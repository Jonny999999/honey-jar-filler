#include "filler_fsm.h"

#include <stdbool.h>
#include <math.h>

#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app.h"
#include "gate.h"
#include "motor.h"
#include "config.h"
#include "scale_hx711.h"

// FSM loop period.
#define FSM_TICK_MS 10

// Consider scale stale if no update for this long.
#define SCALE_STALE_US (2 * 1000 * 1000)

// Target window for VERIFY_TARGET (grams).
#define TARGET_TOL_LOW  3.0f
#define TARGET_TOL_HIGH 5.0f

static const char *TAG = "filler_fsm";

static TaskHandle_t s_task;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile filler_state_t s_state = FILLER_IDLE;
static volatile filler_fault_t s_fault = FLT_NONE;
static volatile uint8_t s_slot_idx = 0;
static volatile bool s_start_req = false;
static volatile bool s_abort_req = false;

static void filler_set_state(filler_state_t st)
{
    taskENTER_CRITICAL(&s_lock);
    s_state = st;
    taskEXIT_CRITICAL(&s_lock);
}

static void filler_set_fault(filler_fault_t flt)
{
    filler_fault_t prev;
    taskENTER_CRITICAL(&s_lock);
    prev = s_fault;
    s_fault = flt;
    taskEXIT_CRITICAL(&s_lock);
    if (flt != prev) {
        ESP_LOGW(TAG, "fault -> %s", filler_fault_name(flt));
    }
}

static bool filler_take_start_req(void)
{
    bool v;
    taskENTER_CRITICAL(&s_lock);
    v = s_start_req;
    s_start_req = false;
    taskEXIT_CRITICAL(&s_lock);
    return v;
}

static bool filler_take_abort_req(void)
{
    bool v;
    taskENTER_CRITICAL(&s_lock);
    v = s_abort_req;
    s_abort_req = false;
    taskEXIT_CRITICAL(&s_lock);
    return v;
}

static void filler_set_slot(uint8_t idx)
{
    taskENTER_CRITICAL(&s_lock);
    s_slot_idx = idx;
    taskEXIT_CRITICAL(&s_lock);
}

static void filler_stop_all(void)
{
    ESP_LOGD(TAG, "stop all outputs");
    motor_set(false);
    (void)gate_close();
}

static bool scale_is_stale(const scale_latest_t *s)
{
    if (!s) return true;
    if (s->ts_us == 0) return true;
    int64_t age = esp_timer_get_time() - s->ts_us;
    return age > SCALE_STALE_US;
}

static void task_filler_fsm(void *arg)
{
    (void)arg;

    filler_state_t state = FILLER_IDLE;
    filler_state_t last_state = FILLER_DONE;
    int64_t state_enter_us = esp_timer_get_time();
    bool target_adjusted = false;
    bool adjust_active = false;
    int64_t adjust_end_us = 0;

    app_params_t params = {0};

    for (;;) {
        // Handle abort at any time.
        if (filler_take_abort_req()) {
            ESP_LOGW(TAG, "abort requested");
            filler_set_fault(FLT_USER_ABORT);
            filler_stop_all();
            filler_set_state(FILLER_FAULT);
            state = FILLER_FAULT;
        }

        // State entry handling.
        if (state != last_state) {
            state_enter_us = esp_timer_get_time();
            app_params_get(&params);
            target_adjusted = false;
            adjust_active = false;
            adjust_end_us = 0;
            ESP_LOGI(TAG, "state -> %s", filler_state_name(state));
            last_state = state;

            // Per-state entry actions.
            switch (state) {
            case FILLER_IDLE:
            case FILLER_DONE:
            case FILLER_FAULT:
                filler_stop_all();
                break;
            case FILLER_FIND_SLOT:
                ESP_LOGD(TAG, "motor on (find slot)");
                motor_set(true);
                break;
            case FILLER_VERIFY_EMPTY:
                ESP_LOGD(TAG, "verify empty: motor off, gate closed");
                motor_set(false);
                (void)gate_close();
                break;
            case FILLER_FILL:
                ESP_LOGD(TAG, "fill: gate open");
                (void)gate_open();
                break;
            case FILLER_VERIFY_TARGET:
                ESP_LOGD(TAG, "verify target: gate closed");
                (void)gate_close();
                break;
            case FILLER_ADVANCE_NEXT:
                ESP_LOGD(TAG, "advance: motor on");
                motor_set(true);
                break;
            }
        }

        // Latest scale snapshot.
        scale_latest_t latest = {0};
        scale_latest_get(&latest);
        float grams = latest.grams;

        switch (state) {
        case FILLER_IDLE:
            // Waiting for start request.
            filler_set_fault(FLT_NONE);
            if (filler_take_start_req()) {
                filler_set_slot(0);
                state = FILLER_FIND_SLOT;
                filler_set_state(state);
            }
            break;

        case FILLER_FIND_SLOT:
            // Advance until the position switch goes LOW or timeout.
            if (gpio_get_level(CONFIG_POS_SWITCH_GPIO) == 0) {
                ESP_LOGI(TAG, "slot found (pos switch low)");
                motor_set(false);
                state = FILLER_VERIFY_EMPTY;
                filler_set_state(state);
            } else if ((esp_timer_get_time() - state_enter_us) > ((int64_t)params.advance_timeout_ms * 1000)) {
                motor_set(false);
                filler_set_fault(FLT_MOTOR_TIMEOUT);
                state = FILLER_FAULT;
                filler_set_state(state);
            }
            break;

        case FILLER_VERIFY_EMPTY:
            // Check for an empty jar; reject heavy or stale readings.
            if (scale_is_stale(&latest)) {
                ESP_LOGE(TAG, "scale stale during verify empty");
                filler_set_fault(FLT_SCALE_TIMEOUT);
                state = FILLER_FAULT;
                filler_set_state(state);
                break;
            }
            if (fabsf(grams) < 50.0f) {
                ESP_LOGI(TAG, "empty jar verified (%.1f g)", (double)grams);
                state = FILLER_FILL;
                filler_set_state(state);
            } else if (grams > (params.target_grams * 0.6f)) {
                ESP_LOGE(TAG, "weight out of range (%.1f g)", (double)grams);
                filler_set_fault(FLT_WEIGHT_RANGE);
                state = FILLER_FAULT;
                filler_set_state(state);
            }
            break;

        case FILLER_FILL: {
            // Open gate fully, then partially close near target.
            if (scale_is_stale(&latest)) {
                ESP_LOGE(TAG, "scale stale during fill");
                filler_set_fault(FLT_SCALE_TIMEOUT);
                state = FILLER_FAULT;
                filler_set_state(state);
                break;
            }
            float near_close = params.target_grams - params.near_close_delta_g;
            if (grams >= params.target_grams) {
                ESP_LOGI(TAG, "target reached: %.1f g", (double)grams);
                (void)gate_close();
                state = FILLER_VERIFY_TARGET;
                filler_set_state(state);
            } else if (grams >= near_close) {
                ESP_LOGD(TAG, "near close: %.1f g -> partial gate", (double)grams);
                (void)gate_set_percent(50.0f);
            } else {
                (void)gate_open();
            }
            if ((esp_timer_get_time() - state_enter_us) > ((int64_t)params.fill_timeout_ms * 1000)) {
                ESP_LOGE(TAG, "fill timeout");
                (void)gate_close();
                filler_set_fault(FLT_SERVO_TIMEOUT);
                state = FILLER_FAULT;
                filler_set_state(state);
            }
            break;
        }

        case FILLER_VERIFY_TARGET: {
            // Dwell, then accept target within tolerance; allow one correction.
            if (scale_is_stale(&latest)) {
                ESP_LOGE(TAG, "scale stale during verify target");
                filler_set_fault(FLT_SCALE_TIMEOUT);
                state = FILLER_FAULT;
                filler_set_state(state);
                break;
            }
            if (adjust_active) {
                if (esp_timer_get_time() >= adjust_end_us) {
                    ESP_LOGD(TAG, "top-up done, closing gate");
                    (void)gate_close();
                    adjust_active = false;
                    state_enter_us = esp_timer_get_time();
                }
                break;
            }
            int64_t elapsed_us = esp_timer_get_time() - state_enter_us;
            if (elapsed_us < ((int64_t)params.motor_dwell_ms * 1000)) {
                break;
            }

            if ((grams >= (params.target_grams - TARGET_TOL_LOW)) &&
                (grams <= (params.target_grams + TARGET_TOL_HIGH))) {
                ESP_LOGI(TAG, "target verified: %.1f g", (double)grams);
                state = FILLER_ADVANCE_NEXT;
                filler_set_state(state);
            } else if (!target_adjusted && grams < (params.target_grams - TARGET_TOL_LOW)) {
                // Single small top-up attempt (non-blocking).
                ESP_LOGI(TAG, "top-up: %.1f g (target %.1f)", (double)grams, (double)params.target_grams);
                (void)gate_set_percent(30.0f);
                adjust_active = true;
                adjust_end_us = esp_timer_get_time() + (300 * 1000);
                target_adjusted = true;
            } else {
                ESP_LOGE(TAG, "target verify failed: %.1f g", (double)grams);
                filler_set_fault(FLT_WEIGHT_RANGE);
                state = FILLER_FAULT;
                filler_set_state(state);
            }
            break;
        }

        case FILLER_ADVANCE_NEXT: {
            // Pulse motor to advance to next slot.
            int64_t elapsed_us = esp_timer_get_time() - state_enter_us;
            if (elapsed_us >= ((int64_t)params.motor_dwell_ms * 1000)) {
                motor_set(false);
                uint8_t cur = filler_get_slot_idx();
                uint8_t next = cur + 1;
                ESP_LOGI(TAG, "advance: slot %u -> %u", (unsigned)cur, (unsigned)next);
                filler_set_slot(next);
                if (next >= params.slots_total) {
                    state = FILLER_DONE;
                } else {
                    state = FILLER_FIND_SLOT;
                }
                filler_set_state(state);
            }
            break;
        }

        case FILLER_DONE:
            // Finished all slots; return to idle.
            state = FILLER_IDLE;
            filler_set_state(state);
            break;

        case FILLER_FAULT:
            // Stop everything and wait for a new start request.
            if (filler_take_start_req()) {
                ESP_LOGI(TAG, "restart after fault");
                filler_set_fault(FLT_NONE);
                filler_set_slot(0);
                state = FILLER_FIND_SLOT;
                filler_set_state(state);
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(FSM_TICK_MS));
    }
}

void filler_start_task(UBaseType_t prio, BaseType_t core)
{
    if (s_task) return;
    xTaskCreatePinnedToCore(task_filler_fsm, "filler_fsm", 4096, NULL, prio, &s_task, core);
}

void filler_request_start(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_start_req = true;
    taskEXIT_CRITICAL(&s_lock);
}

void filler_request_abort(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_abort_req = true;
    taskEXIT_CRITICAL(&s_lock);
}

filler_state_t filler_get_state(void)
{
    filler_state_t v;
    taskENTER_CRITICAL(&s_lock);
    v = s_state;
    taskEXIT_CRITICAL(&s_lock);
    return v;
}

uint8_t filler_get_slot_idx(void)
{
    uint8_t v;
    taskENTER_CRITICAL(&s_lock);
    v = s_slot_idx;
    taskEXIT_CRITICAL(&s_lock);
    return v;
}

filler_fault_t filler_get_fault(void)
{
    filler_fault_t v;
    taskENTER_CRITICAL(&s_lock);
    v = s_fault;
    taskEXIT_CRITICAL(&s_lock);
    return v;
}

const char *filler_state_name(filler_state_t st)
{
    switch (st) {
    case FILLER_IDLE:          return "IDLE";
    case FILLER_FIND_SLOT:     return "FIND_SLOT";
    case FILLER_VERIFY_EMPTY:  return "VERIFY_EMPTY";
    case FILLER_FILL:          return "FILL";
    case FILLER_VERIFY_TARGET: return "VERIFY_TARGET";
    case FILLER_ADVANCE_NEXT:  return "ADVANCE_NEXT";
    case FILLER_DONE:          return "DONE";
    case FILLER_FAULT:         return "FAULT";
    default:                   return "?";
    }
}

const char *filler_fault_name(filler_fault_t flt)
{
    switch (flt) {
    case FLT_NONE:          return "NONE";
    case FLT_NO_JAR:        return "NO_JAR";
    case FLT_WEIGHT_RANGE:  return "WEIGHT_RANGE";
    case FLT_MOTOR_TIMEOUT: return "MOTOR_TIMEOUT";
    case FLT_SERVO_TIMEOUT: return "SERVO_TIMEOUT";
    case FLT_SCALE_TIMEOUT: return "SCALE_TIMEOUT";
    case FLT_EMPTY_HONEY:   return "EMPTY_HONEY";
    case FLT_USER_ABORT:    return "USER_ABORT";
    default:                return "?";
    }
}
