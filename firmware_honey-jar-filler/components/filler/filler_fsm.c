#include "filler_fsm.h"

#include <stdbool.h>
#include <math.h>

#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app.h"
#include "buzzer.h"
#include "gate.h"
#include "motor.h"
#include "config.h"
#include "scale_hx711.h"

// FSM loop period.
#define FSM_TICK_MS 10

// Consider scale stale if no update for this long.
#define SCALE_STALE_US (2 * 1000 * 1000)

// Target window for VERIFY_TARGET (grams).
#define TARGET_TOL_LOW  60.0f // if below that we swith to fill again to adjust/fill some more
#define TARGET_TOL_HIGH 100.0f


// Minimum motor run time before accepting POS switch (avoid immediate stop).
#define FIND_IGNORE_MS 500
// Step used to relax close-early offset after an underweight retry.
#define CLOSE_EARLY_STEP_G 10.0f

static const char *TAG = "filler_fsm";


typedef enum {
    GATE_CMD_NONE = 0,
    GATE_CMD_OPEN,
    GATE_CMD_CLOSE,
    GATE_CMD_PERCENT
} gate_cmd_t;

static TaskHandle_t s_task;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile filler_state_t s_state = FILLER_IDLE;
static volatile filler_fault_t s_fault = FLT_NONE;
static volatile uint8_t s_slot_idx = 0;
static volatile bool s_start_req = false;
static volatile bool s_abort_req = false;
static gate_cmd_t s_gate_cmd = GATE_CMD_NONE;
static float s_gate_pct = -1.0f;

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
    s_gate_cmd = GATE_CMD_CLOSE;
    s_gate_pct = -1.0f;
}

static void gate_open_cached(void)
{
    if (s_gate_cmd != GATE_CMD_OPEN) {
        (void)gate_open();
        s_gate_cmd = GATE_CMD_OPEN;
        s_gate_pct = -1.0f;
    }
}

static void gate_close_cached(void)
{
    if (s_gate_cmd != GATE_CMD_CLOSE) {
        (void)gate_close();
        s_gate_cmd = GATE_CMD_CLOSE;
        s_gate_pct = -1.0f;
    }
}

static void gate_set_percent_cached(float pct)
{
    if (s_gate_cmd != GATE_CMD_PERCENT || s_gate_pct != pct) {
        (void)gate_set_percent(pct);
        s_gate_cmd = GATE_CMD_PERCENT;
        s_gate_pct = pct;
    }
}

static bool scale_is_stale(const scale_latest_t *s)
{
    if (!s) return true;
    if (s->ts_us == 0) return true;
    int64_t age = esp_timer_get_time() - s->ts_us;
    return age > SCALE_STALE_US;
}

static bool scale_require_fresh_or_fault(const char *ctx,
                                         const scale_latest_t *s,
                                         filler_state_t *state)
{
    if (!scale_is_stale(s)) return true;
    if (ctx) {
        ESP_LOGE(TAG, "scale stale (%s)", ctx);
    } else {
        ESP_LOGE(TAG, "scale stale");
    }
    filler_set_fault(FLT_SCALE_TIMEOUT);
    if (state) {
        *state = FILLER_FAULT;
        filler_set_state(*state);
    } else {
        filler_set_state(FILLER_FAULT);
    }
    return false;
}

static void task_filler_fsm(void *arg)
{
    (void)arg;

    filler_state_t state = FILLER_IDLE;
    filler_state_t last_state = FILLER_DONE;
    int64_t state_enter_us = esp_timer_get_time();
    bool near_close_logged = false;
    float close_early_g_cur = 0.0f;

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
            filler_state_t prev_state = last_state;
            state_enter_us = esp_timer_get_time();
            app_params_get(&params);
            near_close_logged = false;
            ESP_LOGI(TAG, "state -> %s", filler_state_name(state));
            last_state = state;

            // Audible state-change feedback.
            if (state == FILLER_FIND_SLOT && prev_state == FILLER_IDLE) {
                buzzer_beep_long(1); // start
            } else if (state == FILLER_DONE) {
                buzzer_beep_short(3); // finish
            } else if (state == FILLER_FAULT) {
                buzzer_beep_long(2); // failure
            } else {
                buzzer_beep_ms(20); // generic state change
            }

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
                gate_close_cached();
                break;
            case FILLER_SLOT_SETTLE:
                ESP_LOGD(TAG, "slot settle: motor off, gate closed");
                motor_set(false);
                gate_close_cached();
                break;
            case FILLER_FILL:
                ESP_LOGD(TAG, "fill: gate open");
                if (last_state != FILLER_VERIFY_TARGET) {
                    close_early_g_cur = params.close_early_g;
                }
                gate_open_cached();
                break;
            case FILLER_DRIP_WAIT:
                ESP_LOGD(TAG, "drip wait: gate closed");
                gate_close_cached();
                break;
            case FILLER_VERIFY_TARGET:
                ESP_LOGD(TAG, "verify target: gate closed");
                gate_close_cached();
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
                ESP_LOGI(TAG, "start: slots_total=%u", (unsigned)params.slots_total);
                filler_set_slot(0);
                state = FILLER_FIND_SLOT;
                filler_set_state(state);
            }
            break;

        case FILLER_FIND_SLOT:
            // Advance until the position switch goes LOW or timeout.
            if ((esp_timer_get_time() - state_enter_us) >= ((int64_t)FIND_IGNORE_MS * 1000) &&
                gpio_get_level(CONFIG_POS_SWITCH_GPIO) == 0) {
                ESP_LOGI(TAG, "slot found (pos switch low)");
                motor_set(false);
                state = FILLER_SLOT_SETTLE;
                filler_set_state(state);
            } else if ((esp_timer_get_time() - state_enter_us) > ((int64_t)params.advance_timeout_ms * 1000)) {
                motor_set(false);
                filler_set_fault(FLT_MOTOR_TIMEOUT);
                state = FILLER_FAULT;
                filler_set_state(state);
            }
            break;

        case FILLER_SLOT_SETTLE: {
            // Let the motor stop fully before reading the scale.
            int64_t elapsed_us = esp_timer_get_time() - state_enter_us;
            if (elapsed_us >= ((int64_t)params.slot_settle_ms * 1000)) {
                state = FILLER_VERIFY_EMPTY;
                filler_set_state(state);
            }
            break;
        }

        case FILLER_VERIFY_EMPTY:
            // Check for an empty jar; reject heavy or stale readings.
            if (!scale_require_fresh_or_fault("verify empty", &latest, &state)) {
                break;
            }
            if (fabsf(grams) < 180.0f) {
                ESP_LOGI(TAG, "empty jar verified (%.1f g)", (double)grams);
                state = FILLER_FILL;
                filler_set_state(state);
            } else if (grams > (params.target_grams * 0.8f)) {
                ESP_LOGE(TAG, "weight out of range (already full?) (%.1f g)", (double)grams);
                filler_set_fault(FLT_WEIGHT_RANGE);
                state = FILLER_FAULT;
                filler_set_state(state);
            } else {
                ESP_LOGE(TAG, "unexpected weight (partially full?)(%.1f g)", (double)grams);
                filler_set_fault(FLT_WEIGHT_RANGE);
                state = FILLER_FAULT;
                filler_set_state(state);

            }
            break;

        case FILLER_FILL: {
            // Open gate fully, then partially close near target.
            if (!scale_require_fresh_or_fault("fill", &latest, &state)) {
                break;
            }
            float near_close = params.target_grams - params.near_close_delta_g;
            float close_early = params.target_grams - close_early_g_cur;
            if (grams >= params.target_grams) {
                ESP_LOGI(TAG, "target reached: %.1f g", (double)grams);
                gate_close_cached();
                state = FILLER_DRIP_WAIT;
                filler_set_state(state);
            } else if (grams >= close_early) {
                ESP_LOGI(TAG, "close-early reached: %.1f g (offset %.1f g)", (double)grams, (double)close_early_g_cur);
                gate_close_cached();
                state = FILLER_DRIP_WAIT;
                filler_set_state(state);
            } else if (grams >= near_close) {
                if (!near_close_logged) {
                    ESP_LOGD(TAG, "near close: %.1f g -> partial gate", (double)grams);
                    near_close_logged = true;
                }
                gate_set_percent_cached(params.near_close_gate_pct);
            } else {
                gate_open_cached();
            }
            if ((esp_timer_get_time() - state_enter_us) > ((int64_t)params.fill_timeout_ms * 1000)) {
                ESP_LOGE(TAG, "fill timeout");
                gate_close_cached();
                filler_set_fault(FLT_SERVO_TIMEOUT);
                state = FILLER_FAULT;
                filler_set_state(state);
            }
            break;
        }

        case FILLER_DRIP_WAIT: {
            // Wait for honey to drip after closing gate.
            int64_t elapsed_us = esp_timer_get_time() - state_enter_us;
            if (elapsed_us >= ((int64_t)params.drip_delay_ms * 1000)) {
                state = FILLER_VERIFY_TARGET;
                filler_set_state(state);
            }
            break;
        }

        case FILLER_VERIFY_TARGET: {
            // Check target window; re-fill if low, fault if high.
            if (!scale_require_fresh_or_fault("verify target", &latest, &state)) {
                break;
            }
            if (grams > (params.target_grams + TARGET_TOL_HIGH)) {
                float over = grams - params.target_grams;
                ESP_LOGE(TAG, "overweight: %.1f g (+%.1f g)", (double)grams, (double)over);
                ESP_LOGW(TAG, "suggestion: increase close_early_g (now %.1f g)", (double)params.close_early_g);
                filler_set_fault(FLT_WEIGHT_RANGE);
                state = FILLER_FAULT;
                filler_set_state(state);
            } else if (grams < (params.target_grams - TARGET_TOL_LOW)) {
                float under = params.target_grams - grams;
                ESP_LOGI(TAG, "underweight: %.1f g (-%.1f g) -> refill", (double)grams, (double)under);
                if (close_early_g_cur > 0.0f) {
                    float prev = close_early_g_cur;
                    close_early_g_cur -= CLOSE_EARLY_STEP_G;
                    if (close_early_g_cur < 0.0f) close_early_g_cur = 0.0f;
                    ESP_LOGI(TAG, "relax close_early_g: %.1f -> %.1f g", (double)prev, (double)close_early_g_cur);
                    ESP_LOGW(TAG, "suggestion: decrease close_early_g (now %.1f g)", (double)params.close_early_g);
                }
                state = FILLER_FILL;
                filler_set_state(state);
            } else {
                ESP_LOGI(TAG, "target verified: %.1f g", (double)grams);
                uint8_t cur = filler_get_slot_idx();
                if ((cur + 1) >= params.slots_total) {
                    state = FILLER_DONE;
                } else {
                    state = FILLER_ADVANCE_NEXT;
                }
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
                ESP_LOGI(TAG, "advance: slot %u -> %u (total=%u)",
                         (unsigned)cur, (unsigned)next, (unsigned)params.slots_total);
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
    case FILLER_SLOT_SETTLE:   return "SLOT_SETTLE";
    case FILLER_DRIP_WAIT:     return "DRIP_WAIT";
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
