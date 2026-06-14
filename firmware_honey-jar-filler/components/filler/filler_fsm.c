#include "filler_fsm.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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
#include "telemetry.h"
#include "filler_strategy.h"

// FSM loop period.
#define FSM_TICK_MS 10

// Consider scale stale if no update for this long.
#define SCALE_STALE_US (2 * 1000 * 1000)

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
static volatile bool s_has_jar_tare = false;
static volatile uint32_t s_run_id = 0;
static float s_jar_tare_g = 0.0f;
static gate_cmd_t s_gate_cmd = GATE_CMD_NONE;
static float s_gate_pct = -1.0f;
static filler_strategy_sample_telemetry_t s_strategy_sample_telemetry = {0};

static void filler_set_state(filler_state_t st)
{
    taskENTER_CRITICAL(&s_lock);
    s_state = st;
    taskEXIT_CRITICAL(&s_lock);
}

static void filler_set_strategy_sample_telemetry(const filler_strategy_sample_telemetry_t *sample)
{
    taskENTER_CRITICAL(&s_lock);
    if (sample) {
        s_strategy_sample_telemetry = *sample;
    } else {
        s_strategy_sample_telemetry = (filler_strategy_sample_telemetry_t){0};
    }
    taskEXIT_CRITICAL(&s_lock);
}

static void filler_clear_strategy_sample_telemetry(void)
{
    filler_set_strategy_sample_telemetry(NULL);
}

static void filler_set_fault(filler_fault_t flt)
{
    filler_fault_t prev;
    uint32_t run_id;
    uint8_t slot;
    taskENTER_CRITICAL(&s_lock);
    prev = s_fault;
    s_fault = flt;
    run_id = s_run_id;
    slot = s_slot_idx;
    taskEXIT_CRITICAL(&s_lock);
    if (flt != prev) {
        ESP_LOGW(TAG, "fault -> %s", filler_fault_name(flt));
        if (flt != FLT_NONE) {
            (void)telemetry_publish_fault(run_id, slot, filler_fault_name(flt));
        }
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

static void jar_tare_clear(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_has_jar_tare = false;
    s_jar_tare_g = 0.0f;
    taskEXIT_CRITICAL(&s_lock);
}

static void jar_tare_set(float grams)
{
    taskENTER_CRITICAL(&s_lock);
    s_has_jar_tare = true;
    s_jar_tare_g = grams;
    taskEXIT_CRITICAL(&s_lock);
}

static bool jar_tare_get(float *out_grams)
{
    bool has;
    taskENTER_CRITICAL(&s_lock);
    has = s_has_jar_tare;
    if (out_grams) {
        *out_grams = s_jar_tare_g;
    }
    taskEXIT_CRITICAL(&s_lock);
    return has;
}

static void filler_stop_all(void)
{
    ESP_LOGD(TAG, "stop all outputs");
    motor_set(false);
    (void)gate_close();
    s_gate_cmd = GATE_CMD_CLOSE;
    s_gate_pct = 0.0f;
}

static void gate_close_cached_label(const char *label)
{
    if (s_gate_cmd != GATE_CMD_CLOSE) {
        (void)gate_close();
        s_gate_cmd = GATE_CMD_CLOSE;
        s_gate_pct = 0.0f;
        (void)telemetry_publish_gate(filler_get_run_id(),
                                     filler_get_slot_idx(),
                                     s_gate_pct,
                                     label ? label : "close");
    }
}

static void gate_close_cached(void)
{
    gate_close_cached_label("close");
}

static void gate_set_percent_cached_label(float pct, const char *label)
{
    if (s_gate_cmd != GATE_CMD_PERCENT || s_gate_pct != pct) {
        (void)gate_set_percent(pct);
        s_gate_cmd = GATE_CMD_PERCENT;
        s_gate_pct = pct;
        (void)telemetry_publish_gate(filler_get_run_id(),
                                     filler_get_slot_idx(),
                                     s_gate_pct,
                                     label ? label : "percent");
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

static uint8_t slot_count_sane(uint8_t configured_slots)
{
    return (configured_slots == 0) ? 1 : configured_slots;
}

static uint8_t slot_next_idx(uint8_t idx, uint8_t configured_slots)
{
    uint8_t slots = slot_count_sane(configured_slots);
    return (uint8_t)((idx + 1) % slots);
}

// Emit one compact end-of-run record that captures the tested preset,
// strategy, final values, and the full runtime parameter snapshot.
static void filler_publish_run_summary(const char *result,
                                       uint32_t run_id,
                                       uint8_t slot_idx,
                                       const char *strategy_name,
                                       const app_params_t *params)
{
    if (!params) return;

    scale_latest_t latest = {0};
    scale_latest_get(&latest);

    float final_weight_g = latest.grams;
    float final_relative_fill_g = final_weight_g;
    float jar_tare_g = 0.0f;
    if (filler_get_jar_tare(&jar_tare_g)) {
        final_relative_fill_g -= jar_tare_g;
    }

    (void)telemetry_publish_run_summary(run_id,
                                        slot_idx,
                                        result,
                                        app_presets_get_name(app_presets_get_active_index()),
                                        strategy_name ? strategy_name : "?",
                                        params,
                                        final_weight_g,
                                        final_relative_fill_g,
                                        CONFIG_HX711_POLL_INTERVAL_MS,
                                        FSM_TICK_MS);
}

static void filler_publish_fill_start(uint32_t run_id,
                                      uint8_t slot_idx,
                                      const char *strategy_name,
                                      const app_params_t *params,
                                      float base_weight_g)
{
    if (!params) return;
    (void)telemetry_publish_fill_start(run_id,
                                       slot_idx,
                                       app_presets_get_name(app_presets_get_active_index()),
                                       strategy_name ? strategy_name : "?",
                                       params,
                                       base_weight_g,
                                       CONFIG_HX711_POLL_INTERVAL_MS,
                                       FSM_TICK_MS);
}

static void filler_publish_fill_summary(uint32_t run_id,
                                        uint8_t slot_idx,
                                        const char *preset_name,
                                        const char *strategy_name,
                                        const app_params_t *params,
                                        const filler_strategy_fill_summary_t *summary)
{
    if (!params || !summary || !summary->valid) return;

    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_FILL_SUMMARY);
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    rec.target_g = (uint32_t)summary->target_g;
    rec.weight_g = summary->final_mass_g;
    rec.relative_fill_g = summary->final_mass_g;
    rec.fill_error_g = summary->fill_error_g;
    rec.measured_dead_time_s = summary->measured_dead_time_s;
    rec.measured_post_close_gain_g = summary->measured_post_close_gain_g;
    rec.measured_fast_rate_gps = summary->measured_fast_rate_gps;
    rec.measured_slow_rate_gps = summary->measured_slow_rate_gps;
    rec.adapted_drip_wait_ms = summary->drip_wait_used_ms;
    rec.refill_count = summary->refill_count;
    rec.learned_dead_time_s = summary->next_dead_time_s;
    rec.learned_post_close_gain_g = summary->next_post_close_gain_g;
    rec.learned_fast_rate_gps = summary->next_fast_rate_gps;
    rec.learned_slow_rate_gps = summary->next_slow_rate_gps;
    rec.adapted_near_close_g = summary->next_near_close_g;
    rec.adapted_close_early_g = summary->next_close_early_g;
    rec.next_drip_wait_ms = summary->next_drip_wait_ms;
    rec.params = *params;
    snprintf(rec.text, sizeof(rec.text), "%s", summary->reason);
    snprintf(rec.preset_name, sizeof(rec.preset_name), "%s", preset_name ? preset_name : "?");
    snprintf(rec.strategy_name, sizeof(rec.strategy_name), "%s", strategy_name ? strategy_name : "?");
    (void)telemetry_publish(&rec);
}

static void task_filler_fsm(void *arg)
{
    (void)arg;

    filler_state_t state = FILLER_IDLE;
    filler_state_t last_state = FILLER_DONE;
    int64_t state_enter_us = esp_timer_get_time();
    uint8_t skipped_slots_streak = 0;
    int64_t last_sample_ts = 0;

    app_params_t params = {0};
    app_fill_strategy_t active_strategy = app_fill_strategy_get_active();
    const filler_strategy_ops_t *strategy_ops = filler_strategy_ops_for_app(active_strategy);
    filler_strategy_runtime_t strategy_rt = {0};
    const filler_strategy_env_t strategy_env = {
        .require_fresh_or_fault = scale_require_fresh_or_fault,
        .set_fault = filler_set_fault,
        .jar_tare_get = jar_tare_get,
        .gate_close_label = gate_close_cached_label,
        .gate_set_percent_label = gate_set_percent_cached_label,
        .set_slot = filler_set_slot,
        .slot_next_idx = slot_next_idx,
        .publish_fill_start = filler_publish_fill_start,
        .set_sample_telemetry = filler_set_strategy_sample_telemetry,
        .clear_sample_telemetry = filler_clear_strategy_sample_telemetry,
        .publish_fill_summary = filler_publish_fill_summary,
    };

    for (;;) {
        app_params_get(&params);

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
            last_sample_ts = 0;
            ESP_LOGI(TAG, "state -> %s", filler_state_name(state));
            (void)telemetry_publish_state(filler_get_run_id(),
                                          filler_get_slot_idx(),
                                          (uint32_t)state,
                                          filler_state_name(state));
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
                filler_clear_strategy_sample_telemetry();
                if (state == FILLER_FAULT) {
                    filler_publish_run_summary("fault",
                                               filler_get_run_id(),
                                               filler_get_slot_idx(),
                                               app_fill_strategy_get_name(active_strategy),
                                               &params);
                    (void)telemetry_publish_run_end(filler_get_run_id(),
                                                    filler_get_slot_idx(),
                                                    "fault");
                }
                filler_stop_all();
                jar_tare_clear();
                skipped_slots_streak = 0;
                break;
            case FILLER_FIND_SLOT:
                filler_clear_strategy_sample_telemetry();
                ESP_LOGD(TAG, "motor on (find slot)");
                motor_set(true);
                jar_tare_clear();
                break;
            case FILLER_VERIFY_EMPTY:
                filler_clear_strategy_sample_telemetry();
                ESP_LOGD(TAG, "verify empty: motor off, gate closed");
                motor_set(false);
                gate_close_cached();
                break;
            case FILLER_SLOT_SETTLE:
                filler_clear_strategy_sample_telemetry();
                ESP_LOGD(TAG, "slot settle: motor off, gate closed");
                motor_set(false);
                gate_close_cached();
                break;
            case FILLER_FILL:
            case FILLER_DRIP_WAIT:
            case FILLER_VERIFY_TARGET: {
                scale_latest_t entry_latest = {0};
                scale_latest_get(&entry_latest);
                filler_strategy_tick_t enter_tick = {
                    .now_us = state_enter_us,
                    .state_enter_us = state_enter_us,
                    .run_id = filler_get_run_id(),
                    .slot_idx = filler_get_slot_idx(),
                    .new_sample = false,
                    .latest = &entry_latest,
                    .params = &params,
                    .preset_index = app_presets_get_active_index(),
                    .preset_name = app_presets_get_name(app_presets_get_active_index()),
                    .strategy_name = app_fill_strategy_get_name(active_strategy),
                };
                if (strategy_ops && strategy_ops->on_enter) {
                    strategy_ops->on_enter(&strategy_rt, state, prev_state, &strategy_env, &enter_tick);
                }
                break;
            }
            }
        }

        // Latest scale snapshot.
        scale_latest_t latest = {0};
        scale_latest_get(&latest);
        float grams = latest.grams;
        bool new_sample = (latest.ts_us != 0 && latest.ts_us != last_sample_ts);
        if (new_sample) {
            last_sample_ts = latest.ts_us;
        }

        switch (state) {
        case FILLER_IDLE:
            // Waiting for start request.
            filler_set_fault(FLT_NONE);
            if (filler_take_start_req()) {
                active_strategy = app_fill_strategy_get_active();
                strategy_ops = filler_strategy_ops_for_app(active_strategy);
                taskENTER_CRITICAL(&s_lock);
                s_run_id++;
                taskEXIT_CRITICAL(&s_lock);
                ESP_LOGI(TAG, "start: carousel_slots=%u strategy=%s",
                         (unsigned)slot_count_sane(params.slots_total),
                         app_fill_strategy_get_name(active_strategy));
                filler_set_slot(0);
                (void)telemetry_publish_run_start(filler_get_run_id(), 0);
                skipped_slots_streak = 0;
                state = FILLER_FIND_SLOT;
                filler_set_state(state);
            }
            break;

        case FILLER_FIND_SLOT:
            // Advance until the position switch goes LOW or timeout.
            if ((esp_timer_get_time() - state_enter_us) >= ((int64_t)params.find_ignore_ms * 1000) &&
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
            if (grams < params.empty_glass_min_g) {
                uint8_t cur = filler_get_slot_idx();
                uint8_t next = slot_next_idx(cur, params.slots_total);
                skipped_slots_streak++;
                ESP_LOGW(TAG, "no jar: %.1f g (min %.1f g) -> skip slot", (double)grams, (double)params.empty_glass_min_g);
                ESP_LOGI(TAG, "slot unavailable: %u -> %u (skip streak=%u/%u)",
                         (unsigned)cur, (unsigned)next,
                         (unsigned)skipped_slots_streak, (unsigned)slot_count_sane(params.slots_total));
                filler_set_slot(next);
                if (skipped_slots_streak >= slot_count_sane(params.slots_total)) {
                    ESP_LOGI(TAG, "stop: full revolution without fillable jar");
                    state = FILLER_DONE;
                } else {
                    state = FILLER_FIND_SLOT;
                }
                filler_set_state(state);
            } else if (grams > params.empty_glass_max_g) {
                uint8_t cur = filler_get_slot_idx();
                uint8_t next = slot_next_idx(cur, params.slots_total);
                skipped_slots_streak++;
                ESP_LOGW(TAG, "jar not empty: %.1f g (max %.1f g) -> skip slot", (double)grams, (double)params.empty_glass_max_g);
                ESP_LOGI(TAG, "slot unavailable: %u -> %u (skip streak=%u/%u)",
                         (unsigned)cur, (unsigned)next,
                         (unsigned)skipped_slots_streak, (unsigned)slot_count_sane(params.slots_total));
                filler_set_slot(next);
                if (skipped_slots_streak >= slot_count_sane(params.slots_total)) {
                    ESP_LOGI(TAG, "stop: full revolution without fillable jar");
                    state = FILLER_DONE;
                } else {
                    state = FILLER_FIND_SLOT;
                }
                filler_set_state(state);
            } else {
                jar_tare_set(grams);
                skipped_slots_streak = 0;
                ESP_LOGI(TAG, "empty jar verified: %.1f g (tare set)", (double)grams);
                state = FILLER_FILL;
                filler_set_state(state);
            }
            break;

        case FILLER_FILL: {
            filler_strategy_tick_t tick = {
                .now_us = esp_timer_get_time(),
                .state_enter_us = state_enter_us,
                .run_id = filler_get_run_id(),
                .slot_idx = filler_get_slot_idx(),
                .new_sample = new_sample,
                .latest = &latest,
                .params = &params,
                .preset_index = app_presets_get_active_index(),
                .preset_name = app_presets_get_name(app_presets_get_active_index()),
                .strategy_name = app_fill_strategy_get_name(active_strategy),
            };
            filler_state_t next_state = strategy_ops ? strategy_ops->step(&strategy_rt, state, &strategy_env, &tick) : state;
            if (next_state != state) {
                if (next_state == FILLER_FIND_SLOT) {
                    skipped_slots_streak = 0;
                }
                state = next_state;
                filler_set_state(state);
            }
            break;
        }

        case FILLER_DRIP_WAIT:
        case FILLER_VERIFY_TARGET: {
            filler_strategy_tick_t tick = {
                .now_us = esp_timer_get_time(),
                .state_enter_us = state_enter_us,
                .run_id = filler_get_run_id(),
                .slot_idx = filler_get_slot_idx(),
                .new_sample = new_sample,
                .latest = &latest,
                .params = &params,
                .preset_index = app_presets_get_active_index(),
                .preset_name = app_presets_get_name(app_presets_get_active_index()),
                .strategy_name = app_fill_strategy_get_name(active_strategy),
            };
            filler_state_t next_state = strategy_ops ? strategy_ops->step(&strategy_rt, state, &strategy_env, &tick) : state;
            if (next_state != state) {
                if (next_state == FILLER_FIND_SLOT) {
                    skipped_slots_streak = 0;
                }
                state = next_state;
                filler_set_state(state);
            }
            break;
        }

        case FILLER_DONE:
            // Nothing fillable found in a full revolution; return to idle.
            filler_publish_run_summary("done",
                                       filler_get_run_id(),
                                       filler_get_slot_idx(),
                                       app_fill_strategy_get_name(active_strategy),
                                       &params);
            (void)telemetry_publish_run_end(filler_get_run_id(), filler_get_slot_idx(), "done");
            state = FILLER_IDLE;
            filler_set_state(state);
            break;

        case FILLER_FAULT:
            // Stop everything and wait for a new start request.
            if (filler_take_start_req()) {
                active_strategy = app_fill_strategy_get_active();
                strategy_ops = filler_strategy_ops_for_app(active_strategy);
                taskENTER_CRITICAL(&s_lock);
                s_run_id++;
                taskEXIT_CRITICAL(&s_lock);
                ESP_LOGI(TAG, "restart after fault (strategy=%s)",
                         app_fill_strategy_get_name(active_strategy));
                filler_set_fault(FLT_NONE);
                filler_set_slot(0);
                (void)telemetry_publish_run_start(filler_get_run_id(), 0);
                skipped_slots_streak = 0;
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

uint32_t filler_get_run_id(void)
{
    uint32_t v;
    taskENTER_CRITICAL(&s_lock);
    v = s_run_id;
    taskEXIT_CRITICAL(&s_lock);
    return v;
}

float filler_get_gate_percent(void)
{
    float v;
    taskENTER_CRITICAL(&s_lock);
    v = s_gate_pct;
    taskEXIT_CRITICAL(&s_lock);
    return v;
}

bool filler_get_jar_tare(float *out_grams)
{
    return jar_tare_get(out_grams);
}

bool filler_get_strategy_sample_telemetry(filler_strategy_sample_telemetry_t *out)
{
    if (!out) return false;
    taskENTER_CRITICAL(&s_lock);
    *out = s_strategy_sample_telemetry;
    taskEXIT_CRITICAL(&s_lock);
    return out->valid;
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
