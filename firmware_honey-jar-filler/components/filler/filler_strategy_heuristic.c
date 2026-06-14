#include "filler_strategy.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

// Step used to relax close-early offset after an underweight retry (grams).
#define CLOSE_EARLY_STEP_G 4.0f

// Require consecutive samples to confirm threshold crossings.
#define THRESH_CONFIRM_COUNT 4

// Derivative-based measurement is shared with the adaptive strategy so both
// modes can later be compared using similar thesis plots.
#define RATE_FILTER_ALPHA 0.18f
#define RESPONSE_THRESHOLD_MIN_G 1.5f
#define RESPONSE_THRESHOLD_MAX_G 6.0f
#define DELTA_RATE_MIN_DT_S 0.05f
#define DELTA_RATE_MAX_DT_S 0.80f

static const char *TAG = "fill_strategy";

static float clampf_local(float value, float min_v, float max_v)
{
    if (value < min_v) return min_v;
    if (value > max_v) return max_v;
    return value;
}

static float ewma(float prev, float sample, float alpha)
{
    if (prev <= 0.0f) return sample;
    return prev + alpha * (sample - prev);
}

static bool stable_above(float value, float threshold, uint8_t *count, uint8_t required)
{
    if (!count || required == 0) return value >= threshold;
    if (value >= threshold) {
        if (*count < required) {
            (*count)++;
            ESP_LOGD(TAG, "stable_above: %.1f >= %.1f -> %u/%u",
                     (double)value, (double)threshold,
                     (unsigned)*count, (unsigned)required);
        }
    } else {
        if (*count != 0) {
            ESP_LOGD(TAG, "stable_above: %.1f < %.1f -> reset",
                     (double)value, (double)threshold);
        }
        *count = 0;
    }
    if (*count >= required) {
        ESP_LOGD(TAG, "stable_above: %.1f >= %.1f -> true",
                 (double)value, (double)threshold);
    }
    return *count >= required;
}

static bool stable_below(float value, float threshold, uint8_t *count, uint8_t required)
{
    if (!count || required == 0) return value <= threshold;
    if (value <= threshold) {
        if (*count < required) {
            (*count)++;
            ESP_LOGD(TAG, "stable_below: %.1f <= %.1f -> %u/%u",
                     (double)value, (double)threshold,
                     (unsigned)*count, (unsigned)required);
        }
    } else {
        if (*count != 0) {
            ESP_LOGD(TAG, "stable_below: %.1f > %.1f -> reset",
                     (double)value, (double)threshold);
        }
        *count = 0;
    }
    if (*count >= required) {
        ESP_LOGD(TAG, "stable_below: %.1f <= %.1f -> true",
                 (double)value, (double)threshold);
    }
    return *count >= required;
}

static const char *heuristic_phase_name(const filler_strategy_runtime_t *rt)
{
    if (!rt) return "closed";
    if (rt->first_close_seen) return "closed";
    if (rt->refill_count > 0 && !rt->near_close_logged) return "refill";
    if (rt->near_close_logged) return "reduced";
    return "fast";
}

static float response_threshold_g(const filler_strategy_tick_t *tick)
{
    float candidate = tick && tick->params ? ((float)tick->params->target_grams * 0.005f) : 2.0f;
    return clampf_local(candidate, RESPONSE_THRESHOLD_MIN_G, RESPONSE_THRESHOLD_MAX_G);
}

static float current_close_early_g(const filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!rt || !tick || !tick->params) return 0.0f;
    float value = (float)tick->params->close_early_g - rt->close_early_relax_g;
    return (value > 0.0f) ? value : 0.0f;
}

static void update_rate_estimates(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!rt || !tick || !tick->new_sample || !tick->latest) return;

    float rel_g = tick->latest->grams - rt->run_base_weight_g;

    if (rt->last_rate_ts_us != 0) {
        float dt_s = (float)(tick->latest->ts_us - rt->last_rate_ts_us) / 1000000.0f;
        if (dt_s >= DELTA_RATE_MIN_DT_S && dt_s <= DELTA_RATE_MAX_DT_S) {
            float delta_g = rel_g - rt->last_rel_g;
            if (delta_g < -0.5f) delta_g = 0.0f;
            rt->raw_rate_gps = delta_g > 0.0f ? (delta_g / dt_s) : 0.0f;
            rt->filtered_rate_gps = ewma(rt->filtered_rate_gps, rt->raw_rate_gps, RATE_FILTER_ALPHA);

            const char *phase = heuristic_phase_name(rt);
            if ((strcmp(phase, "fast") == 0 || strcmp(phase, "refill") == 0) && rt->raw_rate_gps > 0.1f) {
                rt->fast_rate_sum_gps += rt->raw_rate_gps;
                rt->fast_rate_count++;
            } else if (strcmp(phase, "reduced") == 0 && rt->raw_rate_gps > 0.1f) {
                rt->slow_rate_sum_gps += rt->raw_rate_gps;
                rt->slow_rate_count++;
            }
        }
    }

    if (!rt->response_detected && rel_g >= response_threshold_g(tick)) {
        rt->response_detected = true;
        rt->first_response_ts_us = tick->latest->ts_us;
        rt->measured_dead_time_s = clampf_local((float)(tick->latest->ts_us - rt->fill_open_ts_us) / 1000000.0f,
                                                0.10f,
                                                3.00f);
        ESP_LOGI(TAG, "detected dead time: %.3f s", (double)rt->measured_dead_time_s);
    }

    rt->last_rel_g = rel_g;
    rt->last_rate_ts_us = tick->latest->ts_us;
}

static void track_post_close_gain(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!rt || !tick || !tick->latest || !tick->new_sample || !rt->first_close_seen) return;
    float rel_g = tick->latest->grams - rt->run_base_weight_g;
    float gain_g = rel_g - rt->rel_at_first_close_g;
    if (gain_g < 0.0f) gain_g = 0.0f;
    rt->measured_post_close_gain_g = gain_g;
    if (gain_g > rt->last_post_close_gain_g + 0.25f) {
        rt->last_gain_ts_us = tick->latest->ts_us;
    }
    rt->last_post_close_gain_g = gain_g;
}

static void mark_first_close(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!rt || !tick || !tick->latest || rt->first_close_seen) return;
    rt->first_close_seen = true;
    rt->first_close_ts_us = tick->latest->ts_us;
    rt->rel_at_first_close_g = tick->latest->grams - rt->run_base_weight_g;
    rt->last_gain_ts_us = rt->first_close_ts_us;
    rt->rate_at_close_gps = (rt->filtered_rate_gps > 0.0f) ? rt->filtered_rate_gps : rt->raw_rate_gps;
}

static void publish_runtime_snapshot(filler_strategy_runtime_t *rt,
                                     const filler_strategy_env_t *env,
                                     const filler_strategy_tick_t *tick,
                                     float near_close_delta_g,
                                     float close_early_g_cur,
                                     float drip_wait_ms)
{
    if (!rt || !env || !env->set_sample_telemetry || !tick) return;

    filler_strategy_sample_telemetry_t sample = {
        .valid = true,
        .raw_rate_gps = rt->raw_rate_gps,
        .filtered_rate_gps = rt->filtered_rate_gps,
        .measured_dead_time_s = rt->measured_dead_time_s,
        .measured_post_close_gain_g = rt->measured_post_close_gain_g,
        .adapted_near_close_g = near_close_delta_g,
        .adapted_close_early_g = close_early_g_cur,
        .adapted_drip_wait_ms = drip_wait_ms,
        .refill_count = rt->refill_count,
    };
    snprintf(sample.strategy_name, sizeof(sample.strategy_name), "%s", tick->strategy_name ? tick->strategy_name : "?");
    snprintf(sample.gate_phase, sizeof(sample.gate_phase), "%s", heuristic_phase_name(rt));
    env->set_sample_telemetry(&sample);
}

static void publish_summary(filler_strategy_runtime_t *rt,
                            const filler_strategy_env_t *env,
                            const filler_strategy_tick_t *tick,
                            const char *reason,
                            bool success)
{
    if (!rt || !env || !env->publish_fill_summary || !tick || !tick->latest || !tick->params) return;

    float final_rel_g = tick->latest->grams - rt->run_base_weight_g;
    float fast_avg = (rt->fast_rate_count > 0) ? (rt->fast_rate_sum_gps / (float)rt->fast_rate_count) : 0.0f;
    float slow_avg = (rt->slow_rate_count > 0) ? (rt->slow_rate_sum_gps / (float)rt->slow_rate_count) : 0.0f;
    float fill_duration_s = (rt->fill_open_ts_us > 0)
        ? ((float)(tick->now_us - rt->fill_open_ts_us) / 1000000.0f)
        : 0.0f;

    filler_strategy_fill_summary_t summary = {
        .valid = true,
        .final_mass_g = final_rel_g,
        .target_g = (float)tick->params->target_grams,
        .fill_error_g = final_rel_g - (float)tick->params->target_grams,
        .measured_dead_time_s = rt->measured_dead_time_s,
        .measured_post_close_gain_g = rt->measured_post_close_gain_g,
        .measured_fast_rate_gps = fast_avg,
        .measured_slow_rate_gps = slow_avg,
        .rate_at_close_gps = rt->rate_at_close_gps,
        .fill_duration_s = fill_duration_s,
        .drip_wait_used_ms = (float)tick->params->drip_delay_ms,
        .refill_count = rt->refill_count,
    };
    snprintf(summary.reason, sizeof(summary.reason), "%s", reason ? reason : (success ? "ok" : "fault"));
    env->publish_fill_summary(tick->run_id,
                              tick->slot_idx,
                              tick->preset_name,
                              tick->strategy_name,
                              tick->params,
                              &summary);
}

static void heuristic_reset_counters(filler_strategy_runtime_t *rt)
{
    if (!rt) return;
    rt->near_close_logged = false;
    rt->cnt_near_close = 0;
    rt->cnt_close_early = 0;
    rt->cnt_target = 0;
    rt->cnt_under = 0;
    rt->cnt_over = 0;
    rt->sample_count = 0;
}

static void heuristic_on_enter(filler_strategy_runtime_t *rt,
                               filler_state_t state,
                               filler_state_t prev_state,
                               const filler_strategy_env_t *env,
                               const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick) return;

    heuristic_reset_counters(rt);

    switch (state) {
    case FILLER_FILL:
        ESP_LOGD(TAG, "fill: gate open");
        if (prev_state != FILLER_VERIFY_TARGET) {
            memset(rt, 0, sizeof(*rt));
            rt->close_early_relax_g = 0.0f;
            rt->fill_open_ts_us = tick->now_us;
            rt->target_g = (float)tick->params->target_grams;
            if (!env->jar_tare_get || !env->jar_tare_get(&rt->run_base_weight_g)) {
                rt->run_base_weight_g = tick->latest ? tick->latest->grams : 0.0f;
            }
        } else {
            rt->refill_count++;
            rt->first_close_seen = false;
            rt->first_close_ts_us = 0;
            rt->last_gain_ts_us = 0;
            rt->rel_at_first_close_g = 0.0f;
            rt->measured_post_close_gain_g = 0.0f;
            rt->last_post_close_gain_g = 0.0f;
        }
        rt->last_rate_ts_us = 0;
        rt->last_rel_g = 0.0f;
        env->publish_fill_start(tick->run_id,
                                tick->slot_idx,
                                tick->strategy_name,
                                tick->params,
                                tick->latest ? tick->latest->grams : 0.0f);
        env->gate_set_percent_label(tick->params->max_gate_pct, "max_gate");
        publish_runtime_snapshot(rt, env, tick,
                                 (float)tick->params->near_close_delta_g,
                                 current_close_early_g(rt, tick),
                                 (float)tick->params->drip_delay_ms);
        break;
    case FILLER_DRIP_WAIT:
        ESP_LOGD(TAG, "drip wait: gate closed");
        mark_first_close(rt, tick);
        env->gate_close_label("drip_wait");
        publish_runtime_snapshot(rt, env, tick,
                                 (float)tick->params->near_close_delta_g,
                                 current_close_early_g(rt, tick),
                                 (float)tick->params->drip_delay_ms);
        break;
    case FILLER_VERIFY_TARGET:
        ESP_LOGD(TAG, "verify target: gate closed");
        env->gate_close_label("close");
        publish_runtime_snapshot(rt, env, tick,
                                 (float)tick->params->near_close_delta_g,
                                 current_close_early_g(rt, tick),
                                 (float)tick->params->drip_delay_ms);
        break;
    default:
        break;
    }
}

static filler_state_t heuristic_step(filler_strategy_runtime_t *rt,
                                     filler_state_t state,
                                     const filler_strategy_env_t *env,
                                     const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick || !tick->latest || !tick->params) return state;

    if (tick->new_sample && rt->sample_count < 255) {
        rt->sample_count++;
    }

    switch (state) {
    case FILLER_FILL: {
        filler_state_t next_state = state;
        if (!env->require_fresh_or_fault("fill", tick->latest, &next_state)) {
            publish_summary(rt, env, tick, "scale_stale", false);
            env->clear_sample_telemetry();
            return next_state;
        }

        float tare_g = 0.0f;
        bool has_tare = env->jar_tare_get(&tare_g);
        float rel_g = tick->latest->grams - (has_tare ? tare_g : 0.0f);
        update_rate_estimates(rt, tick);
        float close_early_g_cur = current_close_early_g(rt, tick);
        float near_close = (float)tick->params->target_grams - (float)tick->params->near_close_delta_g;
        float close_early = (float)tick->params->target_grams - close_early_g_cur;

        if ((tick->now_us - tick->state_enter_us) > ((int64_t)tick->params->fill_timeout_ms * 1000)) {
            ESP_LOGE(TAG, "fill timeout");
            env->gate_close_label("close");
            env->set_fault(FLT_SERVO_TIMEOUT);
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick,
                                     (float)tick->params->near_close_delta_g,
                                     close_early_g_cur,
                                     (float)tick->params->drip_delay_ms);
            publish_summary(rt, env, tick, "fill_timeout", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }
        if (!tick->new_sample) {
            publish_runtime_snapshot(rt, env, tick,
                                     (float)tick->params->near_close_delta_g,
                                     close_early_g_cur,
                                     (float)tick->params->drip_delay_ms);
            return state;
        }
        if (stable_above(rel_g, (float)tick->params->target_grams, &rt->cnt_target, THRESH_CONFIRM_COUNT)) {
            ESP_LOGI(TAG, "target reached: rel=%.1f g abs=%.1f g",
                     (double)rel_g, (double)tick->latest->grams);
            env->gate_close_label("target");
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick,
                                     (float)tick->params->near_close_delta_g,
                                     close_early_g_cur,
                                     (float)tick->params->drip_delay_ms);
            return FILLER_DRIP_WAIT;
        }
        if (stable_above(rel_g, close_early, &rt->cnt_close_early, THRESH_CONFIRM_COUNT)) {
            ESP_LOGI(TAG, "close-early reached: rel=%.1f g (offset %.1f g)",
                     (double)rel_g, (double)close_early_g_cur);
            env->gate_close_label("close_early");
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick,
                                     (float)tick->params->near_close_delta_g,
                                     close_early_g_cur,
                                     (float)tick->params->drip_delay_ms);
            return FILLER_DRIP_WAIT;
        }
        if (stable_above(rel_g, near_close, &rt->cnt_near_close, THRESH_CONFIRM_COUNT)) {
            if (!rt->near_close_logged) {
                ESP_LOGD(TAG, "near close: rel=%.1f g -> partial gate", (double)rel_g);
                rt->near_close_logged = true;
            }
            env->gate_set_percent_label(tick->params->near_close_gate_pct, "near_close");
            publish_runtime_snapshot(rt, env, tick,
                                     (float)tick->params->near_close_delta_g,
                                     close_early_g_cur,
                                     (float)tick->params->drip_delay_ms);
            return state;
        }

        env->gate_set_percent_label(tick->params->max_gate_pct, "max_gate");
        publish_runtime_snapshot(rt, env, tick,
                                 (float)tick->params->near_close_delta_g,
                                 close_early_g_cur,
                                 (float)tick->params->drip_delay_ms);
        return state;
    }

    case FILLER_DRIP_WAIT:
        track_post_close_gain(rt, tick);
        publish_runtime_snapshot(rt, env, tick,
                                 (float)tick->params->near_close_delta_g,
                                 current_close_early_g(rt, tick),
                                 (float)tick->params->drip_delay_ms);
        if ((tick->now_us - tick->state_enter_us) >= ((int64_t)tick->params->drip_delay_ms * 1000)) {
            return FILLER_VERIFY_TARGET;
        }
        return state;

    case FILLER_VERIFY_TARGET: {
        filler_state_t next_state = state;
        if (!env->require_fresh_or_fault("verify target", tick->latest, &next_state)) {
            publish_summary(rt, env, tick, "scale_stale", false);
            env->clear_sample_telemetry();
            return next_state;
        }

        float tare_g = 0.0f;
        bool has_tare = env->jar_tare_get(&tare_g);
        float rel_g = tick->latest->grams - (has_tare ? tare_g : 0.0f);
        float tol_low_g = (float)tick->params->target_tol_low_g;
        float tol_high_g = (float)tick->params->target_tol_high_g;
        track_post_close_gain(rt, tick);

        if (tick->new_sample &&
            stable_above(rel_g, (float)tick->params->target_grams + tol_high_g, &rt->cnt_over, THRESH_CONFIRM_COUNT)) {
            float over = rel_g - (float)tick->params->target_grams;
            ESP_LOGE(TAG, "overweight: rel=%.1f g (+%.1f g, tol=+%.1f g)",
                     (double)rel_g, (double)over, (double)tol_high_g);
            ESP_LOGW(TAG, "suggestion: increase close_early_g (now %u g)",
                     (unsigned)tick->params->close_early_g);
            env->set_fault(FLT_WEIGHT_RANGE);
            publish_runtime_snapshot(rt, env, tick,
                                     (float)tick->params->near_close_delta_g,
                                     current_close_early_g(rt, tick),
                                     (float)tick->params->drip_delay_ms);
            publish_summary(rt, env, tick, "overweight", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }

        if (tick->new_sample &&
            stable_below(rel_g, (float)tick->params->target_grams - tol_low_g, &rt->cnt_under, THRESH_CONFIRM_COUNT)) {
            float under = (float)tick->params->target_grams - rel_g;
            ESP_LOGI(TAG, "underweight: rel=%.1f g (-%.1f g, tol=-%.1f g) -> refill",
                     (double)rel_g, (double)under, (double)tol_low_g);

            float close_early_g_cur = (float)tick->params->close_early_g - rt->close_early_relax_g;
            if (close_early_g_cur > 0.0f) {
                float prev_g = close_early_g_cur;
                rt->close_early_relax_g += CLOSE_EARLY_STEP_G;
                if (rt->close_early_relax_g > (float)tick->params->close_early_g) {
                    rt->close_early_relax_g = (float)tick->params->close_early_g;
                }
                close_early_g_cur = (float)tick->params->close_early_g - rt->close_early_relax_g;
                if (close_early_g_cur < 0.0f) close_early_g_cur = 0.0f;
                ESP_LOGI(TAG, "relax close_early: %.1f g -> %.1f g",
                         (double)prev_g, (double)close_early_g_cur);
                ESP_LOGW(TAG, "suggestion: decrease close_early_g (now %u g)",
                         (unsigned)tick->params->close_early_g);
            }
            publish_runtime_snapshot(rt, env, tick,
                                     (float)tick->params->near_close_delta_g,
                                     close_early_g_cur,
                                     (float)tick->params->drip_delay_ms);
            return FILLER_FILL;
        }

        if (rt->sample_count >= THRESH_CONFIRM_COUNT) {
            uint8_t next = env->slot_next_idx(tick->slot_idx, tick->params->slots_total);
            ESP_LOGI(TAG, "target verified: rel=%.1f g", (double)rel_g);
            ESP_LOGI(TAG, "slot complete: %u -> %u", (unsigned)tick->slot_idx, (unsigned)next);
            publish_runtime_snapshot(rt, env, tick,
                                     (float)tick->params->near_close_delta_g,
                                     current_close_early_g(rt, tick),
                                     (float)tick->params->drip_delay_ms);
            publish_summary(rt, env, tick, "ok", true);
            env->clear_sample_telemetry();
            env->set_slot(next);
            return FILLER_FIND_SLOT;
        }
        publish_runtime_snapshot(rt, env, tick,
                                 (float)tick->params->near_close_delta_g,
                                 current_close_early_g(rt, tick),
                                 (float)tick->params->drip_delay_ms);
        return state;
    }

    default:
        return state;
    }
}

const filler_strategy_ops_t g_filler_strategy_heuristic_ops = {
    .name = "heuristic",
    .on_enter = heuristic_on_enter,
    .step = heuristic_step,
};
