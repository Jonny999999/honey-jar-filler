#include "filler_strategy.h"

#include "esp_log.h"
#include "esp_timer.h"

// Step used to relax close-early offset after an underweight retry (grams).
#define CLOSE_EARLY_STEP_G 4.0f

// Require consecutive samples to confirm threshold crossings.
#define THRESH_CONFIRM_COUNT 4

static const char *TAG = "fill_strategy";

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
            rt->close_early_relax_g = 0.0f;
        }
        env->publish_fill_start(tick->run_id,
                                tick->slot_idx,
                                tick->strategy_name,
                                tick->params,
                                tick->latest ? tick->latest->grams : 0.0f);
        env->gate_set_percent_label(tick->params->max_gate_pct, "max_gate");
        break;
    case FILLER_DRIP_WAIT:
        ESP_LOGD(TAG, "drip wait: gate closed");
        env->gate_close_label("drip_wait");
        break;
    case FILLER_VERIFY_TARGET:
        ESP_LOGD(TAG, "verify target: gate closed");
        env->gate_close_label("close");
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
            return next_state;
        }

        float tare_g = 0.0f;
        bool has_tare = env->jar_tare_get(&tare_g);
        float rel_g = tick->latest->grams - (has_tare ? tare_g : 0.0f);
        float close_early_g_cur = (float)tick->params->close_early_g - rt->close_early_relax_g;
        if (close_early_g_cur < 0.0f) close_early_g_cur = 0.0f;
        float near_close = (float)tick->params->target_grams - (float)tick->params->near_close_delta_g;
        float close_early = (float)tick->params->target_grams - close_early_g_cur;

        if ((tick->now_us - tick->state_enter_us) > ((int64_t)tick->params->fill_timeout_ms * 1000)) {
            ESP_LOGE(TAG, "fill timeout");
            env->gate_close_label("close");
            env->set_fault(FLT_SERVO_TIMEOUT);
            return FILLER_FAULT;
        }
        if (!tick->new_sample) {
            return state;
        }
        if (stable_above(rel_g, (float)tick->params->target_grams, &rt->cnt_target, THRESH_CONFIRM_COUNT)) {
            ESP_LOGI(TAG, "target reached: rel=%.1f g abs=%.1f g",
                     (double)rel_g, (double)tick->latest->grams);
            env->gate_close_label("target");
            return FILLER_DRIP_WAIT;
        }
        if (stable_above(rel_g, close_early, &rt->cnt_close_early, THRESH_CONFIRM_COUNT)) {
            ESP_LOGI(TAG, "close-early reached: rel=%.1f g (offset %.1f g)",
                     (double)rel_g, (double)close_early_g_cur);
            env->gate_close_label("close_early");
            return FILLER_DRIP_WAIT;
        }
        if (stable_above(rel_g, near_close, &rt->cnt_near_close, THRESH_CONFIRM_COUNT)) {
            if (!rt->near_close_logged) {
                ESP_LOGD(TAG, "near close: rel=%.1f g -> partial gate", (double)rel_g);
                rt->near_close_logged = true;
            }
            env->gate_set_percent_label(tick->params->near_close_gate_pct, "near_close");
            return state;
        }

        env->gate_set_percent_label(tick->params->max_gate_pct, "max_gate");
        return state;
    }

    case FILLER_DRIP_WAIT:
        if ((tick->now_us - tick->state_enter_us) >= ((int64_t)tick->params->drip_delay_ms * 1000)) {
            return FILLER_VERIFY_TARGET;
        }
        return state;

    case FILLER_VERIFY_TARGET: {
        filler_state_t next_state = state;
        if (!env->require_fresh_or_fault("verify target", tick->latest, &next_state)) {
            return next_state;
        }

        float tare_g = 0.0f;
        bool has_tare = env->jar_tare_get(&tare_g);
        float rel_g = tick->latest->grams - (has_tare ? tare_g : 0.0f);
        float tol_low_g = (float)tick->params->target_tol_low_g;
        float tol_high_g = (float)tick->params->target_tol_high_g;

        if (tick->new_sample &&
            stable_above(rel_g, (float)tick->params->target_grams + tol_high_g, &rt->cnt_over, THRESH_CONFIRM_COUNT)) {
            float over = rel_g - (float)tick->params->target_grams;
            ESP_LOGE(TAG, "overweight: rel=%.1f g (+%.1f g, tol=+%.1f g)",
                     (double)rel_g, (double)over, (double)tol_high_g);
            ESP_LOGW(TAG, "suggestion: increase close_early_g (now %u g)",
                     (unsigned)tick->params->close_early_g);
            env->set_fault(FLT_WEIGHT_RANGE);
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
            return FILLER_FILL;
        }

        if (rt->sample_count >= THRESH_CONFIRM_COUNT) {
            uint8_t next = env->slot_next_idx(tick->slot_idx, tick->params->slots_total);
            ESP_LOGI(TAG, "target verified: rel=%.1f g", (double)rel_g);
            ESP_LOGI(TAG, "slot complete: %u -> %u", (unsigned)tick->slot_idx, (unsigned)next);
            env->set_slot(next);
            return FILLER_FIND_SLOT;
        }
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
