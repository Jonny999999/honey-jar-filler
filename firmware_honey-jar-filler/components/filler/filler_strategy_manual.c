#include "filler_strategy.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

// Manual mode uses the existing machine FSM to position the carousel, but the
// actual dosing is fully user-driven with the rotary encoder.
#define MANUAL_GATE_STEP_PCT 4f
#define MANUAL_ADVANCE_DRIP_MS 2000

static const char *TAG = "fill_manual";

static float clamp_pct(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 100.0f) return 100.0f;
    return value;
}

static void manual_publish_snapshot(filler_strategy_runtime_t *rt,
                                    const filler_strategy_env_t *env,
                                    const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !env->set_sample_telemetry || !tick) return;

    filler_strategy_sample_telemetry_t sample = {
        .valid = true,
    };
    snprintf(sample.strategy_name, sizeof(sample.strategy_name), "%s", tick->strategy_name ? tick->strategy_name : "?");
    snprintf(sample.gate_phase, sizeof(sample.gate_phase), "%s", "manual");
    env->set_sample_telemetry(&sample);
}

static void manual_on_enter(filler_strategy_runtime_t *rt,
                            filler_state_t state,
                            filler_state_t prev_state,
                            const filler_strategy_env_t *env,
                            const filler_strategy_tick_t *tick)
{
    (void)prev_state;
    if (!rt || !env || !tick) return;

    switch (state) {
    case FILLER_FILL:
        memset(rt, 0, sizeof(*rt));
        rt->manual_gate_pct = 0.0f;
        rt->manual_pending_advance = false;
        env->gate_close_label("manual_entry");
        env->publish_fill_start(tick->run_id,
                                tick->slot_idx,
                                tick->strategy_name,
                                tick->params,
                                tick->latest ? tick->latest->grams : 0.0f);
        ESP_LOGI(TAG, "manual fill ready: slot=%u gate=0%%, encoder controls gate, button advances",
                 (unsigned)tick->slot_idx);
        manual_publish_snapshot(rt, env, tick);
        break;
    case FILLER_DRIP_WAIT:
        env->gate_close_label("manual_close");
        manual_publish_snapshot(rt, env, tick);
        break;
    case FILLER_VERIFY_TARGET:
        manual_publish_snapshot(rt, env, tick);
        break;
    default:
        break;
    }
}

static filler_state_t manual_step(filler_strategy_runtime_t *rt,
                                  filler_state_t state,
                                  const filler_strategy_env_t *env,
                                  const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick) return state;

    switch (state) {
    case FILLER_FILL: {
        if (env->take_start_request && env->take_start_request()) {
            uint8_t next = env->slot_next_idx ? env->slot_next_idx(tick->slot_idx, tick->params ? tick->params->slots_total : 1) : tick->slot_idx;
            if (rt->manual_gate_pct > 0.0f) {
                rt->manual_pending_advance = true;
                rt->manual_next_slot = next;
                rt->manual_gate_pct = 0.0f;
                env->gate_close_label("manual_drip_close");
                ESP_LOGI(TAG,
                         "manual advance requested: closing gate and waiting %u ms before slot %u -> %u",
                         (unsigned)MANUAL_ADVANCE_DRIP_MS,
                         (unsigned)tick->slot_idx,
                         (unsigned)next);
                manual_publish_snapshot(rt, env, tick);
                return FILLER_DRIP_WAIT;
            }
            env->gate_close_label("manual_advance");
            env->set_slot(next);
            ESP_LOGI(TAG, "manual advance: slot %u -> %u", (unsigned)tick->slot_idx, (unsigned)next);
            manual_publish_snapshot(rt, env, tick);
            return FILLER_FIND_SLOT;
        }

        if (env->take_manual_gate_delta) {
            int32_t delta_steps = env->take_manual_gate_delta();
            if (delta_steps != 0) {
                float prev_pct = rt->manual_gate_pct;
                rt->manual_gate_pct = clamp_pct(rt->manual_gate_pct + ((float)delta_steps * MANUAL_GATE_STEP_PCT));
                if (rt->manual_gate_pct != prev_pct) {
                    env->gate_set_percent_label(rt->manual_gate_pct, "manual_adjust");
                    ESP_LOGI(TAG, "manual gate: %.0f%% -> %.0f%% (delta=%ld)",
                             (double)prev_pct,
                             (double)rt->manual_gate_pct,
                             (long)delta_steps);
                }
            }
        }

        manual_publish_snapshot(rt, env, tick);
        return state;
    }

    case FILLER_DRIP_WAIT:
        if (rt->manual_pending_advance &&
            (tick->now_us - tick->state_enter_us) >= ((int64_t)MANUAL_ADVANCE_DRIP_MS * 1000)) {
            rt->manual_pending_advance = false;
            env->set_slot(rt->manual_next_slot);
            ESP_LOGI(TAG, "manual drip wait complete: advancing to slot %u", (unsigned)rt->manual_next_slot);
            manual_publish_snapshot(rt, env, tick);
            return FILLER_FIND_SLOT;
        }
        manual_publish_snapshot(rt, env, tick);
        return state;

    default:
        return state;
    }
}

const filler_strategy_ops_t g_filler_strategy_manual_ops = {
    .name = "manual",
    .on_enter = manual_on_enter,
    .step = manual_step,
};
