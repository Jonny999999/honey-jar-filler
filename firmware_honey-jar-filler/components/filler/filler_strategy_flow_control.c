#include "filler_strategy.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "esp_log.h"

// Rate estimation filter:
// Rise slowly, fall faster, and reset cleanly when no measurable flow remains.
#define RATE_FILTER_ALPHA_RISE 0.18f
#define RATE_FILTER_ALPHA_FALL 0.55f
#define RATE_MIN_VALID_GPS 2.0f
#define RATE_ZERO_EPS_GPS 0.5f
#define RATE_NO_FLOW_RESET_SAMPLES 2u

// Conservative session-local learning:
// Post-close gain is trusted most because it is measured directly after the
// close command. Rates and dead time still adapt, but less aggressively. The
// first successful fill of a preset dominates its estimate via a count-based
// warmup (see learn_ewma) so the fallback seeds are dropped quickly.
#define LEARN_ALPHA_DEAD_TIME 0.35f
#define LEARN_ALPHA_POST_CLOSE 0.65f
#define LEARN_ALPHA_RATE 0.20f
#define LEARN_ALPHA_DRIP_WAIT 0.35f
#define LEARN_ALPHA_FINISH_TRIM 0.18f
#define LEARN_ALPHA_GATE_START 0.30f

// Verify robustness.
#define VERIFY_CONFIRM_SAMPLES 4u

// Response / dead-time detection. Bounds are generous so thick honey with a
// long transport dead time is not falsely cancelled (the no-response fault only
// guards an empty bucket / total clog); on the first jar the limit falls back
// to NO_RESPONSE_MIN_S because the dead time is not learned yet.
#define RESPONSE_THRESHOLD_MIN_G 1.5f
#define RESPONSE_THRESHOLD_MAX_G 6.0f
#define DEAD_TIME_MIN_S 0.10f
#define DEAD_TIME_MAX_S 15.00f
#define NO_RESPONSE_MIN_S 15.00f
#define NO_RESPONSE_MAX_S 30.00f

// Final-close and phase-transition shaping. After an underfill the close
// threshold is relaxed by ~the measured deficit (scaled, floored at the step)
// so a single refill reaches target instead of many tiny steps.
#define CLOSE_BUFFER_G 1.5f
#define REFILL_RELAX_STEP_G 2.5f
#define REFILL_CLOSE_RELAX_FACTOR 0.90f
#define NEAR_CLOSE_TRANSITION_MARGIN_G 4.0f
#define FLOW_FINISH_TRIM_MIN_G -10.0f
#define FLOW_FINISH_TRIM_MAX_G 12.0f

// Safety limits.
#define FLOW_HARD_OVERFILL_MARGIN_MIN_G 12.0f
#define FLOW_HARD_OVERFILL_MARGIN_EXTRA_G 6.0f
#define SAFE_RATE_MIN_GPS 25.0f
#define SAFE_RATE_MAX_GPS 140.0f
#define SAFE_RATE_MULT 1.8f
// Absolute safe-rate ceiling from the target mass: protects the first jar and
// scales with jar size (a controlled fill should not complete faster than
// this). Small jars intentionally run at a high g/s operating point (see the
// fast gate phase), so this must stay below that intended rate with headroom,
// or the safe-rate reducer fights the controller on every fill instead of
// only reacting to genuine runaways.
#define FLOW_MIN_CONTROLLED_FILL_S 4.0f
// Debounce the safe-rate trigger so the impact spike when honey first hits the
// glass does not reduce the gate; escalate to a fault if the reduced gate still
// cannot tame the flow within the grace period (medium too thin for gate %).
#define RATE_SPIKE_CONFIRM_SAMPLES 3u
#define SAFE_REDUCE_ESCALATE_S 5.0f
// Progressive safe-rate gate reduction: step size scales with how far the
// rate is over the limit, instead of a single fixed cut, and is paced by the
// plant dead time (control_holdoff_ms) so each step's effect is seen before
// reacting again -- reacting on every sample ratcheted the gate closed well
// before the fill's own dead time even elapsed once.
#define SAFE_REDUCE_STEP_MIN_PCT 2.0f
#define SAFE_REDUCE_STEP_MAX_PCT 20.0f
#define SAFE_REDUCE_OVER_RATIO_SOFT 1.05f
#define SAFE_REDUCE_OVER_RATIO_HARD 2.0f

// Controller behavior.
#define FLOW_CTRL_KP_PCT_PER_GPS 0.08f
#define FLOW_CTRL_STEP_MAX_PCT 4.0f
#define FLOW_CTRL_STEP_MIN_PCT 0.8f
#define FLOW_CTRL_DEADBAND_FAST_GPS 5.0f
#define FLOW_CTRL_DEADBAND_SLOW_GPS 2.5f
#define FLOW_CTRL_HOLDOFF_MIN_MS 350.0f
#define FLOW_CTRL_HOLDOFF_MAX_MS 2000.0f

// Sample-to-sample derivative window.
#define DELTA_RATE_MAX_DT_S 0.80f
#define DELTA_RATE_MIN_DT_S 0.05f

// Drip wait learning bounds.
#define DRIP_WAIT_MIN_MS 5000.0f
#define DRIP_WAIT_MAX_MS 60000.0f
#define DRIP_WAIT_SETTLE_MARGIN_MS 1200.0f

typedef struct {
    uint8_t initialized;
    uint32_t successful_fills;
    float dead_time_s;
    float post_close_gain_g;
    float fast_rate_gps;
    float slow_rate_gps;
    float finish_trim_g;
    float fast_start_gate_pct;
    float slow_start_gate_pct;
    float drip_wait_ms;
} flow_learned_entry_t;

typedef enum {
    FLOW_PHASE_FAST_CONTROL = 0,
    FLOW_PHASE_SLOW_CONTROL,
    FLOW_PHASE_CLOSED,
    FLOW_PHASE_REFILL,
} flow_gate_phase_t;

static const char *TAG = "fill_flowctrl";
static flow_learned_entry_t s_learned[APP_PRESET_COUNT];

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

// Learned-estimate update with a count-based warmup. No prev<=0 shortcut, so it
// is also correct for signed quantities, and the first observations dominate so
// a conservative fallback seed is replaced within a couple of jars.
static float learn_ewma(float prev, float sample, float base_alpha, uint32_t observations)
{
    float warmup_alpha = 1.0f / (float)(observations + 1u);
    float alpha = (warmup_alpha > base_alpha) ? warmup_alpha : base_alpha;
    return prev + alpha * (sample - prev);
}

static void clear_live_rate_estimates(filler_strategy_runtime_t *rt)
{
    if (!rt) return;
    rt->raw_rate_gps = 0.0f;
    rt->filtered_rate_gps = 0.0f;
    rt->no_flow_count = 0;
}

static bool stable_above(float value, float threshold, uint8_t *count, uint8_t required)
{
    if (!count || required == 0) return value >= threshold;
    if (value >= threshold) {
        if (*count < required) (*count)++;
    } else {
        *count = 0;
    }
    return *count >= required;
}

static bool stable_below(float value, float threshold, uint8_t *count, uint8_t required)
{
    if (!count || required == 0) return value <= threshold;
    if (value <= threshold) {
        if (*count < required) (*count)++;
    } else {
        *count = 0;
    }
    return *count >= required;
}

static const char *phase_name(flow_gate_phase_t phase)
{
    switch (phase) {
    case FLOW_PHASE_FAST_CONTROL: return "fast_control";
    case FLOW_PHASE_SLOW_CONTROL: return "slow_control";
    case FLOW_PHASE_CLOSED:       return "closed";
    case FLOW_PHASE_REFILL:       return "refill";
    default:                      return "?";
    }
}

static flow_gate_phase_t runtime_phase(const filler_strategy_runtime_t *rt)
{
    if (!rt) return FLOW_PHASE_CLOSED;
    if (rt->first_close_seen) return FLOW_PHASE_CLOSED;
    if (rt->refill_count > 0 && !rt->near_close_logged) return FLOW_PHASE_REFILL;
    if (rt->near_close_logged) return FLOW_PHASE_SLOW_CONTROL;
    return FLOW_PHASE_FAST_CONTROL;
}

static void flow_reset_counters(filler_strategy_runtime_t *rt)
{
    if (!rt) return;
    rt->cnt_near_close = 0;
    rt->cnt_close_early = 0;
    rt->cnt_target = 0;
    rt->cnt_under = 0;
    rt->cnt_over = 0;
    rt->cnt_safe_rate = 0;
    rt->sample_count = 0;
    rt->safe_reduce_ts_us = 0;
    rt->safe_reduce_last_action_us = 0;
}

static float fallback_fast_rate_gps(const app_params_t *params)
{
    float target_g = params ? (float)params->target_grams : 500.0f;
    return clampf_local(target_g * 0.16f, 25.0f, 120.0f);
}

static float fallback_slow_rate_gps(const app_params_t *params)
{
    float target_g = params ? (float)params->target_grams : 500.0f;
    return clampf_local(target_g * 0.045f, 6.0f, 45.0f);
}

static void flow_defaults_from_params(flow_learned_entry_t *entry, const app_params_t *params)
{
    if (!entry || !params) return;
    float poll_s = ((float)CONFIG_HX711_POLL_INTERVAL_MS / 1000.0f) * 2.5f;
    entry->initialized = 1u;
    entry->dead_time_s = clampf_local(poll_s, 0.25f, 1.20f);
    entry->post_close_gain_g = clampf_local((float)params->close_remaining_g * 0.55f, 1.0f, 120.0f);
    entry->fast_rate_gps = fallback_fast_rate_gps(params);
    entry->slow_rate_gps = fallback_slow_rate_gps(params);
    entry->finish_trim_g = 0.0f;
    entry->fast_start_gate_pct = clampf_local((float)params->max_gate_pct, 0.0f, 100.0f);
    entry->slow_start_gate_pct = clampf_local((float)params->slow_gate_pct, 0.0f, 100.0f);
    entry->drip_wait_ms = clampf_local((float)params->drip_delay_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS);
}

static void log_learned_entry(const char *prefix, uint8_t preset_index, const flow_learned_entry_t *entry)
{
    if (!prefix || !entry) return;
    ESP_LOGI(TAG,
             "%s preset=%u dead_time=%.3f s post_close=%.1f g fast_rate=%.1f g/s slow_rate=%.1f g/s finish_trim=%.1f g fast_start=%.1f%% slow_start=%.1f%% drip_wait=%.0f ms fills=%lu",
             prefix,
             (unsigned)preset_index,
             (double)entry->dead_time_s,
             (double)entry->post_close_gain_g,
             (double)entry->fast_rate_gps,
             (double)entry->slow_rate_gps,
             (double)entry->finish_trim_g,
             (double)entry->fast_start_gate_pct,
             (double)entry->slow_start_gate_pct,
             (double)entry->drip_wait_ms,
             (unsigned long)entry->successful_fills);
}

static flow_learned_entry_t *flow_entry_for_preset(uint8_t preset_index, const app_params_t *params)
{
    if (preset_index >= APP_PRESET_COUNT) preset_index = 0;
    flow_learned_entry_t *entry = &s_learned[preset_index];
    if (!entry->initialized) {
        flow_defaults_from_params(entry, params);
        log_learned_entry("initialized flow-control defaults for", preset_index, entry);
    }
    return entry;
}

static float response_threshold_g(const filler_strategy_tick_t *tick)
{
    float candidate = tick && tick->params ? ((float)tick->params->target_grams * 0.005f) : 2.0f;
    return clampf_local(candidate, RESPONSE_THRESHOLD_MIN_G, RESPONSE_THRESHOLD_MAX_G);
}

static float transition_fast_rate_gps(const filler_strategy_runtime_t *rt, const app_params_t *params)
{
    if (!rt) return params ? fallback_fast_rate_gps(params) : 0.0f;
    flow_gate_phase_t phase = runtime_phase(rt);
    if ((phase == FLOW_PHASE_FAST_CONTROL || phase == FLOW_PHASE_REFILL) && rt->filtered_rate_gps > 0.5f) {
        return rt->filtered_rate_gps;
    }
    if (rt->learned_fast_rate_gps > 0.5f) return rt->learned_fast_rate_gps;
    if (rt->filtered_rate_gps > 0.5f) return rt->filtered_rate_gps;
    if (rt->raw_rate_gps > 0.5f) return rt->raw_rate_gps;
    return params ? fallback_fast_rate_gps(params) : 0.0f;
}

static float close_phase_rate_gps(const filler_strategy_runtime_t *rt, const app_params_t *params)
{
    if (!rt) return params ? fallback_slow_rate_gps(params) : 0.0f;
    if (rt->near_close_logged && rt->filtered_rate_gps > 0.5f) return rt->filtered_rate_gps;
    if (rt->learned_slow_rate_gps > 0.5f) return rt->learned_slow_rate_gps;
    return params ? fallback_slow_rate_gps(params) : 0.0f;
}

static float fast_gate_min_pct(const app_params_t *params)
{
    (void)params;
    return 0.0f;
}

static float fast_gate_max_pct(const app_params_t *params)
{
    (void)params;
    return 100.0f;
}

static float slow_gate_max_pct(const app_params_t *params)
{
    (void)params;
    return 100.0f;
}

static float slow_gate_min_pct(const app_params_t *params)
{
    (void)params;
    return 0.0f;
}

static float flow_start_gate_pct(const app_params_t *params)
{
    // Preset gate percentages act as the initial operating points for the two
    // flow-control phases. The controller is still free to move over the full
    // 0..100% range afterward if the measured flow requires it.
    float start_gate = params ? (float)params->max_gate_pct : 80.0f;
    return clampf_local(start_gate, 0.0f, 100.0f);
}

static float flow_slow_phase_start_gate_pct(const app_params_t *params)
{
    float start_gate = params ? (float)params->slow_gate_pct : 20.0f;
    return clampf_local(start_gate, 0.0f, 100.0f);
}

static float flow_runtime_fast_start_gate_pct(const filler_strategy_runtime_t *rt, const app_params_t *params)
{
    if (!rt || rt->learned_fast_start_gate_pct <= 0.0f) {
        return flow_start_gate_pct(params);
    }
    return clampf_local(rt->learned_fast_start_gate_pct, 0.0f, 100.0f);
}

static float flow_runtime_slow_start_gate_pct(const filler_strategy_runtime_t *rt, const app_params_t *params)
{
    if (!rt || rt->learned_slow_start_gate_pct <= 0.0f) {
        return flow_slow_phase_start_gate_pct(params);
    }
    return clampf_local(rt->learned_slow_start_gate_pct, 0.0f, 100.0f);
}

static float control_holdoff_ms(const filler_strategy_runtime_t *rt)
{
    if (!rt) return FLOW_CTRL_HOLDOFF_MIN_MS;

    // Use the preset/session-learned dead time as the startup estimate, then
    // switch to the measured dead time from the current fill once a response
    // has actually been detected. That keeps the controller holdoff dynamic
    // within the run instead of waiting until the next jar to benefit.
    float dead_time_s = rt->learned_dead_time_s;
    if (rt->response_detected && rt->measured_dead_time_s > 0.0f) {
        dead_time_s = rt->measured_dead_time_s;
    }
    return clampf_local(dead_time_s * 1000.0f, FLOW_CTRL_HOLDOFF_MIN_MS, FLOW_CTRL_HOLDOFF_MAX_MS);
}

static float target_rate_gps(const filler_strategy_runtime_t *rt, const app_params_t *params)
{
    if (!rt) return params ? fallback_fast_rate_gps(params) : 0.0f;
    if (rt->near_close_logged) {
        if (rt->learned_slow_rate_gps > 0.5f) return rt->learned_slow_rate_gps;
        return params ? fallback_slow_rate_gps(params) : 0.0f;
    }
    if (rt->learned_fast_rate_gps > 0.5f) return rt->learned_fast_rate_gps;
    return params ? fallback_fast_rate_gps(params) : 0.0f;
}

static void update_control_targets(filler_strategy_runtime_t *rt, const app_params_t *params)
{
    if (!rt) return;
    rt->control_target_rate_gps = target_rate_gps(rt, params);
    rt->control_rate_error_gps = rt->control_target_rate_gps - rt->filtered_rate_gps;
}

static void update_thresholds(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!rt || !tick || !tick->params) return;

    float fast_transition_rate = transition_fast_rate_gps(rt, tick->params);
    float close_rate = close_phase_rate_gps(rt, tick->params);
    float transition_mass_g = rt->learned_dead_time_s * fast_transition_rate;
    float close_dead_mass_g = rt->learned_dead_time_s * close_rate;

    // Final close is still driven by the learned post-close residual mass.
    // The slow-phase transport mass only adds the remaining in-flight portion
    // before the fully closed command can take effect.
    rt->predicted_remaining_g = rt->learned_post_close_gain_g + close_dead_mass_g;

    float close_max = fmaxf((float)tick->params->close_remaining_g * 1.8f, 10.0f);
    float near_max = fmaxf((float)tick->params->slow_remaining_g * 2.0f, 30.0f);
    // A small learned finish trim compensates for repeatable mean fill error
    // that still sits inside tolerance. Positive trim closes later.
    float close_candidate = rt->predicted_remaining_g + CLOSE_BUFFER_G -
                            rt->close_early_relax_g - rt->learned_finish_trim_g;
    float close_threshold = clampf_local(close_candidate, 2.0f, close_max);
    float near_candidate = transition_mass_g + close_threshold + NEAR_CLOSE_TRANSITION_MARGIN_G;

    rt->adapted_close_early_g = close_threshold;
    rt->adapted_near_close_g = clampf_local(near_candidate,
                                            rt->adapted_close_early_g + 3.0f,
                                            near_max);
    rt->adapted_drip_wait_ms = clampf_local(rt->adapted_drip_wait_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS);
}

static void publish_runtime_snapshot(filler_strategy_runtime_t *rt,
                                     const filler_strategy_env_t *env,
                                     const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !env->set_sample_telemetry || !tick) return;

    filler_strategy_sample_telemetry_t sample = {
        .valid = true,
        .raw_rate_gps = rt->raw_rate_gps,
        .filtered_rate_gps = rt->filtered_rate_gps,
        .target_rate_gps = rt->control_target_rate_gps,
        .rate_error_gps = rt->control_rate_error_gps,
        .predicted_remaining_g = rt->predicted_remaining_g,
        .measured_dead_time_s = rt->measured_dead_time_s,
        .measured_post_close_gain_g = rt->measured_post_close_gain_g,
        .learned_dead_time_s = rt->learned_dead_time_s,
        .learned_post_close_gain_g = rt->learned_post_close_gain_g,
        .learned_fast_rate_gps = rt->learned_fast_rate_gps,
        .learned_slow_rate_gps = rt->learned_slow_rate_gps,
        .adapted_near_close_g = rt->adapted_near_close_g,
        .adapted_close_early_g = rt->adapted_close_early_g,
        .adapted_drip_wait_ms = rt->adapted_drip_wait_ms,
        .refill_count = rt->refill_count,
    };
    snprintf(sample.strategy_name, sizeof(sample.strategy_name), "%s", tick->strategy_name ? tick->strategy_name : "?");
    snprintf(sample.gate_phase, sizeof(sample.gate_phase), "%s", phase_name(runtime_phase(rt)));
    env->set_sample_telemetry(&sample);
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
            if (rt->raw_rate_gps < RATE_MIN_VALID_GPS) {
                rt->raw_rate_gps = 0.0f;
            }

            if (rt->raw_rate_gps <= 0.0f) {
                if (rt->no_flow_count < 255) rt->no_flow_count++;
                if (rt->no_flow_count >= RATE_NO_FLOW_RESET_SAMPLES) {
                    rt->filtered_rate_gps = 0.0f;
                } else {
                    rt->filtered_rate_gps = ewma(rt->filtered_rate_gps, 0.0f, RATE_FILTER_ALPHA_FALL);
                }
            } else {
                rt->no_flow_count = 0;
                float alpha = (rt->raw_rate_gps >= rt->filtered_rate_gps) ? RATE_FILTER_ALPHA_RISE : RATE_FILTER_ALPHA_FALL;
                rt->filtered_rate_gps = ewma(rt->filtered_rate_gps, rt->raw_rate_gps, alpha);
            }
            if (rt->filtered_rate_gps < RATE_ZERO_EPS_GPS) {
                rt->filtered_rate_gps = 0.0f;
            }

            flow_gate_phase_t phase = runtime_phase(rt);
            if ((phase == FLOW_PHASE_FAST_CONTROL || phase == FLOW_PHASE_REFILL) && rt->raw_rate_gps > 0.1f) {
                rt->fast_rate_sum_gps += rt->raw_rate_gps;
                rt->fast_rate_count++;
            } else if (phase == FLOW_PHASE_SLOW_CONTROL && rt->raw_rate_gps > 0.1f) {
                rt->slow_rate_sum_gps += rt->raw_rate_gps;
                rt->slow_rate_count++;
            }
        }
    }

    if (!rt->response_detected && rel_g >= response_threshold_g(tick)) {
        rt->response_detected = true;
        rt->first_response_ts_us = tick->latest->ts_us;
        rt->measured_dead_time_s = clampf_local((float)(tick->latest->ts_us - rt->fill_open_ts_us) / 1000000.0f,
                                                DEAD_TIME_MIN_S,
                                                DEAD_TIME_MAX_S);
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
    rt->gate_at_close_pct = rt->control_gate_cmd_pct;
}

static float safe_rate_limit_gps(const filler_strategy_runtime_t *rt, const app_params_t *params)
{
    // Absolute ceiling from the target mass: independent of any learned value so
    // it also protects the first jar. A rate above it would complete the fill in
    // under FLOW_MIN_CONTROLLED_FILL_S, i.e. the medium is far thinner than the
    // gate operating points expect.
    float abs_ceiling = SAFE_RATE_MAX_GPS;
    if (params && params->target_grams > 0) {
        abs_ceiling = (float)params->target_grams / FLOW_MIN_CONTROLLED_FILL_S;
    }
    abs_ceiling = clampf_local(abs_ceiling, SAFE_RATE_MIN_GPS, SAFE_RATE_MAX_GPS);

    // Once a fast rate is learned, also cut back on a run flowing far faster than
    // the learned norm (a thinner batch mid-session). Tighter ceiling wins.
    float learned = rt ? rt->learned_fast_rate_gps : 0.0f;
    if (learned > 1.0f) {
        float rel_ceiling = clampf_local(learned * SAFE_RATE_MULT, SAFE_RATE_MIN_GPS, SAFE_RATE_MAX_GPS);
        return fminf(abs_ceiling, rel_ceiling);
    }
    return abs_ceiling;
}

static bool plausible_positive(float value, float min_v, float max_v)
{
    return value >= min_v && value <= max_v;
}

static void flow_command_gate(filler_strategy_runtime_t *rt,
                              const filler_strategy_env_t *env,
                              float gate_pct,
                              const char *label,
                              int64_t now_us,
                              bool force)
{
    if (!rt || !env || !env->gate_set_percent_label) return;
    gate_pct = clampf_local(gate_pct, 0.0f, 100.0f);
    if (!force && fabsf(gate_pct - rt->control_gate_cmd_pct) < 0.05f) return;
    rt->control_gate_cmd_pct = gate_pct;
    rt->control_last_update_us = now_us;
    env->gate_set_percent_label(gate_pct, label);
}

static bool update_controller_gate(filler_strategy_runtime_t *rt,
                                   const filler_strategy_env_t *env,
                                   const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick || !tick->params || !tick->new_sample || !rt->response_detected || rt->first_close_seen) {
        return false;
    }

    float holdoff_ms = control_holdoff_ms(rt);
    if (rt->control_last_update_us > 0 &&
        (tick->now_us - rt->control_last_update_us) < (int64_t)(holdoff_ms * 1000.0f)) {
        return false;
    }

    update_control_targets(rt, tick->params);
    float target_rate = rt->control_target_rate_gps;
    float error_gps = rt->control_rate_error_gps;
    float deadband = rt->near_close_logged ? FLOW_CTRL_DEADBAND_SLOW_GPS : FLOW_CTRL_DEADBAND_FAST_GPS;
    if (fabsf(error_gps) <= deadband) {
        return false;
    }

    float gate_min = rt->near_close_logged ? slow_gate_min_pct(tick->params) : fast_gate_min_pct(tick->params);
    float gate_max = rt->near_close_logged ? slow_gate_max_pct(tick->params) : fast_gate_max_pct(tick->params);
    float step_pct = clampf_local(error_gps * FLOW_CTRL_KP_PCT_PER_GPS, -FLOW_CTRL_STEP_MAX_PCT, FLOW_CTRL_STEP_MAX_PCT);
    if (fabsf(step_pct) < FLOW_CTRL_STEP_MIN_PCT) {
        step_pct = (step_pct >= 0.0f) ? FLOW_CTRL_STEP_MIN_PCT : -FLOW_CTRL_STEP_MIN_PCT;
    }

    float next_gate = clampf_local(rt->control_gate_cmd_pct + step_pct, gate_min, gate_max);
    if (fabsf(next_gate - rt->control_gate_cmd_pct) < 0.05f) {
        return false;
    }

    ESP_LOGI(TAG,
             "flow ctrl phase=%s target_rate=%.1f g/s measured=%.1f g/s error=%.1f g/s gate %.1f%%->%.1f%% holdoff=%.0f ms",
             phase_name(runtime_phase(rt)),
             (double)target_rate,
             (double)rt->filtered_rate_gps,
             (double)error_gps,
             (double)rt->control_gate_cmd_pct,
             (double)next_gate,
             (double)holdoff_ms);
    flow_command_gate(rt, env, next_gate, "flow_ctrl", tick->now_us, false);
    return true;
}

static void publish_summary_and_learn(filler_strategy_runtime_t *rt,
                                      const filler_strategy_env_t *env,
                                      const filler_strategy_tick_t *tick,
                                      const char *reason,
                                      bool success)
{
    if (!rt || !env || !env->publish_fill_summary || !tick || !tick->latest || !tick->params) return;

    flow_learned_entry_t *entry = flow_entry_for_preset(tick->preset_index, tick->params);
    float final_rel_g = tick->latest->grams - rt->run_base_weight_g;
    float fast_avg = (rt->fast_rate_count > 0) ? (rt->fast_rate_sum_gps / (float)rt->fast_rate_count) : 0.0f;
    float slow_avg = (rt->slow_rate_count > 0) ? (rt->slow_rate_sum_gps / (float)rt->slow_rate_count) : 0.0f;
    float fill_error_g = final_rel_g - (float)tick->params->target_grams;
    float fill_duration_s = (rt->fill_open_ts_us > 0)
        ? ((float)(tick->now_us - rt->fill_open_ts_us) / 1000000.0f)
        : 0.0f;
    float settled_wait_ms = rt->first_close_seen && rt->last_gain_ts_us >= rt->first_close_ts_us
        ? ((float)(rt->last_gain_ts_us - rt->first_close_ts_us) / 1000.0f) + DRIP_WAIT_SETTLE_MARGIN_MS
        : rt->adapted_drip_wait_ms;

    bool plausible = success &&
                     rt->response_detected &&
                     plausible_positive(rt->measured_dead_time_s, DEAD_TIME_MIN_S, DEAD_TIME_MAX_S) &&
                     plausible_positive(rt->measured_post_close_gain_g, 0.0f, 150.0f) &&
                     plausible_positive(fast_avg, 5.0f, 400.0f) &&
                     (slow_avg == 0.0f || plausible_positive(slow_avg, 1.0f, 250.0f));

    flow_learned_entry_t before = *entry;
    if (plausible) {
        // Warmup so the first successful fill fully adopts its measurement
        // instead of blending with the fallback seed.
        uint32_t obs = before.successful_fills;
        entry->dead_time_s = learn_ewma(entry->dead_time_s, rt->measured_dead_time_s, LEARN_ALPHA_DEAD_TIME, obs);
        entry->post_close_gain_g = learn_ewma(entry->post_close_gain_g, rt->measured_post_close_gain_g, LEARN_ALPHA_POST_CLOSE, obs);
        entry->fast_rate_gps = learn_ewma(entry->fast_rate_gps, fast_avg, LEARN_ALPHA_RATE, obs);
        if (slow_avg > 0.0f) entry->slow_rate_gps = learn_ewma(entry->slow_rate_gps, slow_avg, LEARN_ALPHA_RATE, obs);
        // finish_trim is SIGNED: use a plain EWMA (no prev<=0 shortcut, no
        // warmup) so a negative/zero trim is filtered, not replaced wholesale.
        float trim_sample = clampf_local(-fill_error_g, FLOW_FINISH_TRIM_MIN_G, FLOW_FINISH_TRIM_MAX_G);
        entry->finish_trim_g += LEARN_ALPHA_FINISH_TRIM * (trim_sample - entry->finish_trim_g);
        entry->finish_trim_g = clampf_local(entry->finish_trim_g,
                                            FLOW_FINISH_TRIM_MIN_G,
                                            FLOW_FINISH_TRIM_MAX_G);
        if (rt->gate_at_slow_entry_pct > 0.0f) {
            entry->fast_start_gate_pct = learn_ewma(entry->fast_start_gate_pct,
                                                    clampf_local(rt->gate_at_slow_entry_pct, 0.0f, 100.0f),
                                                    LEARN_ALPHA_GATE_START, obs);
        }
        if (rt->gate_at_close_pct > 0.0f) {
            entry->slow_start_gate_pct = learn_ewma(entry->slow_start_gate_pct,
                                                    clampf_local(rt->gate_at_close_pct, 0.0f, 100.0f),
                                                    LEARN_ALPHA_GATE_START, obs);
        }
        // Drip wait stays a gentle plain EWMA from the generous seed so it never
        // snaps short on a single fast-settling run.
        entry->drip_wait_ms = ewma(entry->drip_wait_ms,
                                   clampf_local(settled_wait_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS),
                                   LEARN_ALPHA_DRIP_WAIT);
        entry->successful_fills++;
        ESP_LOGI(TAG,
                 "learned update reason=%s dead_time %.3f->%.3f s post_close %.1f->%.1f g fast_rate %.1f->%.1f g/s slow_rate %.1f->%.1f g/s finish_trim %.1f->%.1f g fast_start %.1f->%.1f%% slow_start %.1f->%.1f%% drip_wait %.0f->%.0f ms",
                 reason ? reason : "ok",
                 (double)before.dead_time_s, (double)entry->dead_time_s,
                 (double)before.post_close_gain_g, (double)entry->post_close_gain_g,
                 (double)before.fast_rate_gps, (double)entry->fast_rate_gps,
                 (double)before.slow_rate_gps, (double)entry->slow_rate_gps,
                 (double)before.finish_trim_g, (double)entry->finish_trim_g,
                 (double)before.fast_start_gate_pct, (double)entry->fast_start_gate_pct,
                 (double)before.slow_start_gate_pct, (double)entry->slow_start_gate_pct,
                 (double)before.drip_wait_ms, (double)entry->drip_wait_ms);
    } else {
        ESP_LOGI(TAG,
                 "learning skipped reason=%s success=%d response=%d dead_time=%.3f s post_close=%.1f g fast_rate=%.1f g/s slow_rate=%.1f g/s",
                 reason ? reason : "fault",
                 success ? 1 : 0,
                 rt->response_detected ? 1 : 0,
                 (double)rt->measured_dead_time_s,
                 (double)rt->measured_post_close_gain_g,
                 (double)fast_avg,
                 (double)slow_avg);
    }

    filler_strategy_fill_summary_t summary = {
        .valid = true,
        .final_mass_g = final_rel_g,
        .target_g = (float)tick->params->target_grams,
        .fill_error_g = fill_error_g,
        .measured_dead_time_s = rt->measured_dead_time_s,
        .measured_post_close_gain_g = rt->measured_post_close_gain_g,
        .measured_fast_rate_gps = fast_avg,
        .measured_slow_rate_gps = slow_avg,
        .rate_at_close_gps = rt->rate_at_close_gps,
        .fill_duration_s = fill_duration_s,
        .drip_wait_used_ms = rt->adapted_drip_wait_ms,
        .refill_count = rt->refill_count,
        .used_dead_time_s = rt->learned_dead_time_s,
        .used_post_close_gain_g = rt->learned_post_close_gain_g,
        .used_fast_rate_gps = rt->learned_fast_rate_gps,
        .used_slow_rate_gps = rt->learned_slow_rate_gps,
        .used_near_close_g = rt->adapted_near_close_g,
        .used_close_early_g = rt->adapted_close_early_g,
        .next_dead_time_s = entry->dead_time_s,
        .next_post_close_gain_g = entry->post_close_gain_g,
        .next_fast_rate_gps = entry->fast_rate_gps,
        .next_slow_rate_gps = entry->slow_rate_gps,
        .next_near_close_g = rt->adapted_near_close_g,
        .next_close_early_g = rt->adapted_close_early_g,
        .next_drip_wait_ms = entry->drip_wait_ms,
    };
    snprintf(summary.reason, sizeof(summary.reason), "%s", reason ? reason : (success ? "ok" : "fault"));
    env->publish_fill_summary(tick->run_id,
                              tick->slot_idx,
                              tick->preset_name,
                              tick->strategy_name,
                              tick->params,
                              &summary);
}

static void flow_on_enter(filler_strategy_runtime_t *rt,
                          filler_state_t state,
                          filler_state_t prev_state,
                          const filler_strategy_env_t *env,
                          const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick || !tick->params) return;

    switch (state) {
    case FILLER_FILL: {
        flow_learned_entry_t *entry = flow_entry_for_preset(tick->preset_index, tick->params);
        if (prev_state != FILLER_VERIFY_TARGET) {
            memset(rt, 0, sizeof(*rt));
            rt->fill_open_ts_us = tick->now_us;
            rt->active_preset_index = tick->preset_index;
            rt->target_g = (float)tick->params->target_grams;
            rt->learned_dead_time_s = entry->dead_time_s;
            rt->learned_post_close_gain_g = entry->post_close_gain_g;
            rt->learned_fast_rate_gps = entry->fast_rate_gps;
            rt->learned_slow_rate_gps = entry->slow_rate_gps;
            rt->learned_finish_trim_g = entry->finish_trim_g;
            rt->learned_fast_start_gate_pct = entry->fast_start_gate_pct;
            rt->learned_slow_start_gate_pct = entry->slow_start_gate_pct;
            if (!env->jar_tare_get || !env->jar_tare_get(&rt->run_base_weight_g)) {
                rt->run_base_weight_g = tick->latest ? tick->latest->grams : 0.0f;
            }
            rt->control_gate_cmd_pct = flow_runtime_fast_start_gate_pct(rt, tick->params);
            rt->adapted_drip_wait_ms = entry->drip_wait_ms;
            log_learned_entry("starting flow-control fill with", tick->preset_index, entry);
        } else {
            rt->refill_count++;
            // close_early_relax was already increased (proportional to the
            // deficit) in the verify-underweight branch that triggered this
            // refill; do not add a second fixed step here.
            rt->first_close_seen = false;
            rt->first_close_ts_us = 0;
            rt->last_gain_ts_us = 0;
            rt->rel_at_first_close_g = 0.0f;
            rt->measured_post_close_gain_g = 0.0f;
            rt->last_post_close_gain_g = 0.0f;
            rt->control_gate_cmd_pct = clampf_local(fmaxf(rt->control_gate_cmd_pct, fast_gate_min_pct(tick->params) + 2.0f),
                                                    fast_gate_min_pct(tick->params),
                                                    fast_gate_max_pct(tick->params));
            ESP_LOGI(TAG,
                     "refill attempt=%u relaxed_close_early=%.1f g reopen_gate=%.1f%%",
                     (unsigned)rt->refill_count,
                     (double)rt->close_early_relax_g,
                     (double)rt->control_gate_cmd_pct);
        }

        rt->last_rate_ts_us = 0;
        rt->last_rel_g = 0.0f;
        rt->control_last_update_us = 0;
        clear_live_rate_estimates(rt);
        flow_reset_counters(rt);
        update_thresholds(rt, tick);
        update_control_targets(rt, tick->params);
        ESP_LOGI(TAG,
                 "flow thresholds near_close=%.1f g close_early=%.1f g predicted_remaining=%.1f g finish_trim=%.1f g target_fast=%.1f g/s target_slow=%.1f g/s drip_wait=%.0f ms start_gate=%.1f%% slow_start_gate=%.1f%%",
                 (double)rt->adapted_near_close_g,
                 (double)rt->adapted_close_early_g,
                 (double)rt->predicted_remaining_g,
                 (double)rt->learned_finish_trim_g,
                 (double)rt->learned_fast_rate_gps,
                 (double)rt->learned_slow_rate_gps,
                 (double)rt->adapted_drip_wait_ms,
                 (double)rt->control_gate_cmd_pct,
                 (double)flow_runtime_slow_start_gate_pct(rt, tick->params));
        env->publish_fill_start(tick->run_id,
                                tick->slot_idx,
                                tick->strategy_name,
                                tick->params,
                                tick->latest ? tick->latest->grams : 0.0f);
        flow_command_gate(rt,
                          env,
                          rt->control_gate_cmd_pct,
                          (prev_state == FILLER_VERIFY_TARGET) ? "flow_refill_open" : "flow_open",
                          tick->now_us,
                          true);
        publish_runtime_snapshot(rt, env, tick);
        break;
    }
    case FILLER_DRIP_WAIT:
        flow_reset_counters(rt);
        mark_first_close(rt, tick);
        env->gate_close_label("drip_wait");
        publish_runtime_snapshot(rt, env, tick);
        break;
    case FILLER_VERIFY_TARGET:
        flow_reset_counters(rt);
        env->gate_close_label("close");
        publish_runtime_snapshot(rt, env, tick);
        break;
    default:
        break;
    }
}

static filler_state_t flow_step(filler_strategy_runtime_t *rt,
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
            env->clear_sample_telemetry();
            publish_summary_and_learn(rt, env, tick, "scale_stale", false);
            return next_state;
        }

        float rel_g = tick->latest->grams - rt->run_base_weight_g;
        update_rate_estimates(rt, tick);
        update_thresholds(rt, tick);
        update_control_targets(rt, tick->params);

        float hard_margin_g = fmaxf((float)tick->params->target_tol_high_g + FLOW_HARD_OVERFILL_MARGIN_EXTRA_G,
                                    FLOW_HARD_OVERFILL_MARGIN_MIN_G);
        if (rel_g > (float)tick->params->target_grams + hard_margin_g) {
            ESP_LOGE(TAG, "hard safety overweight: rel=%.1f g", (double)rel_g);
            env->gate_close_label("safety_close");
            env->set_fault(FLT_WEIGHT_RANGE);
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "hard_overfill", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }

        float no_response_limit_s = clampf_local(fmaxf(rt->learned_dead_time_s * 2.5f, rt->learned_dead_time_s + 0.8f),
                                                 NO_RESPONSE_MIN_S,
                                                 NO_RESPONSE_MAX_S);
        if (!rt->response_detected && ((tick->now_us - rt->fill_open_ts_us) / 1000000.0f) > no_response_limit_s) {
            ESP_LOGE(TAG, "no weight response after opening");
            env->gate_close_label("no_response");
            env->set_fault(FLT_EMPTY_HONEY);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "no_response", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }

        // Overall fill timeout (per attempt): flow responded but then stalled
        // before target (bucket emptied / partial clog). Without this the fill
        // would hang. Uses the FSM state-enter time so each refill attempt is
        // bounded (fill_open_ts_us is not reset on refill in this strategy).
        if ((tick->now_us - tick->state_enter_us) > ((int64_t)tick->params->fill_timeout_ms * 1000)) {
            ESP_LOGE(TAG, "fill timeout after %.1f s (rel=%.1f g, target=%u g) -> fault",
                     (double)((tick->now_us - tick->state_enter_us) / 1000000.0f),
                     (double)rel_g,
                     (unsigned)tick->params->target_grams);
            env->gate_close_label("fill_timeout");
            env->set_fault(FLT_SERVO_TIMEOUT);
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "fill_timeout", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }

        float safe_limit_gps = safe_rate_limit_gps(rt, tick->params);
        // Debounce over consecutive samples so the impact spike when honey first
        // hits the glass does not act on the safe-rate limit.
        if (tick->new_sample) {
            if (rt->filtered_rate_gps > safe_limit_gps) {
                if (rt->cnt_safe_rate < 255) rt->cnt_safe_rate++;
            } else {
                rt->cnt_safe_rate = 0;
            }
        }
        if (rt->cnt_safe_rate >= RATE_SPIKE_CONFIRM_SAMPLES) {
            if (rt->safe_reduce_ts_us == 0) {
                rt->safe_reduce_ts_us = tick->now_us;
            }
            // Escalate to a fault if stepping the gate down cannot bring the flow
            // under the limit within the grace period: the medium is too thin for
            // the gate operating points and the operator must lower them.
            float safe_reduce_elapsed_s = (float)(tick->now_us - rt->safe_reduce_ts_us) / 1000000.0f;
            if (safe_reduce_elapsed_s > SAFE_REDUCE_ESCALATE_S) {
                ESP_LOGE(TAG,
                         "flow runaway: filtered_rate=%.1f g/s still above limit=%.1f g/s after %.1f s at gate %.1f%% -> fault (lower gate %%)",
                         (double)rt->filtered_rate_gps,
                         (double)safe_limit_gps,
                         (double)safe_reduce_elapsed_s,
                         (double)rt->control_gate_cmd_pct);
                env->gate_close_label("runaway_close");
                env->set_fault(FLT_WEIGHT_RANGE);
                mark_first_close(rt, tick);
                publish_runtime_snapshot(rt, env, tick);
                publish_summary_and_learn(rt, env, tick, "flow_runaway", false);
                env->clear_sample_telemetry();
                return FILLER_FAULT;
            }
            // Pace corrective steps by the plant dead time so each step's effect
            // is actually seen before reacting again, instead of ratcheting the
            // gate down on every incoming sample.
            float holdoff_ms = control_holdoff_ms(rt);
            bool first_step = (rt->safe_reduce_last_action_us == 0);
            bool holdoff_elapsed = !first_step &&
                (tick->now_us - rt->safe_reduce_last_action_us) >= (int64_t)(holdoff_ms * 1000.0f);
            if (first_step || holdoff_elapsed) {
                // Scale the step with how far over the limit the rate is: just
                // over the limit gets a small nudge, far over (a real runaway)
                // still gets a large, fast step.
                float over_ratio = (safe_limit_gps > 0.0f) ? (rt->filtered_rate_gps / safe_limit_gps) : SAFE_REDUCE_OVER_RATIO_HARD;
                float severity = clampf_local((over_ratio - SAFE_REDUCE_OVER_RATIO_SOFT) /
                                              (SAFE_REDUCE_OVER_RATIO_HARD - SAFE_REDUCE_OVER_RATIO_SOFT), 0.0f, 1.0f);
                float reduce_step = SAFE_REDUCE_STEP_MIN_PCT + severity * (SAFE_REDUCE_STEP_MAX_PCT - SAFE_REDUCE_STEP_MIN_PCT);
                float gate_min = rt->near_close_logged ? slow_gate_min_pct(tick->params) : fast_gate_min_pct(tick->params);
                float reduced_gate = clampf_local(rt->control_gate_cmd_pct - reduce_step,
                                                  gate_min,
                                                  fast_gate_max_pct(tick->params));
                ESP_LOGW(TAG,
                         "safe-rate reduction filtered_rate=%.1f g/s limit=%.1f g/s gate %.1f%%->%.1f%% (step=%.1f%%, over=%.2fx)",
                         (double)rt->filtered_rate_gps,
                         (double)safe_limit_gps,
                         (double)rt->control_gate_cmd_pct,
                         (double)reduced_gate,
                         (double)reduce_step,
                         (double)over_ratio);
                flow_command_gate(rt, env, reduced_gate, "safe_reduce", tick->now_us, true);
                rt->safe_reduce_last_action_us = tick->now_us;
            }
            publish_runtime_snapshot(rt, env, tick);
            return state;
        }
        // Flow back under the safe limit: clear the escalation timer.
        rt->safe_reduce_ts_us = 0;
        rt->safe_reduce_last_action_us = 0;

        if (!tick->new_sample) {
            publish_runtime_snapshot(rt, env, tick);
            return state;
        }

        float remaining_g = (float)tick->params->target_grams - rel_g;
        if (rel_g >= (float)tick->params->target_grams || remaining_g <= rt->adapted_close_early_g) {
            ESP_LOGI(TAG,
                     "flow final close remaining=%.1f g threshold=%.1f g predicted_remaining=%.1f g",
                     (double)remaining_g,
                     (double)rt->adapted_close_early_g,
                     (double)rt->predicted_remaining_g);
            env->gate_close_label("flow_close");
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick);
            return FILLER_DRIP_WAIT;
        }

        if (remaining_g <= rt->adapted_near_close_g && !rt->near_close_logged) {
            rt->near_close_logged = true;
            rt->gate_at_slow_entry_pct = rt->control_gate_cmd_pct;
            float slow_start_gate = flow_runtime_slow_start_gate_pct(rt, tick->params);
            if (rt->control_gate_cmd_pct > slow_start_gate) {
                flow_command_gate(rt, env, slow_start_gate, "flow_slow_enter", tick->now_us, true);
            }
            update_control_targets(rt, tick->params);
            ESP_LOGI(TAG,
                     "flow slow-phase enter remaining=%.1f g threshold=%.1f g target_rate=%.1f g/s gate=%.1f%% start_gate=%.1f%%",
                     (double)remaining_g,
                     (double)rt->adapted_near_close_g,
                     (double)rt->control_target_rate_gps,
                     (double)rt->control_gate_cmd_pct,
                     (double)slow_start_gate);
            publish_runtime_snapshot(rt, env, tick);
            return state;
        }

        (void)update_controller_gate(rt, env, tick);
        publish_runtime_snapshot(rt, env, tick);
        return state;
    }

    case FILLER_DRIP_WAIT:
        update_rate_estimates(rt, tick);
        update_control_targets(rt, tick->params);
        track_post_close_gain(rt, tick);
        if ((tick->now_us - tick->state_enter_us) >= ((int64_t)rt->adapted_drip_wait_ms * 1000.0f)) {
            ESP_LOGI(TAG,
                     "drip wait complete waited=%.0f ms post_close_gain=%.1f g",
                     (double)rt->adapted_drip_wait_ms,
                     (double)rt->measured_post_close_gain_g);
            publish_runtime_snapshot(rt, env, tick);
            return FILLER_VERIFY_TARGET;
        }
        publish_runtime_snapshot(rt, env, tick);
        return state;

    case FILLER_VERIFY_TARGET: {
        filler_state_t next_state = state;
        if (!env->require_fresh_or_fault("verify target", tick->latest, &next_state)) {
            env->clear_sample_telemetry();
            publish_summary_and_learn(rt, env, tick, "scale_stale", false);
            return next_state;
        }

        update_rate_estimates(rt, tick);
        update_control_targets(rt, tick->params);
        track_post_close_gain(rt, tick);
        float rel_g = tick->latest->grams - rt->run_base_weight_g;
        float tol_low_g = (float)tick->params->target_tol_low_g;
        float tol_high_g = (float)tick->params->target_tol_high_g;

        if (tick->new_sample &&
            stable_above(rel_g,
                         (float)tick->params->target_grams + tol_high_g,
                         &rt->cnt_over,
                         VERIFY_CONFIRM_SAMPLES)) {
            ESP_LOGE(TAG, "flow overweight: rel=%.1f g", (double)rel_g);
            env->set_fault(FLT_WEIGHT_RANGE);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "overweight", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }

        if (tick->new_sample &&
            stable_below(rel_g,
                         (float)tick->params->target_grams - tol_low_g,
                         &rt->cnt_under,
                         VERIFY_CONFIRM_SAMPLES)) {
            float under = (float)tick->params->target_grams - rel_g;
            ESP_LOGI(TAG, "flow underweight: rel=%.1f g (-%.1f g) -> refill",
                     (double)rel_g,
                     (double)under);
            // Relax by ~the measured deficit so one refill reaches target.
            float relax_step = fmaxf(under * REFILL_CLOSE_RELAX_FACTOR, REFILL_RELAX_STEP_G);
            rt->close_early_relax_g += relax_step;
            ESP_LOGI(TAG, "relax close_early by %.1f g (deficit %.1f g) -> total_relax=%.1f g",
                     (double)relax_step, (double)under, (double)rt->close_early_relax_g);
            publish_runtime_snapshot(rt, env, tick);
            return FILLER_FILL;
        }

        if (rt->sample_count >= VERIFY_CONFIRM_SAMPLES) {
            uint8_t next = env->slot_next_idx(tick->slot_idx, tick->params->slots_total);
            ESP_LOGI(TAG,
                     "flow target verified rel=%.1f g target=%u g tol=[-%u,+%u] confirm=%u",
                     (double)rel_g,
                     (unsigned)tick->params->target_grams,
                     (unsigned)tick->params->target_tol_low_g,
                     (unsigned)tick->params->target_tol_high_g,
                     (unsigned)VERIFY_CONFIRM_SAMPLES);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "ok", true);
            env->clear_sample_telemetry();
            env->set_slot(next);
            return FILLER_FIND_SLOT;
        }

        publish_runtime_snapshot(rt, env, tick);
        return state;
    }

    default:
        return state;
    }
}

const filler_strategy_ops_t g_filler_strategy_flow_control_ops = {
    .name = "flow-control",
    .on_enter = flow_on_enter,
    .step = flow_step,
};
