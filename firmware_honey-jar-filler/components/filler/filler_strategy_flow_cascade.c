#include "filler_strategy.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "esp_log.h"

// Continuous ("regelungsbasierter") dosing strategy: a cascade controller with
// an inner PI rate loop, an outer deceleration rate profile, feedforward gate
// via an online-estimated plant gain, and a Smith predictor for dead-time
// compensation. All plant parameters (dead time, gain, rates, post-close gain,
// drip) are identified online from the same measurements the adaptive strategy
// already uses, so a run stays stable even with rough first-try defaults.

// Rate estimation filter (shared shape with the other strategies).
#define RATE_FILTER_ALPHA_RISE 0.18f
#define RATE_FILTER_ALPHA_FALL 0.55f
#define RATE_MIN_VALID_GPS 2.0f
#define RATE_ZERO_EPS_GPS 0.5f
#define RATE_NO_FLOW_RESET_SAMPLES 2u

// Learning: first successful fill dominates via the count warmup (learn_ewma).
#define LEARN_ALPHA_DEAD_TIME 0.35f
#define LEARN_ALPHA_POST_CLOSE 0.65f
#define LEARN_ALPHA_RATE 0.20f
#define LEARN_ALPHA_GATE_GAIN 0.20f
#define LEARN_ALPHA_DRIP_WAIT 0.35f
#define LEARN_ALPHA_FINISH_TRIM 0.18f

#define VERIFY_CONFIRM_SAMPLES 4u

// Response / dead-time detection. Generous so thick honey (long dead time) and
// the first jar are not falsely cancelled.
#define RESPONSE_THRESHOLD_MIN_G 1.5f
#define RESPONSE_THRESHOLD_MAX_G 6.0f
#define DEAD_TIME_MIN_S 0.10f
#define DEAD_TIME_MAX_S 15.00f
#define NO_RESPONSE_MIN_S 15.00f
#define NO_RESPONSE_MAX_S 30.00f

// Final-close / refill shaping.
#define CLOSE_BUFFER_G 1.5f
#define REFILL_RELAX_STEP_G 2.5f
#define REFILL_CLOSE_RELAX_FACTOR 0.90f
#define FINISH_TRIM_MIN_G -10.0f
#define FINISH_TRIM_MAX_G 12.0f

// Safety limits.
#define HARD_OVERFILL_MARGIN_MIN_G 12.0f
#define HARD_OVERFILL_MARGIN_EXTRA_G 6.0f
#define SAFE_RATE_MIN_GPS 25.0f
#define SAFE_RATE_MAX_GPS 140.0f
#define SAFE_RATE_MULT 1.8f
// Lower bound on how fast a "controlled" fill may run (target_g / this =
// abs_ceiling in safe_rate_limit_gps). Small jars run at a high g/s operating
// point on purpose (see fast_rate_gps), so this must stay below that intended
// rate with headroom, or the safe-rate reducer fights the cascade controller
// on every fill instead of only reacting to genuine runaways.
#define MIN_CONTROLLED_FILL_S 4.0f
#define RATE_SPIKE_CONFIRM_SAMPLES 3u
#define SAFE_REDUCE_ESCALATE_S 5.0f
// Progressive safe-rate gate reduction: step size scales with how far the
// rate is over the limit (ratio-based), instead of a single fixed cut. A rate
// just over the limit gets a small nudge and then waits a full dead time to
// see the effect before reacting again; a rate far over the limit (a real
// runaway) still gets a large, fast step. Without pacing by the dead time,
// the fixed-step version reacted on every sample -- much faster than the
// plant could ever respond -- and ratcheted the gate closed in a handful of
// samples well before the fill's own dead time even elapsed once.
#define SAFE_REDUCE_STEP_MIN_PCT 2.0f
#define SAFE_REDUCE_STEP_MAX_PCT 20.0f
#define SAFE_REDUCE_OVER_RATIO_SOFT 1.05f
#define SAFE_REDUCE_OVER_RATIO_HARD 2.0f

// Gate ceiling ramp. The very first fill for a preset must stay at (or below)
// the preset's configured max_gate_pct -- opening straight to the machine's
// full 100% before anything about the medium/jar is known would be a
// critical, messy failure mode. Once a preset has accumulated enough
// successful fills to trust the learned rates and plant gain K, relax the
// ceiling toward true machine max so the cascade is not permanently capped at
// a value that was only ever meant as a first-fill safety margin.
#define GATE_CEILING_MAX_PCT 100.0f
#define GATE_CEILING_RELAX_FILLS 6u

// Cascade controller. Deliberately gentle: the feedforward gate carries the
// operating point, so the PI only trims. Updates are paced by the dead time so
// the loop never reacts faster than the plant can answer.
#define CASC_KP_PCT_PER_GPS 1.2f
#define CASC_KI_PCT_PER_GPS_S 0.40f
#define CASC_STEP_MAX_PCT 6.0f
#define CASC_INTEG_LIMIT_PCT 35.0f
#define CASC_DEADBAND_GPS 0.6f
#define CASC_HOLDOFF_MIN_MS 300.0f
#define CASC_HOLDOFF_MAX_MS 2500.0f

// Outer deceleration profile: full fast rate while far from target, then a
// linear taper down to the slow rate as the (in-flight-compensated) remaining
// mass shrinks into the deceleration band.
#define CASC_DECEL_BAND_MIN_G 20.0f
#define CASC_DECEL_BAND_MULT 1.6f
#define CASC_RATE_MIN_FLOOR_GPS 3.0f

// Plant-gain (K̂ = rate per % gate) plausibility bounds for the feedforward and
// Smith model.
#define CASC_GATE_GAIN_MIN 0.02f
#define CASC_GATE_GAIN_MAX 3.00f
// Live plant-gain identification: only sample K̂ once the gate has been held
// longer than the dead time (plus a settle margin), so the measured rate
// reflects the current gate rather than the one commanded a dead time ago.
#define CASC_GAIN_SETTLE_MARGIN_S 0.5f
#define CASC_GAIN_LIVE_ALPHA 0.10f

// Smith predictor model.
#define CASC_MODEL_TAU_S 0.60f
#define SMITH_DELAY_MAX 256

#define DELTA_RATE_MAX_DT_S 0.80f
#define DELTA_RATE_MIN_DT_S 0.05f
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
    float gate_gain_gps_per_pct;   // K̂ for feedforward + Smith model
    float finish_trim_g;
    float drip_wait_ms;
} cascade_learned_entry_t;

typedef enum {
    CASC_PHASE_FAST = 0,
    CASC_PHASE_SLOW,
    CASC_PHASE_CLOSED,
    CASC_PHASE_REFILL,
} cascade_phase_t;

static const char *TAG = "fill_cascade";
static cascade_learned_entry_t s_learned[APP_PRESET_COUNT];

// Smith-predictor model state (only one fill runs at a time, so file-scope is
// fine and keeps the shared runtime struct small).
static float s_model_rate_gps;       // ŷ  (delay-free model rate)
static float s_model_delayed_gps;    // ŷ_d (model rate seen through the delay)
static float s_model_hist[SMITH_DELAY_MAX];
static uint16_t s_model_head;
static uint16_t s_model_count;
static float s_integ_pct;            // PI integrator (gate %)

static float clampf_local(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float ewma(float prev, float sample, float alpha)
{
    if (prev <= 0.0f) return sample;
    return prev + alpha * (sample - prev);
}

// Count-warmup EWMA: signed-safe, first observations dominate the seed.
static float learn_ewma(float prev, float sample, float base_alpha, uint32_t observations)
{
    float warmup = 1.0f / (float)(observations + 1u);
    float alpha = (warmup > base_alpha) ? warmup : base_alpha;
    return prev + alpha * (sample - prev);
}

static void clear_live_rate_estimates(filler_strategy_runtime_t *rt)
{
    if (!rt) return;
    rt->raw_rate_gps = 0.0f;
    rt->filtered_rate_gps = 0.0f;
    rt->no_flow_count = 0;
}

static void cascade_model_reset(void)
{
    s_model_rate_gps = 0.0f;
    s_model_delayed_gps = 0.0f;
    s_model_head = 0;
    s_model_count = 0;
    memset(s_model_hist, 0, sizeof(s_model_hist));
}

static bool stable_above(float value, float threshold, uint8_t *count, uint8_t required)
{
    if (!count || required == 0) return value >= threshold;
    if (value >= threshold) { if (*count < required) (*count)++; } else { *count = 0; }
    return *count >= required;
}

static bool stable_below(float value, float threshold, uint8_t *count, uint8_t required)
{
    if (!count || required == 0) return value <= threshold;
    if (value <= threshold) { if (*count < required) (*count)++; } else { *count = 0; }
    return *count >= required;
}

static const char *phase_name(cascade_phase_t phase)
{
    switch (phase) {
    case CASC_PHASE_FAST:   return "fast_control";
    case CASC_PHASE_SLOW:   return "slow_control";
    case CASC_PHASE_CLOSED: return "closed";
    case CASC_PHASE_REFILL: return "refill";
    default:                return "?";
    }
}

static cascade_phase_t runtime_phase(const filler_strategy_runtime_t *rt)
{
    if (!rt) return CASC_PHASE_CLOSED;
    if (rt->first_close_seen) return CASC_PHASE_CLOSED;
    if (rt->refill_count > 0 && !rt->near_close_logged) return CASC_PHASE_REFILL;
    if (rt->near_close_logged) return CASC_PHASE_SLOW;
    return CASC_PHASE_FAST;
}

static void cascade_reset_counters(filler_strategy_runtime_t *rt)
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

static void cascade_defaults_from_params(cascade_learned_entry_t *entry, const app_params_t *params)
{
    if (!entry || !params) return;
    float poll_s = ((float)CONFIG_HX711_POLL_INTERVAL_MS / 1000.0f) * 2.5f;
    entry->initialized = 1u;
    entry->dead_time_s = clampf_local(poll_s, 0.25f, 1.20f);
    entry->post_close_gain_g = clampf_local((float)params->close_remaining_g * 0.55f, 1.0f, 120.0f);
    entry->fast_rate_gps = fallback_fast_rate_gps(params);
    entry->slow_rate_gps = fallback_slow_rate_gps(params);
    // Seed K̂ from the fast rate reached near the preset's max gate opening.
    float max_gate = clampf_local((float)params->max_gate_pct, 5.0f, 100.0f);
    entry->gate_gain_gps_per_pct = clampf_local(entry->fast_rate_gps / max_gate,
                                                CASC_GATE_GAIN_MIN, CASC_GATE_GAIN_MAX);
    entry->finish_trim_g = 0.0f;
    entry->drip_wait_ms = clampf_local((float)params->drip_delay_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS);
}

static void log_learned_entry(const char *prefix, uint8_t idx, const cascade_learned_entry_t *e)
{
    if (!prefix || !e) return;
    ESP_LOGI(TAG,
             "%s preset=%u dead_time=%.3f s post_close=%.1f g fast=%.1f g/s slow=%.1f g/s K=%.3f g/s/%% trim=%.1f g drip=%.0f ms fills=%lu",
             prefix, (unsigned)idx,
             (double)e->dead_time_s, (double)e->post_close_gain_g,
             (double)e->fast_rate_gps, (double)e->slow_rate_gps,
             (double)e->gate_gain_gps_per_pct, (double)e->finish_trim_g,
             (double)e->drip_wait_ms, (unsigned long)e->successful_fills);
}

static cascade_learned_entry_t *cascade_entry_for_preset(uint8_t idx, const app_params_t *params)
{
    if (idx >= APP_PRESET_COUNT) idx = 0;
    cascade_learned_entry_t *e = &s_learned[idx];
    if (!e->initialized) {
        cascade_defaults_from_params(e, params);
        log_learned_entry("initialized cascade defaults for", idx, e);
    }
    return e;
}

// Confidence-based gate ceiling: preset max_gate_pct on an untried preset,
// linearly relaxing to GATE_CEILING_MAX_PCT once GATE_CEILING_RELAX_FILLS
// successful fills have been learned for it.
static float gate_ceiling_for_entry(const cascade_learned_entry_t *entry, const app_params_t *params)
{
    float preset_cap = clampf_local(params ? (float)params->max_gate_pct : GATE_CEILING_MAX_PCT, 5.0f, 100.0f);
    uint32_t fills = entry ? entry->successful_fills : 0u;
    float progress = clampf_local((float)fills / (float)GATE_CEILING_RELAX_FILLS, 0.0f, 1.0f);
    return preset_cap + progress * (GATE_CEILING_MAX_PCT - preset_cap);
}

static float response_threshold_g(const filler_strategy_tick_t *tick)
{
    float candidate = tick && tick->params ? ((float)tick->params->target_grams * 0.005f) : 2.0f;
    return clampf_local(candidate, RESPONSE_THRESHOLD_MIN_G, RESPONSE_THRESHOLD_MAX_G);
}

static bool plausible_positive(float v, float lo, float hi) { return v >= lo && v <= hi; }

static bool model_is_usable(const filler_strategy_runtime_t *rt)
{
    // Only trust the Smith prediction once the plant gain is plausibly known;
    // otherwise the controller falls back to the raw measured rate.
    return rt && rt->learned_gate_gain_gps_per_pct >= CASC_GATE_GAIN_MIN &&
           rt->learned_gate_gain_gps_per_pct <= CASC_GATE_GAIN_MAX &&
           rt->learned_fast_rate_gps > 1.0f;
}

// Update the delay-free plant model ŷ and the delayed model output ŷ_d from the
// current gate command. Runs every new weight sample.
static void cascade_update_model(filler_strategy_runtime_t *rt, float dt_s)
{
    if (!rt || dt_s <= 0.0f) return;
    float k = rt->learned_gate_gain_gps_per_pct;
    if (k < CASC_GATE_GAIN_MIN) k = CASC_GATE_GAIN_MIN;
    float ss = k * rt->control_gate_cmd_pct;                 // steady-state model rate
    float a = clampf_local(dt_s / CASC_MODEL_TAU_S, 0.0f, 1.0f);
    s_model_rate_gps += a * (ss - s_model_rate_gps);

    s_model_hist[s_model_head] = s_model_rate_gps;
    s_model_head = (uint16_t)((s_model_head + 1u) % SMITH_DELAY_MAX);
    if (s_model_count < SMITH_DELAY_MAX) s_model_count++;

    int delay_samples = (int)lroundf(rt->learned_dead_time_s / dt_s);
    if (delay_samples < 0) delay_samples = 0;
    if (delay_samples > SMITH_DELAY_MAX - 1) delay_samples = SMITH_DELAY_MAX - 1;
    if (delay_samples >= (int)s_model_count) {
        // Not enough history yet: the model output "L ago" was still zero
        // (initial condition). This is what keeps the predictor from winding up
        // during the initial dead time, when the measured rate is also zero.
        s_model_delayed_gps = 0.0f;
    } else {
        int idx = (int)s_model_head - 1 - delay_samples;
        idx %= SMITH_DELAY_MAX;
        if (idx < 0) idx += SMITH_DELAY_MAX;
        s_model_delayed_gps = s_model_hist[idx];
    }
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
            if (rt->raw_rate_gps < RATE_MIN_VALID_GPS) rt->raw_rate_gps = 0.0f;

            if (rt->raw_rate_gps <= 0.0f) {
                if (rt->no_flow_count < 255) rt->no_flow_count++;
                if (rt->no_flow_count >= RATE_NO_FLOW_RESET_SAMPLES) rt->filtered_rate_gps = 0.0f;
                else rt->filtered_rate_gps = ewma(rt->filtered_rate_gps, 0.0f, RATE_FILTER_ALPHA_FALL);
            } else {
                rt->no_flow_count = 0;
                float alpha = (rt->raw_rate_gps >= rt->filtered_rate_gps) ? RATE_FILTER_ALPHA_RISE : RATE_FILTER_ALPHA_FALL;
                rt->filtered_rate_gps = ewma(rt->filtered_rate_gps, rt->raw_rate_gps, alpha);
            }
            if (rt->filtered_rate_gps < RATE_ZERO_EPS_GPS) rt->filtered_rate_gps = 0.0f;

            cascade_phase_t phase = runtime_phase(rt);
            if ((phase == CASC_PHASE_FAST || phase == CASC_PHASE_REFILL) && rt->raw_rate_gps > 0.1f) {
                rt->fast_rate_sum_gps += rt->raw_rate_gps;
                rt->fast_rate_count++;
            } else if (phase == CASC_PHASE_SLOW && rt->raw_rate_gps > 0.1f) {
                rt->slow_rate_sum_gps += rt->raw_rate_gps;
                rt->slow_rate_count++;
            }

            // Live, dead-time-aware plant-gain identification. Sampling only in
            // steady state (gate held longer than the dead time) captures the
            // local nonlinear gain at the current operating point and updates
            // K̂ within the fill, so the feedforward and Smith model track as
            // the controller sweeps fast -> slow.
            float gate_stable_s = (rt->control_last_update_us > 0)
                ? (float)(tick->latest->ts_us - rt->control_last_update_us) / 1000000.0f : 0.0f;
            if (rt->filtered_rate_gps > 1.0f && rt->control_gate_cmd_pct > 5.0f &&
                gate_stable_s > (rt->learned_dead_time_s + CASC_GAIN_SETTLE_MARGIN_S)) {
                float k = rt->filtered_rate_gps / rt->control_gate_cmd_pct;
                if (k >= CASC_GATE_GAIN_MIN && k <= CASC_GATE_GAIN_MAX) {
                    rt->learned_gate_gain_gps_per_pct =
                        ewma(rt->learned_gate_gain_gps_per_pct, k, CASC_GAIN_LIVE_ALPHA);
                    if (rt->gate_gain_count < 65535) rt->gate_gain_count++;
                }
            }

            if (!rt->first_close_seen) cascade_update_model(rt, dt_s);
        }
    }

    if (!rt->response_detected && rel_g >= response_threshold_g(tick)) {
        rt->response_detected = true;
        rt->first_response_ts_us = tick->latest->ts_us;
        rt->measured_dead_time_s = clampf_local((float)(tick->latest->ts_us - rt->fill_open_ts_us) / 1000000.0f,
                                                DEAD_TIME_MIN_S, DEAD_TIME_MAX_S);
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
    if (gain_g > rt->last_post_close_gain_g + 0.25f) rt->last_gain_ts_us = tick->latest->ts_us;
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
    float abs_ceiling = SAFE_RATE_MAX_GPS;
    if (params && params->target_grams > 0) abs_ceiling = (float)params->target_grams / MIN_CONTROLLED_FILL_S;
    abs_ceiling = clampf_local(abs_ceiling, SAFE_RATE_MIN_GPS, SAFE_RATE_MAX_GPS);
    float learned = rt ? rt->learned_fast_rate_gps : 0.0f;
    if (learned > 1.0f) {
        float rel = clampf_local(learned * SAFE_RATE_MULT, SAFE_RATE_MIN_GPS, SAFE_RATE_MAX_GPS);
        return fminf(abs_ceiling, rel);
    }
    return abs_ceiling;
}

static void cascade_command_gate(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                                 float gate_pct, const char *label, int64_t now_us, bool force)
{
    if (!rt || !env || !env->gate_set_percent_label) return;
    gate_pct = clampf_local(gate_pct, 0.0f, 100.0f);
    if (!force && fabsf(gate_pct - rt->control_gate_cmd_pct) < 0.05f) return;
    rt->control_gate_cmd_pct = gate_pct;
    rt->control_last_update_us = now_us;
    env->gate_set_percent_label(gate_pct, label);
}

static float control_holdoff_ms(const filler_strategy_runtime_t *rt)
{
    if (!rt) return CASC_HOLDOFF_MIN_MS;
    float dead_s = rt->learned_dead_time_s;
    if (rt->response_detected && rt->measured_dead_time_s > 0.0f) dead_s = rt->measured_dead_time_s;
    return clampf_local(dead_s * 1000.0f, CASC_HOLDOFF_MIN_MS, CASC_HOLDOFF_MAX_MS);
}

// Outer loop: deceleration rate profile. Returns the rate setpoint ṁ*.
static float profile_target_rate_gps(const filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick,
                                     float remaining_g)
{
    float fast = (rt->learned_fast_rate_gps > 1.0f) ? rt->learned_fast_rate_gps : fallback_fast_rate_gps(tick->params);
    float slow = (rt->learned_slow_rate_gps > 0.5f) ? rt->learned_slow_rate_gps : fallback_slow_rate_gps(tick->params);
    if (slow < CASC_RATE_MIN_FLOOR_GPS) slow = CASC_RATE_MIN_FLOOR_GPS;
    if (fast < slow) fast = slow;

    // Compensate for the mass still in flight so the taper is referenced to the
    // mass that will actually still be controllable.
    float in_flight = rt->learned_dead_time_s * fmaxf(0.0f, rt->filtered_rate_gps) + rt->learned_post_close_gain_g;
    float eff_remaining = remaining_g - in_flight;
    if (eff_remaining < 0.0f) eff_remaining = 0.0f;

    float band = fmaxf(CASC_DECEL_BAND_MIN_G, rt->learned_post_close_gain_g * CASC_DECEL_BAND_MULT +
                                              rt->learned_dead_time_s * fast);
    float frac = clampf_local(eff_remaining / band, 0.0f, 1.0f);
    return slow + (fast - slow) * frac;
}

static void update_control_targets(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick, float remaining_g)
{
    if (!rt || !tick) return;
    rt->control_target_rate_gps = profile_target_rate_gps(rt, tick, remaining_g);
    float fb = model_is_usable(rt)
        ? (s_model_rate_gps + (rt->filtered_rate_gps - s_model_delayed_gps))  // Smith prediction
        : rt->filtered_rate_gps;
    rt->control_rate_error_gps = rt->control_target_rate_gps - fb;
}

// Inner loop: PI on the rate error with feedforward gate and anti-windup.
static void cascade_run_controller(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                                   const filler_strategy_tick_t *tick, float remaining_g)
{
    if (!rt || !env || !tick || !tick->new_sample || !rt->response_detected || rt->first_close_seen) return;

    float holdoff_ms = control_holdoff_ms(rt);
    if (rt->control_last_update_us > 0 &&
        (tick->now_us - rt->control_last_update_us) < (int64_t)(holdoff_ms * 1000.0f)) {
        return;
    }
    float holdoff_s = holdoff_ms / 1000.0f;

    update_control_targets(rt, tick, remaining_g);
    float error = rt->control_rate_error_gps;
    if (fabsf(error) <= CASC_DEADBAND_GPS) {
        // Within deadband: hold gate, let the integrator rest.
        return;
    }

    // Confidence-based ceiling (preset max_gate_pct until the preset has
    // proven itself, then relaxing toward machine max -- see gate_ceiling_for_entry).
    float gate_cap = clampf_local(rt->gate_ceiling_pct, 5.0f, 100.0f);
    float k = clampf_local(rt->learned_gate_gain_gps_per_pct, CASC_GATE_GAIN_MIN, CASC_GATE_GAIN_MAX);
    float gate_ff = rt->control_target_rate_gps / k;              // feedforward operating point
    s_integ_pct = clampf_local(s_integ_pct + CASC_KI_PCT_PER_GPS_S * error * holdoff_s,
                               -CASC_INTEG_LIMIT_PCT, CASC_INTEG_LIMIT_PCT);
    float raw_gate = gate_ff + CASC_KP_PCT_PER_GPS * error + s_integ_pct;
    float next_gate = clampf_local(raw_gate, 0.0f, gate_cap);

    // Anti-windup: on saturation, unwind the integrator by the clipped amount.
    if (raw_gate != next_gate) s_integ_pct += (next_gate - raw_gate);

    // Rate-limit the actuator step for smoothness.
    float step = clampf_local(next_gate - rt->control_gate_cmd_pct, -CASC_STEP_MAX_PCT, CASC_STEP_MAX_PCT);
    next_gate = clampf_local(rt->control_gate_cmd_pct + step, 0.0f, gate_cap);

    if (fabsf(next_gate - rt->control_gate_cmd_pct) < 0.05f) return;
    ESP_LOGD(TAG, "ctrl phase=%s mstar=%.1f err=%.1f gate %.1f->%.1f ff=%.1f integ=%.1f",
             phase_name(runtime_phase(rt)), (double)rt->control_target_rate_gps, (double)error,
             (double)rt->control_gate_cmd_pct, (double)next_gate, (double)gate_ff, (double)s_integ_pct);
    cascade_command_gate(rt, env, next_gate, "cascade_ctrl", tick->now_us, false);
}

static void update_thresholds(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!rt || !tick || !tick->params) return;
    float close_rate = (rt->near_close_logged && rt->filtered_rate_gps > 0.5f)
        ? rt->filtered_rate_gps
        : ((rt->learned_slow_rate_gps > 0.5f) ? rt->learned_slow_rate_gps : fallback_slow_rate_gps(tick->params));
    float close_dead_mass = rt->learned_dead_time_s * close_rate;
    rt->predicted_remaining_g = rt->learned_post_close_gain_g + close_dead_mass;

    float close_max = fmaxf((float)tick->params->close_remaining_g * 1.8f, 10.0f);
    float close_candidate = rt->predicted_remaining_g + CLOSE_BUFFER_G -
                            rt->close_early_relax_g - rt->learned_finish_trim_g;
    rt->adapted_close_early_g = clampf_local(close_candidate, 2.0f, close_max);
    rt->adapted_drip_wait_ms = clampf_local(rt->adapted_drip_wait_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS);
}

static void publish_runtime_snapshot(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                                     const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !env->set_sample_telemetry || !tick) return;
    filler_strategy_sample_telemetry_t s = {
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
        .adapted_close_early_g = rt->adapted_close_early_g,
        .adapted_drip_wait_ms = rt->adapted_drip_wait_ms,
        .model_rate_gps = s_model_rate_gps,
        .model_delayed_gps = s_model_delayed_gps,
        .control_integ_pct = s_integ_pct,
        .gate_gain_gps_per_pct = rt->learned_gate_gain_gps_per_pct,
        .refill_count = rt->refill_count,
    };
    snprintf(s.strategy_name, sizeof(s.strategy_name), "%s", tick->strategy_name ? tick->strategy_name : "?");
    snprintf(s.gate_phase, sizeof(s.gate_phase), "%s", phase_name(runtime_phase(rt)));
    env->set_sample_telemetry(&s);
}

static void publish_summary_and_learn(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                                      const filler_strategy_tick_t *tick, const char *reason, bool success)
{
    if (!rt || !env || !env->publish_fill_summary || !tick || !tick->latest || !tick->params) return;
    cascade_learned_entry_t *entry = cascade_entry_for_preset(tick->preset_index, tick->params);
    float final_rel_g = tick->latest->grams - rt->run_base_weight_g;
    float fast_avg = (rt->fast_rate_count > 0) ? (rt->fast_rate_sum_gps / (float)rt->fast_rate_count) : 0.0f;
    float slow_avg = (rt->slow_rate_count > 0) ? (rt->slow_rate_sum_gps / (float)rt->slow_rate_count) : 0.0f;
    float fill_error_g = final_rel_g - (float)tick->params->target_grams;
    float fill_duration_s = (rt->fill_open_ts_us > 0) ? ((float)(tick->now_us - rt->fill_open_ts_us) / 1000000.0f) : 0.0f;
    float settled_wait_ms = rt->first_close_seen && rt->last_gain_ts_us >= rt->first_close_ts_us
        ? ((float)(rt->last_gain_ts_us - rt->first_close_ts_us) / 1000.0f) + DRIP_WAIT_SETTLE_MARGIN_MS
        : rt->adapted_drip_wait_ms;

    bool plausible = success && rt->response_detected &&
                     plausible_positive(rt->measured_dead_time_s, DEAD_TIME_MIN_S, DEAD_TIME_MAX_S) &&
                     plausible_positive(rt->measured_post_close_gain_g, 0.0f, 150.0f) &&
                     plausible_positive(fast_avg, 5.0f, 400.0f) &&
                     (slow_avg == 0.0f || plausible_positive(slow_avg, 1.0f, 250.0f));

    cascade_learned_entry_t before = *entry;
    if (plausible) {
        uint32_t obs = before.successful_fills;
        entry->dead_time_s = learn_ewma(entry->dead_time_s, rt->measured_dead_time_s, LEARN_ALPHA_DEAD_TIME, obs);
        entry->post_close_gain_g = learn_ewma(entry->post_close_gain_g, rt->measured_post_close_gain_g, LEARN_ALPHA_POST_CLOSE, obs);
        entry->fast_rate_gps = learn_ewma(entry->fast_rate_gps, fast_avg, LEARN_ALPHA_RATE, obs);
        if (slow_avg > 0.0f) entry->slow_rate_gps = learn_ewma(entry->slow_rate_gps, slow_avg, LEARN_ALPHA_RATE, obs);
        // Persist the live-refined (steady-state) plant gain, not a noisy mean.
        if (rt->gate_gain_count > 0) entry->gate_gain_gps_per_pct = clampf_local(
            learn_ewma(entry->gate_gain_gps_per_pct, rt->learned_gate_gain_gps_per_pct, LEARN_ALPHA_GATE_GAIN, obs),
            CASC_GATE_GAIN_MIN, CASC_GATE_GAIN_MAX);
        // finish_trim is signed: plain EWMA (no warmup, no prev<=0 shortcut).
        float trim_sample = clampf_local(-fill_error_g, FINISH_TRIM_MIN_G, FINISH_TRIM_MAX_G);
        entry->finish_trim_g += LEARN_ALPHA_FINISH_TRIM * (trim_sample - entry->finish_trim_g);
        entry->finish_trim_g = clampf_local(entry->finish_trim_g, FINISH_TRIM_MIN_G, FINISH_TRIM_MAX_G);
        entry->drip_wait_ms = ewma(entry->drip_wait_ms,
                                   clampf_local(settled_wait_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS),
                                   LEARN_ALPHA_DRIP_WAIT);
        entry->successful_fills++;
        ESP_LOGI(TAG,
                 "learned reason=%s fills=%lu dead %.3f->%.3f post %.1f->%.1f fast %.1f->%.1f slow %.1f->%.1f K %.3f->%.3f trim %.1f->%.1f drip %.0f->%.0f",
                 reason ? reason : "ok", (unsigned long)obs,
                 (double)before.dead_time_s, (double)entry->dead_time_s,
                 (double)before.post_close_gain_g, (double)entry->post_close_gain_g,
                 (double)before.fast_rate_gps, (double)entry->fast_rate_gps,
                 (double)before.slow_rate_gps, (double)entry->slow_rate_gps,
                 (double)before.gate_gain_gps_per_pct, (double)entry->gate_gain_gps_per_pct,
                 (double)before.finish_trim_g, (double)entry->finish_trim_g,
                 (double)before.drip_wait_ms, (double)entry->drip_wait_ms);
    } else {
        ESP_LOGI(TAG, "learning skipped reason=%s success=%d dead=%.3f post=%.1f fast=%.1f",
                 reason ? reason : "fault", success ? 1 : 0,
                 (double)rt->measured_dead_time_s, (double)rt->measured_post_close_gain_g, (double)fast_avg);
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
        .used_close_early_g = rt->adapted_close_early_g,
        .next_dead_time_s = entry->dead_time_s,
        .next_post_close_gain_g = entry->post_close_gain_g,
        .next_fast_rate_gps = entry->fast_rate_gps,
        .next_slow_rate_gps = entry->slow_rate_gps,
        .next_close_early_g = rt->adapted_close_early_g,
        .next_drip_wait_ms = entry->drip_wait_ms,
    };
    snprintf(summary.reason, sizeof(summary.reason), "%s", reason ? reason : (success ? "ok" : "fault"));
    env->publish_fill_summary(tick->run_id, tick->slot_idx, tick->preset_name, tick->strategy_name, tick->params, &summary);
}

static void cascade_on_enter(filler_strategy_runtime_t *rt, filler_state_t state, filler_state_t prev_state,
                             const filler_strategy_env_t *env, const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick || !tick->params) return;

    switch (state) {
    case FILLER_FILL: {
        cascade_learned_entry_t *entry = cascade_entry_for_preset(tick->preset_index, tick->params);
        if (prev_state != FILLER_VERIFY_TARGET) {
            memset(rt, 0, sizeof(*rt));
            cascade_model_reset();
            s_integ_pct = 0.0f;
            rt->fill_open_ts_us = tick->now_us;
            rt->active_preset_index = tick->preset_index;
            rt->target_g = (float)tick->params->target_grams;
            rt->learned_dead_time_s = entry->dead_time_s;
            rt->learned_post_close_gain_g = entry->post_close_gain_g;
            rt->learned_fast_rate_gps = entry->fast_rate_gps;
            rt->learned_slow_rate_gps = entry->slow_rate_gps;
            rt->learned_gate_gain_gps_per_pct = entry->gate_gain_gps_per_pct;
            rt->learned_finish_trim_g = entry->finish_trim_g;
            if (!env->jar_tare_get || !env->jar_tare_get(&rt->run_base_weight_g)) {
                rt->run_base_weight_g = tick->latest ? tick->latest->grams : 0.0f;
            }
            rt->adapted_drip_wait_ms = entry->drip_wait_ms;
            rt->gate_ceiling_pct = gate_ceiling_for_entry(entry, tick->params);
            // Start at the feedforward gate for the initial fast setpoint.
            float k = clampf_local(entry->gate_gain_gps_per_pct, CASC_GATE_GAIN_MIN, CASC_GATE_GAIN_MAX);
            rt->control_gate_cmd_pct = clampf_local(entry->fast_rate_gps / k, 0.0f, rt->gate_ceiling_pct);
            log_learned_entry("starting cascade fill with", tick->preset_index, entry);
            ESP_LOGI(TAG, "gate ceiling=%.1f%% (preset max=%u%%, successful_fills=%lu)",
                     (double)rt->gate_ceiling_pct, (unsigned)tick->params->max_gate_pct,
                     (unsigned long)entry->successful_fills);
        } else {
            rt->refill_count++;
            rt->first_close_seen = false;
            rt->first_close_ts_us = 0;
            rt->last_gain_ts_us = 0;
            rt->rel_at_first_close_g = 0.0f;
            rt->measured_post_close_gain_g = 0.0f;
            rt->last_post_close_gain_g = 0.0f;
            s_integ_pct = 0.0f;
            cascade_model_reset();
            rt->control_gate_cmd_pct = clampf_local(rt->control_gate_cmd_pct, 2.0f, rt->gate_ceiling_pct);
            ESP_LOGI(TAG, "refill attempt=%u relaxed_close_early=%.1f g reopen_gate=%.1f%%",
                     (unsigned)rt->refill_count, (double)rt->close_early_relax_g, (double)rt->control_gate_cmd_pct);
        }
        rt->last_rate_ts_us = 0;
        rt->last_rel_g = 0.0f;
        rt->control_last_update_us = 0;
        clear_live_rate_estimates(rt);
        cascade_reset_counters(rt);
        update_thresholds(rt, tick);
        update_control_targets(rt, tick, (float)tick->params->target_grams);
        ESP_LOGI(TAG,
                 "cascade start close_early=%.1f g predicted_remaining=%.1f g K=%.3f g/s/%% dead=%.2f s fast=%.1f g/s slow=%.1f g/s start_gate=%.1f%%",
                 (double)rt->adapted_close_early_g, (double)rt->predicted_remaining_g,
                 (double)rt->learned_gate_gain_gps_per_pct, (double)rt->learned_dead_time_s,
                 (double)rt->learned_fast_rate_gps, (double)rt->learned_slow_rate_gps,
                 (double)rt->control_gate_cmd_pct);
        env->publish_fill_start(tick->run_id, tick->slot_idx, tick->strategy_name, tick->params,
                                tick->latest ? tick->latest->grams : 0.0f);
        cascade_command_gate(rt, env, rt->control_gate_cmd_pct,
                             (prev_state == FILLER_VERIFY_TARGET) ? "cascade_refill_open" : "cascade_open",
                             tick->now_us, true);
        publish_runtime_snapshot(rt, env, tick);
        break;
    }
    case FILLER_DRIP_WAIT:
        cascade_reset_counters(rt);
        mark_first_close(rt, tick);
        env->gate_close_label("drip_wait");
        publish_runtime_snapshot(rt, env, tick);
        break;
    case FILLER_VERIFY_TARGET:
        cascade_reset_counters(rt);
        env->gate_close_label("close");
        publish_runtime_snapshot(rt, env, tick);
        break;
    default:
        break;
    }
}

static filler_state_t cascade_step(filler_strategy_runtime_t *rt, filler_state_t state,
                                   const filler_strategy_env_t *env, const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick || !tick->latest || !tick->params) return state;
    if (tick->new_sample && rt->sample_count < 255) rt->sample_count++;

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
        float remaining_g = (float)tick->params->target_grams - rel_g;

        float hard_margin = fmaxf((float)tick->params->target_tol_high_g + HARD_OVERFILL_MARGIN_EXTRA_G, HARD_OVERFILL_MARGIN_MIN_G);
        if (rel_g > (float)tick->params->target_grams + hard_margin) {
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
                                                 NO_RESPONSE_MIN_S, NO_RESPONSE_MAX_S);
        if (!rt->response_detected && ((tick->now_us - rt->fill_open_ts_us) / 1000000.0f) > no_response_limit_s) {
            ESP_LOGE(TAG, "no weight response after opening");
            env->gate_close_label("no_response");
            env->set_fault(FLT_EMPTY_HONEY);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "no_response", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }

        if ((tick->now_us - tick->state_enter_us) > ((int64_t)tick->params->fill_timeout_ms * 1000)) {
            ESP_LOGE(TAG, "fill timeout after %.1f s (rel=%.1f g) -> fault",
                     (double)((tick->now_us - tick->state_enter_us) / 1000000.0f), (double)rel_g);
            env->gate_close_label("fill_timeout");
            env->set_fault(FLT_SERVO_TIMEOUT);
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "fill_timeout", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }

        float safe_limit_gps = safe_rate_limit_gps(rt, tick->params);
        if (tick->new_sample) {
            if (rt->filtered_rate_gps > safe_limit_gps) { if (rt->cnt_safe_rate < 255) rt->cnt_safe_rate++; }
            else rt->cnt_safe_rate = 0;
        }
        if (rt->cnt_safe_rate >= RATE_SPIKE_CONFIRM_SAMPLES) {
            if (rt->safe_reduce_ts_us == 0) rt->safe_reduce_ts_us = tick->now_us;
            float elapsed_s = (float)(tick->now_us - rt->safe_reduce_ts_us) / 1000000.0f;
            if (elapsed_s > SAFE_REDUCE_ESCALATE_S) {
                ESP_LOGE(TAG, "flow runaway: rate=%.1f g/s > limit=%.1f g/s after %.1f s at gate %.1f%% -> fault (lower gate %%)",
                         (double)rt->filtered_rate_gps, (double)safe_limit_gps, (double)elapsed_s, (double)rt->control_gate_cmd_pct);
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
                float reduced = clampf_local(rt->control_gate_cmd_pct - reduce_step, 0.0f, 100.0f);
                ESP_LOGW(TAG, "safe-rate reduction rate=%.1f g/s limit=%.1f g/s gate %.1f->%.1f (step=%.1f%%, over=%.2fx)",
                         (double)rt->filtered_rate_gps, (double)safe_limit_gps, (double)rt->control_gate_cmd_pct,
                         (double)reduced, (double)reduce_step, (double)over_ratio);
                s_integ_pct = 0.0f;
                cascade_command_gate(rt, env, reduced, "safe_reduce", tick->now_us, true);
                rt->safe_reduce_last_action_us = tick->now_us;
            }
            publish_runtime_snapshot(rt, env, tick);
            return state;
        }
        rt->safe_reduce_ts_us = 0;
        rt->safe_reduce_last_action_us = 0;

        if (!tick->new_sample) {
            publish_runtime_snapshot(rt, env, tick);
            return state;
        }

        // Final close: predicted final mass reaches target.
        if (rel_g >= (float)tick->params->target_grams || remaining_g <= rt->adapted_close_early_g) {
            ESP_LOGI(TAG, "cascade close remaining=%.1f g threshold=%.1f g predicted=%.1f g",
                     (double)remaining_g, (double)rt->adapted_close_early_g, (double)rt->predicted_remaining_g);
            env->gate_close_label("cascade_close");
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick);
            return FILLER_DRIP_WAIT;
        }

        // Mark the slow-control phase once inside the deceleration band (for
        // phase-aware rate learning); the profile makes the transition smooth.
        if (!rt->near_close_logged && remaining_g <= fmaxf(CASC_DECEL_BAND_MIN_G, rt->learned_post_close_gain_g * CASC_DECEL_BAND_MULT)) {
            rt->near_close_logged = true;
            rt->gate_at_slow_entry_pct = rt->control_gate_cmd_pct;
        }

        cascade_run_controller(rt, env, tick, remaining_g);
        publish_runtime_snapshot(rt, env, tick);
        return state;
    }

    case FILLER_DRIP_WAIT:
        update_rate_estimates(rt, tick);
        track_post_close_gain(rt, tick);
        if ((tick->now_us - tick->state_enter_us) >= ((int64_t)rt->adapted_drip_wait_ms * 1000.0f)) {
            ESP_LOGI(TAG, "drip wait complete waited=%.0f ms post_close_gain=%.1f g",
                     (double)rt->adapted_drip_wait_ms, (double)rt->measured_post_close_gain_g);
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
        track_post_close_gain(rt, tick);
        float rel_g = tick->latest->grams - rt->run_base_weight_g;
        float tol_low_g = (float)tick->params->target_tol_low_g;
        float tol_high_g = (float)tick->params->target_tol_high_g;

        if (tick->new_sample && stable_above(rel_g, (float)tick->params->target_grams + tol_high_g, &rt->cnt_over, VERIFY_CONFIRM_SAMPLES)) {
            ESP_LOGE(TAG, "cascade overweight: rel=%.1f g", (double)rel_g);
            env->set_fault(FLT_WEIGHT_RANGE);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "overweight", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }
        if (tick->new_sample && stable_below(rel_g, (float)tick->params->target_grams - tol_low_g, &rt->cnt_under, VERIFY_CONFIRM_SAMPLES)) {
            float under = (float)tick->params->target_grams - rel_g;
            ESP_LOGI(TAG, "cascade underweight: rel=%.1f g (-%.1f g) -> refill", (double)rel_g, (double)under);
            float relax = fmaxf(under * REFILL_CLOSE_RELAX_FACTOR, REFILL_RELAX_STEP_G);
            rt->close_early_relax_g += relax;
            ESP_LOGI(TAG, "relax close_early by %.1f g (deficit %.1f g) -> total_relax=%.1f g",
                     (double)relax, (double)under, (double)rt->close_early_relax_g);
            publish_runtime_snapshot(rt, env, tick);
            return FILLER_FILL;
        }
        if (rt->sample_count >= VERIFY_CONFIRM_SAMPLES) {
            uint8_t next = env->slot_next_idx(tick->slot_idx, tick->params->slots_total);
            ESP_LOGI(TAG, "cascade target verified rel=%.1f g target=%u g tol=[-%u,+%u]",
                     (double)rel_g, (unsigned)tick->params->target_grams,
                     (unsigned)tick->params->target_tol_low_g, (unsigned)tick->params->target_tol_high_g);
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

const filler_strategy_ops_t g_filler_strategy_flow_cascade_ops = {
    .name = "flow-cascade",
    .on_enter = cascade_on_enter,
    .step = cascade_step,
};
