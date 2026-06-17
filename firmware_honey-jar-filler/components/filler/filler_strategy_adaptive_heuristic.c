#include "filler_strategy.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "nvs.h"

#define ADAPT_NAMESPACE "fill_adapt"
#define ADAPT_KEY_STORE "learn_v1"
#define ADAPT_STORE_VERSION 1u

// Persistence / session behavior:
// 0 => start after each boot from preset-derived defaults and learn only within
//      the current runtime session
// 1 => restore learned values from NVS and persist updates across boots
#define ADAPT_ENABLE_PERSISTENT_LEARNING 0

// Filtering / learning aggressiveness:
// Use a slower rise and faster fall so the filtered rate follows real flow but
// does not stay artificially high once the measured increase stops.
#define RATE_FILTER_ALPHA_RISE 0.18f
#define RATE_FILTER_ALPHA_FALL 0.55f
#define RATE_MIN_VALID_GPS 2.0f
#define RATE_ZERO_EPS_GPS 0.5f
#define RATE_NO_FLOW_RESET_SAMPLES 2u
#define LEARN_ALPHA_DEAD_TIME 0.35f
#define LEARN_ALPHA_POST_CLOSE 0.65f
#define LEARN_ALPHA_RATE 0.20f
#define LEARN_ALPHA_DRIP_WAIT 0.35f

// VERIFY_TARGET robustness:
// Require a few consecutive confirming samples before deciding under/overweight.
#define VERIFY_CONFIRM_SAMPLES 4u

// Response / dead-time detection:
// Response threshold defines when the first real weight movement is accepted.
#define RESPONSE_THRESHOLD_MIN_G 1.5f
#define RESPONSE_THRESHOLD_MAX_G 6.0f
#define DEAD_TIME_MIN_S 0.10f
#define DEAD_TIME_MAX_S 10.00f
#define NO_RESPONSE_MIN_S 7.00f
#define NO_RESPONSE_MAX_S 12.00f
#define NO_RESPONSE_DEAD_TIME_MULT 4.0f
#define NO_RESPONSE_EXTRA_S 1.5f

// Adaptive threshold shaping:
// CLOSE_BUFFER_G keeps closing slightly conservative, REFILL_RELAX_STEP_G makes
// the next refill attempt less conservative after an underfill.
#define CLOSE_BUFFER_G 1.5f
#define REFILL_RELAX_STEP_G 2.5f
#define REFILL_MIN_OPEN_MS 1000.0f

// Safety limits:
// Hard overfill margin faults instead of trying to recover. Safe-rate limits
// reduce the gate opening when the observed fill rate looks implausibly high.
#define ADAPT_HARD_OVERFILL_MARGIN_MIN_G 1.0f
#define ADAPT_HARD_OVERFILL_MARGIN_EXTRA_G 6.0f
#define NEAR_CLOSE_TRANSITION_MARGIN_G 4.0f
#define SAFE_RATE_MIN_GPS 10.0f
#define SAFE_RATE_MAX_GPS 100.0f
#define SAFE_RATE_MULT 1.8f

// Sample-to-sample rate estimation window:
// Ignore very short/long deltas because they distort derivative estimates.
#define DELTA_RATE_MAX_DT_S 0.80f
#define DELTA_RATE_MIN_DT_S 0.05f

// Drip-wait adaptation:
// Keep learned wait times within conservative bounds and retain some settling
// margin after the last observed post-close mass increase.
#define DRIP_WAIT_MIN_MS 5000.0f
#define DRIP_WAIT_MAX_MS 60000.0f
#define DRIP_WAIT_SETTLE_MARGIN_MS 1200.0f

// Learned across successful runs for one preset. These values form the
// conservative starting point for the next fill of the same preset.
typedef struct {
    uint8_t initialized;
    uint32_t successful_fills;
    float dead_time_s;
    float post_close_gain_g;
    float fast_rate_gps;
    float slow_rate_gps;
    float drip_wait_ms;
} adaptive_learned_entry_t;

// One learned entry is stored for each preset so different honey presets do
// not poison each other's adaptive estimates.
typedef struct {
    uint16_t version;
    uint8_t preset_count;
    adaptive_learned_entry_t presets[APP_PRESET_COUNT];
} adaptive_store_t;

typedef enum {
    PHASE_FAST = 0,
    PHASE_REDUCED,
    PHASE_CLOSED,
    PHASE_REFILL,
} adaptive_gate_phase_t;

static const char *TAG = "fill_adaptive";
static adaptive_store_t s_store;
static bool s_store_loaded;

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

static const char *phase_name(adaptive_gate_phase_t phase)
{
    switch (phase) {
    case PHASE_FAST:    return "fast";
    case PHASE_REDUCED: return "reduced";
    case PHASE_CLOSED:  return "closed";
    case PHASE_REFILL:  return "refill";
    default:            return "?";
    }
}

static adaptive_gate_phase_t runtime_phase(const filler_strategy_runtime_t *rt)
{
    if (!rt) return PHASE_CLOSED;
    if (rt->first_close_seen) return PHASE_CLOSED;
    if (rt->refill_count > 0 && !rt->near_close_logged) return PHASE_REFILL;
    if (rt->near_close_logged) return PHASE_REDUCED;
    return PHASE_FAST;
}

static void reset_state_counters(filler_strategy_runtime_t *rt)
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

static void adaptive_defaults_from_params(adaptive_learned_entry_t *entry, const app_params_t *params)
{
    if (!entry || !params) return;
    float poll_s = ((float)CONFIG_HX711_POLL_INTERVAL_MS / 1000.0f) * 2.5f;
    entry->initialized = 1;
    entry->dead_time_s = clampf_local(poll_s, 0.25f, 1.20f);
    entry->post_close_gain_g = clampf_local((float)params->close_early_g * 0.55f, 1.0f, 120.0f);
    entry->fast_rate_gps = 0.0f;
    entry->slow_rate_gps = 0.0f;
    entry->drip_wait_ms = clampf_local((float)params->drip_delay_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS);
}

static void log_learned_entry(const char *prefix, uint8_t preset_index, const adaptive_learned_entry_t *entry)
{
    if (!prefix || !entry) return;
    ESP_LOGI(TAG,
             "%s preset=%u dead_time=%.3f s post_close=%.1f g fast_rate=%.1f g/s slow_rate=%.1f g/s drip_wait=%.0f ms fills=%lu",
             prefix,
             (unsigned)preset_index,
             (double)entry->dead_time_s,
             (double)entry->post_close_gain_g,
             (double)entry->fast_rate_gps,
             (double)entry->slow_rate_gps,
             (double)entry->drip_wait_ms,
             (unsigned long)entry->successful_fills);
}

static void adaptive_store_apply_defaults(adaptive_store_t *store)
{
    if (!store) return;
    *store = (adaptive_store_t){
        .version = ADAPT_STORE_VERSION,
        .preset_count = APP_PRESET_COUNT,
    };
}

static bool adaptive_store_valid(const adaptive_store_t *store)
{
    return store &&
           store->version == ADAPT_STORE_VERSION &&
           store->preset_count == APP_PRESET_COUNT;
}

static void adaptive_store_load_once(void)
{
    if (s_store_loaded) return;

    adaptive_store_apply_defaults(&s_store);
    if (!ADAPT_ENABLE_PERSISTENT_LEARNING) {
        ESP_LOGI(TAG, "adaptive learning persistence disabled, starting from preset defaults");
        s_store_loaded = true;
        return;
    }

    nvs_handle_t h;
    if (nvs_open(ADAPT_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(s_store);
        adaptive_store_t tmp = {0};
        if (nvs_get_blob(h, ADAPT_KEY_STORE, &tmp, &len) == ESP_OK &&
            len == sizeof(tmp) &&
            adaptive_store_valid(&tmp)) {
            s_store = tmp;
            ESP_LOGI(TAG, "loaded adaptive learning store from NVS");
        }
        nvs_close(h);
    }
    s_store_loaded = true;
}

static void adaptive_store_save(void)
{
    if (!ADAPT_ENABLE_PERSISTENT_LEARNING) {
        return;
    }

    nvs_handle_t h;
    if (nvs_open(ADAPT_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "failed to open adaptive NVS store");
        return;
    }
    esp_err_t err = nvs_set_blob(h, ADAPT_KEY_STORE, &s_store, sizeof(s_store));
    if (err == ESP_OK) err = nvs_commit(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to save adaptive store: %s", esp_err_to_name(err));
    }
    nvs_close(h);
}

static adaptive_learned_entry_t *adaptive_entry_for_preset(uint8_t preset_index, const app_params_t *params)
{
    adaptive_store_load_once();
    if (preset_index >= APP_PRESET_COUNT) preset_index = 0;
    adaptive_learned_entry_t *entry = &s_store.presets[preset_index];
    if (!entry->initialized) {
        adaptive_defaults_from_params(entry, params);
        log_learned_entry("initialized adaptive defaults for", preset_index, entry);
    }
    return entry;
}

static float response_threshold_g(const filler_strategy_tick_t *tick)
{
    float candidate = tick && tick->params ? ((float)tick->params->target_grams * 0.005f) : 2.0f;
    return clampf_local(candidate, RESPONSE_THRESHOLD_MIN_G, RESPONSE_THRESHOLD_MAX_G);
}

static float transition_fast_rate_gps(const filler_strategy_runtime_t *rt)
{
    if (!rt) return 0.0f;
    adaptive_gate_phase_t phase = runtime_phase(rt);
    if ((phase == PHASE_FAST || phase == PHASE_REFILL) && rt->filtered_rate_gps > 0.5f) {
        return rt->filtered_rate_gps;
    }
    if (rt->learned_fast_rate_gps > 0.5f) return rt->learned_fast_rate_gps;
    if (rt->filtered_rate_gps > 0.5f) return rt->filtered_rate_gps;
    if (rt->raw_rate_gps > 0.5f) return rt->raw_rate_gps;
    return 0.0f;
}

static void update_adapted_thresholds(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!rt || !tick || !tick->params) return;

    float fast_transition_rate_gps = transition_fast_rate_gps(rt);
    float transition_mass_g = rt->learned_dead_time_s * fast_transition_rate_gps;

    // Close timing is dominated by the empirically measured residual mass after
    // a full-close command. This measured post-close gain already captures the
    // real plant behavior better than reconstructing it mainly from rate and
    // dead time again.
    rt->predicted_remaining_g = rt->learned_post_close_gain_g;

    float close_max = fmaxf((float)tick->params->close_early_g * 1.8f, 8.0f);
    float near_max = fmaxf((float)tick->params->near_close_delta_g * 2.0f, 20.0f);
    float close_candidate = rt->predicted_remaining_g + CLOSE_BUFFER_G - rt->close_early_relax_g;
    // Reducing from fast to slow should happen early enough that the mass
    // already "in flight" at the fast rate can clear before the slower final
    // phase is relied on. Therefore near-close is modeled as fast dead-time
    // mass plus the later full-close threshold.
    float near_candidate = transition_mass_g + clampf_local(close_candidate, 2.0f, close_max) + NEAR_CLOSE_TRANSITION_MARGIN_G;

    rt->adapted_close_early_g = clampf_local(close_candidate, 2.0f, close_max);
    rt->adapted_near_close_g = clampf_local(near_candidate,
                                            rt->adapted_close_early_g + 3.0f,
                                            near_max);
    rt->adapted_drip_wait_ms = clampf_local(rt->adapted_drip_wait_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS);
}

static float near_close_transition_mass_g(const filler_strategy_runtime_t *rt)
{
    if (!rt) return 0.0f;
    return rt->learned_dead_time_s * transition_fast_rate_gps(rt);
}

static float adaptive_no_response_limit_s(const filler_strategy_runtime_t *rt)
{
    float learned_dead_time_s = rt ? rt->learned_dead_time_s : 0.0f;
    return clampf_local(fmaxf(learned_dead_time_s * NO_RESPONSE_DEAD_TIME_MULT,
                              learned_dead_time_s + NO_RESPONSE_EXTRA_S),
                        NO_RESPONSE_MIN_S,
                        NO_RESPONSE_MAX_S);
}

static void publish_runtime_snapshot(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env, const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !env->set_sample_telemetry || !tick) return;

    filler_strategy_sample_telemetry_t sample = {
        .valid = true,
        .raw_rate_gps = rt->raw_rate_gps,
        .filtered_rate_gps = rt->filtered_rate_gps,
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

            adaptive_gate_phase_t phase = runtime_phase(rt);
            if ((phase == PHASE_FAST || phase == PHASE_REFILL) && rt->raw_rate_gps > 0.1f) {
                rt->fast_rate_sum_gps += rt->raw_rate_gps;
                rt->fast_rate_count++;
            } else if (phase == PHASE_REDUCED && rt->raw_rate_gps > 0.1f) {
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
}

static float safe_rate_limit_gps(const filler_strategy_runtime_t *rt)
{
    float reference = rt ? rt->learned_fast_rate_gps : 0.0f;
    if (reference < 1.0f && rt) reference = rt->filtered_rate_gps;
    if (reference < 1.0f) reference = SAFE_RATE_MIN_GPS;
    return clampf_local(reference * SAFE_RATE_MULT, SAFE_RATE_MIN_GPS, SAFE_RATE_MAX_GPS);
}

static bool plausible_positive(float value, float min_v, float max_v)
{
    return value >= min_v && value <= max_v;
}

static void publish_summary_and_learn(filler_strategy_runtime_t *rt,
                                      const filler_strategy_env_t *env,
                                      const filler_strategy_tick_t *tick,
                                      const char *reason,
                                      bool success)
{
    if (!rt || !env || !env->publish_fill_summary || !tick || !tick->latest || !tick->params) return;

    // Learned across runs: dead time, post-close gain, representative fast/slow
    // fill rates, and a conservative drip-wait recommendation. Per-run
    // measurements stay in the runtime struct and are only committed when the
    // finished fill is plausible and fault-free.
    adaptive_learned_entry_t *entry = adaptive_entry_for_preset(tick->preset_index, tick->params);
    float final_rel_g = tick->latest->grams - rt->run_base_weight_g;
    float fast_avg = (rt->fast_rate_count > 0) ? (rt->fast_rate_sum_gps / (float)rt->fast_rate_count) : 0.0f;
    float slow_avg = (rt->slow_rate_count > 0) ? (rt->slow_rate_sum_gps / (float)rt->slow_rate_count) : 0.0f;
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
                     (fast_avg == 0.0f || plausible_positive(fast_avg, 0.5f, 400.0f)) &&
                     (slow_avg == 0.0f || plausible_positive(slow_avg, 0.2f, 250.0f));

    adaptive_learned_entry_t before = *entry;

    if (plausible) {
        entry->dead_time_s = ewma(entry->dead_time_s, rt->measured_dead_time_s, LEARN_ALPHA_DEAD_TIME);
        entry->post_close_gain_g = ewma(entry->post_close_gain_g, rt->measured_post_close_gain_g, LEARN_ALPHA_POST_CLOSE);
        if (fast_avg > 0.0f) entry->fast_rate_gps = ewma(entry->fast_rate_gps, fast_avg, LEARN_ALPHA_RATE);
        if (slow_avg > 0.0f) entry->slow_rate_gps = ewma(entry->slow_rate_gps, slow_avg, LEARN_ALPHA_RATE);
        entry->drip_wait_ms = ewma(entry->drip_wait_ms,
                                   clampf_local(settled_wait_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS),
                                   LEARN_ALPHA_DRIP_WAIT);
        entry->successful_fills++;
        adaptive_store_save();
        ESP_LOGI(TAG,
                 "learned update reason=%s dead_time %.3f->%.3f s (a=%.2f) post_close %.1f->%.1f g (a=%.2f) fast_rate %.1f->%.1f g/s (a=%.2f) slow_rate %.1f->%.1f g/s (a=%.2f) drip_wait %.0f->%.0f ms (a=%.2f)",
                 reason ? reason : "ok",
                 (double)before.dead_time_s, (double)entry->dead_time_s,
                 (double)LEARN_ALPHA_DEAD_TIME,
                 (double)before.post_close_gain_g, (double)entry->post_close_gain_g,
                 (double)LEARN_ALPHA_POST_CLOSE,
                 (double)before.fast_rate_gps, (double)entry->fast_rate_gps,
                 (double)LEARN_ALPHA_RATE,
                 (double)before.slow_rate_gps, (double)entry->slow_rate_gps,
                 (double)LEARN_ALPHA_RATE,
                 (double)before.drip_wait_ms, (double)entry->drip_wait_ms,
                 (double)LEARN_ALPHA_DRIP_WAIT);
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
        .fill_error_g = final_rel_g - (float)tick->params->target_grams,
        .measured_dead_time_s = rt->measured_dead_time_s,
        .measured_post_close_gain_g = rt->measured_post_close_gain_g,
        .measured_fast_rate_gps = fast_avg,
        .measured_slow_rate_gps = slow_avg,
        .rate_at_close_gps = rt->rate_at_close_gps,
        .fill_duration_s = fill_duration_s,
        .drip_wait_used_ms = rt->adapted_drip_wait_ms,
        .refill_count = rt->refill_count,
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

static void adaptive_on_enter(filler_strategy_runtime_t *rt,
                              filler_state_t state,
                              filler_state_t prev_state,
                              const filler_strategy_env_t *env,
                              const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick || !tick->params) return;

    switch (state) {
    case FILLER_FILL: {
        adaptive_learned_entry_t *entry = adaptive_entry_for_preset(tick->preset_index, tick->params);
        if (prev_state != FILLER_VERIFY_TARGET) {
            // Measured only within the current fill: response timing, live rate
            // estimates, post-close gain, and refill count for this jar.
            memset(rt, 0, sizeof(*rt));
            rt->fill_open_ts_us = tick->now_us;
            rt->run_base_weight_g = tick->latest ? tick->latest->grams : 0.0f;
            rt->active_preset_index = tick->preset_index;
            rt->target_g = (float)tick->params->target_grams;
            rt->measured_dead_time_s = 0.0f;
            rt->adapted_drip_wait_ms = entry->drip_wait_ms;
            log_learned_entry("starting adaptive fill with", tick->preset_index, entry);
        } else {
            rt->refill_count++;
            rt->fill_open_ts_us = tick->now_us;
            rt->first_close_seen = false;
            rt->rel_at_first_close_g = 0.0f;
            rt->measured_post_close_gain_g = 0.0f;
            rt->last_post_close_gain_g = 0.0f;
            rt->first_close_ts_us = 0;
            rt->last_gain_ts_us = 0;
            ESP_LOGI(TAG,
                     "refill attempt=%u relaxed_close_early=%.1f g",
                     (unsigned)rt->refill_count,
                     (double)rt->close_early_relax_g);
        }
        rt->fill_eval_after_ts_us = (tick->latest && tick->latest->ts_us != 0) ? tick->latest->ts_us : tick->now_us;

        rt->last_rate_ts_us = 0;
        rt->last_rel_g = 0.0f;
        clear_live_rate_estimates(rt);
        reset_state_counters(rt);
        rt->learned_dead_time_s = entry->dead_time_s;
        rt->learned_post_close_gain_g = entry->post_close_gain_g;
        rt->learned_fast_rate_gps = entry->fast_rate_gps;
        rt->learned_slow_rate_gps = entry->slow_rate_gps;
        update_adapted_thresholds(rt, tick);
        ESP_LOGI(TAG,
                 "adaptive thresholds near_close=%.1f g close_early=%.1f g close_post=%.1f g near_transition=%.1f g drip_wait=%.0f ms gate_fast=%u%% gate_reduced=%u%% no_response_limit=%.2f s response_threshold=%.1f g",
                 (double)rt->adapted_near_close_g,
                 (double)rt->adapted_close_early_g,
                 (double)rt->predicted_remaining_g,
                 (double)near_close_transition_mass_g(rt),
                 (double)rt->adapted_drip_wait_ms,
                 (unsigned)tick->params->max_gate_pct,
                 (unsigned)tick->params->near_close_gate_pct,
                 (double)adaptive_no_response_limit_s(rt),
                 (double)response_threshold_g(tick));
        env->publish_fill_start(tick->run_id,
                                tick->slot_idx,
                                tick->strategy_name,
                                tick->params,
                                tick->latest ? tick->latest->grams : 0.0f);
        env->gate_set_percent_label(tick->params->max_gate_pct, (prev_state == FILLER_VERIFY_TARGET) ? "refill_open" : "max_gate");
        publish_runtime_snapshot(rt, env, tick);
        break;
    }
    case FILLER_DRIP_WAIT:
        reset_state_counters(rt);
        mark_first_close(rt, tick);
        env->gate_close_label("drip_wait");
        publish_runtime_snapshot(rt, env, tick);
        break;
    case FILLER_VERIFY_TARGET:
        reset_state_counters(rt);
        env->gate_close_label("close");
        publish_runtime_snapshot(rt, env, tick);
        break;
    default:
        break;
    }
}

static filler_state_t adaptive_step(filler_strategy_runtime_t *rt,
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
        bool sample_after_open = tick->new_sample && tick->latest->ts_us > rt->fill_eval_after_ts_us;
        float open_elapsed_ms = (float)(tick->now_us - rt->fill_open_ts_us) / 1000.0f;
        if (sample_after_open) {
            update_rate_estimates(rt, tick);
        }
        update_adapted_thresholds(rt, tick);

        // Hard stop against obvious mess scenarios: if the process is already
        // far above target, close immediately and fault instead of trying to
        // recover adaptively.
        float hard_margin_g = fmaxf((float)tick->params->target_tol_high_g + ADAPT_HARD_OVERFILL_MARGIN_EXTRA_G,
                                    ADAPT_HARD_OVERFILL_MARGIN_MIN_G);
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

        float no_response_limit_s = adaptive_no_response_limit_s(rt);
        if (!rt->response_detected && ((tick->now_us - rt->fill_open_ts_us) / 1000000.0f) > no_response_limit_s) {
            ESP_LOGE(TAG,
                     "no weight response after opening elapsed=%.2f s limit=%.2f s rel=%.1f g threshold=%.1f g learned_dead_time=%.2f s",
                     (double)((tick->now_us - rt->fill_open_ts_us) / 1000000.0f),
                     (double)no_response_limit_s,
                     (double)rel_g,
                     (double)response_threshold_g(tick),
                     (double)rt->learned_dead_time_s);
            env->gate_close_label("no_response");
            env->set_fault(FLT_EMPTY_HONEY);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "no_response", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }

        float safe_limit_gps = safe_rate_limit_gps(rt);
        // If the estimated rate rises well above the learned/plausible range,
        // reduce to the preset's reduced gate opening before the target region
        // to stay conservative.
        if (rt->filtered_rate_gps > safe_limit_gps) {
            if (!rt->near_close_logged) {
                ESP_LOGI(TAG,
                         "safe-rate reduction filtered_rate=%.1f g/s limit=%.1f g/s -> gate=%u%%",
                         (double)rt->filtered_rate_gps,
                         (double)safe_limit_gps,
                         (unsigned)tick->params->near_close_gate_pct);
            }
            env->gate_set_percent_label(tick->params->near_close_gate_pct, "safe_reduce");
            rt->near_close_logged = true;
            publish_runtime_snapshot(rt, env, tick);
            return state;
        }

        if (!sample_after_open) {
            publish_runtime_snapshot(rt, env, tick);
            return state;
        }

        if (rt->refill_count > 0 && open_elapsed_ms < REFILL_MIN_OPEN_MS) {
            env->gate_set_percent_label(tick->params->max_gate_pct, "refill_hold_open");
            publish_runtime_snapshot(rt, env, tick);
            return state;
        }

        float remaining_g = (float)tick->params->target_grams - rel_g;
        if (rel_g >= (float)tick->params->target_grams) {
            ESP_LOGI(TAG, "target reached directly rel=%.1f g -> close", (double)rel_g);
            env->gate_close_label("target");
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick);
            return FILLER_DRIP_WAIT;
        }
        if (remaining_g <= rt->adapted_close_early_g) {
            ESP_LOGI(TAG,
                     "adaptive close remaining=%.1f g threshold=%.1f g learned_post_close=%.1f g",
                     (double)remaining_g,
                     (double)rt->adapted_close_early_g,
                     (double)rt->predicted_remaining_g);
            env->gate_close_label("adaptive_close");
            mark_first_close(rt, tick);
            publish_runtime_snapshot(rt, env, tick);
            return FILLER_DRIP_WAIT;
        }
        if (remaining_g <= rt->adapted_near_close_g) {
            if (!rt->near_close_logged) {
                ESP_LOGI(TAG,
                         "adaptive near-close remaining=%.1f g threshold=%.1f g transition_mass=%.1f g close_threshold=%.1f g -> gate=%u%%",
                         (double)remaining_g,
                         (double)rt->adapted_near_close_g,
                         (double)near_close_transition_mass_g(rt),
                         (double)rt->adapted_close_early_g,
                         (unsigned)tick->params->near_close_gate_pct);
            }
            env->gate_set_percent_label(tick->params->near_close_gate_pct, "adaptive_near");
            rt->near_close_logged = true;
            publish_runtime_snapshot(rt, env, tick);
            return state;
        }

        env->gate_set_percent_label(tick->params->max_gate_pct,
                                    (rt->refill_count > 0) ? "refill_open" : "max_gate");
        publish_runtime_snapshot(rt, env, tick);
        return state;
    }

    case FILLER_DRIP_WAIT:
        update_rate_estimates(rt, tick);
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
        track_post_close_gain(rt, tick);
        float rel_g = tick->latest->grams - rt->run_base_weight_g;
        float tol_low_g = (float)tick->params->target_tol_low_g;
        float tol_high_g = (float)tick->params->target_tol_high_g;

        if (tick->new_sample &&
            stable_above(rel_g,
                         (float)tick->params->target_grams + tol_high_g,
                         &rt->cnt_over,
                         VERIFY_CONFIRM_SAMPLES)) {
            ESP_LOGE(TAG, "adaptive overweight: rel=%.1f g", (double)rel_g);
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
            ESP_LOGI(TAG, "adaptive underweight: rel=%.1f g (-%.1f g) -> refill",
                     (double)rel_g, (double)under);
            rt->close_early_relax_g += REFILL_RELAX_STEP_G;
            ESP_LOGI(TAG, "relax close_early by %.1f g -> total_relax=%.1f g",
                     (double)REFILL_RELAX_STEP_G,
                     (double)rt->close_early_relax_g);
            publish_runtime_snapshot(rt, env, tick);
            return FILLER_FILL;
        }

        if (rt->sample_count >= VERIFY_CONFIRM_SAMPLES) {
            ESP_LOGI(TAG,
                     "verify target confirmed rel=%.1f g target=%u g tol=[-%u,+%u] confirm=%u",
                     (double)rel_g,
                     (unsigned)tick->params->target_grams,
                     (unsigned)tick->params->target_tol_low_g,
                     (unsigned)tick->params->target_tol_high_g,
                     (unsigned)VERIFY_CONFIRM_SAMPLES);
            publish_runtime_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "ok", true);
            env->clear_sample_telemetry();
            uint8_t next = env->slot_next_idx(tick->slot_idx, tick->params->slots_total);
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

const filler_strategy_ops_t g_filler_strategy_adaptive_heuristic_ops = {
    .name = "adaptive-heuristic",
    .on_enter = adaptive_on_enter,
    .step = adaptive_step,
};
