// adaptive-gate: the adaptive heuristic plus learned gate openings.
//
// Same two-phase FSM as adaptive-heuristic (fast -> reduced -> close -> drip
// wait -> verify, refill on underweight) and the same between-fill learning of
// dead time, post-close gain and drip wait. What is new: the two gate openings
// are no longer fixed preset percentages but are LEARNED so that each phase
// runs at a target flow rate:
//
//   fast target   r_f* = target_g / auto_fill_time_s          (speed knob)
//   slow target   r_s* = AG_SLOW_RATE_PCT * min(r_f*, achieved fast rate)
//
// Deliberately NOT a continuous controller (that is flow-cascade). The gate is
// moved only on OBVIOUS evidence, so it does not hunt:
//
//   between fills  each phase's steady rate is measured (least-squares slope
//                  over a constant-gate window after the dead time). If it is
//                  outside a +-AG_DEADBAND band around the target, the learned
//                  gate moves part of the way (ratio rule on the open part
//                  gate-onset). Inside the band nothing changes. => a slowly
//                  draining bucket produces an occasional step, not a ramp.
//   within a fill  only gross deviations act, at most a few times per fill:
//                  flow > AG_INRUN_TOO_FAST x target (always), flow <
//                  target / AG_INRUN_TOO_SLOW while plenty of mass remains
//                  (first jar / honey change), and "nothing flows" (raise the
//                  gate before the no-response fault). The reduced phase is
//                  latched, so the gate can never toggle fast<->reduced.
//
// A regime change (bucket topped up, other honey) is detected when the first
// steady fast-phase rate of a fill is far off what the learned observation
// predicts at that gate, or the dead time jumps: the learning warmup is reset
// so the next measurements dominate instead of being averaged in slowly.
//
// Learned values are persisted to NVS, but only as a REFERENCE: after a boot
// the warmup counter restarts (first fills dominate again) and the first fill
// opens derated, because the bucket may have been refilled or the honey
// changed while the machine was off. All parent safeguards stay: hard
// overfill, no response, fill timeout, safe-rate cutback + runaway fault.

#include "filler_strategy.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "nvs.h"

#define AG_NAMESPACE "fill_agate"
#define AG_KEY_STORE "learn_v1"
#define AG_STORE_VERSION 1u
#define AG_ENABLE_PERSISTENT_LEARNING 1

// ---- gate model / learning ------------------------------------------------
// Flow-onset gate: the % the gaskets must clear before honey flows. Only used
// for the ratio rule (scale the OPEN part gate-onset). Measured 13 % (dark
// honey sweep), ~19 % thin dummy, ~7 % thick honey; an error here only makes a
// correction step a bit too small/large - the deadband loop still converges.
#define AG_ONSET_PCT 15.0f
#define AG_MIN_FLOW_MARGIN_PCT 3.0f     // never learn a gate below onset + this
#define AG_GATE_CAP_PCT 100.0f
// First-ever opening. Real honey ranges from ~66 g/s at 50 % (dark thin) to
// ~4 g/s at 50 % (thick, needs ~100 %). 50 % is the middle: thick honey flows
// at once and is opened further in-fill, thin honey is caught by the safe-rate
// cutback / in-fill too-fast rule within ~1 s (no spill on a 500 g jar).
#define AG_COLD_FAST_GATE_PCT 50.0f
#define AG_COLD_SLOW_GATE_PCT 24.0f     // slow gate before anything is known
// Reduced-phase rate as % of the fast rate. Sim sweep: 25-50 % all equivalent,
// 10 % crawls at the onset cliff (refills) -> fixed, not a menu parameter.
#define AG_SLOW_RATE_PCT 25.0f
#define AG_SLOW_MAX_OF_ACHIEVED 0.5f    // ... but at most half the achieved fast rate
// First-ever post-close (Nachlauf) guess, from the jar size instead of the
// legacy close_remaining_g (whose default 150 g caused ~3 refills/session).
// Measured Nachlauf: thin ~5 g @300 g, thick ~7-9 g @500 g.
#define AG_POST_CLOSE_SEED_FRAC 0.015f
#define AG_DEADBAND 0.15f               // +-15 % rate error => leave the gate alone
#define AG_LEARN_ALPHA_GATE 0.75f       // fraction of the needed correction per fill
// The bucket drains, so the rate drifts DOWN fill after fill. An upward
// correction therefore aims slightly above the target: the rate then starts at
// the top of the band and drifts through it (fewer corrections, mean ~ target).
#define AG_DRAIN_LEAD 0.08f
// The reduced opening sits just above the flow onset, where gate->flow is
// steepest and the onset assumption least accurate: learn it more cautiously.
#define AG_SLOW_DEADBAND 0.25f
#define AG_SLOW_LEARN_ALPHA 0.5f
#define AG_SLOW_STEP_UP_MAX_PCT 3.0f
// Going UP only costs time (close scaling covers accuracy), the no-flow cliff
// is only below: a clearly too slow reduced phase moves up at once, further.
#define AG_SLOW_CLEARLY_SLOW 1.5f
#define AG_SLOW_STEP_UP_CLEAR_PCT 10.0f
#define AG_SLOW_STEP_DOWN_MAX_PCT 1.5f  // going down risks the no-flow cliff
// "Barely anything flowed at gate g" is certain one-sided evidence that the
// onset is above g: the reduced opening never goes below that again.
#define AG_FLOOR_EVIDENCE_FRAC 0.4f     // slow obs below this x target => floor
#define AG_FLOOR_MARGIN_PCT 1.0f
// Overshoot damping (model-free): a correction that reverses the previous one
// means the step was too big for this honey (onset/curve differ from the
// assumption) -> halve the step gain; same direction again -> let it recover.
#define AG_FLIP_GAIN_MIN 0.25f
#define AG_FLIP_GAIN_RECOVER 1.5f
#define AG_RATIO_MIN 0.5f               // per-fill correction ratio bounds
#define AG_RATIO_MAX 2.0f
#define AG_STEP_UP_MAX_PCT 15.0f        // per-fill gate step limits (learning)
#define AG_STEP_DOWN_MAX_PCT 25.0f
#define AG_BOOT_DERATE 0.75f            // first fill after boot: open part x this
#define AG_BOOT_SLOW_DERATE 0.85f
#define AG_REGIME_HI 1.6f               // measured/expected rate => regime change
#define AG_REGIME_LO 0.6f
#define AG_REGIME_DEAD_TIME 1.6f        // dead-time ratio => regime change

// ---- steady-rate measurement ----------------------------------------------
#define AG_WINDOW_MIN_S 2.0f            // >= drip period: averaged, not per-drip
#define AG_SLIDE_WINDOW_S 2.0f          // sliding window for in-fill checks (sees steps)
#define AG_SLIDE_MIN_SPAN_S 1.5f
#define AG_RB_N 32u                     // ring buffer >= slide window at 10 Hz
#define AG_COARSE_WINDOW_S 1.0f         // short reduced phase: coarse obs, "too fast" only
#define AG_COARSE_TOO_FAST 1.6f
#define AG_SETTLE_AFTER_RESPONSE_S 1.5f // first-contact blob + flow lag
#define AG_SETTLE_AFTER_CHANGE_S 1.0f   // after (learned dead time) for gate moves

// ---- in-fill corrections (only gross deviations) ---------------------------
#define AG_INRUN_TOO_FAST 1.5f          // on the long (cumulative) window
#define AG_INRUN_TOO_FAST_SLIDE 2.0f    // on the 2 s sliding window (drips make it noisy)
#define AG_INRUN_TOO_SLOW 2.0f
#define AG_INRUN_UP_RATIO_MAX 2.5f
#define AG_INRUN_UP_STEP_MAX_PCT 30.0f
#define AG_INRUN_DOWN_RATIO_MIN 0.35f
#define AG_INRUN_MAX_CORRECTIONS 3u
#define AG_INRUN_UP_MIN_REMAINING_FRAC 0.40f
#define AG_NOFLOW_BUMP_PCT 15.0f        // nothing comes out -> open this much more
#define AG_NOFLOW_BUMP_AFTER_S 8.0f     // ... after this long without response
#define AG_NOFLOW_MAX_BUMPS 8u          // effectively: keep opening up to the cap
// "No flow" after a response (a blob, a bump against the scale): the steady
// rate is ~0. Never a state to stay in -> open further regardless of the
// correction budget / direction lock.
#define AG_ZERO_FLOW_GPS 0.5f
#define AG_ZERO_FLOW_TARGET_FRAC 0.10f
#define AG_SLOW_STALL_FRAC 0.4f         // reduced phase below this x target = crawling
#define AG_SLOW_STALL_BUMP_PCT 3.0f
#define AG_SLOW_STALL_MAX 2u
#define AG_SLOW_TOO_FAST 2.0f           // reduced phase above this x target = cut now

// ---- close threshold vs actual slow rate ------------------------------------
#define AG_CLOSE_SCALE_MIN 0.6f        // with a full steady window
#define AG_CLOSE_SCALE_MIN_SLIDE 0.75f  // with only the 2 s sliding window
#define AG_CLOSE_SCALE_MAX 2.0f
#define AG_CLOSE_SCALE_DEADBAND 0.20f

// ---- inherited from adaptive-heuristic ------------------------------------
#define RATE_FILTER_ALPHA_RISE 0.18f
#define RATE_FILTER_ALPHA_FALL 0.55f
#define RATE_FILTER_MEDIUM_ALPHA_RISE 0.10f
#define RATE_FILTER_MEDIUM_ALPHA_FALL 0.28f
#define RATE_FILTER_SLOW_ALPHA_RISE 0.05f
#define RATE_FILTER_SLOW_ALPHA_FALL 0.16f
#define RATE_CONTROL_MIN_VALID_GPS 2.0f
#define RATE_ZERO_EPS_GPS 0.15f
#define RATE_NO_FLOW_RESET_SAMPLES 2u
#define LEARN_ALPHA_DEAD_TIME 0.35f
#define LEARN_ALPHA_POST_CLOSE 0.65f
#define LEARN_ALPHA_RATE 0.20f
#define LEARN_ALPHA_DRIP_WAIT 0.35f
#define NEAR_CLOSE_CONTROL_MARGIN_MIN_G 4.0f
#define NEAR_CLOSE_CONTROL_MARGIN_MAX_G 40.0f
#define VERIFY_CONFIRM_SAMPLES 4u
#define RESPONSE_THRESHOLD_MIN_G 1.5f
#define RESPONSE_THRESHOLD_MAX_G 6.0f
#define DEAD_TIME_MIN_S 0.10f
#define DEAD_TIME_MAX_S 15.00f
// Measured from the LAST no-flow bump, i.e. normally with the gate at 100 %.
// Waiting with the gate open over an empty bucket cannot spill, while very
// thick/cold honey may need long to form a strand -> generous.
#define NO_RESPONSE_MIN_S 30.00f
#define NO_RESPONSE_MAX_S 45.00f
#define NO_RESPONSE_DEAD_TIME_MULT 4.0f
#define NO_RESPONSE_EXTRA_S 1.5f
#define CLOSE_BUFFER_G 1.5f
#define REFILL_RELAX_STEP_G 2.5f
#define REFILL_CLOSE_RELAX_FACTOR 0.90f
#define NEAR_CLOSE_REFILL_RELAX_STEP_G 3.0f
#define NEAR_CLOSE_REFILL_RELAX_MAX_G 12.0f
#define REFILL_MIN_OPEN_MS 1000.0f
#define ADAPT_HARD_OVERFILL_MARGIN_MIN_G 1.0f
#define ADAPT_HARD_OVERFILL_MARGIN_EXTRA_G 6.0f
#define SAFE_RATE_MIN_GPS 10.0f
#define SAFE_RATE_MAX_GPS 150.0f
#define SAFE_RATE_MULT 1.8f
#define ADAPT_MIN_CONTROLLED_FILL_S 4.0f
#define RATE_SPIKE_CONFIRM_SAMPLES 3u
#define SAFE_REDUCE_ESCALATE_S 5.0f
#define DELTA_RATE_MAX_DT_S 0.80f
#define DELTA_RATE_MIN_DT_S 0.05f
#define DRIP_WAIT_MIN_MS 5000.0f
#define DRIP_WAIT_MAX_MS 60000.0f
#define DRIP_WAIT_SETTLE_MARGIN_MS 1200.0f

enum { PH_FAST = 0, PH_SLOW = 1, PH_COUNT = 2 };

// Learned per preset; persisted (reference only, see header comment).
typedef struct {
    uint8_t initialized;
    uint8_t has_gates;                 // gates were learned at least once
    uint32_t successful_fills;         // warmup counter (reset at boot / regime change)
    uint32_t total_fills;
    float dead_time_s;
    float post_close_gain_g;
    float fast_rate_gps;               // avg raw rate in the fast phase (as parent)
    float slow_rate_gps;
    float drip_wait_ms;
    float gate_pct[PH_COUNT];          // learned opening per phase
    float obs_gate_pct[PH_COUNT];      // last steady observation per phase
    float obs_rate_gps[PH_COUNT];
    float close_rate_gps;              // steady slow rate the post-close gain belongs to
    float slow_pending_ratio;          // out-of-band slow observation waiting for confirmation
    float slow_floor_pct;              // learned: below this the reduced phase (nearly) stalls
    int8_t last_dir[PH_COUNT];         // direction of the last gate change (+1/-1)
    float step_gain[PH_COUNT];         // overshoot damping, 0 = unset (1)
} ag_entry_t;

typedef struct {
    uint16_t version;
    uint8_t preset_count;
    ag_entry_t presets[APP_PRESET_COUNT];
} ag_store_t;

// Per-fill state of the gate learning (one fill runs at a time).
typedef struct {
    bool first_after_boot;
    bool regime_change;
    bool expectation_checked;
    bool reduced_latched;
    float target_gps[PH_COUNT];
    float gate_cmd[PH_COUNT];          // current opening per phase (in-fill corrected)
    float gate_start[PH_COUNT];        // what the fill started with ("used_")
    uint8_t corrections;               // in-fill corrections so far
    uint8_t noflow_bumps;
    uint8_t slow_bumps;
    float open_rate;                   // first steady rate of the opening segment
    float open_gate_cmd;               // gate this fill opened at
    float open_gate;
    int64_t last_bump_us;
    // current constant-gate segment
    int phase;
    float seg_gate;
    int64_t seg_cmd_us;                // when this gate value was commanded
    bool seg_from_closed;              // segment started by opening from closed
    int64_t seg_steady_us;             // samples from here on are steady
    double s_n, s_t, s_w, s_tt, s_tw;
    float seg_rate;
    float seg_window_s;
    bool seg_valid;
    float seg_coarse_rate;             // same LS, valid from AG_COARSE_WINDOW_S
    bool seg_coarse_valid;
    float rb_t[AG_RB_N];               // steady samples of this segment (sliding window)
    float rb_w[AG_RB_N];
    uint8_t rb_head;
    uint8_t rb_count;
    float slide_rate;
    bool slide_valid;
    float close_rate_meas;             // sliding rate at the close command
    uint8_t slow_cuts;
    float floor_evidence;              // highest reduced gate this fill that (nearly) did not flow
    int8_t dir_lock[PH_COUNT];         // in-fill corrections never reverse within a fill:
                                       // -1 = already cut (no opening), +1 = already opened (no cut)
    // last valid segment per phase (learning input)
    bool obs_valid[PH_COUNT];
    float obs_gate[PH_COUNT];
    float obs_rate[PH_COUNT];
    float obs_window[PH_COUNT];
    bool obs_from_correction[PH_COUNT];
    // safe-rate event
    bool safe_event;
    float safe_gate;
    float safe_rate;
    float close_scale;
    float close_ref_rate;              // steady slow rate the learned post-close gain belongs to
} ag_fill_t;

static const char *TAG = "fill_agate";
static ag_store_t s_store;
static bool s_store_loaded;
static ag_fill_t s_fill;

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

static float learn_ewma(float prev, float sample, float base_alpha, uint32_t observations)
{
    float alpha = base_alpha;
    float warmup_alpha = 1.0f / (float)(observations + 1u);
    if (warmup_alpha > alpha) alpha = warmup_alpha;
    return prev + alpha * (sample - prev);
}

static float gate_min_pct(void) { return AG_ONSET_PCT + AG_MIN_FLOW_MARGIN_PCT; }

// Effective flow onset: the geometric assumption, raised (never lowered) by the
// learned no-flow floor - proven evidence that nothing flows below it.
static float s_onset_eff = AG_ONSET_PCT;
static float s_slow_floor_now;   // learned reduced-gate floor of the active preset

// Ratio rule on the open part of the gate: the flow is ~proportional to
// (gate - onset), so scaling that part by r*/r targets r*.
static float gate_for_ratio(float gate, float ratio)
{
    float open = fmaxf(gate - s_onset_eff, AG_MIN_FLOW_MARGIN_PCT);
    return s_onset_eff + open * ratio;
}

// An in-fill correction of the fast gate proves this honey flows differently
// than the learned reference (thinner/thicker batch, bucket topped up): the
// reduced opening of THIS fill moves by the same factor on the open part.
static void follow_slow_gate(float fast_before, float fast_after);

// ------------------------------------------------------------ store -------

static void store_defaults(ag_store_t *store)
{
    *store = (ag_store_t){ .version = AG_STORE_VERSION, .preset_count = APP_PRESET_COUNT };
}

static void store_load_once(void)
{
    if (s_store_loaded) return;
    store_defaults(&s_store);
    s_store_loaded = true;
    if (!AG_ENABLE_PERSISTENT_LEARNING) return;
    nvs_handle_t h;
    if (nvs_open(AG_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    ag_store_t tmp;
    size_t len = sizeof(tmp);
    if (nvs_get_blob(h, AG_KEY_STORE, &tmp, &len) == ESP_OK && len == sizeof(tmp) &&
        tmp.version == AG_STORE_VERSION && tmp.preset_count == APP_PRESET_COUNT) {
        s_store = tmp;
        // Reference only: learning starts fresh (warmup) after every boot, and
        // the first fill of each preset is derated (see on_enter).
        for (unsigned i = 0; i < APP_PRESET_COUNT; ++i) {
            ag_entry_t *e = &s_store.presets[i];
            e->successful_fills = 0;
            // correction history belongs to the old session; the floor is
            // geometric evidence and stays (a honey change resets it)
            e->slow_pending_ratio = 0.0f;
            for (int ph = 0; ph < PH_COUNT; ++ph) { e->last_dir[ph] = 0; e->step_gain[ph] = 1.0f; }
        }
        ESP_LOGI(TAG, "loaded learned reference from NVS (warmup restarts)");
    }
    nvs_close(h);
}

static void store_save(void)
{
    if (!AG_ENABLE_PERSISTENT_LEARNING) return;
    nvs_handle_t h;
    if (nvs_open(AG_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "failed to open NVS store");
        return;
    }
    esp_err_t err = nvs_set_blob(h, AG_KEY_STORE, &s_store, sizeof(s_store));
    if (err == ESP_OK) err = nvs_commit(h);
    if (err != ESP_OK) ESP_LOGW(TAG, "failed to save store: %s", esp_err_to_name(err));
    nvs_close(h);
}

static void entry_defaults(ag_entry_t *e, const app_params_t *p)
{
    float poll_s = ((float)CONFIG_HX711_POLL_INTERVAL_MS / 1000.0f) * 2.5f;
    memset(e, 0, sizeof(*e));
    e->initialized = 1;
    e->dead_time_s = clampf_local(poll_s, 0.25f, 1.20f);
    e->post_close_gain_g = clampf_local((float)p->target_grams * AG_POST_CLOSE_SEED_FRAC, 2.0f, 15.0f);
    e->drip_wait_ms = clampf_local((float)p->drip_delay_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS);
    e->gate_pct[PH_FAST] = AG_COLD_FAST_GATE_PCT;
    e->gate_pct[PH_SLOW] = clampf_local(fminf(AG_COLD_SLOW_GATE_PCT, e->gate_pct[PH_FAST]),
                                        gate_min_pct(), AG_GATE_CAP_PCT);
}

static ag_entry_t *entry_for(uint8_t preset, const app_params_t *p, bool *was_persisted)
{
    store_load_once();
    if (preset >= APP_PRESET_COUNT) preset = 0;
    ag_entry_t *e = &s_store.presets[preset];
    if (was_persisted) *was_persisted = e->initialized != 0;
    if (!e->initialized) {
        entry_defaults(e, p);
        ESP_LOGI(TAG, "preset=%u cold start: fast_gate=%.1f%% slow_gate=%.1f%%",
                 (unsigned)preset, (double)e->gate_pct[PH_FAST], (double)e->gate_pct[PH_SLOW]);
    }
    s_onset_eff = fmaxf(AG_ONSET_PCT, e->slow_floor_pct - AG_FLOOR_MARGIN_PCT);
    s_slow_floor_now = e->slow_floor_pct;
    return e;
}

// ---------------------------------------------------------- targets -------

static float fast_target_gps(const app_params_t *p)
{
    float t = (p->auto_fill_time_s > 0) ? (float)p->auto_fill_time_s : 25.0f;
    return (float)p->target_grams / t;
}

static float slow_target_gps(const app_params_t *p, const ag_entry_t *e)
{
    // Accuracy depends on the ABSOLUTE rate at close, so the reduced phase aims
    // at a fixed fraction of the target rate. A gate-saturated fast phase
    // (thick honey at 100 %) may deliver less: then at most half of what it
    // really achieves, so the reduced phase still is a reduction. No loop: the
    // fast gate is pinned at its cap in exactly that case.
    float slow = fast_target_gps(p) * AG_SLOW_RATE_PCT / 100.0f;
    if (e && e->obs_rate_gps[PH_FAST] > 0.1f) slow = fminf(slow, e->obs_rate_gps[PH_FAST] * AG_SLOW_MAX_OF_ACHIEVED);
    return fmaxf(0.5f, slow);
}

// --------------------------------------------- inherited rate machinery ---

static float rate_sample_clamped(float delta_g, float dt_s)
{
    if (dt_s <= 0.0f) return 0.0f;
    if (delta_g < -0.5f) delta_g = 0.0f;
    return (delta_g > 0.0f) ? (delta_g / dt_s) : 0.0f;
}

static float history_window_rate_gps(const filler_strategy_runtime_t *rt, uint8_t intervals)
{
    if (!rt || rt->rate_hist_count <= intervals) return 0.0f;
    float dt_s = (float)(rt->rate_hist_ts_us[0] - rt->rate_hist_ts_us[intervals]) / 1000000.0f;
    if (dt_s < DELTA_RATE_MIN_DT_S * (float)intervals || dt_s > DELTA_RATE_MAX_DT_S * (float)intervals) return 0.0f;
    return rate_sample_clamped(rt->rate_hist_rel_g[0] - rt->rate_hist_rel_g[intervals], dt_s);
}

static void push_rate_history(filler_strategy_runtime_t *rt, float rel_g, int64_t ts_us)
{
    for (int i = 4; i > 0; --i) {
        rt->rate_hist_rel_g[i] = rt->rate_hist_rel_g[i - 1];
        rt->rate_hist_ts_us[i] = rt->rate_hist_ts_us[i - 1];
    }
    rt->rate_hist_rel_g[0] = rel_g;
    rt->rate_hist_ts_us[0] = ts_us;
    if (rt->rate_hist_count < 5) rt->rate_hist_count++;
}

static float filtered_rate_step(float prev, float sample, float alpha_rise, float alpha_fall)
{
    float alpha = (sample >= prev) ? alpha_rise : alpha_fall;
    float filtered = ewma(prev, sample, alpha);
    return (filtered < RATE_ZERO_EPS_GPS) ? 0.0f : filtered;
}

static void clear_live_rate_estimates(filler_strategy_runtime_t *rt)
{
    rt->raw_rate_gps = 0.0f;
    rt->rate_2sample_gps = 0.0f;
    rt->rate_4sample_gps = 0.0f;
    rt->filtered_rate_gps = 0.0f;
    rt->filtered_rate_medium_gps = 0.0f;
    rt->filtered_rate_slow_gps = 0.0f;
    rt->no_flow_count = 0;
    rt->rate_hist_count = 0;
}

static bool stable_count(bool cond, uint8_t *count, uint8_t required)
{
    if (cond) {
        if (*count < required) (*count)++;
    } else {
        *count = 0;
    }
    return *count >= required;
}

static float response_threshold_g(const filler_strategy_tick_t *tick)
{
    float c = tick && tick->params ? ((float)tick->params->target_grams * 0.005f) : 2.0f;
    return clampf_local(c, RESPONSE_THRESHOLD_MIN_G, RESPONSE_THRESHOLD_MAX_G);
}

static void reset_state_counters(filler_strategy_runtime_t *rt)
{
    rt->cnt_near_close = 0;
    rt->cnt_close_early = 0;
    rt->cnt_target = 0;
    rt->cnt_under = 0;
    rt->cnt_over = 0;
    rt->cnt_safe_rate = 0;
    rt->sample_count = 0;
    rt->safe_reduce_ts_us = 0;
}

// ------------------------------------------------- steady segment (LS) ----

static float settle_dead_time_s(const filler_strategy_runtime_t *rt)
{
    float L = rt->learned_dead_time_s;
    if (rt->response_detected && rt->measured_dead_time_s > 0.0f) L = rt->measured_dead_time_s;
    return L;
}

static void seg_store_obs(void)
{
    int ph = s_fill.phase;
    // "barely anything flowed at this reduced gate" is kept even if a later
    // segment of the same phase becomes the learning observation
    if (ph == PH_SLOW && (s_fill.seg_valid || s_fill.seg_coarse_valid)) {
        float r = s_fill.seg_valid ? s_fill.seg_rate : s_fill.seg_coarse_rate;
        if (r < s_fill.target_gps[PH_SLOW] * AG_FLOOR_EVIDENCE_FRAC &&
            s_fill.seg_gate + AG_FLOOR_MARGIN_PCT > s_fill.floor_evidence) {
            s_fill.floor_evidence = s_fill.seg_gate + AG_FLOOR_MARGIN_PCT;
        }
    }
    if (!s_fill.seg_valid) {
        // a reduced phase too short for a full window still proves "clearly
        // too fast" (the case that never self-corrects otherwise: too fast ->
        // phase too short -> no observation -> gate never lowered)
        if (ph == PH_SLOW && s_fill.seg_coarse_valid &&
            s_fill.seg_coarse_rate > s_fill.target_gps[PH_SLOW] * AG_COARSE_TOO_FAST) {
            s_fill.obs_valid[ph] = true;
            s_fill.obs_gate[ph] = s_fill.seg_gate;
            s_fill.obs_rate[ph] = s_fill.seg_coarse_rate;
            s_fill.obs_window[ph] = s_fill.seg_window_s;
        }
        return;
    }
    if (s_fill.seg_from_closed && s_fill.open_rate <= 0.0f) {
        s_fill.open_rate = s_fill.seg_rate;
        s_fill.open_gate = s_fill.seg_gate;
    }
    s_fill.obs_valid[ph] = true;
    s_fill.obs_gate[ph] = s_fill.seg_gate;
    s_fill.obs_rate[ph] = s_fill.seg_rate;
    s_fill.obs_window[ph] = s_fill.seg_window_s;
}

// A new commanded gate value starts a new measurement segment. The previous
// segment, if it got a valid steady window, becomes that phase's observation.
static void seg_begin(const filler_strategy_runtime_t *rt, int phase, float gate, int64_t now_us, bool from_closed)
{
    seg_store_obs();
    s_fill.phase = phase;
    s_fill.seg_gate = gate;
    s_fill.seg_cmd_us = now_us;
    s_fill.seg_from_closed = from_closed;
    s_fill.seg_steady_us = 0;   // resolved in seg_update (depends on the response)
    if (!from_closed) {
        s_fill.seg_steady_us = now_us + (int64_t)((settle_dead_time_s(rt) + AG_SETTLE_AFTER_CHANGE_S) * 1e6f);
    }
    s_fill.s_n = s_fill.s_t = s_fill.s_w = s_fill.s_tt = s_fill.s_tw = 0.0;
    s_fill.seg_rate = 0.0f;
    s_fill.seg_window_s = 0.0f;
    s_fill.seg_valid = false;
    s_fill.seg_coarse_rate = 0.0f;
    s_fill.seg_coarse_valid = false;
    s_fill.rb_head = 0;
    s_fill.rb_count = 0;
    s_fill.slide_rate = 0.0f;
    s_fill.slide_valid = false;
}

// Least-squares slope over the most recent AG_SLIDE_WINDOW_S of steady samples.
static void slide_update(void)
{
    s_fill.slide_valid = false;
    if (s_fill.rb_count < 8) return;
    unsigned newest = (s_fill.rb_head + AG_RB_N - 1u) % AG_RB_N;
    float t_end = s_fill.rb_t[newest];
    double n = 0, st_ = 0, sw = 0, stt = 0, stw = 0;
    float t_first = t_end;
    for (unsigned k = 0; k < s_fill.rb_count; ++k) {
        unsigned i = (newest + AG_RB_N - k) % AG_RB_N;
        float t = s_fill.rb_t[i];
        if (t_end - t > AG_SLIDE_WINDOW_S) break;
        double tt = (double)(t - t_end);
        n += 1; st_ += tt; sw += s_fill.rb_w[i]; stt += tt * tt; stw += tt * s_fill.rb_w[i];
        t_first = t;
    }
    if (n < 8 || t_end - t_first < AG_SLIDE_MIN_SPAN_S) return;
    double var = stt - st_ * st_ / n;
    if (var <= 1e-9) return;
    float r = (float)((stw - st_ * sw / n) / var);
    s_fill.slide_rate = r > 0.0f ? r : 0.0f;
    s_fill.slide_valid = true;
}

static void seg_update(const filler_strategy_runtime_t *rt, int64_t ts_us, float rel_g)
{
    if (s_fill.seg_from_closed && s_fill.seg_steady_us == 0) {
        if (!rt->response_detected) return;
        s_fill.seg_steady_us = rt->first_response_ts_us + (int64_t)(AG_SETTLE_AFTER_RESPONSE_S * 1e6f);
    }
    if (s_fill.seg_steady_us == 0 || ts_us < s_fill.seg_steady_us) return;
    double t = (double)(ts_us - s_fill.seg_steady_us) / 1e6;
    double w = (double)rel_g;
    s_fill.s_n += 1.0;
    s_fill.s_t += t;
    s_fill.s_w += w;
    s_fill.s_tt += t * t;
    s_fill.s_tw += t * w;
    s_fill.seg_window_s = (float)t;
    s_fill.rb_t[s_fill.rb_head] = (float)t;
    s_fill.rb_w[s_fill.rb_head] = rel_g;
    s_fill.rb_head = (uint8_t)((s_fill.rb_head + 1u) % AG_RB_N);
    if (s_fill.rb_count < AG_RB_N) s_fill.rb_count++;
    slide_update();
    if (s_fill.s_n < 5.0 || t < AG_COARSE_WINDOW_S) return;
    double var = s_fill.s_tt - s_fill.s_t * s_fill.s_t / s_fill.s_n;
    if (var <= 1e-9) return;
    double cov = s_fill.s_tw - s_fill.s_t * s_fill.s_w / s_fill.s_n;
    float rate = (float)(cov / var);
    if (rate < 0.0f) rate = 0.0f;
    s_fill.seg_coarse_rate = rate;
    s_fill.seg_coarse_valid = true;
    if (t < AG_WINDOW_MIN_S) return;
    s_fill.seg_rate = rate;
    s_fill.seg_valid = true;
}

// ----------------------------------------------------- phase gate set ------

static void command_gate(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                         int phase, float gate, const char *label, int64_t now_us, bool remember)
{
    gate = clampf_local(gate, gate_min_pct(), AG_GATE_CAP_PCT);
    if (remember) s_fill.gate_cmd[phase] = gate;
    if (s_fill.phase != phase || fabsf(s_fill.seg_gate - gate) > 0.05f) {
        seg_begin(rt, phase, gate, now_us, false);
    }
    env->gate_set_percent_label(gate, label);
}

static void command_phase_gate(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                               int phase, float gate, const char *label, int64_t now_us)
{
    command_gate(rt, env, phase, gate, label, now_us, true);
}

static void follow_slow_gate(float fast_before, float fast_after)
{
    if (s_fill.reduced_latched) return;
    float k = (fast_after - s_onset_eff) / fmaxf(fast_before - s_onset_eff, AG_MIN_FLOW_MARGIN_PCT);
    k = clampf_local(k, AG_INRUN_DOWN_RATIO_MIN, AG_INRUN_UP_RATIO_MAX);
    float before = s_fill.gate_cmd[PH_SLOW];
    float g = gate_for_ratio(before, k);
    g = clampf_local(g, fmaxf(gate_min_pct(), s_slow_floor_now), fmaxf(fast_after, gate_min_pct()));
    s_fill.gate_cmd[PH_SLOW] = g;
    ESP_LOGI(TAG, "reduced gate follows the in-fill correction: %.1f%% -> %.1f%%", (double)before, (double)g);
}

// ------------------------------------------------- thresholds (parent) ----

static float transition_fast_rate_gps(const filler_strategy_runtime_t *rt)
{
    if (!s_fill.reduced_latched && rt->filtered_rate_gps > 0.5f) return rt->filtered_rate_gps;
    if (rt->learned_fast_rate_gps > 0.5f) return rt->learned_fast_rate_gps;
    if (rt->filtered_rate_gps > 0.5f) return rt->filtered_rate_gps;
    return 0.0f;
}

// Mass the reduced phase must still deliver: in-flight mass during the
// reduction dead time + a slow-phase dwell long enough for a steady rate
// observation (dead time + settle + window), at the slow TARGET rate.
static float near_window_g(const filler_strategy_runtime_t *rt, const app_params_t *p)
{
    // this fill's own measured dead time once known: after a honey change the
    // learned one belongs to the old honey (thick L x thin rate = a reduced
    // phase starting hundreds of grams early)
    float L = settle_dead_time_s(rt);
    float dwell_s = fmaxf(2.5f, L + AG_SETTLE_AFTER_CHANGE_S + AG_WINDOW_MIN_S + 1.5f);
    float margin = clampf_local(s_fill.target_gps[PH_SLOW] * dwell_s,
                                NEAR_CLOSE_CONTROL_MARGIN_MIN_G, NEAR_CLOSE_CONTROL_MARGIN_MAX_G);
    float transition = L * transition_fast_rate_gps(rt);
    return fmaxf(3.0f, transition + margin - rt->near_close_relax_g);
}

static void update_thresholds(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    const app_params_t *p = tick->params;
    // Post-close gain was learned at the slow rate the previous fills closed
    // at; if this fill's steady slow rate is clearly different (bucket topped
    // up, gate still adapting) scale it - first-order, the in-flight mass is
    // proportional to the rate.
    float scale = 1.0f;
    float now_rate = 0.0f;
    bool in_slow = s_fill.reduced_latched && s_fill.phase == PH_SLOW;
    bool steady = in_slow && s_fill.seg_valid;
    bool sliding = in_slow && s_fill.slide_valid;
    // reference: the rate previous fills closed at (the learned post-close gain
    // belongs to it); before any, the slow design rate
    float ref = s_fill.close_ref_rate > 0.1f ? s_fill.close_ref_rate : s_fill.target_gps[PH_SLOW];
    if (sliding) now_rate = s_fill.slide_rate;
    else if (s_fill.reduced_latched) now_rate = rt->filtered_rate_medium_gps;
    if (now_rate > 0.1f && ref > 0.1f) {
        float r = now_rate / ref;
        if (sliding && r > 1.0f + AG_CLOSE_SCALE_DEADBAND) {
            scale = clampf_local(r, 1.0f, AG_CLOSE_SCALE_MAX);           // faster: close earlier
        } else if (steady && r < 1.0f - AG_CLOSE_SCALE_DEADBAND) {
            scale = clampf_local(r, AG_CLOSE_SCALE_MIN, 1.0f);           // slower: close later
        } else if (sliding && r < 1.0f - AG_CLOSE_SCALE_DEADBAND) {
            scale = clampf_local(r, AG_CLOSE_SCALE_MIN_SLIDE, 1.0f);
        } else if (!sliding && r > 2.0f) {
            // no steady slow window yet: the filtered rate still carries the
            // fast-phase transition, so only a GROSS excess closes earlier
            scale = clampf_local(r * 0.5f, 1.0f, AG_CLOSE_SCALE_MAX);
        }
    }
    s_fill.close_scale = scale;
    rt->predicted_remaining_g = rt->learned_post_close_gain_g * scale;
    float close_max = fmaxf((float)p->target_grams * 0.15f, 8.0f);
    rt->adapted_close_early_g = clampf_local(rt->predicted_remaining_g + CLOSE_BUFFER_G - rt->close_early_relax_g,
                                             2.0f, close_max);
    float near_max = fmaxf((float)p->target_grams * 0.5f, 20.0f);
    rt->adapted_near_close_g = clampf_local(rt->adapted_close_early_g + near_window_g(rt, p),
                                            rt->adapted_close_early_g + 3.0f, near_max);
    rt->adapted_drip_wait_ms = clampf_local(rt->adapted_drip_wait_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS);
}

static float no_response_limit_s(const filler_strategy_runtime_t *rt)
{
    float L = rt->learned_dead_time_s;
    return clampf_local(fmaxf(L * NO_RESPONSE_DEAD_TIME_MULT, L + NO_RESPONSE_EXTRA_S),
                        NO_RESPONSE_MIN_S, NO_RESPONSE_MAX_S);
}

static float safe_rate_limit_gps(const filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    float abs_ceiling = SAFE_RATE_MAX_GPS;
    if (tick->params->target_grams > 0) abs_ceiling = (float)tick->params->target_grams / ADAPT_MIN_CONTROLLED_FILL_S;
    abs_ceiling = clampf_local(abs_ceiling, SAFE_RATE_MIN_GPS, SAFE_RATE_MAX_GPS);
    // Relative ceiling against the TARGET rate (not the learned one: the gate
    // is regulated to the target, so that is the expected norm).
    float rel = clampf_local(s_fill.target_gps[PH_FAST] * SAFE_RATE_MULT * AG_INRUN_TOO_FAST,
                             SAFE_RATE_MIN_GPS, SAFE_RATE_MAX_GPS);
    return fminf(abs_ceiling, rel);
}

// ---------------------------------------------------------- telemetry -----

static const char *phase_label(const filler_strategy_runtime_t *rt)
{
    if (rt->first_close_seen) return "closed";
    if (rt->refill_count > 0 && !s_fill.reduced_latched) return "refill";
    return s_fill.reduced_latched ? "reduced" : "fast";
}

static void publish_snapshot(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                             const filler_strategy_tick_t *tick)
{
    if (!env->set_sample_telemetry) return;
    int ph = s_fill.reduced_latched ? PH_SLOW : PH_FAST;
    int last = s_fill.obs_valid[PH_SLOW] && s_fill.reduced_latched ? PH_SLOW : PH_FAST;
    filler_strategy_sample_telemetry_t s = {
        .valid = true,
        .raw_rate_gps = rt->raw_rate_gps,
        .rate_2sample_gps = rt->rate_2sample_gps,
        .rate_4sample_gps = rt->rate_4sample_gps,
        .filtered_rate_gps = rt->filtered_rate_gps,
        .filtered_rate_medium_gps = rt->filtered_rate_medium_gps,
        .filtered_rate_slow_gps = rt->filtered_rate_slow_gps,
        .target_rate_gps = s_fill.target_gps[ph],
        .rate_error_gps = s_fill.seg_valid ? (s_fill.target_gps[ph] - s_fill.seg_rate) : 0.0f,
        .predicted_remaining_g = rt->predicted_remaining_g,
        .measured_dead_time_s = rt->measured_dead_time_s,
        .measured_post_close_gain_g = rt->measured_post_close_gain_g,
        .measured_near_close_gain_g = rt->measured_near_close_gain_g,
        .learned_dead_time_s = rt->learned_dead_time_s,
        .learned_post_close_gain_g = rt->learned_post_close_gain_g,
        .learned_fast_rate_gps = rt->learned_fast_rate_gps,
        .learned_slow_rate_gps = rt->learned_slow_rate_gps,
        .adapted_near_close_g = rt->adapted_near_close_g,
        .adapted_close_early_g = rt->adapted_close_early_g,
        .adapted_drip_wait_ms = rt->adapted_drip_wait_ms,
        .flow_onset_gate_pct = s_onset_eff,
        .gain_obs_gate_pct = s_fill.obs_gate[last],
        .gain_obs_rate_gps = s_fill.obs_rate[last],
        .gain_obs_count = (float)s_fill.corrections,
        .control_rate_gps = s_fill.seg_valid ? s_fill.seg_rate : 0.0f,
        .control_integ_pct = s_fill.gate_cmd[ph],
        .refill_count = rt->refill_count,
    };
    snprintf(s.strategy_name, sizeof(s.strategy_name), "%s", tick->strategy_name ? tick->strategy_name : "?");
    snprintf(s.gate_phase, sizeof(s.gate_phase), "%s", phase_label(rt));
    env->set_sample_telemetry(&s);
}

// ---------------------------------------------------- gate learning -------

// Between-fill update of one phase's learned gate from this fill's steady
// observation. Returns true if it changed.
static bool learn_phase_gate(ag_entry_t *e, int ph, const app_params_t *p, float *out_ratio)
{
    float target = s_fill.target_gps[ph];
    if (!s_fill.obs_valid[ph] || target <= 0.0f) return false;
    float og = s_fill.obs_gate[ph];
    float orate = s_fill.obs_rate[ph];
    if (ph == PH_SLOW && orate < target * AG_FLOOR_EVIDENCE_FRAC) {
        // Crawling right above the onset: the observation says "higher", but a
        // ratio from a near-zero rate is meaningless there (the cliff). Remember
        // the floor and step up by one small increment from the highest gate
        // this fill already tried (an in-fill bump may not have been observed).
        float fl = og + AG_FLOOR_MARGIN_PCT;
        if (fl > e->slow_floor_pct) {
            e->slow_floor_pct = fl;
            ESP_LOGI(TAG, "slow floor -> %.1f%% (%.2f g/s at %.1f%%)", (double)fl, (double)orate, (double)og);
        }
        float before = e->gate_pct[ph];
        float next = fmaxf(fmaxf(og + AG_SLOW_STEP_UP_MAX_PCT, s_fill.gate_cmd[PH_SLOW]), e->slow_floor_pct);
        next = clampf_local(fminf(next, before + 2.0f * AG_SLOW_STEP_UP_MAX_PCT), gate_min_pct(), AG_GATE_CAP_PCT);
        e->slow_pending_ratio = 0.0f;
        e->gate_pct[ph] = next;
        e->obs_gate_pct[ph] = og;
        e->obs_rate_gps[ph] = orate;
        if (out_ratio) *out_ratio = (orate > 0.2f) ? target / orate : AG_RATIO_MAX;
        return fabsf(next - before) > 0.05f;
    }
    float ratio = (orate > 0.2f) ? target / orate : AG_RATIO_MAX;
    if (out_ratio) *out_ratio = ratio;
    float before = e->gate_pct[ph];
    float desired = og;
    bool slow = (ph == PH_SLOW);
    bool out_of_band = fabsf(ratio - 1.0f) > (slow ? AG_SLOW_DEADBAND : AG_DEADBAND);
    // The reduced-phase window is short and sits right above the onset, so a
    // single observation is noisy (drips): act only when two consecutive fills
    // agree on the direction, on their geometric mean. A regime change or an
    // in-fill correction is evidence enough on its own.
    bool clearly_slow = slow && ratio > AG_SLOW_CLEARLY_SLOW;
    if (slow && !clearly_slow && !s_fill.regime_change && !s_fill.obs_from_correction[ph]) {
        float pend = e->slow_pending_ratio;
        if (!out_of_band) {
            e->slow_pending_ratio = 0.0f;
        } else if (pend > 0.0f && ((pend > 1.0f) == (ratio > 1.0f))) {
            ratio = sqrtf(pend * ratio);
            e->slow_pending_ratio = 0.0f;
            out_of_band = fabsf(ratio - 1.0f) > AG_SLOW_DEADBAND;
        } else {
            e->slow_pending_ratio = ratio;
            out_of_band = false;
        }
        if (out_ratio) *out_ratio = ratio;
    }
    if (out_of_band) {
        // lead only for natural drain drift; after an in-fill correction the
        // observation sits next to a gate that was just proven too fast
        bool drift = ratio > 1.0f && !s_fill.obs_from_correction[ph] && !s_fill.safe_event;
        float aim = drift ? ratio * (1.0f + AG_DRAIN_LEAD) : ratio;
        desired = gate_for_ratio(og, clampf_local(aim, AG_RATIO_MIN, AG_RATIO_MAX));
    }
    float alpha = (e->successful_fills == 0 || s_fill.regime_change) ? 1.0f
                : (slow ? AG_SLOW_LEARN_ALPHA : AG_LEARN_ALPHA_GATE);
    if (e->step_gain[ph] <= 0.0f) e->step_gain[ph] = 1.0f;
    int8_t dir = (desired > before + 0.05f) ? 1 : (desired < before - 0.05f) ? -1 : 0;
    if (dir != 0 && e->last_dir[ph] != 0) {
        e->step_gain[ph] = (dir != e->last_dir[ph])
            ? fmaxf(AG_FLIP_GAIN_MIN, e->step_gain[ph] * 0.5f)
            : fminf(1.0f, e->step_gain[ph] * AG_FLIP_GAIN_RECOVER);
    }
    if (dir != 0) e->last_dir[ph] = dir;
    float next = before + alpha * e->step_gain[ph] * (desired - before);
    // step limits only bound the slow drift learning; an in-fill correction
    // already proved the new gate on the plant
    if (slow || !s_fill.obs_from_correction[ph]) {
        float up = slow ? (clearly_slow ? AG_SLOW_STEP_UP_CLEAR_PCT : AG_SLOW_STEP_UP_MAX_PCT) : AG_STEP_UP_MAX_PCT;
        float down = slow ? AG_SLOW_STEP_DOWN_MAX_PCT : AG_STEP_DOWN_MAX_PCT;
        next = clampf_local(next, before - down, before + up);
    }
    float lo = (ph == PH_SLOW) ? fmaxf(gate_min_pct(), e->slow_floor_pct) : gate_min_pct();
    next = clampf_local(next, lo, AG_GATE_CAP_PCT);
    e->gate_pct[ph] = next;
    e->obs_gate_pct[ph] = og;
    e->obs_rate_gps[ph] = orate;
    (void)p;
    return fabsf(next - before) > 0.05f;
}

// --------------------------------------------------- summary + learning ---

static void publish_summary_and_learn(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                                      const filler_strategy_tick_t *tick, const char *reason, bool success)
{
    if (!env->publish_fill_summary || !tick->latest || !tick->params) return;
    const app_params_t *p = tick->params;
    ag_entry_t *e = entry_for(tick->preset_index, p, NULL);
    seg_store_obs();   // the segment running at close is an observation too

    float final_rel_g = tick->latest->grams - rt->run_base_weight_g;
    float fast_avg = rt->fast_rate_count ? rt->fast_rate_sum_gps / (float)rt->fast_rate_count : 0.0f;
    float slow_avg = rt->slow_rate_count ? rt->slow_rate_sum_gps / (float)rt->slow_rate_count : 0.0f;
    float duration_s = rt->fill_open_ts_us > 0 ? (float)(tick->now_us - rt->fill_open_ts_us) / 1e6f : 0.0f;
    float settled_wait_ms = rt->first_close_seen && rt->last_gain_ts_us >= rt->first_close_ts_us
        ? ((float)(rt->last_gain_ts_us - rt->first_close_ts_us) / 1000.0f) + DRIP_WAIT_SETTLE_MARGIN_MS
        : rt->adapted_drip_wait_ms;
    bool plausible = success && rt->response_detected &&
                     rt->measured_dead_time_s >= DEAD_TIME_MIN_S && rt->measured_dead_time_s <= DEAD_TIME_MAX_S &&
                     rt->measured_post_close_gain_g >= 0.0f && rt->measured_post_close_gain_g <= 150.0f;

    if (s_fill.regime_change) {
        e->successful_fills = 0;        // relearn: next measurements dominate
        e->slow_floor_pct = 0.0f;       // another honey may start flowing elsewhere
        e->slow_pending_ratio = 0.0f;
        for (int i = 0; i < PH_COUNT; ++i) { e->last_dir[i] = 0; e->step_gain[i] = 1.0f; }
    }
    ag_entry_t before = *e;
    uint32_t obs = before.successful_fills;
    if (plausible) {
        // The dead time is the time to build a strand of fixed mass (L ~ m/q,
        // fitted on the thick-honey session: L*rate ~ const). If this fill
        // opened at a much lower flow than its fast phase finally ran at (cold
        // start, in-fill corrections), scale it to the final operating point.
        float dead_obs = rt->measured_dead_time_s;
        if (s_fill.open_rate > 0.1f && s_fill.obs_valid[PH_FAST] && s_fill.obs_rate[PH_FAST] > s_fill.open_rate * 1.3f) {
            float k = clampf_local(s_fill.open_rate / s_fill.obs_rate[PH_FAST], 0.25f, 1.0f);
            dead_obs = clampf_local(dead_obs * k, DEAD_TIME_MIN_S, DEAD_TIME_MAX_S);
            ESP_LOGI(TAG, "dead time %.2f s measured at %.1f g/s -> %.2f s at %.1f g/s", (double)rt->measured_dead_time_s,
                     (double)s_fill.open_rate, (double)dead_obs, (double)s_fill.obs_rate[PH_FAST]);
        }
        e->dead_time_s = learn_ewma(e->dead_time_s, dead_obs, LEARN_ALPHA_DEAD_TIME, obs);
        e->post_close_gain_g = learn_ewma(e->post_close_gain_g, rt->measured_post_close_gain_g, LEARN_ALPHA_POST_CLOSE, obs);
        if (fast_avg > 0.0f) e->fast_rate_gps = learn_ewma(e->fast_rate_gps, fast_avg, LEARN_ALPHA_RATE, obs);
        if (slow_avg > 0.0f) e->slow_rate_gps = learn_ewma(e->slow_rate_gps, slow_avg, LEARN_ALPHA_RATE, obs);
        // the steady slow rate this post-close gain belongs to (close scaling)
        float cr = s_fill.close_rate_meas > 0.1f ? s_fill.close_rate_meas
                 : (s_fill.obs_valid[PH_SLOW] ? s_fill.obs_rate[PH_SLOW] : 0.0f);
        if (cr > 0.1f) e->close_rate_gps = learn_ewma(e->close_rate_gps, cr, LEARN_ALPHA_POST_CLOSE, obs);
        e->drip_wait_ms = ewma(e->drip_wait_ms, clampf_local(settled_wait_ms, DRIP_WAIT_MIN_MS, DRIP_WAIT_MAX_MS),
                               LEARN_ALPHA_DRIP_WAIT);
    }

    // Gate learning also runs on faulted fills: a steady observation is
    // physically valid regardless of how the jar ended (a timeout says "open
    // more", an overfill "open less"). Not on stale scale / no response.
    bool gate_learn = rt->response_detected && strcmp(reason ? reason : "", "scale_stale") != 0;
    float ratio_f = 1.0f, ratio_s = 1.0f;
    bool ch_f = false, ch_s = false;
    if (gate_learn && s_fill.floor_evidence > e->slow_floor_pct) {
        e->slow_floor_pct = s_fill.floor_evidence;
        ESP_LOGI(TAG, "slow floor -> %.1f%% (reduced phase crawled below it)", (double)e->slow_floor_pct);
        if (e->gate_pct[PH_SLOW] < e->slow_floor_pct) e->gate_pct[PH_SLOW] = e->slow_floor_pct;
    }
    if (gate_learn) {
        float fast_before = e->gate_pct[PH_FAST];
        ch_f = learn_phase_gate(e, PH_FAST, p, &ratio_f);
        if (s_fill.safe_event) {
            // the safe-rate cutback measured "far too fast" before any steady
            // window: never start the next fill above the gate that tripped it
            float g = gate_for_ratio(s_fill.safe_gate,
                                     clampf_local(s_fill.target_gps[PH_FAST] / fmaxf(s_fill.safe_rate, 0.1f),
                                                  AG_INRUN_DOWN_RATIO_MIN, 1.0f));
            if (g < e->gate_pct[PH_FAST]) { e->gate_pct[PH_FAST] = fmaxf(g, gate_min_pct()); ch_f = true; }
        }
        if (s_fill.obs_valid[PH_SLOW]) {
            ch_s = learn_phase_gate(e, PH_SLOW, p, &ratio_s);
        } else if (ch_f && fast_before < AG_GATE_CAP_PCT - 0.5f) {
            // no own slow observation: the head drift seen in the fast phase
            // affects the slow phase the same way
            float k = (e->gate_pct[PH_FAST] - s_onset_eff) / fmaxf(fast_before - s_onset_eff, AG_MIN_FLOW_MARGIN_PCT);
            e->gate_pct[PH_SLOW] = clampf_local(gate_for_ratio(e->gate_pct[PH_SLOW], clampf_local(k, AG_RATIO_MIN, AG_RATIO_MAX)),
                                                fmaxf(gate_min_pct(), e->slow_floor_pct), AG_GATE_CAP_PCT);
            ch_s = true;
        }
        // the slow phase must stay slower than the fast phase
        if (e->gate_pct[PH_SLOW] > e->gate_pct[PH_FAST]) e->gate_pct[PH_SLOW] = e->gate_pct[PH_FAST];
        e->has_gates = 1;
    }
    if (plausible) e->successful_fills++;
    e->total_fills++;
    store_save();

    ESP_LOGI(TAG,
             "learn reason=%s fills=%lu gates fast %.1f->%.1f%% (obs %.1f g/s @%.1f%% target %.1f ratio %.2f) "
             "slow %.1f->%.1f%% (obs %.1f g/s @%.1f%% target %.1f ratio %.2f) L %.2f->%.2f s post %.1f->%.1f g%s%s",
             reason ? reason : "?", (unsigned long)obs,
             (double)before.gate_pct[PH_FAST], (double)e->gate_pct[PH_FAST],
             (double)s_fill.obs_rate[PH_FAST], (double)s_fill.obs_gate[PH_FAST],
             (double)s_fill.target_gps[PH_FAST], (double)ratio_f,
             (double)before.gate_pct[PH_SLOW], (double)e->gate_pct[PH_SLOW],
             (double)s_fill.obs_rate[PH_SLOW], (double)s_fill.obs_gate[PH_SLOW],
             (double)s_fill.target_gps[PH_SLOW], (double)ratio_s,
             (double)before.dead_time_s, (double)e->dead_time_s,
             (double)before.post_close_gain_g, (double)e->post_close_gain_g,
             s_fill.regime_change ? " [regime change]" : "", s_fill.safe_event ? " [safe-rate]" : "");
    (void)ch_f; (void)ch_s;

    filler_strategy_fill_summary_t summary = {
        .valid = true,
        .final_mass_g = final_rel_g,
        .target_g = (float)p->target_grams,
        .fill_error_g = final_rel_g - (float)p->target_grams,
        .measured_dead_time_s = rt->measured_dead_time_s,
        .measured_post_close_gain_g = rt->measured_post_close_gain_g,
        .measured_near_close_gain_g = rt->measured_near_close_gain_g,
        .measured_fast_rate_gps = fast_avg,
        .measured_slow_rate_gps = slow_avg,
        .rate_at_close_gps = rt->rate_at_close_gps,
        .rate_2sample_at_close_gps = rt->rate_2sample_at_close_gps,
        .rate_4sample_at_close_gps = rt->rate_4sample_at_close_gps,
        .filtered_rate_medium_at_close_gps = rt->filtered_rate_medium_at_close_gps,
        .filtered_rate_slow_at_close_gps = rt->filtered_rate_slow_at_close_gps,
        .fill_duration_s = duration_s,
        .drip_wait_used_ms = rt->adapted_drip_wait_ms,
        .refill_count = rt->refill_count,
        .used_dead_time_s = rt->learned_dead_time_s,
        .used_post_close_gain_g = rt->learned_post_close_gain_g,
        .used_fast_rate_gps = rt->learned_fast_rate_gps,
        .used_slow_rate_gps = rt->learned_slow_rate_gps,
        .used_near_close_g = rt->adapted_near_close_g,
        .used_close_early_g = rt->adapted_close_early_g,
        .next_dead_time_s = e->dead_time_s,
        .next_post_close_gain_g = e->post_close_gain_g,
        .next_fast_rate_gps = e->fast_rate_gps,
        .next_slow_rate_gps = e->slow_rate_gps,
        .next_drip_wait_ms = e->drip_wait_ms,
        .used_onset_gate_pct = s_onset_eff,
        .next_onset_gate_pct = fmaxf(AG_ONSET_PCT, e->slow_floor_pct - AG_FLOOR_MARGIN_PCT),
        .used_fast_gate_pct = s_fill.gate_start[PH_FAST],
        .used_slow_gate_pct = s_fill.gate_start[PH_SLOW],
        .next_fast_gate_pct = e->gate_pct[PH_FAST],
        .next_slow_gate_pct = e->gate_pct[PH_SLOW],
        .target_fast_rate_gps = s_fill.target_gps[PH_FAST],
        .target_slow_rate_gps = s_fill.target_gps[PH_SLOW],
        .obs_fast_rate_gps = s_fill.obs_valid[PH_FAST] ? s_fill.obs_rate[PH_FAST] : 0.0f,
        .obs_slow_rate_gps = s_fill.obs_valid[PH_SLOW] ? s_fill.obs_rate[PH_SLOW] : 0.0f,
        .gate_corrections = s_fill.corrections,
        .regime_change = s_fill.regime_change ? 1u : 0u,
    };
    snprintf(summary.reason, sizeof(summary.reason), "%s", reason ? reason : (success ? "ok" : "fault"));
    env->publish_fill_summary(tick->run_id, tick->slot_idx, tick->preset_name, tick->strategy_name, p, &summary);
}

// ---------------------------------------------------- rate estimates ------

static void update_rate_estimates(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!tick->new_sample || !tick->latest) return;
    float rel_g = tick->latest->grams - rt->run_base_weight_g;
    push_rate_history(rt, rel_g, tick->latest->ts_us);
    if (rt->last_rate_ts_us != 0) {
        float dt_s = (float)(tick->latest->ts_us - rt->last_rate_ts_us) / 1000000.0f;
        if (dt_s >= DELTA_RATE_MIN_DT_S && dt_s <= DELTA_RATE_MAX_DT_S) {
            rt->raw_rate_gps = rate_sample_clamped(rel_g - rt->last_rel_g, dt_s);
            rt->rate_2sample_gps = history_window_rate_gps(rt, 2);
            rt->rate_4sample_gps = history_window_rate_gps(rt, 4);
            float ctl = rt->raw_rate_gps >= RATE_CONTROL_MIN_VALID_GPS ? rt->raw_rate_gps : 0.0f;
            if (ctl <= 0.0f) {
                if (rt->no_flow_count < 255) rt->no_flow_count++;
                rt->filtered_rate_gps = (rt->no_flow_count >= RATE_NO_FLOW_RESET_SAMPLES)
                    ? 0.0f : ewma(rt->filtered_rate_gps, 0.0f, RATE_FILTER_ALPHA_FALL);
            } else {
                rt->no_flow_count = 0;
                rt->filtered_rate_gps = filtered_rate_step(rt->filtered_rate_gps, ctl, RATE_FILTER_ALPHA_RISE, RATE_FILTER_ALPHA_FALL);
            }
            rt->filtered_rate_medium_gps = filtered_rate_step(rt->filtered_rate_medium_gps, rt->raw_rate_gps,
                                                              RATE_FILTER_MEDIUM_ALPHA_RISE, RATE_FILTER_MEDIUM_ALPHA_FALL);
            rt->filtered_rate_slow_gps = filtered_rate_step(rt->filtered_rate_slow_gps, rt->raw_rate_gps,
                                                            RATE_FILTER_SLOW_ALPHA_RISE, RATE_FILTER_SLOW_ALPHA_FALL);
            if (!rt->first_close_seen && rt->raw_rate_gps > 0.1f) {
                if (!s_fill.reduced_latched) { rt->fast_rate_sum_gps += rt->raw_rate_gps; rt->fast_rate_count++; }
                else { rt->slow_rate_sum_gps += rt->raw_rate_gps; rt->slow_rate_count++; }
            }
        }
    }
    if (!rt->response_detected && rel_g >= response_threshold_g(tick)) {
        rt->response_detected = true;
        rt->first_response_ts_us = tick->latest->ts_us;
        rt->measured_dead_time_s = clampf_local((float)(tick->latest->ts_us - rt->fill_open_ts_us) / 1e6f,
                                                DEAD_TIME_MIN_S, DEAD_TIME_MAX_S);
        ESP_LOGI(TAG, "detected dead time: %.3f s", (double)rt->measured_dead_time_s);
    }
    if (!rt->first_close_seen) seg_update(rt, tick->latest->ts_us, rel_g);
    rt->last_rel_g = rel_g;
    rt->last_rate_ts_us = tick->latest->ts_us;
}

static void track_post_close_gain(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!tick->latest || !tick->new_sample || !rt->first_close_seen) return;
    float gain_g = fmaxf(0.0f, tick->latest->grams - rt->run_base_weight_g - rt->rel_at_first_close_g);
    rt->measured_post_close_gain_g = gain_g;
    if (gain_g > rt->last_post_close_gain_g + 0.25f) rt->last_gain_ts_us = tick->latest->ts_us;
    rt->last_post_close_gain_g = gain_g;
}

static void mark_first_close(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
{
    if (!tick->latest || rt->first_close_seen) return;
    if (s_fill.phase == PH_SLOW && s_fill.slide_valid) s_fill.close_rate_meas = s_fill.slide_rate;
    else if (s_fill.phase == PH_SLOW && s_fill.seg_coarse_valid) s_fill.close_rate_meas = s_fill.seg_coarse_rate;
    else s_fill.close_rate_meas = rt->filtered_rate_medium_gps;   // rough, but the post-close
                                                                  // gain learned now belongs to it
    seg_store_obs();
    rt->first_close_seen = true;
    rt->first_close_ts_us = tick->latest->ts_us;
    rt->rel_at_first_close_g = tick->latest->grams - rt->run_base_weight_g;
    rt->last_gain_ts_us = rt->first_close_ts_us;
    rt->rate_at_close_gps = rt->filtered_rate_gps > 0.0f ? rt->filtered_rate_gps : rt->raw_rate_gps;
    rt->rate_2sample_at_close_gps = rt->rate_2sample_gps;
    rt->rate_4sample_at_close_gps = rt->rate_4sample_gps;
    rt->filtered_rate_medium_at_close_gps = rt->filtered_rate_medium_gps;
    rt->filtered_rate_slow_at_close_gps = rt->filtered_rate_slow_gps;
    if (rt->measured_near_close_valid) {
        rt->measured_near_close_gain_g = fmaxf(0.0f, rt->rel_at_first_close_g - rt->rel_at_near_close_g);
    }
}

// --------------------------------------------- in-fill gross corrections --

static void check_regime(filler_strategy_runtime_t *rt, const ag_entry_t *e)
{
    if (s_fill.expectation_checked || !s_fill.seg_valid || s_fill.phase != PH_FAST) return;
    s_fill.expectation_checked = true;
    if (!e->has_gates || e->obs_rate_gps[PH_FAST] <= 0.1f) return;
    float open_now = fmaxf(s_fill.seg_gate - s_onset_eff, AG_MIN_FLOW_MARGIN_PCT);
    float open_obs = fmaxf(e->obs_gate_pct[PH_FAST] - s_onset_eff, AG_MIN_FLOW_MARGIN_PCT);
    float expected = e->obs_rate_gps[PH_FAST] * open_now / open_obs;
    float dev = s_fill.seg_rate / fmaxf(expected, 0.1f);
    // dead time depends on the flow itself (L ~ m/q), so compare it only when
    // this fill opened at about the gate the reference was observed at
    bool same_gate = fabsf(s_fill.open_gate_cmd - e->obs_gate_pct[PH_FAST]) < 5.0f;
    float dt_dev = (same_gate && e->dead_time_s > 0.0f && rt->measured_dead_time_s > 0.0f)
        ? rt->measured_dead_time_s / e->dead_time_s : 1.0f;
    if (dev > AG_REGIME_HI || dev < AG_REGIME_LO || dt_dev > AG_REGIME_DEAD_TIME || dt_dev < 1.0f / AG_REGIME_DEAD_TIME) {
        s_fill.regime_change = true;
        ESP_LOGW(TAG, "regime change: rate %.1f g/s vs expected %.1f g/s (x%.2f), dead time x%.2f -> relearn",
                 (double)s_fill.seg_rate, (double)expected, (double)dev, (double)dt_dev);
    }
}

// Only OBVIOUS deviations act inside a fill. Returns true if the gate moved.
static bool inrun_correct_fast(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                               const filler_strategy_tick_t *tick, float remaining_g)
{
    if (s_fill.reduced_latched || s_fill.phase != PH_FAST) return false;
    if (!s_fill.seg_valid && !s_fill.slide_valid) return false;
    float target = s_fill.target_gps[PH_FAST];
    if (s_fill.seg_valid && s_fill.slide_valid && s_fill.seg_gate < AG_GATE_CAP_PCT - 0.5f &&
        fmaxf(s_fill.seg_rate, s_fill.slide_rate) < fmaxf(AG_ZERO_FLOW_GPS, target * AG_ZERO_FLOW_TARGET_FRAC)) {
        float g0 = s_fill.seg_gate;
        float g1 = clampf_local(g0 + AG_NOFLOW_BUMP_PCT, gate_min_pct(), AG_GATE_CAP_PCT);
        s_fill.corrections++;
        s_fill.dir_lock[PH_FAST] = 1;
        ESP_LOGW(TAG, "in-fill no_flow: rate %.2f g/s over %.1f s -> fast gate %.1f%% -> %.1f%%",
                 (double)fmaxf(s_fill.seg_rate, s_fill.slide_rate), (double)s_fill.seg_window_s, (double)g0, (double)g1);
        s_fill.obs_from_correction[PH_FAST] = true;
        command_phase_gate(rt, env, PH_FAST, g1, "no_flow", tick->now_us);
        return true;
    }
    if (s_fill.corrections >= AG_INRUN_MAX_CORRECTIONS) return false;
    // too fast: the sliding window (sees a step, e.g. the bucket topped up
    // mid-fill); too slow: the cumulative AND the sliding rate must agree
    bool cum_fast = s_fill.seg_valid && s_fill.seg_rate > target * AG_INRUN_TOO_FAST;
    bool slide_fast = s_fill.slide_valid && s_fill.slide_rate > target * AG_INRUN_TOO_FAST_SLIDE;
    float r_slow = (s_fill.seg_valid && s_fill.slide_valid) ? fmaxf(s_fill.seg_rate, s_fill.slide_rate) : 1e9f;
    float r = 0.0f;
    float g = s_fill.seg_gate;
    float next = g;
    const char *why = NULL;
    int8_t dir = 0;
    if ((cum_fast || slide_fast) && s_fill.dir_lock[PH_FAST] <= 0) {
        // size by the calmer cumulative rate unless only the step detector saw it
        r = cum_fast ? s_fill.seg_rate : s_fill.slide_rate;
        next = gate_for_ratio(g, clampf_local(target / r, AG_INRUN_DOWN_RATIO_MIN, 1.0f));
        why = "too_fast";
        dir = -1;
    } else if (s_fill.dir_lock[PH_FAST] >= 0 && (r = r_slow) * AG_INRUN_TOO_SLOW < target && g < AG_GATE_CAP_PCT - 0.5f &&
               remaining_g > (float)tick->params->target_grams * AG_INRUN_UP_MIN_REMAINING_FRAC) {
        float ratio = (r > 0.2f) ? fminf(target / r, AG_INRUN_UP_RATIO_MAX) : AG_INRUN_UP_RATIO_MAX;
        next = fminf(gate_for_ratio(g, ratio), g + AG_INRUN_UP_STEP_MAX_PCT);
        why = "too_slow";
        dir = 1;
    }
    if (!why) return false;
    next = clampf_local(next, gate_min_pct(), AG_GATE_CAP_PCT);
    if (fabsf(next - g) < 0.5f) return false;
    s_fill.corrections++;
    s_fill.dir_lock[PH_FAST] = dir;
    ESP_LOGW(TAG, "in-fill %s: rate %.1f g/s target %.1f g/s over %.1f s -> fast gate %.1f%% -> %.1f%%",
             why, (double)r, (double)target, (double)s_fill.seg_window_s, (double)g, (double)next);
    s_fill.obs_from_correction[PH_FAST] = true;
    follow_slow_gate(g, next);
    command_phase_gate(rt, env, PH_FAST, next, why, tick->now_us);
    return true;
}

// ---------------------------------------------------------- on_enter ------

static void ag_on_enter(filler_strategy_runtime_t *rt, filler_state_t state, filler_state_t prev_state,
                        const filler_strategy_env_t *env, const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick || !tick->params) return;
    switch (state) {
    case FILLER_FILL: {
        ag_entry_t *e = entry_for(tick->preset_index, tick->params, NULL);
        if (prev_state != FILLER_VERIFY_TARGET) {
            memset(rt, 0, sizeof(*rt));
            memset(&s_fill, 0, sizeof(s_fill));
            rt->fill_open_ts_us = tick->now_us;
            rt->run_base_weight_g = tick->latest ? tick->latest->grams : 0.0f;
            rt->active_preset_index = tick->preset_index;
            rt->target_g = (float)tick->params->target_grams;
            rt->adapted_drip_wait_ms = e->drip_wait_ms;
            s_fill.target_gps[PH_FAST] = fast_target_gps(tick->params);
            s_fill.target_gps[PH_SLOW] = slow_target_gps(tick->params, e);
            s_fill.gate_cmd[PH_FAST] = e->gate_pct[PH_FAST];
            s_fill.gate_cmd[PH_SLOW] = e->gate_pct[PH_SLOW];
            // first fill after a boot with a persisted reference: open derated
            // (bucket may have been topped up, honey changed while off)
            if (e->has_gates && e->successful_fills == 0 && e->total_fills > 0) {
                s_fill.first_after_boot = true;
                s_fill.gate_cmd[PH_FAST] = clampf_local(gate_for_ratio(e->gate_pct[PH_FAST], AG_BOOT_DERATE),
                                                        gate_min_pct(), AG_GATE_CAP_PCT);
                s_fill.gate_cmd[PH_SLOW] = clampf_local(gate_for_ratio(e->gate_pct[PH_SLOW], AG_BOOT_SLOW_DERATE),
                                                        gate_min_pct(), AG_GATE_CAP_PCT);
            }
            s_fill.gate_start[PH_FAST] = s_fill.gate_cmd[PH_FAST];
            s_fill.open_gate_cmd = s_fill.gate_cmd[PH_FAST];
            s_fill.gate_start[PH_SLOW] = s_fill.gate_cmd[PH_SLOW];
            ESP_LOGI(TAG, "start fill preset=%u fills=%lu/%lu target %.1f/%.1f g/s gates fast=%.1f%% slow=%.1f%%%s",
                     (unsigned)tick->preset_index, (unsigned long)e->successful_fills, (unsigned long)e->total_fills,
                     (double)s_fill.target_gps[PH_FAST], (double)s_fill.target_gps[PH_SLOW],
                     (double)s_fill.gate_cmd[PH_FAST], (double)s_fill.gate_cmd[PH_SLOW],
                     s_fill.first_after_boot ? " (first after boot: derated)" : "");
        } else {
            rt->refill_count++;
            rt->fill_open_ts_us = tick->now_us;
            rt->first_close_seen = false;
            rt->rel_at_first_close_g = 0.0f;
            rt->rel_at_near_close_g = 0.0f;
            rt->measured_post_close_gain_g = 0.0f;
            rt->measured_near_close_gain_g = 0.0f;
            rt->measured_near_close_valid = false;
            rt->last_post_close_gain_g = 0.0f;
            rt->first_close_ts_us = 0;
            rt->last_gain_ts_us = 0;
            // a refill tops up with the reduced opening: it is a small amount
            s_fill.reduced_latched = true;
            ESP_LOGI(TAG, "refill attempt=%u relaxed_close=%.1f g", (unsigned)rt->refill_count,
                     (double)rt->close_early_relax_g);
        }
        rt->fill_eval_after_ts_us = (tick->latest && tick->latest->ts_us != 0) ? tick->latest->ts_us : tick->now_us;
        rt->last_rate_ts_us = 0;
        rt->last_rel_g = 0.0f;
        clear_live_rate_estimates(rt);
        reset_state_counters(rt);
        rt->learned_dead_time_s = e->dead_time_s;
        rt->learned_post_close_gain_g = e->post_close_gain_g;
        rt->learned_fast_rate_gps = e->fast_rate_gps;
        rt->learned_slow_rate_gps = e->slow_rate_gps;
        s_fill.close_ref_rate = e->close_rate_gps;
        update_thresholds(rt, tick);
        env->publish_fill_start(tick->run_id, tick->slot_idx, tick->strategy_name, tick->params,
                                tick->latest ? tick->latest->grams : 0.0f);
        int ph = s_fill.reduced_latched ? PH_SLOW : PH_FAST;
        float g = clampf_local(s_fill.gate_cmd[ph], gate_min_pct(), AG_GATE_CAP_PCT);
        s_fill.last_bump_us = tick->now_us;
        // a refill resumes from closed, but rel_g is already near target so the
        // response detector cannot re-arm: treat it like a gate change (settle
        // by the learned dead time) instead of waiting for a response
        seg_begin(rt, ph, g, tick->now_us, prev_state != FILLER_VERIFY_TARGET);
        env->gate_set_percent_label(g, prev_state == FILLER_VERIFY_TARGET ? "refill_open" : "auto_open");
        publish_snapshot(rt, env, tick);
        break;
    }
    case FILLER_DRIP_WAIT:
        reset_state_counters(rt);
        mark_first_close(rt, tick);
        env->gate_close_label("drip_wait");
        publish_snapshot(rt, env, tick);
        break;
    case FILLER_VERIFY_TARGET:
        reset_state_counters(rt);
        env->gate_close_label("close");
        publish_snapshot(rt, env, tick);
        break;
    default:
        break;
    }
}

static filler_state_t fault_out(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                                const filler_strategy_tick_t *tick, const char *label, filler_fault_t flt,
                                const char *reason)
{
    env->gate_close_label(label);
    env->set_fault(flt);
    mark_first_close(rt, tick);
    publish_snapshot(rt, env, tick);
    publish_summary_and_learn(rt, env, tick, reason, false);
    env->clear_sample_telemetry();
    return FILLER_FAULT;
}

// -------------------------------------------------------------- step ------

static filler_state_t ag_step(filler_strategy_runtime_t *rt, filler_state_t state,
                              const filler_strategy_env_t *env, const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick || !tick->latest || !tick->params) return state;
    const app_params_t *p = tick->params;
    if (tick->new_sample && rt->sample_count < 255) rt->sample_count++;

    switch (state) {
    case FILLER_FILL: {
        filler_state_t next_state = state;
        if (!env->require_fresh_or_fault("fill", tick->latest, &next_state)) {
            env->clear_sample_telemetry();
            publish_summary_and_learn(rt, env, tick, "scale_stale", false);
            return next_state;
        }
        ag_entry_t *e = entry_for(tick->preset_index, p, NULL);
        float rel_g = tick->latest->grams - rt->run_base_weight_g;
        bool sample_after_open = tick->new_sample && tick->latest->ts_us > rt->fill_eval_after_ts_us;
        float open_elapsed_ms = (float)(tick->now_us - rt->fill_open_ts_us) / 1000.0f;
        if (sample_after_open) update_rate_estimates(rt, tick);
        update_thresholds(rt, tick);

        // --- parent safeguards ---
        float hard_margin_g = fmaxf((float)p->target_tol_high_g + ADAPT_HARD_OVERFILL_MARGIN_EXTRA_G,
                                    ADAPT_HARD_OVERFILL_MARGIN_MIN_G);
        if (rel_g > (float)p->target_grams + hard_margin_g) {
            ESP_LOGE(TAG, "hard safety overweight: rel=%.1f g", (double)rel_g);
            return fault_out(rt, env, tick, "safety_close", FLT_WEIGHT_RANGE, "hard_overfill");
        }
        // nothing comes out: open more (a couple of times) before giving up
        if (!rt->response_detected && rt->refill_count == 0) {
            float since_bump_s = (float)(tick->now_us - s_fill.last_bump_us) / 1e6f;
            float bump_after_s = fmaxf(AG_NOFLOW_BUMP_AFTER_S, rt->learned_dead_time_s * 2.0f);
            if (since_bump_s > bump_after_s && s_fill.noflow_bumps < AG_NOFLOW_MAX_BUMPS &&
                s_fill.gate_cmd[PH_FAST] < AG_GATE_CAP_PCT - 0.5f) {
                s_fill.noflow_bumps++;
                s_fill.last_bump_us = tick->now_us;
                float g = s_fill.gate_cmd[PH_FAST] + AG_NOFLOW_BUMP_PCT;
                ESP_LOGW(TAG, "no flow after %.1f s at %.1f%% -> open to %.1f%%",
                         (double)since_bump_s, (double)s_fill.gate_cmd[PH_FAST], (double)g);
                s_fill.obs_from_correction[PH_FAST] = true;
                s_fill.gate_cmd[PH_FAST] = clampf_local(g, gate_min_pct(), AG_GATE_CAP_PCT);
                s_fill.seg_gate = s_fill.gate_cmd[PH_FAST];   // still the opening segment
                env->gate_set_percent_label(s_fill.gate_cmd[PH_FAST], "noflow_bump");
            }
            if ((float)(tick->now_us - s_fill.last_bump_us) / 1e6f > no_response_limit_s(rt)) {
                ESP_LOGE(TAG, "no weight response at %.1f%% -> fault", (double)s_fill.gate_cmd[PH_FAST]);
                return fault_out(rt, env, tick, "no_response", FLT_EMPTY_HONEY, "no_response");
            }
        }
        if ((tick->now_us - rt->fill_open_ts_us) > ((int64_t)p->fill_timeout_ms * 1000)) {
            ESP_LOGE(TAG, "fill timeout (rel=%.1f g) -> fault", (double)rel_g);
            return fault_out(rt, env, tick, "fill_timeout", FLT_SERVO_TIMEOUT, "fill_timeout");
        }
        float safe_limit = safe_rate_limit_gps(rt, tick);
        if (tick->new_sample) {
            if (rt->filtered_rate_gps > safe_limit) {
                if (rt->cnt_safe_rate < 255) rt->cnt_safe_rate++;
            } else {
                rt->cnt_safe_rate = 0;
            }
        }
        if (rt->cnt_safe_rate >= RATE_SPIKE_CONFIRM_SAMPLES) {
            if (rt->safe_reduce_ts_us == 0) {
                rt->safe_reduce_ts_us = tick->now_us;
                // also lower this fill's fast opening so leaving the cutback
                // does not run straight back into it (no limit cycle)
                float g = s_fill.gate_cmd[PH_FAST];
                float ng = gate_for_ratio(g, clampf_local(safe_limit * 0.6f / fmaxf(rt->filtered_rate_gps, 0.1f),
                                                          AG_INRUN_DOWN_RATIO_MIN, 1.0f));
                s_fill.safe_event = true;
                s_fill.safe_gate = g;
                s_fill.safe_rate = rt->filtered_rate_gps;
                s_fill.gate_cmd[PH_FAST] = clampf_local(fminf(ng, g), gate_min_pct(), AG_GATE_CAP_PCT);
                follow_slow_gate(g, s_fill.gate_cmd[PH_FAST]);
                ESP_LOGW(TAG, "safe-rate cutback rate=%.1f g/s limit=%.1f g/s -> gate %.1f%% (fast gate now %.1f%%)",
                         (double)rt->filtered_rate_gps, (double)safe_limit,
                         (double)fminf(s_fill.gate_cmd[PH_SLOW], s_fill.gate_cmd[PH_FAST]), (double)s_fill.gate_cmd[PH_FAST]);
            }
            float elapsed = (float)(tick->now_us - rt->safe_reduce_ts_us) / 1e6f;
            if (elapsed > SAFE_REDUCE_ESCALATE_S) {
                ESP_LOGE(TAG, "flow runaway: %.1f g/s above %.1f g/s for %.1f s at reduced gate -> fault",
                         (double)rt->filtered_rate_gps, (double)safe_limit, (double)elapsed);
                return fault_out(rt, env, tick, "runaway_close", FLT_WEIGHT_RANGE, "flow_runaway");
            }
            int ph = s_fill.reduced_latched ? PH_SLOW : PH_FAST;
            command_gate(rt, env, ph, fminf(s_fill.gate_cmd[PH_SLOW], s_fill.gate_cmd[PH_FAST]),
                         "safe_reduce", tick->now_us, false);
            publish_snapshot(rt, env, tick);
            return state;
        }
        rt->safe_reduce_ts_us = 0;

        if (!sample_after_open) {
            publish_snapshot(rt, env, tick);
            return state;
        }
        if (rt->refill_count > 0 && open_elapsed_ms < REFILL_MIN_OPEN_MS) {
            publish_snapshot(rt, env, tick);
            return state;
        }

        float remaining_g = (float)p->target_grams - rel_g;
        if (rel_g >= (float)p->target_grams) {
            ESP_LOGI(TAG, "target reached directly rel=%.1f g -> close", (double)rel_g);
            env->gate_close_label("target");
            mark_first_close(rt, tick);
            publish_snapshot(rt, env, tick);
            return FILLER_DRIP_WAIT;
        }
        if (remaining_g <= rt->adapted_close_early_g) {
            ESP_LOGI(TAG, "close remaining=%.1f g threshold=%.1f g (post %.1f g x%.2f)", (double)remaining_g,
                     (double)rt->adapted_close_early_g, (double)rt->learned_post_close_gain_g, (double)s_fill.close_scale);
            env->gate_close_label("adaptive_close");
            mark_first_close(rt, tick);
            publish_snapshot(rt, env, tick);
            return FILLER_DRIP_WAIT;
        }
        if (!s_fill.reduced_latched && remaining_g <= rt->adapted_near_close_g) {
            s_fill.reduced_latched = true;    // one-way: never back to the fast gate
            rt->near_close_logged = true;
            rt->measured_near_close_valid = true;
            rt->rel_at_near_close_g = rel_g;
            ESP_LOGI(TAG, "reduced phase remaining=%.1f g threshold=%.1f g -> gate %.1f%% (target %.1f g/s)",
                     (double)remaining_g, (double)rt->adapted_near_close_g, (double)s_fill.gate_cmd[PH_SLOW],
                     (double)s_fill.target_gps[PH_SLOW]);
            command_phase_gate(rt, env, PH_SLOW, fminf(s_fill.gate_cmd[PH_SLOW], s_fill.gate_cmd[PH_FAST]),
                               "auto_reduced", tick->now_us);
            publish_snapshot(rt, env, tick);
            return state;
        }

        if (!s_fill.reduced_latched) {
            check_regime(rt, e);
            if (!inrun_correct_fast(rt, env, tick, remaining_g)) {
                command_phase_gate(rt, env, PH_FAST, s_fill.gate_cmd[PH_FAST],
                                   rt->refill_count > 0 ? "refill_open" : "auto_fast", tick->now_us);
            }
        } else {
            // reduced phase: only "way too fast" (cut now) and "it stalled"
            // move the gate
            if (s_fill.phase == PH_SLOW && s_fill.slide_valid && s_fill.slow_cuts < 1u && s_fill.dir_lock[PH_SLOW] <= 0 &&
                s_fill.slide_rate > s_fill.target_gps[PH_SLOW] * AG_SLOW_TOO_FAST) {
                s_fill.slow_cuts++;
                s_fill.corrections++;
                s_fill.dir_lock[PH_SLOW] = -1;
                float g = gate_for_ratio(s_fill.seg_gate,
                                         clampf_local(s_fill.target_gps[PH_SLOW] / s_fill.slide_rate,
                                                      AG_INRUN_DOWN_RATIO_MIN, 1.0f));
                g = fmaxf(g, fmaxf(gate_min_pct(), e->slow_floor_pct));
                ESP_LOGW(TAG, "reduced phase too fast (%.1f g/s, target %.1f) -> slow gate %.1f%% -> %.1f%%",
                         (double)s_fill.slide_rate, (double)s_fill.target_gps[PH_SLOW], (double)s_fill.seg_gate, (double)g);
                s_fill.obs_from_correction[PH_SLOW] = true;
                command_phase_gate(rt, env, PH_SLOW, g, "slow_cut", tick->now_us);
            } else if (s_fill.phase == PH_SLOW && s_fill.seg_valid &&
                ((s_fill.seg_rate < s_fill.target_gps[PH_SLOW] * AG_SLOW_STALL_FRAC &&
                  s_fill.slow_bumps < AG_SLOW_STALL_MAX && s_fill.dir_lock[PH_SLOW] >= 0) ||
                 // nothing flows at all (e.g. a cut below this honey's onset):
                 // never a state to stay in -> open again despite the lock
                 (s_fill.seg_rate < AG_ZERO_FLOW_GPS && s_fill.seg_gate < AG_GATE_CAP_PCT - 0.5f))) {
                s_fill.corrections++;
                s_fill.slow_bumps++;
                s_fill.dir_lock[PH_SLOW] = 1;
                float g = s_fill.seg_gate + AG_SLOW_STALL_BUMP_PCT;
                ESP_LOGW(TAG, "reduced phase stalled (%.2f g/s) -> slow gate %.1f%% -> %.1f%%",
                         (double)s_fill.seg_rate, (double)s_fill.seg_gate, (double)g);
                s_fill.obs_from_correction[PH_SLOW] = true;
                command_phase_gate(rt, env, PH_SLOW, g, "slow_stall", tick->now_us);
            } else {
                command_phase_gate(rt, env, PH_SLOW, s_fill.gate_cmd[PH_SLOW],
                                   rt->refill_count > 0 ? "refill_open" : "auto_reduced", tick->now_us);
            }
        }
        publish_snapshot(rt, env, tick);
        return state;
    }

    case FILLER_DRIP_WAIT:
        update_rate_estimates(rt, tick);
        track_post_close_gain(rt, tick);
        if ((tick->now_us - tick->state_enter_us) >= ((int64_t)rt->adapted_drip_wait_ms * 1000)) {
            ESP_LOGI(TAG, "drip wait complete waited=%.0f ms post_close_gain=%.1f g",
                     (double)rt->adapted_drip_wait_ms, (double)rt->measured_post_close_gain_g);
            publish_snapshot(rt, env, tick);
            return FILLER_VERIFY_TARGET;
        }
        publish_snapshot(rt, env, tick);
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
        if (tick->new_sample &&
            stable_count(rel_g >= (float)p->target_grams + (float)p->target_tol_high_g, &rt->cnt_over, VERIFY_CONFIRM_SAMPLES)) {
            ESP_LOGE(TAG, "overweight: rel=%.1f g", (double)rel_g);
            env->set_fault(FLT_WEIGHT_RANGE);
            publish_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "overweight", false);
            env->clear_sample_telemetry();
            return FILLER_FAULT;
        }
        if (tick->new_sample &&
            stable_count(rel_g <= (float)p->target_grams - (float)p->target_tol_low_g, &rt->cnt_under, VERIFY_CONFIRM_SAMPLES)) {
            float under = (float)p->target_grams - rel_g;
            float relax = fmaxf(under * REFILL_CLOSE_RELAX_FACTOR, REFILL_RELAX_STEP_G);
            rt->close_early_relax_g += relax;
            rt->near_close_relax_g = fminf(rt->near_close_relax_g + NEAR_CLOSE_REFILL_RELAX_STEP_G, NEAR_CLOSE_REFILL_RELAX_MAX_G);
            ESP_LOGI(TAG, "underweight rel=%.1f g (-%.1f g) -> refill, relax close by %.1f g", (double)rel_g,
                     (double)under, (double)relax);
            publish_snapshot(rt, env, tick);
            return FILLER_FILL;
        }
        if (rt->sample_count >= VERIFY_CONFIRM_SAMPLES) {
            ESP_LOGI(TAG, "verify target confirmed rel=%.1f g target=%u g", (double)rel_g, (unsigned)p->target_grams);
            publish_snapshot(rt, env, tick);
            publish_summary_and_learn(rt, env, tick, "ok", true);
            env->clear_sample_telemetry();
            env->set_slot(env->slot_next_idx(tick->slot_idx, p->slots_total));
            return FILLER_FIND_SLOT;
        }
        publish_snapshot(rt, env, tick);
        return state;
    }

    default:
        return state;
    }
}

const filler_strategy_ops_t g_filler_strategy_adaptive_gate_ops = {
    .name = "adaptive-gate",
    .on_enter = ag_on_enter,
    .step = ag_step,
};
