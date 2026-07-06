#include "filler_strategy.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

// Scripted plant-probe strategy: runs a fixed, hard-coded timed gate script and
// logs telemetry exactly like the dosing strategies, without any target or
// learning logic. Its only purpose is to capture a clean, repeatable Strecke
// (plant) response for the thesis - dead time, rate build-up per gate step,
// and post-close drip - so it can be discussed in isolation.
//
// Edit k_seq[] to change the script. Each step commands a gate opening for a
// fixed duration; a 0 % step closes the gate (e.g. to observe the drip). After
// the last step the fill is reported as a success and the carousel advances.
typedef struct {
    float gate_pct;
    uint32_t duration_ms;
} seq_step_t;

static const seq_step_t k_seq[] = {
    { 30.0f,  5000 },   // partial open
    { 90.0f,  5000 },   // near-full open
    {  0.0f, 10000 },   // close and observe drip / settling
};
#define SEQ_STEP_COUNT (sizeof(k_seq) / sizeof(k_seq[0]))

// Rate estimation (same shape/constants as the dosing strategies) so the chart
// shows the same derived signals.
#define RATE_FILTER_ALPHA_RISE 0.18f
#define RATE_FILTER_ALPHA_FALL 0.55f
#define RATE_MIN_VALID_GPS 2.0f
#define RATE_ZERO_EPS_GPS 0.5f
#define RATE_NO_FLOW_RESET_SAMPLES 2u
#define DELTA_RATE_MIN_DT_S 0.05f
#define DELTA_RATE_MAX_DT_S 0.80f

static const char *TAG = "fill_sequence";

static float ewma(float prev, float sample, float alpha)
{
    if (prev <= 0.0f) return sample;
    return prev + alpha * (sample - prev);
}

static void seq_update_rate(filler_strategy_runtime_t *rt, const filler_strategy_tick_t *tick)
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
        }
    }
    rt->last_rel_g = rel_g;
    rt->last_rate_ts_us = tick->latest->ts_us;
}

static void seq_publish_snapshot(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                                 const filler_strategy_tick_t *tick, const char *phase)
{
    if (!rt || !env || !env->set_sample_telemetry || !tick) return;
    filler_strategy_sample_telemetry_t s = {
        .valid = true,
        .raw_rate_gps = rt->raw_rate_gps,
        .filtered_rate_gps = rt->filtered_rate_gps,
    };
    snprintf(s.strategy_name, sizeof(s.strategy_name), "%s", tick->strategy_name ? tick->strategy_name : "?");
    snprintf(s.gate_phase, sizeof(s.gate_phase), "%s", phase ? phase : "sequence");
    env->set_sample_telemetry(&s);
}

static uint32_t seq_total_ms(void)
{
    uint32_t total = 0;
    for (size_t i = 0; i < SEQ_STEP_COUNT; ++i) total += k_seq[i].duration_ms;
    return total;
}

// Map elapsed time to a step index; returns SEQ_STEP_COUNT when the script is done.
static size_t seq_step_for_elapsed(uint32_t elapsed_ms)
{
    uint32_t acc = 0;
    for (size_t i = 0; i < SEQ_STEP_COUNT; ++i) {
        acc += k_seq[i].duration_ms;
        if (elapsed_ms < acc) return i;
    }
    return SEQ_STEP_COUNT;
}

static void seq_command_step(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env, size_t step)
{
    if (step >= SEQ_STEP_COUNT) return;
    float gate = k_seq[step].gate_pct;
    if (fabsf(gate - rt->control_gate_cmd_pct) < 0.05f && rt->sample_count > 0) return;
    rt->control_gate_cmd_pct = gate;
    char label[24];
    snprintf(label, sizeof(label), "seq_s%u_%.0f", (unsigned)step, (double)gate);
    if (gate <= 0.05f) env->gate_close_label(label);
    else env->gate_set_percent_label(gate, label);
    ESP_LOGI(TAG, "sequence step %u -> gate %.0f%% for %u ms", (unsigned)step, (double)gate, (unsigned)k_seq[step].duration_ms);
}

static void seq_publish_summary(filler_strategy_runtime_t *rt, const filler_strategy_env_t *env,
                                const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !env->publish_fill_summary || !tick || !tick->latest || !tick->params) return;
    float final_rel_g = tick->latest->grams - rt->run_base_weight_g;
    filler_strategy_fill_summary_t summary = {
        .valid = true,
        .final_mass_g = final_rel_g,
        .target_g = (float)tick->params->target_grams,
        .fill_error_g = final_rel_g - (float)tick->params->target_grams,
        .fill_duration_s = (rt->fill_open_ts_us > 0) ? ((float)(tick->now_us - rt->fill_open_ts_us) / 1000000.0f) : 0.0f,
    };
    snprintf(summary.reason, sizeof(summary.reason), "%s", "sequence");
    env->publish_fill_summary(tick->run_id, tick->slot_idx, tick->preset_name, tick->strategy_name, tick->params, &summary);
}

static void sequence_on_enter(filler_strategy_runtime_t *rt, filler_state_t state, filler_state_t prev_state,
                              const filler_strategy_env_t *env, const filler_strategy_tick_t *tick)
{
    (void)prev_state;
    if (!rt || !env || !tick) return;
    if (state != FILLER_FILL) return;

    memset(rt, 0, sizeof(*rt));
    rt->fill_open_ts_us = tick->now_us;
    if (!env->jar_tare_get || !env->jar_tare_get(&rt->run_base_weight_g)) {
        rt->run_base_weight_g = tick->latest ? tick->latest->grams : 0.0f;
    }
    env->publish_fill_start(tick->run_id, tick->slot_idx, tick->strategy_name, tick->params,
                            tick->latest ? tick->latest->grams : 0.0f);
    ESP_LOGI(TAG, "scripted sequence start: %u steps, total %u ms", (unsigned)SEQ_STEP_COUNT, (unsigned)seq_total_ms());
    seq_command_step(rt, env, 0);
    seq_publish_snapshot(rt, env, tick, "seq_s0");
}

static filler_state_t sequence_step(filler_strategy_runtime_t *rt, filler_state_t state,
                                    const filler_strategy_env_t *env, const filler_strategy_tick_t *tick)
{
    if (!rt || !env || !tick || !tick->latest || !tick->params) return state;
    if (state != FILLER_FILL) return state;

    if (tick->new_sample && rt->sample_count < 255) rt->sample_count++;
    seq_update_rate(rt, tick);

    uint32_t elapsed_ms = (uint32_t)((tick->now_us - tick->state_enter_us) / 1000);
    size_t step = seq_step_for_elapsed(elapsed_ms);

    if (step >= SEQ_STEP_COUNT) {
        env->gate_close_label("seq_done");
        seq_publish_snapshot(rt, env, tick, "seq_done");
        seq_publish_summary(rt, env, tick);
        env->clear_sample_telemetry();
        uint8_t next = env->slot_next_idx(tick->slot_idx, tick->params->slots_total);
        ESP_LOGI(TAG, "sequence complete: final=%.1f g, slot %u -> %u",
                 (double)(tick->latest->grams - rt->run_base_weight_g), (unsigned)tick->slot_idx, (unsigned)next);
        env->set_slot(next);
        return FILLER_FIND_SLOT;
    }

    seq_command_step(rt, env, step);
    char phase[16];
    snprintf(phase, sizeof(phase), "seq_s%u", (unsigned)step);
    seq_publish_snapshot(rt, env, tick, phase);
    return state;
}

const filler_strategy_ops_t g_filler_strategy_sequence_ops = {
    .name = "sequence",
    .on_enter = sequence_on_enter,
    .step = sequence_step,
};
