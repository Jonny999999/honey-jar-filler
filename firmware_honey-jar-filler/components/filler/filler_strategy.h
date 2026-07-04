#ifndef FILLER_STRATEGY_H
#define FILLER_STRATEGY_H

#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "filler_fsm.h"
#include "scale_hx711.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool near_close_logged;
    float close_early_relax_g;
    uint8_t cnt_near_close;
    uint8_t cnt_close_early;
    uint8_t cnt_target;
    uint8_t cnt_under;
    uint8_t cnt_over;
    uint8_t no_flow_count;
    uint8_t sample_count;
    uint8_t cnt_safe_rate;
    bool response_detected;
    bool first_close_seen;
    uint8_t active_preset_index;
    uint8_t refill_count;
    int64_t fill_open_ts_us;
    int64_t fill_eval_after_ts_us;
    int64_t first_response_ts_us;
    int64_t first_close_ts_us;
    int64_t last_rate_ts_us;
    int64_t last_gain_ts_us;
    int64_t safe_reduce_ts_us;
    float run_base_weight_g;
    float last_rel_g;
    float last_post_close_gain_g;
    float raw_rate_gps;
    float rate_2sample_gps;
    float rate_4sample_gps;
    float filtered_rate_gps;
    float filtered_rate_medium_gps;
    float filtered_rate_slow_gps;
    float fast_rate_sum_gps;
    float slow_rate_sum_gps;
    uint16_t fast_rate_count;
    uint16_t slow_rate_count;
    float gate_gain_sum;
    uint16_t gate_gain_count;
    uint8_t rate_hist_count;
    float rate_hist_rel_g[5];
    int64_t rate_hist_ts_us[5];
    float measured_dead_time_s;
    float measured_post_close_gain_g;
    float measured_near_close_gain_g;
    float rate_at_close_gps;
    float rate_2sample_at_close_gps;
    float rate_4sample_at_close_gps;
    float filtered_rate_medium_at_close_gps;
    float filtered_rate_slow_at_close_gps;
    float learned_dead_time_s;
    float learned_post_close_gain_g;
    float learned_fast_rate_gps;
    float learned_slow_rate_gps;
    float learned_gate_gain_gps_per_pct;
    float learned_finish_trim_g;
    float learned_fast_start_gate_pct;
    float learned_slow_start_gate_pct;
    float learned_near_close_bias_g;
    float adapted_near_close_g;
    float adapted_close_early_g;
    float adapted_drip_wait_ms;
    float predicted_remaining_g;
    float rel_at_first_close_g;
    float rel_at_near_close_g;
    float near_close_transition_estimate_g;
    float near_close_relax_g;
    bool measured_near_close_valid;
    float target_g;
    float control_gate_cmd_pct;
    float gate_at_slow_entry_pct;
    float gate_at_close_pct;
    float control_target_rate_gps;
    float control_rate_error_gps;
    int64_t control_last_update_us;
    float manual_gate_pct;
    bool manual_pending_advance;
    uint8_t manual_next_slot;
} filler_strategy_runtime_t;

typedef struct {
    bool valid;
    float final_mass_g;
    float target_g;
    float fill_error_g;
    float measured_dead_time_s;
    float measured_post_close_gain_g;
    float measured_near_close_gain_g;
    float measured_fast_rate_gps;
    float measured_slow_rate_gps;
    float rate_at_close_gps;
    float rate_2sample_at_close_gps;
    float rate_4sample_at_close_gps;
    float filtered_rate_medium_at_close_gps;
    float filtered_rate_slow_at_close_gps;
    float fill_duration_s;
    float drip_wait_used_ms;
    uint32_t refill_count;
    // "used_*" captures the adaptive estimates that were active for this fill
    // before any new learning update was applied at the end.
    float used_dead_time_s;
    float used_post_close_gain_g;
    float used_fast_rate_gps;
    float used_slow_rate_gps;
    float used_near_close_bias_g;
    float used_near_close_g;
    float used_close_early_g;
    // "next_*" values are meaningful for learning strategies. Fixed/manual
    // strategies leave them at zero because no cross-run learning is applied.
    float next_dead_time_s;
    float next_post_close_gain_g;
    float next_fast_rate_gps;
    float next_slow_rate_gps;
    float next_near_close_bias_g;
    float next_near_close_g;
    float next_close_early_g;
    float next_drip_wait_ms;
    char reason[24];
} filler_strategy_fill_summary_t;

typedef struct {
    bool (*require_fresh_or_fault)(const char *ctx, const scale_latest_t *s, filler_state_t *state);
    void (*set_fault)(filler_fault_t flt);
    bool (*jar_tare_get)(float *out_grams);
    bool (*take_start_request)(void);
    int32_t (*take_manual_gate_delta)(void);
    void (*gate_close_label)(const char *label);
    void (*gate_set_percent_label)(float pct, const char *label);
    void (*set_slot)(uint8_t idx);
    uint8_t (*slot_next_idx)(uint8_t idx, uint8_t configured_slots);
    void (*publish_fill_start)(uint32_t run_id,
                               uint8_t slot_idx,
                               const char *strategy_name,
                               const app_params_t *params,
                               float base_weight_g);
    void (*set_sample_telemetry)(const filler_strategy_sample_telemetry_t *sample);
    void (*clear_sample_telemetry)(void);
    void (*publish_fill_summary)(uint32_t run_id,
                                 uint8_t slot_idx,
                                 const char *preset_name,
                                 const char *strategy_name,
                                 const app_params_t *params,
                                 const filler_strategy_fill_summary_t *summary);
} filler_strategy_env_t;

typedef struct {
    int64_t now_us;
    int64_t state_enter_us;
    uint32_t run_id;
    uint8_t slot_idx;
    bool new_sample;
    const scale_latest_t *latest;
    const app_params_t *params;
    uint8_t preset_index;
    const char *preset_name;
    const char *strategy_name;
} filler_strategy_tick_t;

typedef struct {
    const char *name;
    void (*on_enter)(filler_strategy_runtime_t *rt,
                     filler_state_t state,
                     filler_state_t prev_state,
                     const filler_strategy_env_t *env,
                     const filler_strategy_tick_t *tick);
    filler_state_t (*step)(filler_strategy_runtime_t *rt,
                           filler_state_t state,
                           const filler_strategy_env_t *env,
                           const filler_strategy_tick_t *tick);
} filler_strategy_ops_t;

const filler_strategy_ops_t *filler_strategy_ops_for_app(app_fill_strategy_t strategy);
const filler_strategy_ops_t *filler_strategy_active_ops(void);

#ifdef __cplusplus
}
#endif

#endif // FILLER_STRATEGY_H
