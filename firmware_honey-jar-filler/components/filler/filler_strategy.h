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
    uint8_t sample_count;
    bool response_detected;
    bool first_close_seen;
    uint8_t active_preset_index;
    uint8_t refill_count;
    int64_t fill_open_ts_us;
    int64_t first_response_ts_us;
    int64_t first_close_ts_us;
    int64_t last_rate_ts_us;
    int64_t last_gain_ts_us;
    float run_base_weight_g;
    float last_rel_g;
    float last_post_close_gain_g;
    float raw_rate_gps;
    float filtered_rate_gps;
    float fast_rate_sum_gps;
    float slow_rate_sum_gps;
    uint16_t fast_rate_count;
    uint16_t slow_rate_count;
    float measured_dead_time_s;
    float measured_post_close_gain_g;
    float learned_dead_time_s;
    float learned_post_close_gain_g;
    float learned_fast_rate_gps;
    float learned_slow_rate_gps;
    float adapted_near_close_g;
    float adapted_close_early_g;
    float adapted_drip_wait_ms;
    float predicted_remaining_g;
    float rel_at_first_close_g;
    float target_g;
} filler_strategy_runtime_t;

typedef struct {
    bool valid;
    float final_mass_g;
    float target_g;
    float fill_error_g;
    float measured_dead_time_s;
    float measured_post_close_gain_g;
    float measured_fast_rate_gps;
    float measured_slow_rate_gps;
    float drip_wait_used_ms;
    uint32_t refill_count;
    float next_dead_time_s;
    float next_post_close_gain_g;
    float next_fast_rate_gps;
    float next_slow_rate_gps;
    float next_near_close_g;
    float next_close_early_g;
    float next_drip_wait_ms;
    char reason[24];
} filler_strategy_fill_summary_t;

typedef struct {
    bool (*require_fresh_or_fault)(const char *ctx, const scale_latest_t *s, filler_state_t *state);
    void (*set_fault)(filler_fault_t flt);
    bool (*jar_tare_get)(float *out_grams);
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
