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
} filler_strategy_runtime_t;

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
} filler_strategy_env_t;

typedef struct {
    int64_t now_us;
    int64_t state_enter_us;
    uint32_t run_id;
    uint8_t slot_idx;
    bool new_sample;
    const scale_latest_t *latest;
    const app_params_t *params;
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
