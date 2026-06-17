#ifndef FILLER_FSM_H
#define FILLER_FSM_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FILLER_IDLE = 0,
    FILLER_FIND_SLOT,
    FILLER_VERIFY_EMPTY,
    FILLER_FILL,
    FILLER_SLOT_SETTLE,
    FILLER_DRIP_WAIT,
    FILLER_VERIFY_TARGET,
    FILLER_DONE,
    FILLER_FAULT
} filler_state_t;

typedef enum {
    FLT_NONE = 0,
    FLT_NO_JAR,
    FLT_WEIGHT_RANGE,
    FLT_MOTOR_TIMEOUT,
    FLT_SERVO_TIMEOUT,
    FLT_SCALE_TIMEOUT,
    FLT_EMPTY_HONEY,
    FLT_USER_ABORT
} filler_fault_t;

#define FILLER_STRATEGY_NAME_MAX 24u
#define FILLER_GATE_PHASE_MAX    16u

typedef struct {
    bool valid;
    char strategy_name[FILLER_STRATEGY_NAME_MAX];
    char gate_phase[FILLER_GATE_PHASE_MAX];
    float raw_rate_gps;
    float rate_2sample_gps;
    float rate_4sample_gps;
    float filtered_rate_gps;
    float filtered_rate_medium_gps;
    float filtered_rate_slow_gps;
    float target_rate_gps;
    float rate_error_gps;
    float predicted_remaining_g;
    float measured_dead_time_s;
    float measured_post_close_gain_g;
    float measured_near_close_gain_g;
    // Learning / experimental strategies publish learned or live-adapted
    // values here. Fixed heuristic mode reuses the same fields for the
    // currently active static thresholds/waits so comparison charts can share
    // common axes/labels.
    float learned_dead_time_s;
    float learned_post_close_gain_g;
    float learned_fast_rate_gps;
    float learned_slow_rate_gps;
    float learned_near_close_bias_g;
    float adapted_near_close_g;
    float adapted_close_early_g;
    float adapted_drip_wait_ms;
    uint32_t refill_count;
} filler_strategy_sample_telemetry_t;

void filler_start_task(UBaseType_t prio, BaseType_t core); // creates the FSM task
void filler_request_start(void);  // set an atomic flag
void filler_request_abort(void);  // set an atomic flag
void filler_request_manual_gate_delta(int32_t delta_pct_steps); // accumulate manual gate change requests
filler_state_t filler_get_state(void);
uint8_t filler_get_slot_idx(void);
filler_fault_t filler_get_fault(void);
uint32_t filler_get_run_id(void);
float filler_get_gate_percent(void);
bool filler_get_jar_tare(float *out_grams);
bool filler_get_strategy_sample_telemetry(filler_strategy_sample_telemetry_t *out);
const char *filler_state_name(filler_state_t st);
const char *filler_fault_name(filler_fault_t flt);

#ifdef __cplusplus
}
#endif

#endif // FILLER_FSM_H
