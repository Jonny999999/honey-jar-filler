#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TELEMETRY_TEXT_MAX 24u
#define TELEMETRY_NAME_MAX 24u

typedef enum {
    TELEMETRY_KIND_BOOT = 0,
    TELEMETRY_KIND_NOTE,
    TELEMETRY_KIND_PRESET,
    TELEMETRY_KIND_RUN_START,
    TELEMETRY_KIND_RUN_END,
    TELEMETRY_KIND_FILL_START,
    TELEMETRY_KIND_FILL_SUMMARY,
    TELEMETRY_KIND_RUN_SUMMARY,
    TELEMETRY_KIND_STATE,
    TELEMETRY_KIND_FAULT,
    TELEMETRY_KIND_GATE,
    TELEMETRY_KIND_SAMPLE,
} telemetry_kind_t;

typedef struct {
    int64_t ts_us;
    telemetry_kind_t kind;
    uint32_t run_id;
    int32_t slot_idx;
    uint32_t state;
    uint32_t preset_index;
    uint32_t target_g;
    uint32_t refill_count;
    uint32_t scale_period_ms_cfg;
    uint32_t fsm_period_ms_cfg;
    float weight_g;
    float relative_fill_g;
    float gate_pct;
    float rate_raw_gps;
    float rate_filtered_gps;
    float target_rate_gps;
    float rate_error_gps;
    float predicted_remaining_g;
    float measured_dead_time_s;
    float rate_at_close_gps;
    float fill_duration_s;
    float learned_dead_time_s;
    float learned_post_close_gain_g;
    float learned_fast_rate_gps;
    float learned_slow_rate_gps;
    float adapted_near_close_g;
    float adapted_close_early_g;
    float adapted_drip_wait_ms;
    float next_drip_wait_ms;
    float fill_error_g;
    float measured_post_close_gain_g;
    float measured_fast_rate_gps;
    float measured_slow_rate_gps;
    char preset_name[TELEMETRY_NAME_MAX];
    char strategy_name[TELEMETRY_NAME_MAX];
    app_params_t params;
    char text[TELEMETRY_TEXT_MAX];
} telemetry_record_t;

esp_err_t telemetry_start_task(UBaseType_t prio, BaseType_t core);
bool telemetry_is_ready(void);
bool telemetry_publish(const telemetry_record_t *rec);

void telemetry_record_init(telemetry_record_t *rec, telemetry_kind_t kind);
bool telemetry_publish_boot(const char *text);
bool telemetry_publish_note(const char *text);
bool telemetry_publish_preset(uint32_t preset_index, const char *preset_name);
bool telemetry_publish_run_start(uint32_t run_id, int32_t slot_idx);
bool telemetry_publish_run_end(uint32_t run_id, int32_t slot_idx, const char *reason);
bool telemetry_publish_fill_start(uint32_t run_id,
                                  int32_t slot_idx,
                                  const char *preset_name,
                                  const char *strategy_name,
                                  const app_params_t *params,
                                  float base_weight_g,
                                  uint32_t scale_period_ms_cfg,
                                  uint32_t fsm_period_ms_cfg);
bool telemetry_publish_run_summary(uint32_t run_id,
                                   int32_t slot_idx,
                                   const char *result,
                                   const char *preset_name,
                                   const char *strategy_name,
                                   const app_params_t *params,
                                   float final_weight_g,
                                   float final_relative_fill_g,
                                   uint32_t scale_period_ms_cfg,
                                   uint32_t fsm_period_ms_cfg);
bool telemetry_publish_state(uint32_t run_id, int32_t slot_idx, uint32_t state, const char *state_name);
bool telemetry_publish_fault(uint32_t run_id, int32_t slot_idx, const char *fault_name);
bool telemetry_publish_gate(uint32_t run_id, int32_t slot_idx, float gate_pct, const char *label);
bool telemetry_publish_sample(const telemetry_record_t *rec);
bool telemetry_publish_sample_compact(int64_t ts_us,
                                      uint32_t run_id,
                                      int32_t slot_idx,
                                      uint32_t state,
                                      uint32_t target_g,
                                      float weight_g,
                                      float relative_fill_g,
                                      float gate_pct);

const char *telemetry_kind_name(telemetry_kind_t kind);

#ifdef __cplusplus
}
#endif

#endif // TELEMETRY_H
