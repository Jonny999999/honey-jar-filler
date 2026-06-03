#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TELEMETRY_TEXT_MAX 24u

typedef enum {
    TELEMETRY_KIND_BOOT = 0,
    TELEMETRY_KIND_NOTE,
    TELEMETRY_KIND_PRESET,
    TELEMETRY_KIND_RUN_START,
    TELEMETRY_KIND_RUN_END,
    TELEMETRY_KIND_STATE,
    TELEMETRY_KIND_FAULT,
    TELEMETRY_KIND_SAMPLE,
} telemetry_kind_t;

typedef struct {
    int64_t ts_us;
    telemetry_kind_t kind;
    uint32_t run_id;
    int32_t slot_idx;
    int32_t raw;
    float weight_g;
    float value0;
    float value1;
    uint32_t u32_0;
    uint32_t u32_1;
    char text[TELEMETRY_TEXT_MAX];
} telemetry_record_t;

esp_err_t telemetry_start_task(UBaseType_t prio, BaseType_t core);
bool telemetry_is_ready(void);
bool telemetry_publish(const telemetry_record_t *rec);

void telemetry_record_init(telemetry_record_t *rec, telemetry_kind_t kind);
bool telemetry_publish_boot(const char *text);
bool telemetry_publish_note(const char *text);
bool telemetry_publish_preset(uint32_t preset_index, const char *preset_name);

const char *telemetry_kind_name(telemetry_kind_t kind);

#ifdef __cplusplus
}
#endif

#endif // TELEMETRY_H
