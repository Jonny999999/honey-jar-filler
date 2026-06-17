#include "telemetry.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define TELEMETRY_QUEUE_LEN 128u

static QueueHandle_t s_queue;
static TaskHandle_t s_task;
static const char *TAG = "telemetry";

static void telemetry_copy_text(char *dst, size_t dst_len, const char *src)
{
    if (!dst || dst_len == 0) return;
    dst[0] = '\0';
    if (!src) return;
    snprintf(dst, dst_len, "%s", src);
}

static void telemetry_escape_json(const char *src, char *dst, size_t dst_len)
{
    if (!dst || dst_len == 0) return;
    dst[0] = '\0';
    if (!src) return;

    size_t w = 0;
    for (size_t i = 0; src[i] != '\0' && w + 2 < dst_len; ++i) {
        char c = src[i];
        if (c == '"' || c == '\\') {
            if (w + 2 >= dst_len) break;
            dst[w++] = '\\';
            dst[w++] = c;
        } else if ((unsigned char)c < 0x20) {
            if (w + 1 >= dst_len) break;
            dst[w++] = '_';
        } else {
            dst[w++] = c;
        }
    }
    dst[w] = '\0';
}

static void telemetry_emit_params_json(const app_params_t *params)
{
    // Run summaries embed the full active parameter snapshot so each recorded
    // experiment stays self-describing even after presets evolve later.
    if (!params) {
        printf("\"version\":0");
        return;
    }

    printf("\"version\":%u", (unsigned)params->version);
#define APP_PARAM_FLOAT(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope) \
    printf(",\"%s\":%.3f", #field, (double)params->field);
#define APP_PARAM_U32(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope)   \
    printf(",\"%s\":%u", #field, (unsigned)params->field);
#define APP_PARAM_U8(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope)    \
    printf(",\"%s\":%u", #field, (unsigned)params->field);
    APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8)
#undef APP_PARAM_FLOAT
#undef APP_PARAM_U32
#undef APP_PARAM_U8
}

const char *telemetry_kind_name(telemetry_kind_t kind)
{
    switch (kind) {
    case TELEMETRY_KIND_BOOT:      return "boot";
    case TELEMETRY_KIND_NOTE:      return "note";
    case TELEMETRY_KIND_PRESET:    return "preset";
    case TELEMETRY_KIND_RUN_START: return "run_start";
    case TELEMETRY_KIND_RUN_END:   return "run_end";
    case TELEMETRY_KIND_FILL_START:return "fill_start";
    case TELEMETRY_KIND_FILL_SUMMARY:return "fill_summary";
    case TELEMETRY_KIND_RUN_SUMMARY:return "run_summary";
    case TELEMETRY_KIND_STATE:     return "state";
    case TELEMETRY_KIND_FAULT:     return "fault";
    case TELEMETRY_KIND_GATE:      return "gate";
    case TELEMETRY_KIND_SAMPLE:    return "sample";
    default:                       return "?";
    }
}

void telemetry_record_init(telemetry_record_t *rec, telemetry_kind_t kind)
{
    if (!rec) return;
    *rec = (telemetry_record_t){0};
    rec->ts_us = esp_timer_get_time();
    rec->kind = kind;
    rec->slot_idx = -1;
}

bool telemetry_is_ready(void)
{
    return s_queue != NULL;
}

bool telemetry_publish(const telemetry_record_t *rec)
{
    if (!rec || !s_queue) return false;
    return xQueueSendToBack(s_queue, rec, 0) == pdPASS;
}

bool telemetry_publish_boot(const char *text)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_BOOT);
    telemetry_copy_text(rec.text, sizeof(rec.text), text);
    return telemetry_publish(&rec);
}

bool telemetry_publish_note(const char *text)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_NOTE);
    telemetry_copy_text(rec.text, sizeof(rec.text), text);
    return telemetry_publish(&rec);
}

bool telemetry_publish_preset(uint32_t preset_index, const char *preset_name)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_PRESET);
    rec.preset_index = preset_index;
    telemetry_copy_text(rec.text, sizeof(rec.text), preset_name);
    return telemetry_publish(&rec);
}

bool telemetry_publish_run_start(uint32_t run_id, int32_t slot_idx)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_RUN_START);
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    return telemetry_publish(&rec);
}

bool telemetry_publish_run_end(uint32_t run_id, int32_t slot_idx, const char *reason)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_RUN_END);
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    telemetry_copy_text(rec.text, sizeof(rec.text), reason);
    return telemetry_publish(&rec);
}

bool telemetry_publish_fill_start(uint32_t run_id,
                                  int32_t slot_idx,
                                  const char *preset_name,
                                  const char *strategy_name,
                                  const app_params_t *params,
                                  float base_weight_g,
                                  uint32_t scale_period_ms_cfg,
                                  uint32_t fsm_period_ms_cfg)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_FILL_START);
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    rec.weight_g = base_weight_g;
    rec.scale_period_ms_cfg = scale_period_ms_cfg;
    rec.fsm_period_ms_cfg = fsm_period_ms_cfg;
    if (params) {
        rec.target_g = params->target_grams;
        rec.params = *params;
    }
    telemetry_copy_text(rec.text, sizeof(rec.text), "start");
    telemetry_copy_text(rec.preset_name, sizeof(rec.preset_name), preset_name);
    telemetry_copy_text(rec.strategy_name, sizeof(rec.strategy_name), strategy_name);
    return telemetry_publish(&rec);
}

bool telemetry_publish_run_summary(uint32_t run_id,
                                   int32_t slot_idx,
                                   const char *result,
                                   const char *preset_name,
                                   const char *strategy_name,
                                   const app_params_t *params,
                                   float final_weight_g,
                                   float final_relative_fill_g,
                                   uint32_t scale_period_ms_cfg,
                                   uint32_t fsm_period_ms_cfg)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_RUN_SUMMARY);
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    rec.weight_g = final_weight_g;
    rec.relative_fill_g = final_relative_fill_g;
    rec.scale_period_ms_cfg = scale_period_ms_cfg;
    rec.fsm_period_ms_cfg = fsm_period_ms_cfg;
    if (params) {
        rec.target_g = params->target_grams;
        rec.params = *params;
    }
    telemetry_copy_text(rec.text, sizeof(rec.text), result);
    telemetry_copy_text(rec.preset_name, sizeof(rec.preset_name), preset_name);
    telemetry_copy_text(rec.strategy_name, sizeof(rec.strategy_name), strategy_name);
    return telemetry_publish(&rec);
}

bool telemetry_publish_state(uint32_t run_id, int32_t slot_idx, uint32_t state, const char *state_name)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_STATE);
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    rec.state = state;
    telemetry_copy_text(rec.text, sizeof(rec.text), state_name);
    return telemetry_publish(&rec);
}

bool telemetry_publish_fault(uint32_t run_id, int32_t slot_idx, const char *fault_name)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_FAULT);
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    telemetry_copy_text(rec.text, sizeof(rec.text), fault_name);
    return telemetry_publish(&rec);
}

bool telemetry_publish_gate(uint32_t run_id, int32_t slot_idx, float gate_pct, const char *label)
{
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_GATE);
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    rec.gate_pct = gate_pct;
    telemetry_copy_text(rec.text, sizeof(rec.text), label);
    return telemetry_publish(&rec);
}

bool telemetry_publish_sample(const telemetry_record_t *rec)
{
    if (!rec) return false;
    telemetry_record_t copy = *rec;
    if (copy.ts_us == 0) {
        copy.ts_us = esp_timer_get_time();
    }
    copy.kind = TELEMETRY_KIND_SAMPLE;
    return telemetry_publish(&copy);
}

bool telemetry_publish_sample_compact(int64_t ts_us,
                                      uint32_t run_id,
                                      int32_t slot_idx,
                                      uint32_t state,
                                      uint32_t target_g,
                                      float weight_g,
                                      float relative_fill_g,
                                      float gate_pct)
{
    // Common fast-path for sampled process data: callers provide the values
    // they already know, and telemetry owns the record layout/transport.
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_SAMPLE);
    rec.ts_us = ts_us;
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    rec.state = state;
    rec.target_g = target_g;
    rec.weight_g = weight_g;
    rec.relative_fill_g = relative_fill_g;
    rec.gate_pct = gate_pct;
    return telemetry_publish(&rec);
}

static void telemetry_emit_record(const telemetry_record_t *rec)
{
    char text_escaped[(TELEMETRY_TEXT_MAX * 2) + 1];
    char preset_escaped[(TELEMETRY_NAME_MAX * 2) + 1];
    char strategy_escaped[(TELEMETRY_NAME_MAX * 2) + 1];
    telemetry_escape_json(rec->text, text_escaped, sizeof(text_escaped));
    telemetry_escape_json(rec->preset_name, preset_escaped, sizeof(preset_escaped));
    telemetry_escape_json(rec->strategy_name, strategy_escaped, sizeof(strategy_escaped));

    // Hold stdout across the whole record so telemetry lines cannot interleave
    // with other console writers while a JSON object is still being emitted.
    flockfile(stdout);

    // Keep the on-wire JSON compact and event-specific so later parsing and
    // plotting scripts do not have to deal with many always-empty fields.
    switch (rec->kind) {
    case TELEMETRY_KIND_SAMPLE:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"sample\",\"run_id\":%" PRIu32
               ",\"slot_idx\":%" PRId32 ",\"state\":%" PRIu32 ",\"target_g\":%" PRIu32
               ",\"weight_g\":%.3f,\"relative_fill_g\":%.3f,\"gate_pct\":%.3f",
               rec->ts_us,
               rec->run_id,
               rec->slot_idx,
               rec->state,
               rec->target_g,
               (double)rec->weight_g,
               (double)rec->relative_fill_g,
               (double)rec->gate_pct);
        if (strategy_escaped[0] || text_escaped[0]) {
            printf(",\"strategy_name\":\"%s\",\"gate_phase\":\"%s\""
                   ",\"rate_raw_gps\":%.3f,\"rate_filtered_gps\":%.3f"
                   ",\"target_rate_gps\":%.3f,\"rate_error_gps\":%.3f"
                   ",\"predicted_remaining_g\":%.3f,\"measured_dead_time_s\":%.3f"
                   ",\"measured_post_close_gain_g\":%.3f"
                   ",\"learned_dead_time_s\":%.3f,\"learned_post_close_gain_g\":%.3f"
                   ",\"learned_fast_rate_gps\":%.3f,\"learned_slow_rate_gps\":%.3f"
                   ",\"adapted_near_close_g\":%.3f,\"adapted_close_early_g\":%.3f"
                   ",\"adapted_drip_wait_ms\":%.3f,\"refill_count\":%" PRIu32,
                   strategy_escaped,
                   text_escaped,
                   (double)rec->rate_raw_gps,
                   (double)rec->rate_filtered_gps,
                   (double)rec->target_rate_gps,
                   (double)rec->rate_error_gps,
                   (double)rec->predicted_remaining_g,
                   (double)rec->measured_dead_time_s,
                   (double)rec->measured_post_close_gain_g,
                   (double)rec->learned_dead_time_s,
                   (double)rec->learned_post_close_gain_g,
                   (double)rec->learned_fast_rate_gps,
                   (double)rec->learned_slow_rate_gps,
                   (double)rec->adapted_near_close_g,
                   (double)rec->adapted_close_early_g,
                   (double)rec->adapted_drip_wait_ms,
                   rec->refill_count);
        }
        printf("}\n");
        break;
    case TELEMETRY_KIND_STATE:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"state\",\"run_id\":%" PRIu32
               ",\"slot_idx\":%" PRId32 ",\"state\":%" PRIu32 ",\"text\":\"%s\"}\n",
               rec->ts_us,
               rec->run_id,
               rec->slot_idx,
               rec->state,
               text_escaped);
        break;
    case TELEMETRY_KIND_GATE:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"gate\",\"run_id\":%" PRIu32
               ",\"slot_idx\":%" PRId32 ",\"gate_pct\":%.3f,\"text\":\"%s\"}\n",
               rec->ts_us,
               rec->run_id,
               rec->slot_idx,
               (double)rec->gate_pct,
               text_escaped);
        break;
    case TELEMETRY_KIND_FAULT:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"fault\",\"run_id\":%" PRIu32
               ",\"slot_idx\":%" PRId32 ",\"text\":\"%s\"}\n",
               rec->ts_us,
               rec->run_id,
               rec->slot_idx,
               text_escaped);
        break;
    case TELEMETRY_KIND_RUN_START:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"run_start\",\"run_id\":%" PRIu32
               ",\"slot_idx\":%" PRId32 "}\n",
               rec->ts_us,
               rec->run_id,
               rec->slot_idx);
        break;
    case TELEMETRY_KIND_RUN_END:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"run_end\",\"run_id\":%" PRIu32
               ",\"slot_idx\":%" PRId32 ",\"text\":\"%s\"}\n",
               rec->ts_us,
               rec->run_id,
               rec->slot_idx,
               text_escaped);
        break;
    case TELEMETRY_KIND_FILL_START:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"fill_start\",\"run_id\":%" PRIu32
               ",\"slot_idx\":%" PRId32 ",\"text\":\"%s\",\"preset_name\":\"%s\""
               ",\"strategy_name\":\"%s\",\"target_g\":%" PRIu32
               ",\"base_weight_g\":%.3f,\"scale_period_ms_cfg\":%" PRIu32
               ",\"fsm_period_ms_cfg\":%" PRIu32 ",\"params\":{",
               rec->ts_us,
               rec->run_id,
               rec->slot_idx,
               text_escaped,
               preset_escaped,
               strategy_escaped,
               rec->target_g,
               (double)rec->weight_g,
               rec->scale_period_ms_cfg,
               rec->fsm_period_ms_cfg);
        telemetry_emit_params_json(&rec->params);
        printf("}}\n");
        break;
    case TELEMETRY_KIND_FILL_SUMMARY:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"fill_summary\",\"run_id\":%" PRIu32
               ",\"slot_idx\":%" PRId32 ",\"text\":\"%s\",\"preset_name\":\"%s\""
               ",\"strategy_name\":\"%s\",\"target_g\":%" PRIu32
               ",\"final_mass_g\":%.3f,\"final_relative_fill_g\":%.3f"
               ",\"fill_error_g\":%.3f,\"measured_dead_time_s\":%.3f"
               ",\"measured_post_close_gain_g\":%.3f,\"measured_fast_rate_gps\":%.3f"
               ",\"measured_slow_rate_gps\":%.3f,\"rate_at_close_gps\":%.3f"
               ",\"fill_duration_s\":%.3f,\"drip_wait_used_ms\":%.3f"
               ",\"refill_count\":%" PRIu32 ",\"next_dead_time_s\":%.3f"
               ",\"next_post_close_gain_g\":%.3f,\"next_fast_rate_gps\":%.3f"
               ",\"next_slow_rate_gps\":%.3f,\"next_near_close_g\":%.3f"
               ",\"next_close_early_g\":%.3f,\"next_drip_wait_ms\":%.3f"
               ",\"params\":{",
               rec->ts_us,
               rec->run_id,
               rec->slot_idx,
               text_escaped,
               preset_escaped,
               strategy_escaped,
               rec->target_g,
               (double)rec->weight_g,
               (double)rec->relative_fill_g,
               (double)rec->fill_error_g,
               (double)rec->measured_dead_time_s,
               (double)rec->measured_post_close_gain_g,
               (double)rec->measured_fast_rate_gps,
               (double)rec->measured_slow_rate_gps,
               (double)rec->rate_at_close_gps,
               (double)rec->fill_duration_s,
               (double)rec->adapted_drip_wait_ms,
               rec->refill_count,
               (double)rec->learned_dead_time_s,
               (double)rec->learned_post_close_gain_g,
               (double)rec->learned_fast_rate_gps,
               (double)rec->learned_slow_rate_gps,
               (double)rec->adapted_near_close_g,
               (double)rec->adapted_close_early_g,
               (double)rec->next_drip_wait_ms);
        telemetry_emit_params_json(&rec->params);
        printf("}}\n");
        break;
    case TELEMETRY_KIND_RUN_SUMMARY:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"run_summary\",\"run_id\":%" PRIu32
               ",\"slot_idx\":%" PRId32 ",\"text\":\"%s\",\"preset_name\":\"%s\""
               ",\"strategy_name\":\"%s\",\"target_g\":%" PRIu32
               ",\"final_weight_g\":%.3f,\"final_relative_fill_g\":%.3f"
               ",\"scale_period_ms_cfg\":%" PRIu32 ",\"fsm_period_ms_cfg\":%" PRIu32
               ",\"params\":{",
               rec->ts_us,
               rec->run_id,
               rec->slot_idx,
               text_escaped,
               preset_escaped,
               strategy_escaped,
               rec->target_g,
               (double)rec->weight_g,
               (double)rec->relative_fill_g,
               rec->scale_period_ms_cfg,
               rec->fsm_period_ms_cfg);
        telemetry_emit_params_json(&rec->params);
        printf("}}\n");
        break;
    case TELEMETRY_KIND_PRESET:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"preset\",\"preset_index\":%" PRIu32
               ",\"text\":\"%s\"}\n",
               rec->ts_us,
               rec->preset_index,
               text_escaped);
        break;
    case TELEMETRY_KIND_BOOT:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"boot\",\"text\":\"%s\"}\n",
               rec->ts_us,
               text_escaped);
        break;
    case TELEMETRY_KIND_NOTE:
    default:
        printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"%s\",\"text\":\"%s\"}\n",
               rec->ts_us,
               telemetry_kind_name(rec->kind),
               text_escaped);
        break;
    }
    fflush(stdout);
    funlockfile(stdout);
}

static void telemetry_task(void *arg)
{
    (void)arg;

    telemetry_record_t rec;
    for (;;) {
        if (xQueueReceive(s_queue, &rec, portMAX_DELAY) == pdPASS) {
            telemetry_emit_record(&rec);
        }
    }
}

esp_err_t telemetry_start_task(UBaseType_t prio, BaseType_t core)
{
    if (s_task) return ESP_OK;

    s_queue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(telemetry_record_t));
    if (!s_queue) {
        ESP_LOGE(TAG, "queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(telemetry_task,
                                            "telemetry",
                                            4096,
                                            NULL,
                                            prio,
                                            &s_task,
                                            core);
    if (ok != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        s_task = NULL;
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "started");
    return ESP_OK;
}
