#include "telemetry.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define TELEMETRY_QUEUE_LEN 64u

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

const char *telemetry_kind_name(telemetry_kind_t kind)
{
    switch (kind) {
    case TELEMETRY_KIND_BOOT:      return "boot";
    case TELEMETRY_KIND_NOTE:      return "note";
    case TELEMETRY_KIND_PRESET:    return "preset";
    case TELEMETRY_KIND_RUN_START: return "run_start";
    case TELEMETRY_KIND_RUN_END:   return "run_end";
    case TELEMETRY_KIND_STATE:     return "state";
    case TELEMETRY_KIND_FAULT:     return "fault";
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
    rec->raw = 0;
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
    rec.u32_0 = preset_index;
    telemetry_copy_text(rec.text, sizeof(rec.text), preset_name);
    return telemetry_publish(&rec);
}

static void telemetry_emit_record(const telemetry_record_t *rec)
{
    char text_escaped[(TELEMETRY_TEXT_MAX * 2) + 1];
    telemetry_escape_json(rec->text, text_escaped, sizeof(text_escaped));

    printf("TEL {\"ts_us\":%" PRId64 ",\"kind\":\"%s\",\"run_id\":%" PRIu32
           ",\"slot_idx\":%" PRId32 ",\"raw\":%" PRId32 ",\"weight_g\":%.3f"
           ",\"value0\":%.3f,\"value1\":%.3f,\"u32_0\":%" PRIu32
           ",\"u32_1\":%" PRIu32 ",\"text\":\"%s\"}\n",
           rec->ts_us,
           telemetry_kind_name(rec->kind),
           rec->run_id,
           rec->slot_idx,
           rec->raw,
           (double)rec->weight_g,
           (double)rec->value0,
           (double)rec->value1,
           rec->u32_0,
           rec->u32_1,
           text_escaped);
    fflush(stdout);
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
