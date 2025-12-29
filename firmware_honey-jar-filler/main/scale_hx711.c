#include "esp_log.h"
#include <inttypes.h>
#include "esp_timer.h"

#include "scale_hx711.h"
#include "nvs_flash.h"
#include "nvs.h"

#define NVS_NAMESPACE  "hx711"
#define NVS_KEY_OFFS   "offset"
#define NVS_KEY_SCALE  "scale"
#define NVS_KEY_VALID  "valid"

#define MUTEX_TIMEOUT_MS 5000

static const char *TAG = "scale_hx711";


static inline bool scale_lock(scale_hx711_t *s, uint32_t timeout_ms)
{
    return s->mtx && xSemaphoreTake(s->mtx, pdMS_TO_TICKS(timeout_ms)) == pdPASS;
}
static inline void scale_unlock(scale_hx711_t *s)
{
    if (s->mtx) xSemaphoreGive(s->mtx);
}


static esp_err_t scale_nvs_load(scale_hx711_t *s)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    int32_t off = 0;
    float   sc  = 0.0f;
    uint8_t ok  = 0;

    // offset
    err = nvs_get_i32(h, NVS_KEY_OFFS, &off);
    if (err != ESP_OK) { nvs_close(h); return err; }

    // scale (float via blob)
    size_t len = sizeof(sc);
    err = nvs_get_blob(h, NVS_KEY_SCALE, &sc, &len);
    if (err != ESP_OK || len != sizeof(sc)) { nvs_close(h); return (err == ESP_OK) ? ESP_ERR_INVALID_SIZE : err; }

    // valid flag
    err = nvs_get_u8(h, NVS_KEY_VALID, &ok);
    nvs_close(h);
    if (err != ESP_OK) return err;

    s->offset_raw = off;
    s->scale_cpg  = (sc != 0.0f) ? sc : 1.0f;
    s->calibrated = ok ? true : false;

    ESP_LOGI(TAG, "NVS: loaded offset=%" PRId32 ", scale=%.6f, calibrated=%d",
             s->offset_raw, s->scale_cpg, s->calibrated);
    return ESP_OK;
}


#define DELAY_BEFORE_NVS_WRITE 1

static esp_err_t scale_nvs_save(const scale_hx711_t *s)
{
    #if DELAY_BEFORE_NVS_WRITE
    ESP_LOGI(TAG, "waiting 2s before open");
    vTaskDelay(pdMS_TO_TICKS(1000));
    #endif
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    uint8_t ok = s->calibrated ? 1 : 0;

    err = nvs_set_i32(h, NVS_KEY_OFFS, s->offset_raw);
    if (err == ESP_OK) err = nvs_set_blob(h, NVS_KEY_SCALE, &s->scale_cpg, sizeof(s->scale_cpg));
    if (err == ESP_OK) err = nvs_set_u8(h, NVS_KEY_VALID, ok);

    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "NVS: saved  offset=%" PRId32 ", scale=%.6f, calibrated=%d",
                 s->offset_raw, s->scale_cpg, s->calibrated);
    } else {
        ESP_LOGW(TAG, "NVS: save failed: %s", esp_err_to_name(err));
    }
    return err;
}



// helper: average N samples from HX711
static esp_err_t read_avg_counts(scale_hx711_t *s, uint16_t samples, int32_t *avg_out)
{
    if (!avg_out || samples == 0) return ESP_ERR_INVALID_ARG;
    // mutex to prevent collisions when e.g. running tare or calibration while polling
    if (!scale_lock(s, MUTEX_TIMEOUT_MS)) return ESP_ERR_TIMEOUT;

    // Make sure data is ready at least once before reading loop
    esp_err_t err = hx711_wait(&s->dev, 1000 /* ms timeout */);
    if (err != ESP_OK) {
        scale_unlock(s);
        ESP_LOGE(TAG, "read_avg_counts: hx711_wait failed (%d)", err);
        return err;
    }

    // use the upstream driver's averaging helper if available:
    err = hx711_read_average(&s->dev, samples, avg_out);
    scale_unlock(s);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hx711_read_average failed (%d)", err);
        return err;
    }

    return ESP_OK;
}



esp_err_t scale_hx711_init(scale_hx711_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;

    // create per-instance mutex once
    if (!s->mtx) {
        s->mtx = xSemaphoreCreateMutex();
        if (!s->mtx) return ESP_ERR_NO_MEM;
    }

    // reset struct defaults
    s->offset_raw  = 0;
    s->scale_cpg   = 1.0f;  // counts per gram fallback
    s->calibrated  = false;

    s->dev.dout    = CONFIG_HX711_DT_GPIO;
    s->dev.pd_sck  = CONFIG_HX711_SCK_GPIO;
    s->dev.gain    = HX711_GAIN_A_128; // channel A, gain 128 (typical load cell mode)

    esp_err_t err = hx711_init(&s->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init: hx711_init failed (%d)", err);
        return err;
    }

    // sanity: wait once so we know chip is alive
    err = hx711_wait(&s->dev, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init: hx711_wait failed (%d)", err);
        return err;
    }

    // try to load previous calibration from nvs
    ESP_LOGI(TAG, "init: trying to load old calibration from nvs...");
    (void)scale_nvs_load(s); // keeps defaults unchanged if not found

    ESP_LOGI(TAG, "init: HX711 init successful (DT=%d, SCK=%d)",
             (int)CONFIG_HX711_DT_GPIO, (int)CONFIG_HX711_SCK_GPIO);

    return ESP_OK;
}



esp_err_t scale_hx711_tare(scale_hx711_t *s, uint16_t samples)
{
    if (!s) return ESP_ERR_INVALID_ARG;

    int32_t avg_raw = 0;
    esp_err_t err = read_avg_counts(s, samples, &avg_raw);
    if (err != ESP_OK) return err;

    if (!scale_lock(s, MUTEX_TIMEOUT_MS)) return ESP_ERR_TIMEOUT;
    s->offset_raw = avg_raw;
    scale_unlock(s);
    ESP_LOGI(TAG, "TARE: offset_raw=%" PRId32 " (avg of %u)", avg_raw, samples);

    // update persistent calibration in nvs
    (void)scale_nvs_save(s);

    return ESP_OK;
}



esp_err_t scale_hx711_calibrate(scale_hx711_t *s,
                                float grams_on_scale,
                                uint16_t samples)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    if (grams_on_scale <= 0.0f) return ESP_ERR_INVALID_ARG;

    int32_t avg_raw = 0;
    esp_err_t err = read_avg_counts(s, samples, &avg_raw);
    if (err != ESP_OK) return err;

    int32_t diff = avg_raw - s->offset_raw;
    if (diff == 0) {
        ESP_LOGW(TAG, "CALIBRATE: diff=0, check weight?");
        diff = 1; // avoid div0
    }

    if (!scale_lock(s, MUTEX_TIMEOUT_MS)) return ESP_ERR_TIMEOUT;
    s->scale_cpg  = (float)diff / grams_on_scale; // counts per gram
    s->calibrated = true;
    scale_unlock(s);

    ESP_LOGI(TAG,
        "CALIBRATE: diff=%" PRId32 " counts @ %.3f g -> scale=%.6f counts/g",
        diff, grams_on_scale, s->scale_cpg);

    // save new calibration to nvs (persistent - gets loaded after next startup/init)
    (void)scale_nvs_save(s);

    return ESP_OK;
}



esp_err_t scale_hx711_read_raw(scale_hx711_t *s,
                               uint16_t samples,
                               int32_t *raw_out)
{
    if (!s || !raw_out) return ESP_ERR_INVALID_ARG;

    int32_t avg_raw = 0;
    esp_err_t err = read_avg_counts(s, samples, &avg_raw);
    if (err != ESP_OK) return err;

    *raw_out = avg_raw;
    return ESP_OK;
}



esp_err_t scale_hx711_read_grams(scale_hx711_t *s,
                                 uint16_t samples,
                                 int32_t *raw_out,
                                 float *grams_out,
                                 bool  *is_valid)
{
    if (!s) return ESP_ERR_INVALID_ARG;

    int32_t avg_raw = 0;
    esp_err_t err = read_avg_counts(s, samples, &avg_raw);
    if (err != ESP_OK) return err;

    if (raw_out) {
        *raw_out = avg_raw;
    }

    if (grams_out) {
        float grams = (float)(avg_raw - s->offset_raw);
        if (s->scale_cpg != 0.0f) {
            grams /= s->scale_cpg;
        }
        *grams_out = grams;
    }

    if (is_valid) {
        *is_valid = s->calibrated;
    }

    return ESP_OK;
}






// ---------- Poll task ----------
typedef struct {
    scale_hx711_t *s;
    uint16_t samples_avg;
    TickType_t period;
    QueueHandle_t q;
} poll_cfg_t;

static void task_scale_poll(void *arg)
{
    poll_cfg_t cfg = *(poll_cfg_t*)arg;  // copy config locally
    vPortFree(arg);

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = cfg.period;

    // rate-limit warnings
    uint32_t overrun_count = 0;
    int64_t last_warn_us = 0;

    for (;;) {
        // --- work start
        int64_t t0 = esp_timer_get_time();

        int32_t raw = 0; float g = 0.0f; bool ok = false;
        if (scale_hx711_read_grams(cfg.s, cfg.samples_avg, &raw, &g, &ok) == ESP_OK) {
            scale_sample_t msg = {
                .raw = raw,
                .grams = g,
                .valid = ok,
                .ts_us = esp_timer_get_time()
            };
            if (xQueueSend(cfg.q, &msg, 0) != pdPASS) {
                // if queue_len == 1 this overwrites; otherwise it's a no-op
                xQueueOverwrite(cfg.q, &msg);
            }
        }

        // --- compute loop duration / overrun
        int64_t t1 = esp_timer_get_time();
        int64_t loop_us = t1 - t0;

        // Check whether next wake time is already in the past (overrun)
        TickType_t next = last + period;
        TickType_t now_ticks = xTaskGetTickCount();
        bool overrun = (now_ticks >= next);

        if (overrun) {
            overrun_count++;
            // throttle WARNs to ~1/s
            int64_t now_us = t1;
            if (now_us - last_warn_us > 20e6) {
                last_warn_us = now_us;

                // Estimate minimum viable period based on samples_avg and HX711 rate.

                ESP_LOGW(TAG,
                         "task_scale_poll can't keep up: actual-period=%lld ms, target-period=%u ms, samples_avg=%u, overruns=%u. "
                         "\n  Hint: decrease 'samples_avg' OR increase 'period'. "
                         "actual-single-readout-time=%u ms",
                         (long long)loop_us/1000,
                         (unsigned) (period * 1000 / configTICK_RATE_HZ),
                         (unsigned) cfg.samples_avg,
                         (unsigned) overrun_count,
                         (unsigned)loop_us/cfg.samples_avg/1000);
            }
        }

        // Sleep until next tick boundary (if late, this returns immediately)
        vTaskDelayUntil(&last, period);
    }
}





esp_err_t scale_hx711_start_poll(scale_hx711_t *s,
                                 uint16_t samples_avg,
                                 TickType_t period,
                                 UBaseType_t queue_len,
                                 QueueHandle_t *out_queue)
{
    if (!s || !out_queue || queue_len == 0 || samples_avg == 0) return ESP_ERR_INVALID_ARG;

    QueueHandle_t q = xQueueCreate(queue_len, sizeof(scale_sample_t));
    if (!q) return ESP_ERR_NO_MEM;

    // Heap-copy the config for the task
    poll_cfg_t *cfg = pvPortMalloc(sizeof(*cfg));
    if (!cfg) { vQueueDelete(q); return ESP_ERR_NO_MEM; }
    cfg->s = s;
    cfg->samples_avg = samples_avg;
    cfg->period = period;
    cfg->q = q;

    // Pin to core 1, low priority
    if (xTaskCreatePinnedToCore(task_scale_poll, "scale_poll", 4096, cfg, 2, NULL, 1) != pdPASS) {
        vPortFree(cfg);
        vQueueDelete(q);
        return ESP_ERR_NO_MEM;
    }

    *out_queue = q;
    return ESP_OK;
}