#include "esp_log.h"
#include <inttypes.h>
#include <stdio.h>
#include "esp_timer.h"

#include "scale_hx711.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "app.h"
#include "filler_fsm.h"
#include "telemetry.h"

#define NVS_NAMESPACE  "hx711"
#define NVS_KEY_OFFS   "offset"
#define NVS_KEY_SCALE  "scale"
#define NVS_KEY_VALID  "valid"

#define MUTEX_TIMEOUT_MS 5000

static const char *TAG = "scale_hx711";

// Latest snapshot storage (guarded by mutex to avoid torn reads).
static SemaphoreHandle_t s_latest_mtx;
static scale_latest_t s_latest;
static scale_hx711_t *s_default;

void scale_latest_set(const scale_latest_t *s)
{
    if (!s) return;

    if (!s_latest_mtx) {
        s_latest_mtx = xSemaphoreCreateMutex();
        if (!s_latest_mtx) {
            s_latest = *s;
            return;
        }
    }

    if (xSemaphoreTake(s_latest_mtx, portMAX_DELAY) != pdPASS) return;
    s_latest = *s;
    xSemaphoreGive(s_latest_mtx);
}

void scale_latest_get(scale_latest_t *out)
{
    if (!out) return;

    if (!s_latest_mtx) {
        *out = s_latest;
        return;
    }

    if (xSemaphoreTake(s_latest_mtx, portMAX_DELAY) != pdPASS) return;
    *out = s_latest;
    xSemaphoreGive(s_latest_mtx);
}


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
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS: open for load failed: %s", esp_err_to_name(err));
        return err;
    }

    int32_t off = 0;
    float   sc  = 0.0f;
    uint8_t ok  = 0;

    // offset
    err = nvs_get_i32(h, NVS_KEY_OFFS, &off);
    if (err != ESP_OK) {
        nvs_close(h);
        ESP_LOGW(TAG, "NVS: missing key '%s': %s", NVS_KEY_OFFS, esp_err_to_name(err));
        return err;
    }

    // scale (float via blob)
    size_t len = sizeof(sc);
    err = nvs_get_blob(h, NVS_KEY_SCALE, &sc, &len);
    if (err != ESP_OK || len != sizeof(sc)) {
        nvs_close(h);
        err = (err == ESP_OK) ? ESP_ERR_INVALID_SIZE : err;
        ESP_LOGW(TAG, "NVS: invalid key '%s': %s", NVS_KEY_SCALE, esp_err_to_name(err));
        return err;
    }

    // valid flag
    err = nvs_get_u8(h, NVS_KEY_VALID, &ok);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS: missing key '%s': %s", NVS_KEY_VALID, esp_err_to_name(err));
        return err;
    }

    s->offset_raw = off;
    s->scale_cpg  = (sc != 0.0f) ? sc : 1.0f;
    s->calibrated = ok ? true : false;

    ESP_LOGI(TAG, "NVS: loaded offset=%" PRId32 ", scale=%.6f, calibrated=%d",
             s->offset_raw, s->scale_cpg, s->calibrated);
    return ESP_OK;
}



static esp_err_t scale_nvs_save(const scale_hx711_t *s)
{
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
    ESP_LOGD(TAG, "read_avg_counts: start samples=%u", (unsigned)samples);
    // mutex to prevent collisions when e.g. running tare or calibration while polling
    if (!scale_lock(s, MUTEX_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "read_avg_counts: mutex timeout after %u ms", (unsigned)MUTEX_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }

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

    ESP_LOGD(TAG, "read_avg_counts: done avg_raw=%" PRId32, *avg_out);

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
    err = scale_nvs_load(s); // keeps defaults unchanged if not found
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "init: keeping defaults (offset=%" PRId32 ", scale=%.6f, calibrated=%d)",
                 s->offset_raw, s->scale_cpg, s->calibrated);
    }

    ESP_LOGI(TAG, "init: HX711 init successful (DT=%d, SCK=%d)",
             (int)CONFIG_HX711_DT_GPIO, (int)CONFIG_HX711_SCK_GPIO);

    return ESP_OK;
}



esp_err_t scale_hx711_tare(scale_hx711_t *s, uint16_t samples)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "TARE: start samples=%u", (unsigned)samples);

    int32_t avg_raw = 0;
    esp_err_t err = read_avg_counts(s, samples, &avg_raw);
    if (err != ESP_OK) return err;

    if (!scale_lock(s, MUTEX_TIMEOUT_MS)) return ESP_ERR_TIMEOUT;
    s->offset_raw = avg_raw;
    scale_unlock(s);
    ESP_LOGI(TAG, "TARE: offset_raw=%" PRId32 " (avg of %u)", avg_raw, samples);

    // update persistent calibration in nvs
    err = scale_nvs_save(s);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TARE: NVS save failed: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}



esp_err_t scale_hx711_calibrate(scale_hx711_t *s,
                                float grams_on_scale,
                                uint16_t samples)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    if (grams_on_scale <= 0.0f) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "CALIBRATE: start ref=%.3f g samples=%u offset_raw=%" PRId32,
             grams_on_scale, (unsigned)samples, s->offset_raw);

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
        "CALIBRATE: raw=%" PRId32 " diff=%" PRId32 " counts @ %.3f g -> scale=%.6f counts/g",
        avg_raw, diff, grams_on_scale, s->scale_cpg);

    // save new calibration to nvs (persistent - gets loaded after next startup/init)
    err = scale_nvs_save(s);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CALIBRATE: NVS save failed: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}

void scale_hx711_set_default(scale_hx711_t *s)
{
    s_default = s;
}

esp_err_t scale_hx711_tare_default(uint16_t samples)
{
    if (!s_default) return ESP_ERR_INVALID_STATE;
    return scale_hx711_tare(s_default, samples);
}

esp_err_t scale_hx711_calibrate_default(float grams_on_scale, uint16_t samples)
{
    if (!s_default) return ESP_ERR_INVALID_STATE;
    return scale_hx711_calibrate(s_default, grams_on_scale, samples);
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
        ESP_LOGD(TAG,
                 "read_grams: raw=%" PRId32 " offset=%" PRId32 " scale=%.6f grams=%.3f valid=%d",
                 avg_raw, s->offset_raw, s->scale_cpg, grams, s->calibrated);
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

typedef struct {
    TickType_t period;
    QueueHandle_t q;
} fake_poll_cfg_t;

// Publish one telemetry sample using the real HX711 sample timestamp and a
// lightweight snapshot of the current controller state.
static bool scale_publish_telemetry_sample(int64_t ts_us, float weight_g)
{
    uint32_t run_id = filler_get_run_id();
    int32_t slot_idx = (int32_t)filler_get_slot_idx();
    uint32_t state = (uint32_t)filler_get_state();
    float gate_pct = filler_get_gate_percent();

    app_params_t params = {0};
    app_params_get(&params);

    float jar_tare_g = 0.0f;
    float relative_fill_g = weight_g;
    if (filler_get_jar_tare(&jar_tare_g)) {
        relative_fill_g = weight_g - jar_tare_g;
    }
    telemetry_record_t rec;
    telemetry_record_init(&rec, TELEMETRY_KIND_SAMPLE);
    rec.ts_us = ts_us;
    rec.run_id = run_id;
    rec.slot_idx = slot_idx;
    rec.state = state;
    rec.target_g = params.target_grams;
    rec.weight_g = weight_g;
    rec.relative_fill_g = relative_fill_g;
    rec.gate_pct = gate_pct;

    filler_strategy_sample_telemetry_t strategy_sample = {0};
    if (filler_get_strategy_sample_telemetry(&strategy_sample) && strategy_sample.valid) {
        snprintf(rec.strategy_name, sizeof(rec.strategy_name), "%s", strategy_sample.strategy_name);
        snprintf(rec.text, sizeof(rec.text), "%s", strategy_sample.gate_phase);
        rec.rate_raw_gps = strategy_sample.raw_rate_gps;
        rec.rate_filtered_gps = strategy_sample.filtered_rate_gps;
        rec.predicted_remaining_g = strategy_sample.predicted_remaining_g;
        rec.measured_dead_time_s = strategy_sample.measured_dead_time_s;
        rec.measured_post_close_gain_g = strategy_sample.measured_post_close_gain_g;
        rec.learned_dead_time_s = strategy_sample.learned_dead_time_s;
        rec.learned_post_close_gain_g = strategy_sample.learned_post_close_gain_g;
        rec.learned_fast_rate_gps = strategy_sample.learned_fast_rate_gps;
        rec.learned_slow_rate_gps = strategy_sample.learned_slow_rate_gps;
        rec.adapted_near_close_g = strategy_sample.adapted_near_close_g;
        rec.adapted_close_early_g = strategy_sample.adapted_close_early_g;
        rec.adapted_drip_wait_ms = strategy_sample.adapted_drip_wait_ms;
        rec.refill_count = strategy_sample.refill_count;
    }

    return telemetry_publish_sample(&rec);
}

// Keep queue/latest publishing consistent between the real and fake producers.
static void scale_publish_sample(QueueHandle_t q, int32_t raw, float grams, bool valid, int64_t ts_us)
{
    static int64_t s_last_drop_warn_us;

    scale_latest_t latest = {
        .raw = raw,
        .grams = grams,
        .valid = valid,
        .ts_us = ts_us
    };
    scale_latest_set(&latest);

    scale_sample_t msg = {
        .raw = raw,
        .grams = grams,
        .valid = valid,
        .ts_us = ts_us
    };
    if (xQueueSend(q, &msg, 0) != pdPASS) {
        xQueueOverwrite(q, &msg);
    }

    if (!scale_publish_telemetry_sample(ts_us, grams)) {
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_last_drop_warn_us > 1000000) {
            s_last_drop_warn_us = now_us;
            ESP_LOGW(TAG, "telemetry sample publish dropped");
        }
    }
}

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
            int64_t ts = esp_timer_get_time();
            scale_publish_sample(cfg.q, raw, g, ok, ts);
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

static float fake_scale_weight_for_step(uint32_t step)
{
    const uint32_t phase = step % 64u;

    // One repeatable synthetic cycle: idle -> jar present -> fill ramp -> drip tail.
    if (phase < 12u) {
        return 0.0f;
    }
    if (phase < 20u) {
        return 185.0f;
    }
    if (phase < 36u) {
        const float frac = (float)(phase - 20u) / 16.0f;
        return 185.0f + frac * 320.0f;
    }
    if (phase < 44u) {
        const float frac = (float)(phase - 36u) / 8.0f;
        return 505.0f + frac * 8.0f;
    }
    if (phase < 52u) {
        const float frac = (float)(phase - 44u) / 8.0f;
        return 513.0f - frac * 6.0f;
    }
    return 507.0f;
}

static void task_scale_fake_poll(void *arg)
{
    fake_poll_cfg_t cfg = *(fake_poll_cfg_t *)arg;
    vPortFree(arg);

    TickType_t last = xTaskGetTickCount();
    uint32_t step = 0;

    for (;;) {
        float grams = fake_scale_weight_for_step(step++);
        // Raw counts are synthetic here; grams are the meaningful test signal.
        int32_t raw = (int32_t)(grams * 100.0f);
        int64_t ts_us = esp_timer_get_time();

        scale_publish_sample(cfg.q, raw, grams, true, ts_us);
        vTaskDelayUntil(&last, cfg.period);
    }
}

static esp_err_t scale_start_poll_task(TaskFunction_t task_fn,
                                       const char *task_name,
                                       void *cfg,
                                       QueueHandle_t q)
{
    if (xTaskCreatePinnedToCore(task_fn, task_name, 4096, cfg,
                                CONFIG_TASK_PRIO_HX711, NULL, CONFIG_TASK_CORE_HX711) != pdPASS) {
        vPortFree(cfg);
        vQueueDelete(q);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
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

    esp_err_t err = scale_start_poll_task(task_scale_poll, "scale_poll", cfg, q);
    if (err != ESP_OK) return err;

    *out_queue = q;
    return ESP_OK;
}

esp_err_t scale_hx711_start_fake_poll(TickType_t period,
                                      UBaseType_t queue_len,
                                      QueueHandle_t *out_queue)
{
    if (!out_queue || queue_len == 0) return ESP_ERR_INVALID_ARG;

    QueueHandle_t q = xQueueCreate(queue_len, sizeof(scale_sample_t));
    if (!q) return ESP_ERR_NO_MEM;

    fake_poll_cfg_t *cfg = pvPortMalloc(sizeof(*cfg));
    if (!cfg) {
        vQueueDelete(q);
        return ESP_ERR_NO_MEM;
    }
    cfg->period = period;
    cfg->q = q;

    esp_err_t err = scale_start_poll_task(task_scale_fake_poll, "scale_fake", cfg, q);
    if (err != ESP_OK) return err;

    *out_queue = q;
    return ESP_OK;
}
