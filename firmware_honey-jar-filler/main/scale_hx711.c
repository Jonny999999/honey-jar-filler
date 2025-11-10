#include "scale_hx711.h"
#include "esp_log.h"
#include <inttypes.h>

static const char *TAG = "scale_hx711";

// helper: average N samples from HX711
static esp_err_t read_avg_counts(scale_hx711_t *s, uint16_t samples, int32_t *avg_out)
{
    if (!avg_out || samples == 0) return ESP_ERR_INVALID_ARG;

    // Make sure data is ready at least once before reading loop
    esp_err_t err = hx711_wait(&s->dev, 1000 /* ms timeout */);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hx711_wait failed (%d)", err);
        return err;
    }

    // use the upstream driver's averaging helper if available:
    // hx711_read_average(&s->dev, samples, avg_out)
    // If that helper doesn't exist in your version, we can roll our own loop.
    err = hx711_read_average(&s->dev, samples, avg_out);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hx711_read_average failed (%d)", err);
        return err;
    }

    return ESP_OK;
}



esp_err_t scale_hx711_init(scale_hx711_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;

    // reset struct defaults
    s->offset_raw  = 0;
    s->scale_cpg   = 1.0f;  // counts per gram fallback
    s->calibrated  = false;

    s->dev.dout    = CONFIG_HX711_DT_GPIO;
    s->dev.pd_sck  = CONFIG_HX711_SCK_GPIO;
    s->dev.gain    = HX711_GAIN_A_128; // channel A, gain 128 (typical load cell mode)

    esp_err_t err = hx711_init(&s->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hx711_init failed (%d)", err);
        return err;
    }

    // sanity: wait once so we know chip is alive
    err = hx711_wait(&s->dev, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hx711_wait failed (%d)", err);
        return err;
    }

    ESP_LOGI(TAG, "HX711 init OK (DT=%d, SCK=%d)",
             (int)CONFIG_HX711_DT_GPIO, (int)CONFIG_HX711_SCK_GPIO);

    return ESP_OK;
}



esp_err_t scale_hx711_tare(scale_hx711_t *s, uint16_t samples)
{
    if (!s) return ESP_ERR_INVALID_ARG;

    int32_t avg_raw = 0;
    esp_err_t err = read_avg_counts(s, samples, &avg_raw);
    if (err != ESP_OK) return err;

    s->offset_raw = avg_raw;
    ESP_LOGI(TAG, "TARE: offset_raw=%" PRId32 " (avg of %u)", avg_raw, samples);
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

    s->scale_cpg  = (float)diff / grams_on_scale; // counts per gram
    s->calibrated = true;

    ESP_LOGI(TAG,
        "CALIBRATE: diff=%" PRId32 " counts @ %.3f g -> scale=%.6f counts/g",
        diff, grams_on_scale, s->scale_cpg);

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
