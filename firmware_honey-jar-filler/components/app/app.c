#include "app.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"

#define NVS_NAMESPACE  "app"
#define NVS_KEY_PARAMS "params_v1"

static SemaphoreHandle_t s_params_mtx;
static app_params_t s_params;

// Centralized defaults for persistent parameters.
static app_params_t app_params_defaults(void)
{
    app_params_t p = {
        .target_grams       = 500.0f,
        .near_close_delta_g = 40.0f,
        .fill_timeout_ms    = 20000,
        .advance_timeout_ms = 4000,
        .motor_dwell_ms     = 400,
        .slots_total        = 6,
    };
    return p;
}

// Update the in-RAM copy (caller must hold mutex).
static void app_params_set_ram(const app_params_t *in)
{
    if (in) {
        s_params = *in;
    }
}

void app_params_init(void)
{
    // Create mutex on first use; fall back to defaults if creation fails.
    if (!s_params_mtx) {
        s_params_mtx = xSemaphoreCreateMutex();
        if (!s_params_mtx) {
            s_params = app_params_defaults();
            return;
        }
    }

    app_params_t defaults = app_params_defaults();

    // Serialize init/load to ensure consistent RAM state.
    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) {
        s_params = defaults;
        return;
    }

    s_params = defaults;

    // Prefer caller-initialized NVS; if not initialized, fall back to defaults only.
    bool need_save = false;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
        s_params = defaults;
        xSemaphoreGive(s_params_mtx);
        return;
    }

    if (err == ESP_OK) {
        size_t len = sizeof(s_params);
        err = nvs_get_blob(h, NVS_KEY_PARAMS, &s_params, &len);
        nvs_close(h);
        if (err != ESP_OK || len != sizeof(s_params)) {
            // Invalid or mismatched blob -> reset to defaults.
            s_params = defaults;
            need_save = true;
        }
    } else {
        // No namespace yet -> write defaults once.
        s_params = defaults;
        need_save = true;
    }

    if (need_save) {
        nvs_handle_t h_wr;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h_wr) == ESP_OK) {
            if (nvs_set_blob(h_wr, NVS_KEY_PARAMS, &s_params, sizeof(s_params)) == ESP_OK) {
                (void)nvs_commit(h_wr);
            }
            nvs_close(h_wr);
        }
    }

    xSemaphoreGive(s_params_mtx);
}

void app_params_get(app_params_t *out)
{
    if (!out || !s_params_mtx) return;
    // Copy the current snapshot while holding the mutex.
    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return;
    *out = s_params;
    xSemaphoreGive(s_params_mtx);
}

esp_err_t app_params_set(const app_params_t *in)
{
    if (!in || !s_params_mtx) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return ESP_ERR_TIMEOUT;

    // Persist first; only update RAM on successful commit.
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, NVS_KEY_PARAMS, in, sizeof(*in));
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }

    if (err == ESP_OK) {
        app_params_set_ram(in);
    }

    xSemaphoreGive(s_params_mtx);
    return err;
}
