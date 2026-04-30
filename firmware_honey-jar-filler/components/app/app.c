#include "app.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "app_params_def.h"

#define NVS_NAMESPACE  "app"
#define NVS_KEY_PARAMS "params_v1"

static SemaphoreHandle_t s_params_mtx;
static app_params_t s_params;
static bool s_params_dirty;
static const char *TAG = "app_params";

// Bump to force defaults reload (when changing defaults/layout).
// APP_PARAMS_VERSION is defined in app_params_def.h

// Centralized defaults for persistent parameters.
static void app_params_apply_defaults(app_params_t *p)
{
    if (!p) return;
    *p = (app_params_t){0};
    // NOTE: bump APP_PARAMS_VERSION above to force defaults reload.
    p->version = APP_PARAMS_VERSION;

#define APP_PARAM_FLOAT(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope) \
    p->field = (def_val);
#define APP_PARAM_U32(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope)   \
    p->field = (def_val);
#define APP_PARAM_U8(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope)    \
    p->field = (def_val);
    APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8)
#undef APP_PARAM_FLOAT
#undef APP_PARAM_U32
#undef APP_PARAM_U8
}

static app_params_t app_params_defaults(void)
{
    app_params_t p;
    app_params_apply_defaults(&p);
    return p;
}

static void app_params_log_all(const app_params_t *cur)
{
    ESP_LOGW(TAG, "app parameters (version=%u):", (unsigned)cur->version);

#define APP_PARAM_FLOAT(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope) \
    ESP_LOGW(TAG, "  %s=%.1f%s%s (default=%.1f min=%.1f max=%.1f step=%.1f) - %s", \
             #field, (double)cur->field,                                       \
             (unit && unit[0]) ? " " : "", (unit && unit[0]) ? unit : "",      \
             (double)(def_val), (double)(min_val), (double)(max_val), (double)(step_val), \
             brief);
#define APP_PARAM_U32(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope) \
    ESP_LOGW(TAG, "  %s=%u%s%s (def=%u min=%u max=%u step=%u) - %s",          \
             #field, (unsigned)cur->field,                                    \
             (unit && unit[0]) ? " " : "", (unit && unit[0]) ? unit : "",     \
             (unsigned)(def_val), (unsigned)(min_val), (unsigned)(max_val), (unsigned)(step_val), \
             brief);
#define APP_PARAM_U8(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope) \
    ESP_LOGW(TAG, "  %s=%u%s%s (def=%u min=%u max=%u step=%u) - %s",          \
             #field, (unsigned)cur->field,                                    \
             (unit && unit[0]) ? " " : "", (unit && unit[0]) ? unit : "",     \
             (unsigned)(def_val), (unsigned)(min_val), (unsigned)(max_val), (unsigned)(step_val), \
             brief);
    APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8)
#undef APP_PARAM_FLOAT
#undef APP_PARAM_U32
#undef APP_PARAM_U8
}

// Update the in-RAM copy (caller must hold mutex).
static void app_params_set_ram(const app_params_t *in)
{
    if (in) {
        s_params = *in;
    }
}

static void app_params_copy_field(uint8_t *dst_base,
                                  const uint8_t *src_base,
                                  const app_param_meta_t *meta)
{
    if (!dst_base || !src_base || !meta) return;

    uint8_t *dst_ptr = dst_base + meta->offset;
    const uint8_t *src_ptr = src_base + meta->offset;

    switch (meta->type) {
    case APP_PARAM_FLOAT:
        *(float *)dst_ptr = *(const float *)src_ptr;
        break;
    case APP_PARAM_U32:
        *(uint32_t *)dst_ptr = *(const uint32_t *)src_ptr;
        break;
    case APP_PARAM_U8:
        *(uint8_t *)dst_ptr = *(const uint8_t *)src_ptr;
        break;
    default:
        break;
    }
}

static esp_err_t app_params_store_locked(const app_params_t *in)
{
    if (!in) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, NVS_KEY_PARAMS, in, sizeof(*in));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void app_params_init(void)
{
    // Create mutex on first use; fall back to defaults if creation fails.
    if (!s_params_mtx) {
        s_params_mtx = xSemaphoreCreateMutex();
        if (!s_params_mtx) {
            s_params = app_params_defaults();
            s_params_dirty = false;
            return;
        }
    }

    app_params_t defaults = app_params_defaults();

    // Serialize init/load to ensure consistent RAM state.
    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) {
        s_params = defaults;
        s_params_dirty = false;
        return;
    }

    s_params = defaults;
    s_params_dirty = false;

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
        if (err != ESP_OK || len != sizeof(s_params) || s_params.version != APP_PARAMS_VERSION) {
            // Invalid or mismatched blob -> reset to defaults.
            if (err == ESP_OK && s_params.version != APP_PARAMS_VERSION) {
                ESP_LOGW(TAG,
                         "params: version mismatch (nvs=%u, expected=%u) -> defaults",
                         (unsigned)s_params.version, (unsigned)APP_PARAMS_VERSION);
            } else if (err != ESP_OK) {
                ESP_LOGW(TAG, "params: load failed (%s) -> defaults", esp_err_to_name(err));
            } else {
                ESP_LOGW(TAG, "params: blob size mismatch -> defaults");
            }
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

    app_params_log_all(&s_params);
    s_params_dirty = false;

    xSemaphoreGive(s_params_mtx);
}

void app_params_defaults_get(app_params_t *out)
{
    if (!out) return;
    app_params_apply_defaults(out);
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

    app_params_set_ram(in);
    s_params_dirty = true;
    esp_err_t err = app_params_store_locked(&s_params);
    if (err == ESP_OK) s_params_dirty = false;
    xSemaphoreGive(s_params_mtx);
    return err;
}

esp_err_t app_params_set_runtime(const app_params_t *in)
{
    if (!in || !s_params_mtx) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return ESP_ERR_TIMEOUT;
    app_params_set_ram(in);
    s_params_dirty = true;
    xSemaphoreGive(s_params_mtx);
    return ESP_OK;
}

esp_err_t app_params_commit(void)
{
    if (!s_params_mtx) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return ESP_ERR_TIMEOUT;
    esp_err_t err = app_params_store_locked(&s_params);
    if (err == ESP_OK) s_params_dirty = false;
    xSemaphoreGive(s_params_mtx);
    return err;
}

bool app_params_is_dirty(void)
{
    bool dirty = false;
    if (!s_params_mtx) return false;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return false;
    dirty = s_params_dirty;
    xSemaphoreGive(s_params_mtx);
    return dirty;
}

void app_params_copy_scope(app_params_t *dst, const app_params_t *src, app_param_scope_t scope)
{
    if (!dst || !src) return;

    size_t count = 0;
    const app_param_meta_t *meta = app_params_meta_get(&count);
    if (!meta || count == 0) return;

    uint8_t *dst_base = (uint8_t *)dst;
    const uint8_t *src_base = (const uint8_t *)src;

    for (size_t i = 0; i < count; ++i) {
        if (meta[i].scope != scope) continue;
        app_params_copy_field(dst_base, src_base, &meta[i]);
    }
}

const char *app_param_scope_name(app_param_scope_t scope)
{
    switch (scope) {
    case APP_PARAM_SCOPE_PRESET:
        return "preset";
    case APP_PARAM_SCOPE_MACHINE:
        return "machine";
    default:
        return "?";
    }
}
