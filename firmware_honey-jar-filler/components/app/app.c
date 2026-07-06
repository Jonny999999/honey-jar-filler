#include "app.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "app_params_def.h"

#define NVS_NAMESPACE         "app"
#define NVS_KEY_LEGACY_PARAMS "params_v1"
#define NVS_KEY_MACHINE       "machine_v1"
#define NVS_KEY_PRESETS       "presets_v1"
#define NVS_KEY_STRATEGY      "strategy_v1"

#define APP_MACHINE_STORE_VERSION 1u
#define APP_PRESET_STORE_VERSION  1u
#define APP_STRATEGY_STORE_VERSION 1u
#define APP_PRESET_DEFAULT_ACTIVE 1u
#define APP_PRESET_TEST_INDEX     3u
#define APP_FILL_STRATEGY_DEFAULT APP_FILL_STRATEGY_HEURISTIC

typedef struct {
    uint16_t version;
    uint16_t params_version;
    app_params_t params;
} app_machine_store_t;

typedef struct {
    uint16_t version;
    uint16_t params_version;
    uint8_t active_index;
    uint8_t preset_count;
    app_params_t presets[APP_PRESET_COUNT];
} app_preset_store_t;

typedef struct {
    uint16_t version;
    uint8_t active_strategy;
} app_strategy_store_t;

static SemaphoreHandle_t s_params_mtx;
static app_params_t s_params;
static bool s_params_dirty;
static app_machine_store_t s_machine_store;
static app_preset_store_t s_preset_store;
static app_strategy_store_t s_strategy_store;
static const char *TAG = "app_params";

static app_params_t app_preset_defaults(uint8_t index);

// Built-in preset names shown in the UI. Their default values are defined below
// as overrides on top of the baseline defaults from app_params_def.h.
static const char *k_preset_names[APP_PRESET_COUNT] = {
    "High viscosity",
    "Medium viscosity",
    "Low viscosity",
    "Testing",
};

static const char *k_strategy_names[APP_FILL_STRATEGY_COUNT] = {
    "heuristic",
    "adaptive-heuristic",
    "flow-control",
    "manual",
    "flow-cascade",
    "sequence",
};

// Centralized defaults for runtime parameters.
static void app_params_apply_defaults(app_params_t *p)
{
    if (!p) return;
    *p = (app_params_t){0};
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

static void app_params_log_all(const app_params_t *cur, const app_params_t *defs)
{
    ESP_LOGW(TAG, "app parameters (version=%u):", (unsigned)cur->version);

#define APP_PARAM_FLOAT(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope) \
    ESP_LOGW(TAG, "  %s=%.1f%s%s (default=%.1f min=%.1f max=%.1f step=%.1f scope=%s) - %s", \
             #field, (double)cur->field,                                       \
             (unit && unit[0]) ? " " : "", (unit && unit[0]) ? unit : "",      \
             (double)defs->field, (double)(min_val), (double)(max_val), (double)(step_val), \
             app_param_scope_name(scope), brief);
#define APP_PARAM_U32(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope) \
    ESP_LOGW(TAG, "  %s=%u%s%s (def=%u min=%u max=%u step=%u scope=%s) - %s",  \
             #field, (unsigned)cur->field,                                    \
             (unit && unit[0]) ? " " : "", (unit && unit[0]) ? unit : "",     \
             (unsigned)defs->field, (unsigned)(min_val), (unsigned)(max_val), (unsigned)(step_val), \
             app_param_scope_name(scope), brief);
#define APP_PARAM_U8(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope) \
    ESP_LOGW(TAG, "  %s=%u%s%s (def=%u min=%u max=%u step=%u scope=%s) - %s",  \
             #field, (unsigned)cur->field,                                    \
             (unit && unit[0]) ? " " : "", (unit && unit[0]) ? unit : "",     \
             (unsigned)defs->field, (unsigned)(min_val), (unsigned)(max_val), (unsigned)(step_val), \
             app_param_scope_name(scope), brief);
    APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8)
#undef APP_PARAM_FLOAT
#undef APP_PARAM_U32
#undef APP_PARAM_U8
}

static app_params_t app_effective_defaults_for_preset(uint8_t index)
{
    return app_preset_defaults(index);
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

static void app_params_set_runtime_locked(const app_params_t *in)
{
    if (!in) return;
    s_params = *in;
    s_params.version = APP_PARAMS_VERSION;
}

static bool app_params_blob_valid(const app_params_t *p)
{
    return p && p->version == APP_PARAMS_VERSION;
}

// Built-in preset defaults only override recipe-scoped fields. Shared machine
// settings still come from the baseline defaults + NVS machine store.
static void app_apply_builtin_preset_defaults(uint8_t index, app_params_t *p)
{
    if (!p) return;

    switch (index) {
    case 0: // High viscosity
        p->target_grams = 500;
        p->target_tol_low_g = 20;
        p->target_tol_high_g = 50;
        p->fill_timeout_ms = 600000;
        p->slow_remaining_g = 30;
        p->slow_gate_pct = 35;
        p->max_gate_pct = 98;
        p->close_remaining_g = 10;
        p->drip_delay_ms = 20000;
        break;
    case 1: // Medium viscosity
        break;
    case 2: // Low viscosity
        p->target_grams = 480;
        p->target_tol_low_g = 20;
        p->target_tol_high_g = 52;
        p->fill_timeout_ms = 600000;
        p->slow_remaining_g = 110;
        p->slow_gate_pct = 14;
        p->max_gate_pct = 22;
        p->close_remaining_g = 5;
        p->drip_delay_ms = 11000;
        break;
    case 3: // Testing
    default:
        break;
    }
}

static app_params_t app_preset_defaults(uint8_t index)
{
    app_params_t p = app_params_defaults();
    app_apply_builtin_preset_defaults(index, &p);
    p.version = APP_PARAMS_VERSION;
    return p;
}

static void app_machine_store_apply_defaults(app_machine_store_t *store)
{
    if (!store) return;
    *store = (app_machine_store_t){
        .version = APP_MACHINE_STORE_VERSION,
        .params_version = APP_PARAMS_VERSION,
        .params = app_params_defaults(),
    };
}

static void app_preset_store_apply_defaults(app_preset_store_t *store)
{
    if (!store) return;

    *store = (app_preset_store_t){
        .version = APP_PRESET_STORE_VERSION,
        .params_version = APP_PARAMS_VERSION,
        .active_index = APP_PRESET_DEFAULT_ACTIVE,
        .preset_count = APP_PRESET_COUNT,
    };

    for (uint8_t i = 0; i < APP_PRESET_COUNT; ++i) {
        store->presets[i] = app_preset_defaults(i);
    }
}

static void app_strategy_store_apply_defaults(app_strategy_store_t *store)
{
    if (!store) return;
    *store = (app_strategy_store_t){
        .version = APP_STRATEGY_STORE_VERSION,
        .active_strategy = APP_FILL_STRATEGY_DEFAULT,
    };
}

static bool app_machine_store_valid(const app_machine_store_t *store)
{
    return store &&
           store->version == APP_MACHINE_STORE_VERSION &&
           store->params_version == APP_PARAMS_VERSION &&
           app_params_blob_valid(&store->params);
}

static bool app_preset_store_valid(const app_preset_store_t *store)
{
    if (!store ||
        store->version != APP_PRESET_STORE_VERSION ||
        store->params_version != APP_PARAMS_VERSION ||
        store->preset_count != APP_PRESET_COUNT ||
        store->active_index >= APP_PRESET_COUNT) {
        return false;
    }

    for (uint8_t i = 0; i < APP_PRESET_COUNT; ++i) {
        if (!app_params_blob_valid(&store->presets[i])) return false;
    }
    return true;
}

static bool app_strategy_store_valid(const app_strategy_store_t *store)
{
    return store &&
           store->version == APP_STRATEGY_STORE_VERSION &&
           store->active_strategy < APP_FILL_STRATEGY_COUNT;
}

static esp_err_t app_machine_store_load(app_machine_store_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t len = sizeof(*out);
    err = nvs_get_blob(h, NVS_KEY_MACHINE, out, &len);
    nvs_close(h);
    if (err != ESP_OK) return err;
    if (len != sizeof(*out) || !app_machine_store_valid(out)) return ESP_ERR_INVALID_RESPONSE;
    return ESP_OK;
}

static esp_err_t app_preset_store_load(app_preset_store_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t len = sizeof(*out);
    err = nvs_get_blob(h, NVS_KEY_PRESETS, out, &len);
    nvs_close(h);
    if (err != ESP_OK) return err;
    if (len != sizeof(*out) || !app_preset_store_valid(out)) return ESP_ERR_INVALID_RESPONSE;
    return ESP_OK;
}

static esp_err_t app_strategy_store_load(app_strategy_store_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t len = sizeof(*out);
    err = nvs_get_blob(h, NVS_KEY_STRATEGY, out, &len);
    nvs_close(h);
    if (err != ESP_OK) return err;
    if (len != sizeof(*out) || !app_strategy_store_valid(out)) return ESP_ERR_INVALID_RESPONSE;
    return ESP_OK;
}

static esp_err_t app_machine_store_save_locked(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, NVS_KEY_MACHINE, &s_machine_store, sizeof(s_machine_store));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t app_preset_store_save_locked(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, NVS_KEY_PRESETS, &s_preset_store, sizeof(s_preset_store));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t app_strategy_store_save_locked(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, NVS_KEY_STRATEGY, &s_strategy_store, sizeof(s_strategy_store));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t app_store_save_all_locked(void)
{
    esp_err_t err = app_machine_store_save_locked();
    if (err != ESP_OK) return err;
    err = app_preset_store_save_locked();
    if (err != ESP_OK) return err;
    return app_strategy_store_save_locked();
}

static void app_sync_stores_from_runtime_locked(void)
{
    s_machine_store.version = APP_MACHINE_STORE_VERSION;
    s_machine_store.params_version = APP_PARAMS_VERSION;
    s_machine_store.params.version = APP_PARAMS_VERSION;
    app_params_copy_scope(&s_machine_store.params, &s_params, APP_PARAM_SCOPE_MACHINE);

    s_preset_store.version = APP_PRESET_STORE_VERSION;
    s_preset_store.params_version = APP_PARAMS_VERSION;
    s_preset_store.preset_count = APP_PRESET_COUNT;
    if (s_preset_store.active_index >= APP_PRESET_COUNT) {
        s_preset_store.active_index = APP_PRESET_DEFAULT_ACTIVE;
    }
    s_preset_store.presets[s_preset_store.active_index].version = APP_PARAMS_VERSION;
    app_params_copy_scope(&s_preset_store.presets[s_preset_store.active_index], &s_params, APP_PARAM_SCOPE_PRESET);
}

static void app_rebuild_runtime_locked(void)
{
    app_params_t merged = app_params_defaults();
    if (s_preset_store.active_index >= APP_PRESET_COUNT) {
        s_preset_store.active_index = APP_PRESET_DEFAULT_ACTIVE;
    }
    app_params_copy_scope(&merged, &s_machine_store.params, APP_PARAM_SCOPE_MACHINE);
    app_params_copy_scope(&merged, &s_preset_store.presets[s_preset_store.active_index], APP_PARAM_SCOPE_PRESET);
    merged.version = APP_PARAMS_VERSION;
    app_params_set_runtime_locked(&merged);
}

static bool app_try_migrate_legacy_locked(void)
{
    app_params_t legacy = {0};
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return false;

    size_t len = sizeof(legacy);
    err = nvs_get_blob(h, NVS_KEY_LEGACY_PARAMS, &legacy, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(legacy) || !app_params_blob_valid(&legacy)) {
        return false;
    }

    app_params_copy_scope(&s_machine_store.params, &legacy, APP_PARAM_SCOPE_MACHINE);
    s_machine_store.params.version = APP_PARAMS_VERSION;

    s_preset_store.active_index = APP_PRESET_TEST_INDEX;
    app_params_copy_scope(&s_preset_store.presets[APP_PRESET_TEST_INDEX], &legacy, APP_PARAM_SCOPE_PRESET);
    s_preset_store.presets[APP_PRESET_TEST_INDEX].version = APP_PARAMS_VERSION;

    app_rebuild_runtime_locked();
    if (app_store_save_all_locked() != ESP_OK) {
        s_params_dirty = true;
    }

    ESP_LOGW(TAG, "migrated legacy params_v1 into machine settings + preset '%s'",
             k_preset_names[APP_PRESET_TEST_INDEX]);
    return true;
}

void app_params_init(void)
{
    if (!s_params_mtx) {
        s_params_mtx = xSemaphoreCreateMutex();
        if (!s_params_mtx) {
            s_params = app_params_defaults();
            s_params_dirty = false;
            return;
        }
    }

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) {
        s_params = app_params_defaults();
        s_params_dirty = false;
        return;
    }

    app_machine_store_apply_defaults(&s_machine_store);
    app_preset_store_apply_defaults(&s_preset_store);
    app_strategy_store_apply_defaults(&s_strategy_store);
    app_rebuild_runtime_locked();
    s_params_dirty = false;

    bool need_save = false;
    bool machine_loaded = false;
    bool presets_loaded = false;
    bool strategy_loaded = false;

    app_machine_store_t machine_loaded_tmp = {0};
    if (app_machine_store_load(&machine_loaded_tmp) == ESP_OK) {
        s_machine_store = machine_loaded_tmp;
        machine_loaded = true;
    } else {
        need_save = true;
    }

    app_preset_store_t presets_loaded_tmp = {0};
    if (app_preset_store_load(&presets_loaded_tmp) == ESP_OK) {
        s_preset_store = presets_loaded_tmp;
        presets_loaded = true;
    } else {
        need_save = true;
    }

    app_strategy_store_t strategy_loaded_tmp = {0};
    if (app_strategy_store_load(&strategy_loaded_tmp) == ESP_OK) {
        s_strategy_store = strategy_loaded_tmp;
        strategy_loaded = true;
    } else {
        need_save = true;
    }

    if (!machine_loaded && !presets_loaded && !strategy_loaded) {
        if (app_try_migrate_legacy_locked()) {
            need_save = false;
        }
    }

    app_rebuild_runtime_locked();

    if (need_save) {
        esp_err_t err = app_store_save_all_locked();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "save defaults failed: %s", esp_err_to_name(err));
            s_params_dirty = true;
        }
    }

    app_params_t effective_defaults = app_effective_defaults_for_preset(s_preset_store.active_index);

    ESP_LOGI(TAG, "active preset: %u '%s'",
             (unsigned)s_preset_store.active_index,
             app_presets_get_name(s_preset_store.active_index));
    ESP_LOGI(TAG, "active strategy: %u '%s'",
             (unsigned)s_strategy_store.active_strategy,
             app_fill_strategy_get_name((app_fill_strategy_t)s_strategy_store.active_strategy));
    app_params_log_all(&s_params, &effective_defaults);

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
    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return;
    *out = s_params;
    xSemaphoreGive(s_params_mtx);
}

esp_err_t app_params_set_runtime(const app_params_t *in)
{
    if (!in || !s_params_mtx) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return ESP_ERR_TIMEOUT;
    app_params_set_runtime_locked(in);
    s_params_dirty = true;
    xSemaphoreGive(s_params_mtx);
    return ESP_OK;
}

esp_err_t app_params_commit(void)
{
    if (!s_params_mtx) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return ESP_ERR_TIMEOUT;
    app_sync_stores_from_runtime_locked();
    esp_err_t err = app_store_save_all_locked();
    if (err == ESP_OK) {
        s_params_dirty = false;
    } else {
        ESP_LOGW(TAG, "commit failed: %s", esp_err_to_name(err));
    }
    xSemaphoreGive(s_params_mtx);
    return err;
}

esp_err_t app_params_set(const app_params_t *in)
{
    if (!in || !s_params_mtx) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return ESP_ERR_TIMEOUT;
    app_params_set_runtime_locked(in);
    s_params_dirty = true;
    app_sync_stores_from_runtime_locked();
    esp_err_t err = app_store_save_all_locked();
    if (err == ESP_OK) {
        s_params_dirty = false;
    } else {
        ESP_LOGW(TAG, "set failed: %s", esp_err_to_name(err));
    }
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

    dst->version = APP_PARAMS_VERSION;
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

size_t app_presets_count(void)
{
    return APP_PRESET_COUNT;
}

uint8_t app_presets_get_active_index(void)
{
    uint8_t index = APP_PRESET_DEFAULT_ACTIVE;
    if (!s_params_mtx) return index;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return index;
    index = s_preset_store.active_index;
    xSemaphoreGive(s_params_mtx);
    return index;
}

const char *app_presets_get_name(uint8_t index)
{
    if (index >= APP_PRESET_COUNT) return "?";
    return k_preset_names[index];
}

esp_err_t app_presets_select(uint8_t index)
{
    if (index >= APP_PRESET_COUNT || !s_params_mtx) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return ESP_ERR_TIMEOUT;
    s_preset_store.active_index = index;
    app_rebuild_runtime_locked();
    s_params_dirty = true;
    esp_err_t err = app_preset_store_save_locked();
    if (err == ESP_OK) {
        s_params_dirty = false;
        ESP_LOGI(TAG, "preset selected: %u '%s'", (unsigned)index, app_presets_get_name(index));
    } else {
        ESP_LOGW(TAG, "preset select save failed: %s", esp_err_to_name(err));
    }
    xSemaphoreGive(s_params_mtx);
    return err;
}

size_t app_fill_strategy_count(void)
{
    return APP_FILL_STRATEGY_COUNT;
}

app_fill_strategy_t app_fill_strategy_get_active(void)
{
    app_fill_strategy_t strategy = APP_FILL_STRATEGY_DEFAULT;
    if (!s_params_mtx) return strategy;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return strategy;
    if (s_strategy_store.active_strategy < APP_FILL_STRATEGY_COUNT) {
        strategy = (app_fill_strategy_t)s_strategy_store.active_strategy;
    }
    xSemaphoreGive(s_params_mtx);
    return strategy;
}

const char *app_fill_strategy_get_name(app_fill_strategy_t strategy)
{
    if ((size_t)strategy >= APP_FILL_STRATEGY_COUNT) return "?";
    return k_strategy_names[strategy];
}

esp_err_t app_fill_strategy_select(app_fill_strategy_t strategy)
{
    if ((size_t)strategy >= APP_FILL_STRATEGY_COUNT || !s_params_mtx) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(s_params_mtx, portMAX_DELAY) != pdPASS) return ESP_ERR_TIMEOUT;
    s_strategy_store.active_strategy = (uint8_t)strategy;
    esp_err_t err = app_strategy_store_save_locked();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "strategy selected: %u '%s'",
                 (unsigned)strategy,
                 app_fill_strategy_get_name(strategy));
    } else {
        ESP_LOGW(TAG, "strategy select save failed: %s", esp_err_to_name(err));
    }
    xSemaphoreGive(s_params_mtx);
    return err;
}
