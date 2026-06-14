#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_PARAM_FLOAT = 0,
    APP_PARAM_U32,
    APP_PARAM_U8,
} app_param_type_t;

typedef enum {
    APP_PARAM_SCOPE_PRESET = 0,
    APP_PARAM_SCOPE_MACHINE,
} app_param_scope_t;

typedef enum {
    APP_FILL_STRATEGY_HEURISTIC = 0,
    APP_FILL_STRATEGY_ADAPTIVE_HEURISTIC,
    APP_FILL_STRATEGY_COUNT,
} app_fill_strategy_t;

#define APP_PRESET_COUNT     4u
#define APP_PRESET_NAME_MAX 16u

typedef struct {
    //=== Meta / versions ===
    uint16_t version;               // bump to force defaults reload when layout/defaults change

#define APP_PARAM_FLOAT(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope) \
    float field;
#define APP_PARAM_U32(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope)   \
    uint32_t field;
#define APP_PARAM_U8(field, label, unit, def_val, min_val, max_val, step_val, brief, detail, group, scope)    \
    uint8_t field;
#include "app_params_def.h"
    APP_PARAMS_DEF_LIST(APP_PARAM_FLOAT, APP_PARAM_U32, APP_PARAM_U8)
#undef APP_PARAM_FLOAT
#undef APP_PARAM_U32
#undef APP_PARAM_U8
} app_params_t;

typedef union {
    float    f;
    uint32_t u32;
    uint8_t  u8;
} app_param_value_t;

typedef struct {
    const char       *name;
    const char       *label;
    const char       *unit;
    const char       *desc_brief;
    const char       *desc_detail;
    const char       *group;
    app_param_type_t  type;
    app_param_scope_t scope;
    size_t            offset;
    app_param_value_t def;
    app_param_value_t min;
    app_param_value_t max;
    app_param_value_t step;
} app_param_meta_t;

void      app_params_init(void);
void      app_params_get(app_params_t *out);
esp_err_t app_params_set_runtime(const app_params_t *in);
esp_err_t app_params_commit(void);
esp_err_t app_params_set(const app_params_t *in);
bool      app_params_is_dirty(void);
void      app_params_defaults_get(app_params_t *out);
void      app_params_copy_scope(app_params_t *dst, const app_params_t *src, app_param_scope_t scope);
const char *app_param_scope_name(app_param_scope_t scope);

size_t    app_presets_count(void);
uint8_t   app_presets_get_active_index(void);
const char *app_presets_get_name(uint8_t index);
esp_err_t app_presets_select(uint8_t index);

size_t    app_fill_strategy_count(void);
app_fill_strategy_t app_fill_strategy_get_active(void);
const char *app_fill_strategy_get_name(app_fill_strategy_t strategy);
esp_err_t app_fill_strategy_select(app_fill_strategy_t strategy);

const app_param_meta_t *app_params_meta_get(size_t *out_count);

#ifdef __cplusplus
}
#endif

#endif // APP_H
