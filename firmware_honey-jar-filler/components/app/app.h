#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    //=== Meta / versions ===
    uint16_t version;               // bump to force defaults reload when layout/defaults change

    //=== Target + verification ===
    float    target_grams;           // target filled mass per jar (grams)
    float    target_tol_low_pct;     // below target by this % -> re-fill (retry window)
    float    target_tol_high_pct;    // above target by this % -> fault (overfill window)
    uint32_t fill_timeout_ms;        // max time allowed in FILL before fault

    //=== Glass detection ===
    float    empty_glass_min_g;      // below this -> no jar present (measured empty jar weight window)
    float    empty_glass_max_g;      // above this -> jar not empty / already filled (empty jar upper bound)

    //=== Honey flow tuning ===
    float    near_close_delta_g;     // when within this many grams of target, partially close gate
    float    near_close_gate_pct;    // partial gate opening (%) used near the target
    float    max_gate_pct;           // max gate opening (%) used during bulk fill (full open cap)
    float    close_early_pct;        // % of target before target to fully close gate (compensates drip/in-flight)
    uint32_t drip_delay_ms;          // wait after closing gate so drips fall into jar


    //=== Mechanics / motion ===
    uint32_t advance_timeout_ms;     // max time to find POS switch when advancing
    uint32_t find_ignore_ms;         // ignore POS switch for this long after motor start (aka motor min time on)
    uint32_t slot_settle_ms;         // wait after slot found so motor/scale settles
    uint8_t  slots_total;            // total number of jars to fill in a run
} app_params_t;

typedef enum {
    APP_PARAM_FLOAT = 0,
    APP_PARAM_U32,
    APP_PARAM_U8,
} app_param_type_t;

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
    size_t            offset;
    app_param_value_t def;
    app_param_value_t min;
    app_param_value_t max;
    app_param_value_t step;
} app_param_meta_t;

void      app_params_init(void);
void      app_params_get(app_params_t *out);
esp_err_t app_params_set(const app_params_t *in);
void      app_params_defaults_get(app_params_t *out);
const app_param_meta_t *app_params_meta_get(size_t *out_count);

#ifdef __cplusplus
}
#endif

#endif // APP_H
