#ifndef APP_H
#define APP_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t version;               // bump to force defaults reload when layout/defaults change
    float    target_grams;           // target filled mass per jar (grams)
    float    near_close_delta_g;     // when within this many grams of target, partially close gate
    float    near_close_gate_pct;    // partial gate opening (%) used near the target
    float    max_gate_pct;           // max gate opening (%) used during bulk fill (full open cap)
    float    close_early_pct;        // % of target before target to fully close gate (compensates drip/in-flight)
    float    empty_glass_min_g;      // below this -> no jar present (measured empty jar weight window)
    float    empty_glass_max_g;      // above this -> jar not empty / already filled (empty jar upper bound)
    float    target_tol_low_pct;     // below target by this % -> re-fill (retry window)
    float    target_tol_high_pct;    // above target by this % -> fault (overfill window)
    uint32_t fill_timeout_ms;        // max time allowed in FILL before fault
    uint32_t advance_timeout_ms;     // max time to find POS switch when advancing
    uint32_t find_ignore_ms;         // ignore POS switch for this long after motor start (aka motor min time on)
    uint32_t drip_delay_ms;          // wait after closing gate so drips fall into jar
    uint32_t slot_settle_ms;         // wait after slot found so motor/scale settles
    uint8_t  slots_total;            // total number of jars to fill in a run
} app_params_t;

void      app_params_init(void);
void      app_params_get(app_params_t *out);
esp_err_t app_params_set(const app_params_t *in);

#ifdef __cplusplus
}
#endif

#endif // APP_H
