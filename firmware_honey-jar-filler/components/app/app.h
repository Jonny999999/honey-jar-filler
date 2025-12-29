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
    float    close_early_g;          // grams before target to fully close gate (compensates drip/in-flight)
    uint32_t fill_timeout_ms;        // max time allowed in FILL before fault
    uint32_t advance_timeout_ms;     // max time to find POS switch when advancing
    uint32_t motor_dwell_ms;         // motor on-time to advance one slot (pulse length)
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
