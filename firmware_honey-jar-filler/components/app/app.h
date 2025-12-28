#ifndef APP_H
#define APP_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    target_grams;           // default 500
    float    near_close_delta_g;     // default 40
    uint32_t fill_timeout_ms;        // default 20000
    uint32_t advance_timeout_ms;     // default 4000
    uint32_t motor_dwell_ms;         // default 400
    uint8_t  slots_total;            // default 6
} app_params_t;

void      app_params_init(void);
void      app_params_get(app_params_t *out);
esp_err_t app_params_set(const app_params_t *in);

#ifdef __cplusplus
}
#endif

#endif // APP_H
