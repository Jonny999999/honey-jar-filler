#ifndef GATE_H
#define GATE_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ledc_mode_t     mode;
    ledc_timer_t    timer;
    ledc_channel_t  ch;
    gpio_num_t      pwm_gpio;  // servo PWM signal GPIO
    gpio_num_t      en_gpio;
    float           open_deg;   // e.g., 160
    float           close_deg;  // e.g., 10
} gate_cfg_t;

esp_err_t gate_init(const gate_cfg_t *cfg);
esp_err_t gate_open(void);
esp_err_t gate_close(void);
esp_err_t gate_set(float deg);   // clamp to [close, open]

#ifdef __cplusplus
}
#endif

#endif // GATE_H
