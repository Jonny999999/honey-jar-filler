#include "gate.h"

#include "driver/gpio.h"
#include "iot_servo.h"

// Servo configuration defaults (standard analog RC servo).
#define SERVO_MAX_ANGLE   180
#define SERVO_MIN_US      500
#define SERVO_MAX_US      2500
#define SERVO_FREQ_HZ     50

// Static runtime config for the gate.
static gate_cfg_t s_cfg;
static bool s_inited = false;

static float gate_clamp(float deg)
{
    if (deg < s_cfg.close_deg) return s_cfg.close_deg;
    if (deg > s_cfg.open_deg)  return s_cfg.open_deg;
    return deg;
}

esp_err_t gate_init(const gate_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    s_cfg = *cfg;

    // Configure enable pin; keep power off until servo is initialized.
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_cfg.en_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(s_cfg.en_gpio, 0);

    // Configure LEDC/servo channel.
    servo_config_t servo_cfg = {
        .max_angle = SERVO_MAX_ANGLE,
        .min_width_us = SERVO_MIN_US,
        .max_width_us = SERVO_MAX_US,
        .freq = SERVO_FREQ_HZ,
        .timer_number = s_cfg.timer,
        .channels = {
            .servo_pin = {s_cfg.pwm_gpio},
            .ch = {s_cfg.ch},
        },
        .channel_number = 1,
    };

    esp_err_t err = iot_servo_init(s_cfg.mode, &servo_cfg);
    if (err != ESP_OK) return err;

    // Power the servo and move to a known safe state.
    gpio_set_level(s_cfg.en_gpio, 1);
    s_inited = true;
    return gate_close();
}

esp_err_t gate_set(float deg)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    float clamped = gate_clamp(deg);
    return iot_servo_write_angle(s_cfg.mode, s_cfg.ch, clamped);
}

esp_err_t gate_open(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    return gate_set(s_cfg.open_deg);
}

esp_err_t gate_close(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    return gate_set(s_cfg.close_deg);
}
