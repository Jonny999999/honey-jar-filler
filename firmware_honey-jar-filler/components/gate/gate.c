#include "gate.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "iot_servo.h"

// Servo configuration defaults (standard analog RC servo).
#define SERVO_MAX_ANGLE   180
#define SERVO_MIN_US      500
#define SERVO_MAX_US      2500
#define SERVO_FREQ_HZ     50

// Static runtime config for the gate.
static gate_cfg_t s_cfg;
static bool s_inited = false;
static bool s_enabled = true;
static const char *TAG = "gate";

static float gate_min_deg(void)
{
    return (s_cfg.open_deg < s_cfg.close_deg) ? s_cfg.open_deg : s_cfg.close_deg;
}

static float gate_max_deg(void)
{
    return (s_cfg.open_deg > s_cfg.close_deg) ? s_cfg.open_deg : s_cfg.close_deg;
}

static float gate_clamp(float deg)
{
    float lo = gate_min_deg();
    float hi = gate_max_deg();
    if (deg < lo) return lo;
    if (deg > hi) return hi;
    return deg;
}

esp_err_t gate_init(const gate_cfg_t *cfg)
{
    if (!cfg) {
        ESP_LOGW(TAG, "init: null config");
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = *cfg;

    ESP_LOGI(TAG, "init: mode=%d timer=%d ch=%d pwm_gpio=%d en_gpio=%d open=%.1f close=%.1f",
             (int)s_cfg.mode, (int)s_cfg.timer, (int)s_cfg.ch,
             (int)s_cfg.pwm_gpio, (int)s_cfg.en_gpio,
             (double)s_cfg.open_deg, (double)s_cfg.close_deg);

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
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "init: iot_servo_init failed (%d)", err);
        return err;
    }

    // Drive a known-safe PWM angle before enabling servo power.
    float safe_deg = gate_clamp(s_cfg.close_deg);
    ESP_LOGD(TAG, "init: pre-power angle=%.1f (clamped)", (double)safe_deg);
    err = iot_servo_write_angle(s_cfg.mode, s_cfg.ch, safe_deg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "init: pre-power write failed (%d)", err);
        return err;
    }

    // Enable power only after PWM is configured and set.
    if (s_enabled) {
        gpio_set_level(s_cfg.en_gpio, 1);
        s_inited = true;
        ESP_LOGI(TAG, "init: power enabled, closing gate");
        return gate_close();
    } else {
        gpio_set_level(s_cfg.en_gpio, 0);
        s_inited = true;
        ESP_LOGI(TAG, "init: power disabled (gate disabled)");
        return ESP_OK;
    }
}

esp_err_t gate_set(float deg)
{
    if (!s_inited) {
        ESP_LOGW(TAG, "set: called before init");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_enabled) {
        ESP_LOGD(TAG, "set ignored (disabled)");
        return ESP_OK;
    }

    float clamped = gate_clamp(deg);
    ESP_LOGI(TAG, "set: req=%.1f", (double)deg);
    ESP_LOGD(TAG, "set: clamped=%.1f (range %.1f..%.1f)",
             (double)clamped, (double)gate_min_deg(), (double)gate_max_deg());
    return iot_servo_write_angle(s_cfg.mode, s_cfg.ch, clamped);
}

esp_err_t gate_set_percent(float pct)
{
    if (!s_inited) {
        ESP_LOGW(TAG, "set_percent: called before init");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_enabled) {
        ESP_LOGD(TAG, "set_percent ignored (disabled)");
        return ESP_OK;
    }

    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    // Map 0..100% to close..open (inversion handled by open/close ordering).
    float span = s_cfg.open_deg - s_cfg.close_deg;
    float deg = s_cfg.close_deg + (span * (pct / 100.0f));
    ESP_LOGI(TAG, "set_percent: pct=%.1f", (double)pct);
    ESP_LOGD(TAG, "set_percent: deg=%.1f (close=%.1f open=%.1f)",
             (double)deg, (double)s_cfg.close_deg, (double)s_cfg.open_deg);
    return gate_set(deg);
}

esp_err_t gate_open(void)
{
    if (!s_inited) {
        ESP_LOGE(TAG, "open: called before init");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_enabled) {
        ESP_LOGD(TAG, "open ignored (disabled)");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "open");
    return gate_set(s_cfg.open_deg);
}

esp_err_t gate_close(void)
{
    if (!s_inited) {
        ESP_LOGW(TAG, "close: called before init");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_enabled) {
        ESP_LOGD(TAG, "close ignored (disabled)");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "close");
    return gate_set(s_cfg.close_deg);
}

void gate_set_enabled(bool enable)
{
    s_enabled = enable;
    if (!s_inited) return;

    if (!enable) {
        gpio_set_level(s_cfg.en_gpio, 0);
        ESP_LOGI(TAG, "disabled (power off)");
    } else {
        gpio_set_level(s_cfg.en_gpio, 1);
        ESP_LOGI(TAG, "enabled (power on)");
    }
}

bool gate_is_enabled(void)
{
    return s_enabled;
}
