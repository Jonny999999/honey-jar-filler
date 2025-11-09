// io_test.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>
#include <inttypes.h>

#include "iotest.h"
// --- your gpio_init_all() is assumed to be called from app_main() before tasks ---

static const char *TAG = "io_test";

// --------- Define the I/O lists (names for logs) ---------
static const gpio_num_t s_inputs[] = {
    CONFIG_ENCODER_A_GPIO,
    CONFIG_ENCODER_B_GPIO,
    CONFIG_ENCODER_SW_GPIO,
    CONFIG_POS_SWITCH_GPIO,
    CONFIG_BUTTON_1_GPIO,
    CONFIG_BUTTON_2_GPIO,
};
static const char *s_in_names[] = {
    "ENC_A", "ENC_B", "ENC_SW", "POS_SW", "BTN1", "BTN2"
};

static const gpio_num_t s_outputs[] = {
    CONFIG_LED1_GPIO,
    CONFIG_LED2_GPIO,
    CONFIG_BUZZER_GPIO,
    CONFIG_MOS_VALVE_GPIO,
    CONFIG_MOS_RESERVE_GPIO,
    CONFIG_OPEN_DRAIN_RESERVE_GPIO,
    CONFIG_RELAY_MOTOR_GPIO,
    CONFIG_RELAY_230V_GPIO,
    CONFIG_SERVO_ENABLE_GPIO,
    // CONFIG_SERVO_PWM_GPIO,
};
static const char *s_out_names[] = {
    "LED1", "LED2", "BUZZ", "MOS_VALVE", "MOS_RES", "OD_RES",
    "RELAY_MOTOR", "RELAY_230V", "SERVO_EN"
};



// --------- Input monitor ---------
// poll all inputs and log on change
void iotest_input_monitor_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(10); // 1 tick @ 100 Hz
    const size_t N = sizeof(s_inputs) / sizeof(s_inputs[0]);

    uint8_t prev[N];

    // initial snapshot + print
    for (size_t i = 0; i < N; ++i) {
        int lvl = gpio_get_level(s_inputs[i]);
        prev[i] = (uint8_t)lvl;
        ESP_LOGI(TAG, "IN %s (GPIO%u) start=%s",
                 s_in_names[i], (unsigned)s_inputs[i], lvl ? "HIGH" : "LOW");
    }

    for (;;) {
        vTaskDelay(period); // always yield ≥1 tick
        for (size_t i = 0; i < N; ++i) {
            uint8_t lvl = (uint8_t)gpio_get_level(s_inputs[i]);
            if (lvl != prev[i]) {
                prev[i] = lvl;
                ESP_LOGI(TAG, "IN %s (GPIO%u) -> %s",
                         s_in_names[i], (unsigned)s_inputs[i], lvl ? "HIGH" : "LOW");
            }
        }
    }
}



// --------- Output toggler (sequential on/off with log) ---------
void iotest_output_toggler_task(void *arg)
{
    const TickType_t on_time  = pdMS_TO_TICKS(500);
    const TickType_t off_time = pdMS_TO_TICKS(500);

    // ensure all LOW (safe) at start
    for (size_t i = 0; i < sizeof(s_outputs)/sizeof(s_outputs[0]); ++i) {
        gpio_set_level(s_outputs[i], 0);
    }
    gpio_set_level(CONFIG_SERVO_PWM_GPIO, 0); // keep PWM line quiet

    while (1) {
        for (size_t i = 0; i < sizeof(s_outputs)/sizeof(s_outputs[0]); ++i) {
            ESP_LOGI(TAG, "OUT %s (GPIO%u) = ON", s_out_names[i], (unsigned)s_outputs[i]);
            gpio_set_level(s_outputs[i], 1);
            vTaskDelay(on_time);

            ESP_LOGI(TAG, "OUT %s (GPIO%u) = OFF", s_out_names[i], (unsigned)s_outputs[i]);
            gpio_set_level(s_outputs[i], 0);
            vTaskDelay(off_time);
        }
    }
}
