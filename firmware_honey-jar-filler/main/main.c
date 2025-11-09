#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "config.h"
#include "iotest.h"

static const char *TAG = "main";

void gpio_init_all(void)
{
    ESP_LOGI(TAG, "Configuring GPIOs (simple)…");

    gpio_config_t io = {0};

    // -------- Inputs (all active-LOW; no internal pulls used) --------
    uint64_t input_mask = 0
        | BIT64(CONFIG_ENCODER_A_GPIO)
        | BIT64(CONFIG_ENCODER_B_GPIO)
        | BIT64(CONFIG_ENCODER_SW_GPIO)
        | BIT64(CONFIG_POS_SWITCH_GPIO)
        | BIT64(CONFIG_BUTTON_1_GPIO)
        | BIT64(CONFIG_BUTTON_2_GPIO);

    io.pin_bit_mask   = input_mask;
    io.mode           = GPIO_MODE_INPUT;
    io.pull_up_en     = GPIO_PULLUP_DISABLE;
    io.pull_down_en   = GPIO_PULLDOWN_DISABLE;
    io.intr_type      = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // -------- Outputs (all push-pull; set LOW after config) --------
    uint64_t output_mask = 0
        | BIT64(CONFIG_BUZZER_GPIO)
        | BIT64(CONFIG_LED1_GPIO)
        | BIT64(CONFIG_LED2_GPIO)
        | BIT64(CONFIG_MOS_VALVE_GPIO)
        | BIT64(CONFIG_MOS_RESERVE_GPIO)
        | BIT64(CONFIG_OPEN_DRAIN_RESERVE_GPIO) // NOTE: currently push-pull
        | BIT64(CONFIG_RELAY_MOTOR_GPIO)
        | BIT64(CONFIG_RELAY_230V_GPIO)         // GPIO2 kept LOW at boot
        | BIT64(CONFIG_SERVO_PWM_GPIO)          // LEDC will re-own later
        | BIT64(CONFIG_SERVO_ENABLE_GPIO);

    io.pin_bit_mask   = output_mask;
    io.mode           = GPIO_MODE_OUTPUT;
    io.pull_up_en     = GPIO_PULLUP_DISABLE;
    io.pull_down_en   = GPIO_PULLDOWN_DISABLE;
    io.intr_type      = GPIO_INTR_DISABLE;
    gpio_config(&io);

    // Safe defaults (everything OFF)
    gpio_set_level(CONFIG_BUZZER_GPIO, 0);
    gpio_set_level(CONFIG_LED1_GPIO, 0);
    gpio_set_level(CONFIG_LED2_GPIO, 0);
    gpio_set_level(CONFIG_MOS_VALVE_GPIO, 0);
    gpio_set_level(CONFIG_MOS_RESERVE_GPIO, 0);
    gpio_set_level(CONFIG_OPEN_DRAIN_RESERVE_GPIO, 0);
    gpio_set_level(CONFIG_RELAY_MOTOR_GPIO, 0);
    gpio_set_level(CONFIG_RELAY_230V_GPIO, 0);  // keep LOW for strap safety
    gpio_set_level(CONFIG_SERVO_PWM_GPIO, 0);   // will be LEDC later
    gpio_set_level(CONFIG_SERVO_ENABLE_GPIO, 0);

    ESP_LOGI(TAG, "GPIOs configured.");
}

void app_main(void)
{

    // 1) init all gpios
    gpio_init_all();

    // 2) start tasks
    xTaskCreatePinnedToCore(iotest_input_monitor_task, "in_mon", 4096, NULL, 1, NULL, 1); // core=1, prio=1
    //xTaskCreatePinnedToCore(iotest_output_toggler_task, "out_chase", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG, "I/O test running: inputs are logged on change; outputs chaser is active.");

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
