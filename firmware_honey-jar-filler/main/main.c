#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "scale_hx711.h"

static const char *TAG = "main";

// Enable/disable calibration sequence on boot.
#define RUN_CALIBRATION 1

void app_main(void)
{
    ESP_LOGI(TAG, "boot: minimal NVS + HX711 init");

    // Init NVS (required for HX711 calibration persistence).
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Init HX711 wrapper.
    static scale_hx711_t scale;
    ESP_ERROR_CHECK(scale_hx711_init(&scale));

#if RUN_CALIBRATION
    // Simple tare + calibration sequence to force NVS write.
    ESP_LOGI(TAG, "calibration: tare in 2 seconds... -> remove weight");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_ERROR_CHECK(scale_hx711_tare(&scale, 16));

    ESP_LOGI(TAG, "calibration: set 500g in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_ERROR_CHECK(scale_hx711_calibrate(&scale, 500.0f, 16));
#endif

    // Idle forever.
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
