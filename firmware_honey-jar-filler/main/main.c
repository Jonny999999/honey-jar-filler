#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "scale_hx711.h"

#include "nvs.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "boot: minimal NVS write test");


    // Init NVS (required for HX711 calibration persistence).
    //ESP_LOGI(TAG, "waiting 2s before nvs init...");
    //vTaskDelay(pdMS_TO_TICKS(2000));
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }


// Issue with crash on nvs write located: strapping pin was pulled high by hx711 module resuling in wrong flash voltage selected
// -> re-routed to gpio 19 using jumper wire
    
#define SIMPLE_WRITE
#ifdef SIMPLE_WRITE
    // Simple NVS write test (no HX711).

    //ESP_LOGI(TAG, "waiting 2s before nvs write...");
    //vTaskDelay(pdMS_TO_TICKS(2000));

    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open("test", NVS_READWRITE, &h));
    uint32_t counter = 0;
    while (1) {
        counter++;
        ESP_ERROR_CHECK(nvs_set_u32(h, "counter", counter));
        ESP_ERROR_CHECK(nvs_commit(h));
        ESP_LOGI(TAG, "NVS write OK: %u", (unsigned)counter);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

#else

    // Init HX711 wrapper.
    static scale_hx711_t scale;
    ESP_ERROR_CHECK(scale_hx711_init(&scale));

    // Simple tare + calibration sequence to force NVS write.
    ESP_LOGI(TAG, "calibration: tare in 2 seconds... -> remove weight");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(scale_hx711_tare(&scale, 16));

    ESP_LOGI(TAG, "calibration: set 500g in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(scale_hx711_calibrate(&scale, 500.0f, 16));

#endif




    // Idle forever.
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
