#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

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

    //ESP_LOGI(TAG, "waiting 2s before nvs write...");
    //vTaskDelay(pdMS_TO_TICKS(2000));

    // Simple NVS write test (no HX711).
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open("test", NVS_READWRITE, &h));
    ESP_ERROR_CHECK(nvs_set_u32(h, "counter", 1));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
    ESP_LOGI(TAG, "NVS write test: OK");

    // Idle forever.
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
