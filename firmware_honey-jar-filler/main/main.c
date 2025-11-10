#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "encoder.h"

#include "config.h"
#include "iotest.h"
#include "scale_hx711.h"

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




// Task for consuming hx711 readout queue and log the new samples
static void task_scale_log_readouts(void *arg)
{
    QueueHandle_t queue_hx711_readouts = (QueueHandle_t)arg;
    scale_sample_t s;

    while(1){
        if (xQueueReceive(queue_hx711_readouts, &s, portMAX_DELAY) == pdPASS) {
            ESP_LOGI("scale_consumer", "received new measurement: t=%lld ms  raw=%" PRId32 "  %.2f g %s",
                     (long long)s.ts_us/1000, s.raw, s.grams, s.valid ? "" : "(not-calibrated!)");
        }
    }
}


// Task for consuming encoder queue and logging all events
static void task_enc_consumer(void *arg)
{
    static const char *TAG = "encoder_consumer";
    QueueHandle_t q = (QueueHandle_t)arg;
    rotary_encoder_event_t ev;

    int32_t position = 0; // simple local pos accumulator

    for (;;) {
        if (xQueueReceive(q, &ev, portMAX_DELAY) != pdPASS) continue;
        switch (ev.type) {
        case RE_ET_CHANGED:
            position += ev.diff; // diff is +-1 per step (or more if accelerated)
            ESP_LOGI(TAG, "STEP %+d  pos=%ld", (int)ev.diff, (long)position);
            break;
        case RE_ET_BTN_PRESSED:
            ESP_LOGI(TAG, "BTN: pressed");
            break;
        case RE_ET_BTN_LONG_PRESSED:
            ESP_LOGI(TAG, "BTN: long pressed");
            break;
        case RE_ET_BTN_RELEASED:
            ESP_LOGI(TAG, "BTN: released");
            break;
        case RE_ET_BTN_CLICKED:
            ESP_LOGI(TAG, "BTN: clicked");
            break;
        default:
            break;
        }
    }
}




void app_main(void)
{
    // init nvs
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    //=== init all gpios ===
    gpio_init_all();


    // === start IO-test tasks ===
    //xTaskCreatePinnedToCore(iotest_input_monitor_task, "in_mon", 4096, NULL, 1, NULL, 1); // core=1, prio=1
    //xTaskCreatePinnedToCore(iotest_output_toggler_task, "out_chase", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG, "I/O test running: inputs are logged on change; outputs chaser is active.");



    //=== HX711 scale ===
    // init HX711 wrapper
    static scale_hx711_t scale;
    ESP_ERROR_CHECK(scale_hx711_init(&scale));

#ifdef SCALE_RUN_CALIBRATION //TODO: trigger this with UI
    // 1. zero readout (determine offset)
    printf("calibration: tare in 2 seconds... -> remove weight from scale\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_ERROR_CHECK(scale_hx711_tare(&scale, 16));
    // 2. calibrate (determine scale)
    printf("calibrating to 500g in 2 seconds... -> pace weight\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_ERROR_CHECK(scale_hx711_calibrate(&scale, 500.0f, 16));
#endif

    // start producer - constantly reads HX711 and updates a queue
    QueueHandle_t queue_hx711_readouts = NULL;
    ESP_ERROR_CHECK(scale_hx711_start_poll(&scale,
                                           CONFIG_HX711_AVG_SAMPLE_COUNT, //samples_avg
                                           pdMS_TO_TICKS(CONFIG_HX711_POLL_INTERVAL_MS), //sample period
                                           1,    // queue length - “latest only"
                                           &queue_hx711_readouts));

    // start task consuming/logging all readouts
    //xTaskCreatePinnedToCore(task_scale_log_readouts, "scale_cons", 4096, (void*)queue_hx711_readouts, 2, NULL, 1);





    //=== Rotary Encoder ===
    // Create the events queue 
    static QueueHandle_t s_enc_queue = NULL;
    s_enc_queue = xQueueCreate(16, sizeof(rotary_encoder_event_t));
    assert(s_enc_queue);

    // Init the encoder library with that queue
    ESP_ERROR_CHECK(rotary_encoder_init(s_enc_queue));

    // Describe encoder pins (button optional)
    static rotary_encoder_t enc = {
        .pin_a  = CONFIG_ENCODER_A_GPIO,      // e.g. GPIO36
        .pin_b  = CONFIG_ENCODER_B_GPIO,      // e.g. GPIO39
        .pin_btn= CONFIG_ENCODER_SW_GPIO,     // e.g. GPIO34, or >=GPIO_NUM_MAX if none
    };
    // Register encoder
    ESP_ERROR_CHECK(rotary_encoder_add(&enc));

    // (Optional) Acceleration for faster turning feel
    //ESP_ERROR_CHECK(rotary_encoder_enable_acceleration(&enc, 400));

    // Start consumer task
    xTaskCreatePinnedToCore(task_enc_consumer, "enc_consumer", 3072,
                            s_enc_queue, 4, NULL, 1);



    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
