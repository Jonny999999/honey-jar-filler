#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "encoder.h"
#include "ssd1306.h"
#include "iot_servo.h"
#include "driver/ledc.h"
#include "motor.h"
#include "gate.h"

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
        //| BIT64(CONFIG_SERVO_PWM_GPIO)          // LEDC will re-own later
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
    //gpio_set_level(CONFIG_SERVO_PWM_GPIO, 0);   // will be LEDC later
    gpio_set_level(CONFIG_SERVO_ENABLE_GPIO, 0);

    ESP_LOGI(TAG, "GPIOs configured.");
}




// Task: log each HX711 scale readout.
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



// Task: log encoder events and button state.
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



// Task: show encoder and scale values on the display.
#define LINE2PIXEL(n) ((n)*8)
#define DISPLAY_UPDATE_INTERVAL_MS 200

typedef struct {
    ssd1306_handle_t disp;
    QueueHandle_t    q_scale;  // scale_sample_t
    QueueHandle_t    q_enc;    // rotary_encoder_event_t
} disp_args_t;

static void task_display(void *arg)
{
    disp_args_t cfg = *(disp_args_t*)arg;
    vPortFree(arg);

    // simple local state
    float   grams = 0.f;
    bool    valid = false;
    int32_t enc_pos = 0;
    int8_t  enc_delta = 0;   // per-frame delta
    uint8_t btn = 0xFF;      // 0xFF = n/a, 0=UP, 1=DOWN
    uint8_t lastLine = 0;

    const TickType_t period = pdMS_TO_TICKS(DISPLAY_UPDATE_INTERVAL_MS);
    TickType_t last = xTaskGetTickCount();

    while(1) {
        //--- drain queues (show freshest data each frame) ---
        // get last scale readout from queue (if used)
        scale_sample_t smp;
        while (cfg.q_scale && xQueueReceive(cfg.q_scale, &smp, 1) == pdPASS) {
            grams = smp.grams;
            valid = smp.valid;
        }

        // also read the latest snapshot (fast-path, no queue)
        scale_latest_t latest;
        scale_latest_get(&latest);
        if (latest.ts_us != 0) {
            grams = latest.grams;
            valid = latest.valid;
        }

        // get last encoder event
        rotary_encoder_event_t ev;
        enc_delta = 0;
        while (cfg.q_enc && xQueueReceive(cfg.q_enc, &ev, 1) == pdPASS) {
            switch (ev.type) {
                case RE_ET_CHANGED:        enc_pos += ev.diff; enc_delta += (int8_t)ev.diff; break;
                case RE_ET_BTN_PRESSED:    btn = 1; break;
                case RE_ET_BTN_RELEASED:   btn = 0; break;
                case RE_ET_BTN_LONG_PRESSED: btn = 1; break;
                default: break;
            }
        }

        //--- draw display content ---
        lastLine = 0;
        char line1[24], line2[32];
        // clear framebuffer:
        ssd1306_clear(cfg.disp);

        // title:
        ssd1306_draw_text_scaled(cfg.disp, 0, LINE2PIXEL(lastLine), "Honey Filler", true, 2);
        lastLine += 2;
        // weight:
        if (valid) snprintf(line1, sizeof(line1), "%.1f g", grams);
        else       snprintf(line1, sizeof(line1), "--.- g");
        ssd1306_draw_text_scaled(cfg.disp, 0, LINE2PIXEL(lastLine), line1, true, 3); // big
        lastLine += 3;
        // encoder data:
        snprintf(line2, sizeof(line2), "Enc:%ld (%+d)  Btn:%s",
                 (long)enc_pos, enc_delta,
                 (btn==0xFF) ? "-" : (btn ? "DOWN" : "UP"));
        ssd1306_draw_text(cfg.disp, 0, LINE2PIXEL(lastLine), line2, true);
        lastLine += 2;

        // flush framebuffer, update display
        ssd1306_display(cfg.disp);

        vTaskDelayUntil(&last, period);
    }
}




// Task: simple servo sweep test.
static void task_servoTest(void *arg)
{
    while(1){
        float angle = 90.0f; // center
        ESP_ERROR_CHECK(gate_set(angle));
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_ERROR_CHECK(gate_close());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    //read back the last set angle (calculated)
}




static void buzzer_beep(uint8_t count, uint32_t on_ms, uint32_t off_ms)
{
    for (uint8_t i = 0; i < count; ++i) {
        gpio_set_level(CONFIG_BUZZER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(CONFIG_BUZZER_GPIO, 0);
        if (i + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
}

static void buzzer_pulse(uint32_t on_ms)
{
    gpio_set_level(CONFIG_BUZZER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    gpio_set_level(CONFIG_BUZZER_GPIO, 0);
}




// Task: button-triggered hardware test sequence (LED, relay, servo, buzzer).
static void task_hardware_test(void *arg)
{
    const TickType_t poll = pdMS_TO_TICKS(10);
    const TickType_t relay_timeout = pdMS_TO_TICKS(5000);
    const int servo_cycles = 1;
    const float servo_fwd = 90.0f;
    const float servo_back = 0.0f;

    buzzer_beep(3, 100, 100);

    while (1) {
        // wait for BUTTON_1 press (active LOW)
        while (gpio_get_level(CONFIG_BUTTON_1_GPIO) != 0) {
            vTaskDelay(poll);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
        if (gpio_get_level(CONFIG_BUTTON_1_GPIO) != 0) {
            continue;
        }

        gpio_set_level(CONFIG_LED1_GPIO, 1);
        gpio_set_level(CONFIG_LED2_GPIO, 1);
        buzzer_beep(1, 100, 100);

        gpio_set_level(CONFIG_RELAY_MOTOR_GPIO, 1);
        TickType_t start = xTaskGetTickCount();
        TickType_t low_start = 0;
        bool seen_high = false;
        bool low_seen = false;
        const TickType_t low_stable_time = pdMS_TO_TICKS(50);
        while (1) {
            int lvl = gpio_get_level(CONFIG_POS_SWITCH_GPIO);
            if (lvl != 0) {
                seen_high = true;
                low_seen = false;
            } else if (seen_high) {
                if (!low_seen) {
                    low_seen = true;
                    low_start = xTaskGetTickCount();
                } else if ((xTaskGetTickCount() - low_start) >= low_stable_time) {
                    break; // falling edge detected (debounced)
                }
            }
            if ((xTaskGetTickCount() - start) >= relay_timeout) {
                break;
            }
            vTaskDelay(poll);
        }
        gpio_set_level(CONFIG_RELAY_MOTOR_GPIO, 0);

        for (int i = 0; i < servo_cycles; ++i) {
            ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, /*channel=*/0, servo_fwd));
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, /*channel=*/0, servo_back));
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        buzzer_beep(2, 100, 100);
        gpio_set_level(CONFIG_LED1_GPIO, 0);
        gpio_set_level(CONFIG_LED2_GPIO, 0);

        // wait for release to avoid re-triggering immediately
        while (gpio_get_level(CONFIG_BUTTON_1_GPIO) == 0) {
            vTaskDelay(poll);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}





// Task: toggle motor with button and beep on POS switch edges.
static void task_pos_switch_test(void *arg)
{
    const TickType_t poll = pdMS_TO_TICKS(10);
    const TickType_t debounce = pdMS_TO_TICKS(30);
    const uint32_t beep_short_ms = 60;
    const uint32_t beep_long_ms = 120;

    bool enabled = false;
    int last_btn = gpio_get_level(CONFIG_BUTTON_1_GPIO);
    int last_pos = gpio_get_level(CONFIG_POS_SWITCH_GPIO);

    gpio_set_level(CONFIG_RELAY_MOTOR_GPIO, 0);

    while (1) {
        int btn = gpio_get_level(CONFIG_BUTTON_1_GPIO);
        if (last_btn != 0 && btn == 0) {
            vTaskDelay(debounce);
            if (gpio_get_level(CONFIG_BUTTON_1_GPIO) == 0) {
                enabled = !enabled;
                gpio_set_level(CONFIG_RELAY_MOTOR_GPIO, enabled ? 1 : 0);
                while (gpio_get_level(CONFIG_BUTTON_1_GPIO) == 0) {
                    vTaskDelay(poll);
                }
            }
        }
        last_btn = btn;

        {
            int pos = gpio_get_level(CONFIG_POS_SWITCH_GPIO);
            if (pos != last_pos) {
                buzzer_pulse(pos ? beep_short_ms : beep_long_ms);
                last_pos = pos;
            }
        }

        vTaskDelay(poll);
    }
}







void app_main(void)
{
    //=======================
    //=== Queues (shared) ===
    //=======================
    // define queues used across multiple tasks
    QueueHandle_t queue_hx711_readouts = NULL;   // created by scale_hx711_start_poll(...)
    static QueueHandle_t queue_encoder_events = NULL;     // created here, used by encoder & UI
    queue_encoder_events = xQueueCreate(16, sizeof(rotary_encoder_event_t));
    assert(queue_encoder_events);


    //==================
    //==== init nvs ====
    //==================
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }


    //=============================
    //=== init Digital IO gpios ===
    //=============================
    gpio_init_all();

    //===================
    //=== Motor relay ===
    //===================
    motor_init(CONFIG_RELAY_MOTOR_GPIO);


    //===================
    //=== HX711 scale ===
    //===================
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



    //======================
    //=== Rotary Encoder ===
    //======================
    // Init the encoder library with that queue
    ESP_ERROR_CHECK(rotary_encoder_init(queue_encoder_events));

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




    //===============
    //=== Display ===
    //===============
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_DISPLAY_SDA_GPIO,
        .scl_io_num = CONFIG_DISPLAY_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_cfg, &bus_handle);
    ssd1306_config_t cfg = {
        .bus = SSD1306_I2C,
        .width = 128,
        .height = 64,
        .iface.i2c = {
            .port = I2C_NUM_0,
            .addr = 0x3C,
            .rst_gpio = GPIO_NUM_NC,
        },
    };
    ssd1306_handle_t disp;
    ssd1306_new_i2c(&cfg, &disp);
    ssd1306_clear(disp);
    ssd1306_draw_text(disp, 0, 0, "INITIALIZED", true);
    ssd1306_display(disp);
    //ssd1306_draw_text_scaled(disp, 0, 4, "SSD1306 I2C", true, 2);




    //=======================
    //==== Gate (Servo) =====
    //=======================
    gate_cfg_t gate_cfg = {
        .mode = LEDC_LOW_SPEED_MODE,
        .timer = LEDC_TIMER_0,
        .ch = LEDC_CHANNEL_0,
        .pwm_gpio = CONFIG_SERVO_PWM_GPIO,
        .en_gpio = CONFIG_SERVO_ENABLE_GPIO,
        .open_deg = 160.0f,
        .close_deg = 10.0f,
    };
    ESP_ERROR_CHECK(gate_init(&gate_cfg));






    //=======================
    //=== Start all tasks ===
    //=======================
    //--- IO-test tasks ---
    #if DEBUG_RUN_IOTEST
    xTaskCreatePinnedToCore(iotest_input_monitor_task, "in_mon", 4096, NULL, 1, NULL, 1); // core=1, prio=1
    xTaskCreatePinnedToCore(iotest_output_toggler_task, "out_chase", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    #endif

    //--- weight scale task ---
    // start producer - constantly reads HX711 and updates a queue
    ESP_ERROR_CHECK(scale_hx711_start_poll(
        &scale,
        CONFIG_HX711_AVG_SAMPLE_COUNT, //samples_avg
        pdMS_TO_TICKS(CONFIG_HX711_POLL_INTERVAL_MS), //sample period
        1,    // queue length - “latest only"
        &queue_hx711_readouts));
    // simple task consuming/logging all readouts
    #if DEBUG_LOG_SCALE_READOUTS
    xTaskCreatePinnedToCore(task_scale_log_readouts, "scale_cons", 4096, (void*)queue_hx711_readouts, 2, NULL, 1);
    #endif

    //--- encoder log task---
    // simple task to consume and log encoder events
    #if DEBUG_LOG_ENCODER_EVENTS
    xTaskCreatePinnedToCore(task_enc_consumer, "enc_consumer", 3072, queue_encoder_events, 4, NULL, 1);
    #endif

    //--- display task ---
    // notes: task conflicts with debug logging tasks above since those take out the events from queue already
    disp_args_t *args = pvPortMalloc(sizeof(*args));
    args->disp    = disp;
    args->q_scale = queue_hx711_readouts;     // recommend queue_len = 1 (latest sample)
    args->q_enc   = queue_encoder_events; // from rotary_encoder_init()
    xTaskCreatePinnedToCore(task_display, "ui_display", 4096, args, 4, NULL, 1);

    //--- servo test task ---
#define DEBUG_RUN_SERVO_TEST 0
    #if DEBUG_RUN_SERVO_TEST
    xTaskCreatePinnedToCore(task_servoTest, "servo_test", 3072, queue_encoder_events, 4, NULL, 1);
    #endif

    //--- hardware test task ---
#define DEBUG_RUN_HARDWARE_TEST 1
    #if DEBUG_RUN_HARDWARE_TEST
    xTaskCreatePinnedToCore(task_hardware_test, "hw_test", 4096, NULL, 4, NULL, 1);
    #endif

    //--- pos switch edge + toggle motor test task ---
#define DEBUG_RUN_POS_SWITCH_EDGE_TEST 0
    #if DEBUG_RUN_POS_SWITC_EDGE_TEST
    xTaskCreatePinnedToCore(task_pos_switch_test, "motor_edge_test", 3072, NULL, 4, NULL, 1);
    #endif








    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
