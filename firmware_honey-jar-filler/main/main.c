#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
//#include "esp_rom_sys.h"
#include "sdkconfig.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "encoder.h"
#include "ssd1306.h"
#include "led_strip.h"
#include "iot_servo.h"
#include "driver/ledc.h"
#include "motor.h"
#include "gate.h"
#include "ui_task.h"
#include "filler_fsm.h"
#include "buzzer.h"

#include "config.h"
#include "iotest.h"
#include "scale_hx711.h"
#include "app.h"

static const char *TAG = "main";


static led_strip_handle_t s_strip;

static uint8_t ws2812_scale(uint8_t v)
{
    return (uint8_t)((v * CONFIG_WS2812_MAX_BRIGHTNESS_PCT) / 100);
}

static void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_strip) return;
    r = ws2812_scale(r);
    g = ws2812_scale(g);
    b = ws2812_scale(b);
    for (int i = 0; i < CONFIG_WS2812_LED_COUNT; ++i) {
        led_strip_set_pixel(s_strip, i, r, g, b);
    }
    (void)led_strip_refresh(s_strip);
}

static void ws2812_clear(void)
{
    if (!s_strip) return;
    (void)led_strip_clear(s_strip);
}

// Startup LED strip self-test (blocking, runs once).
static void ws2812_startup_sequence(void)
{
    if (!s_strip) return;
    const TickType_t delay = pdMS_TO_TICKS(20);

    // Fast lap: red -> green -> blue gradient chase.
    for (int i = 0; i < CONFIG_WS2812_LED_COUNT; ++i) {
        ws2812_clear();
        uint8_t r = (uint8_t)(255 - (i * 255 / (CONFIG_WS2812_LED_COUNT ? CONFIG_WS2812_LED_COUNT : 1)));
        uint8_t g = (uint8_t)(i * 255 / (CONFIG_WS2812_LED_COUNT ? CONFIG_WS2812_LED_COUNT : 1));
        uint8_t b = (uint8_t)(i * 255 / (CONFIG_WS2812_LED_COUNT ? CONFIG_WS2812_LED_COUNT : 1));
        led_strip_set_pixel(s_strip, i, ws2812_scale(r), ws2812_scale(g), ws2812_scale(b));
        (void)led_strip_refresh(s_strip);
        vTaskDelay(delay);
    }

    // Loading bar: fill to idle color one LED at a time.
    ws2812_clear();
    for (int i = 0; i < CONFIG_WS2812_LED_COUNT; ++i) {
        led_strip_set_pixel(s_strip, i, ws2812_scale(0), ws2812_scale(0), ws2812_scale(255));
        (void)led_strip_refresh(s_strip);
        vTaskDelay(delay);
    }
}

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
        // 100% = open, 0% = closed (works with inverted mechanics).
        ESP_ERROR_CHECK(gate_set_percent(100.0f));
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_ERROR_CHECK(gate_set_percent(0.0f));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    //read back the last set angle (calculated)
}





// Task: raw servo calibration using encoder + OLED (no gate clamp).
typedef struct {
    ssd1306_handle_t disp;
    QueueHandle_t    q_enc;
} gate_calib_args_t;

static void task_gate_calibrate(void *arg)
{
    gate_calib_args_t cfg = *(gate_calib_args_t*)arg;
    vPortFree(arg);

    float angle = 90.0f;     // start in a safe mid position
    const float step = 1.0f; // degrees per encoder tick
    bool dirty = true;

    ESP_LOGI(TAG, "gate_cal: start angle=%.1f deg", (double)angle);
    (void)iot_servo_write_angle(LEDC_LOW_SPEED_MODE, /*channel=*/0, angle);

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);

    while (1) {
        rotary_encoder_event_t ev;
        if (xQueueReceive(cfg.q_enc, &ev, pdMS_TO_TICKS(20)) == pdPASS) {
            if (ev.type == RE_ET_CHANGED) {
                angle += (float)ev.diff * step;
                if (angle < 0.0f) angle = 0.0f;
                if (angle > 180.0f) angle = 180.0f;
                ESP_LOGI(TAG, "gate_cal: angle=%.1f deg", (double)angle);
                (void)iot_servo_write_angle(LEDC_LOW_SPEED_MODE, /*channel=*/0, angle);
                dirty = true;
            }
        }

        if (dirty && (xTaskGetTickCount() - last) >= period) {
            last = xTaskGetTickCount();
            dirty = false;

            char line1[24];
            ssd1306_clear(cfg.disp);
            ssd1306_draw_text_scaled(cfg.disp, 0, LINE2PIXEL(0), "Gate Cal", true, 2);
            snprintf(line1, sizeof(line1), "Angle: %5.1f", (double)angle);
            ssd1306_draw_text_scaled(cfg.disp, 0, LINE2PIXEL(3), line1, true, 2);
            ssd1306_display(cfg.disp);
        }
    }
}




// Task: button-triggered hardware test sequence (LED, relay, servo, buzzer).
static void task_hardware_test(void *arg)
{
    const TickType_t poll = pdMS_TO_TICKS(10);
    const TickType_t relay_timeout = pdMS_TO_TICKS(5000);
    const int servo_cycles = 1;

    buzzer_beep_count(3, 100, 100);

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
        buzzer_beep_count(1, 100, 100);

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
            ESP_ERROR_CHECK(gate_set_percent(100));
            vTaskDelay(pdMS_TO_TICKS(1500));
            ESP_ERROR_CHECK(gate_set_percent(0));
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        buzzer_beep_count(2, 100, 100);
        gpio_set_level(CONFIG_LED1_GPIO, 0);
        gpio_set_level(CONFIG_LED2_GPIO, 0);

        // wait for release to avoid re-triggering immediately
        while (gpio_get_level(CONFIG_BUTTON_1_GPIO) == 0) {
            vTaskDelay(poll);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}




// Task: toggle gate between 0% and 100% on button press.
static void task_gate_toggle_test(void *arg)
{
    const TickType_t poll = pdMS_TO_TICKS(10);
    const TickType_t debounce = pdMS_TO_TICKS(30);
    bool is_open = false;

    // Start closed.
    ESP_ERROR_CHECK(gate_set_percent(0.0f));

    while (1) {
        if (gpio_get_level(CONFIG_BUTTON_1_GPIO) == 0) {
            vTaskDelay(debounce);
            if (gpio_get_level(CONFIG_BUTTON_1_GPIO) == 0) {
                is_open = !is_open;
                ESP_ERROR_CHECK(gate_set_percent(is_open ? 100.0f : 0.0f));
                // Wait for release to avoid repeat toggles.
                while (gpio_get_level(CONFIG_BUTTON_1_GPIO) == 0) {
                    vTaskDelay(poll);
                }
            }
        }
        vTaskDelay(poll);
    }
}

// Task: WS2812B strip test (cycle colors).
static void task_ws2812_test(void *arg)
{
    (void)arg;
    const TickType_t chase_delay = pdMS_TO_TICKS(30);
    const TickType_t cycle_delay = pdMS_TO_TICKS(400);
    const int block_len = 3;
    int idx = 0;
    while (1) {
        // Sequence 1: red chase, two full laps.
        for (int lap = 0; lap < 3; ++lap) {
            for (int i = 0; i < CONFIG_WS2812_LED_COUNT; ++i) {
                ws2812_clear();
                if (s_strip) {
                    led_strip_set_pixel(s_strip, i, ws2812_scale(255), 0, 0);
                    (void)led_strip_refresh(s_strip);
                }
                vTaskDelay(chase_delay);
            }
        }

        // Sequence 2: "working" pattern (green with darker green block).
        for (int step = 0; step < (CONFIG_WS2812_LED_COUNT * 3); ++step) {
            if (s_strip) {
                for (int i = 0; i < CONFIG_WS2812_LED_COUNT; ++i) {
                    int rel = (i - idx + CONFIG_WS2812_LED_COUNT) % CONFIG_WS2812_LED_COUNT;
                    bool in_block = (rel < block_len);
                    uint8_t g = in_block ? 80 : 255;
                    led_strip_set_pixel(s_strip, i, 0, ws2812_scale(g), 0);
                }
                (void)led_strip_refresh(s_strip);
            }
            idx = (idx + 1) % CONFIG_WS2812_LED_COUNT;
            vTaskDelay(chase_delay);
        }

        // Sequence 3: full-strip color cycle.
        ws2812_set_all(255, 0, 0);
        vTaskDelay(cycle_delay);
        ws2812_set_all(0, 255, 0);
        vTaskDelay(cycle_delay);
        ws2812_set_all(0, 0, 255);
        vTaskDelay(cycle_delay);
        ws2812_set_all(255, 255, 255);
        vTaskDelay(cycle_delay);
        //ws2812_clear();
        //vTaskDelay(cycle_delay);
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
                buzzer_beep_count(1, pos ? beep_short_ms : beep_long_ms, 0);
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
    // Default to warnings; bump specific components as needed.
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set("main", ESP_LOG_WARN);
    esp_log_level_set("app_params", ESP_LOG_INFO);
    esp_log_level_set("scale_hx711", ESP_LOG_WARN);
    esp_log_level_set("ui_task", ESP_LOG_INFO);
    esp_log_level_set("ui_menu", ESP_LOG_INFO);
    esp_log_level_set("gate", ESP_LOG_WARN);
    esp_log_level_set("buzzer", ESP_LOG_WARN);
    esp_log_level_set("io_test", ESP_LOG_WARN);
    esp_log_level_set("encoder_consumer", ESP_LOG_WARN);
    esp_log_level_set("scale_consumer", ESP_LOG_WARN);
    esp_log_level_set("filler_fsm", ESP_LOG_DEBUG);

    // Load persistent app parameters (targets/timeouts).
    app_params_init();


    //=============================
    //=== init Digital IO gpios ===
    //=============================
    gpio_init_all();


    //=====================
    //=== Buzzer output ===
    //=====================
    buzzer_init(CONFIG_BUZZER_GPIO);
    if (CONFIG_DISABLE_BUZZER) {
        ESP_LOGW(TAG, "Note: buzzer disabled by config");
        buzzer_set_enabled(false);
    }


    //=====================
    //=== WS2812 Strip ====
    //=====================
    #if CONFIG_WS2812_ENABLE
        led_strip_config_t strip_cfg = {
            .strip_gpio_num = CONFIG_WS2812_GPIO,
            .max_leds = CONFIG_WS2812_LED_COUNT,
            .led_model = LED_MODEL_WS2812,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .flags.invert_out = false,
        };
        led_strip_rmt_config_t rmt_cfg = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,
            .flags.with_dma = false,
        };
        if (led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip) != ESP_OK) {
            ESP_LOGW(TAG, "WS2812 init failed");
        } else {
            ws2812_clear();
        }
    #else
        ESP_LOGW(TAG, "WS2812 disabled by config");
    #endif


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

    #define SCALE_RUN_CALIBRATION 0
#if SCALE_RUN_CALIBRATION //TODO: trigger this with UI
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
        // If mechanics are inverted, set open_deg < close_deg.
        .open_deg = 4.0f,
        .close_deg = 102.5f,
    };
    ESP_ERROR_CHECK(gate_init(&gate_cfg));
    if (CONFIG_DISABLE_SERVO) {
        ESP_LOGW(TAG, "Note: gate disabled by config");
        gate_set_enabled(false);
    }






    //=======================
    //=== Start all tasks ===
    //=======================
    //--- buzzer task ---
    buzzer_task_start(3);


    // --- STARTUP SEQUENCE ---
    buzzer_beep_short(3);
    if (CONFIG_WS2812_ENABLE && s_strip) {
        ws2812_startup_sequence();
    }


    //--- weight scale task ---
    // start producer - constantly reads HX711 and updates a queue
    ESP_ERROR_CHECK(scale_hx711_start_poll(
        &scale,
        CONFIG_HX711_AVG_SAMPLE_COUNT, //samples_avg
        pdMS_TO_TICKS(CONFIG_HX711_POLL_INTERVAL_MS), //sample period
        1,    // queue length - “latest only"
        &queue_hx711_readouts));


    //--- debug run modes: pick ONE for safety ---
    #define DEBUG_MODE_NONE         0
    #define DEBUG_MODE_GATE_CALIB   1  // raw servo control with encoder + OLED angle readout
    #define DEBUG_MODE_SERVO_SWEEP  2  // simple gate open/close sweep
    #define DEBUG_MODE_HARDWARE     3  // button-triggered IO test sequence (motor -> pos -> servo)
    #define DEBUG_MODE_GATE_TOGGLE  4  // button toggles gate 0%/100%
    #define DEBUG_MODE_POS_SWITCH   5  // beep on pos switch edge + toggle motor with button
    #define DEBUG_MODE_IOTEST       6  // monitor inputs, cycle through outputs
    #define DEBUG_MODE_WS2812_TEST  7  // WS2812 strip test pattern
    #define DEBUG_MODE_LOG_SCALE    8  // log scale readouts
    #define DEBUG_MODE_LOG_ENCODER  9  // log encoder events

    // select active mode:
    #define DEBUG_MODE DEBUG_MODE_NONE

    //--- one-of test tasks ---
    #if DEBUG_MODE == DEBUG_MODE_GATE_CALIB
    gate_calib_args_t *cal_args = pvPortMalloc(sizeof(*cal_args));
    cal_args->disp  = disp;
    cal_args->q_enc = queue_encoder_events;
    xTaskCreatePinnedToCore(task_gate_calibrate, "gate_cal", 4096, cal_args, 4, NULL, 1);
    #elif DEBUG_MODE == DEBUG_MODE_SERVO_SWEEP
    xTaskCreatePinnedToCore(task_servoTest, "servo_test", 3072, queue_encoder_events, 4, NULL, 1);
    #elif DEBUG_MODE == DEBUG_MODE_HARDWARE
    xTaskCreatePinnedToCore(task_hardware_test, "hw_test", 4096, NULL, 4, NULL, 1);
    #elif DEBUG_MODE == DEBUG_MODE_GATE_TOGGLE
    xTaskCreatePinnedToCore(task_gate_toggle_test, "gate_toggle", 3072, NULL, 4, NULL, 1);
    #elif DEBUG_MODE == DEBUG_MODE_POS_SWITCH
    xTaskCreatePinnedToCore(task_pos_switch_test, "motor_edge_test", 3072, NULL, 4, NULL, 1);
    //--- IO-test tasks ---
    #elif DEBUG_MODE == DEBUG_MODE_IOTEST
    xTaskCreatePinnedToCore(iotest_input_monitor_task, "in_mon", 4096, NULL, 1, NULL, 1); // core=1, prio=1
    xTaskCreatePinnedToCore(iotest_output_toggler_task, "out_chase", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    #elif DEBUG_MODE == DEBUG_MODE_WS2812_TEST
    xTaskCreatePinnedToCore(task_ws2812_test, "ws2812_test", 3072, NULL, 3, NULL, 1);
    #elif DEBUG_MODE == DEBUG_MODE_LOG_SCALE
    xTaskCreatePinnedToCore(task_scale_log_readouts, "scale_cons", 4096, (void*)queue_hx711_readouts, 2, NULL, 1);
    #elif DEBUG_MODE == DEBUG_MODE_LOG_ENCODER
    xTaskCreatePinnedToCore(task_enc_consumer, "enc_consumer", 3072, queue_encoder_events, 4, NULL, 1);

    
    #else // when not in any debug mode, normal startup (FSM + UI task)

    //--- filler FSM task ---
    filler_start_task(CONFIG_TASK_PRIO_FSM, CONFIG_TASK_CORE_FSM);

    //--- UI task ---
    ui_task_set_encoder_queue(queue_encoder_events);
    if (CONFIG_WS2812_ENABLE && s_strip) {
        ui_task_set_led_strip(s_strip);
    }
    ui_task_start(disp, CONFIG_TASK_PRIO_UI, CONFIG_TASK_CORE_UI);

    #endif




    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
