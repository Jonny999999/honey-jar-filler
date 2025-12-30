#include "ui_task.h"

#include <stdio.h>
#include <stdbool.h>

#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "encoder.h"
#include "app.h"
#include "buzzer.h"
#include "config.h"
#include "filler_fsm.h"
#include "led_strip.h"
#include "scale_hx711.h"

// UI refresh rate.
#define UI_PERIOD_MS 200

// Button debounce.
#define UI_BTN_DEBOUNCE_MS 60

// Encoder step for target grams.
#define UI_TARGET_STEP_G 10.0f

// LED strip brightness limit (% of 255). Override in config.h.
#define UI_WS2812_MAX_BRIGHTNESS_PCT CONFIG_WS2812_MAX_BRIGHTNESS_PCT

// Rate-limit encoder beeps to avoid continuous tone.
#define UI_BEEP_MIN_INTERVAL_MS 80

// LED animation for waiting/filling.
#define UI_ANIM_PERIOD_MS 50
#define UI_ANIM_BLOCK_LEN 4
#define UI_ANIM_DIM_PCT 55

#define LINE2PIXEL(n) ((n) * 8)

typedef struct {
    ssd1306_handle_t disp;
} ui_args_t;

static QueueHandle_t s_enc_q;
static TaskHandle_t s_task;
static led_strip_handle_t s_strip;
static const char *TAG = "ui_task";

static uint8_t ui_ws2812_scale(uint8_t v)
{
    return (uint8_t)((v * UI_WS2812_MAX_BRIGHTNESS_PCT) / 100);
}

static void ui_ws2812_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_strip) return;
    r = ui_ws2812_scale(r);
    g = ui_ws2812_scale(g);
    b = ui_ws2812_scale(b);
    for (int i = 0; i < CONFIG_WS2812_LED_COUNT; ++i) {
        led_strip_set_pixel(s_strip, i, r, g, b);
    }
    (void)led_strip_refresh(s_strip);
}

static void ui_ws2812_set_pixel(int idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_strip) return;
    r = ui_ws2812_scale(r);
    g = ui_ws2812_scale(g);
    b = ui_ws2812_scale(b);
    led_strip_set_pixel(s_strip, idx, r, g, b);
}

static void ui_get_state_color(filler_state_t st, filler_fault_t flt,
                               uint8_t *r, uint8_t *g, uint8_t *b, bool *animated)
{
    *animated = false;
    if (st == FILLER_FAULT) {
        (void)flt;
        *r = 255; *g = 0; *b = 0; // red
        return;
    }

    switch (st) {
    case FILLER_IDLE:
        *r = 0; *g = 0; *b = 255; // blue = idle/ready
        break;
    case FILLER_FILL:
        *r = 255; *g = 180; *b = 0; // amber = filling
        *animated = true;
        break;
    case FILLER_SLOT_SETTLE:
    case FILLER_DRIP_WAIT:
        *r = 0; *g = 180; *b = 255; // cyan = waiting/settling
        *animated = true;
        break;
    case FILLER_FIND_SLOT:
    case FILLER_VERIFY_EMPTY:
    case FILLER_VERIFY_TARGET:
    case FILLER_DONE:
    default:
        *r = 0; *g = 255; *b = 0; // green = active/ok
        break;
    }
}

static void ui_render_anim_block(uint8_t r, uint8_t g, uint8_t b, int pos)
{
    if (!s_strip) return;
    int count = CONFIG_WS2812_LED_COUNT;
    if (count <= 0) return;

    uint8_t dr = (uint8_t)((r * UI_ANIM_DIM_PCT) / 100);
    uint8_t dg = (uint8_t)((g * UI_ANIM_DIM_PCT) / 100);
    uint8_t db = (uint8_t)((b * UI_ANIM_DIM_PCT) / 100);

    for (int i = 0; i < count; ++i) {
        ui_ws2812_set_pixel(i, r, g, b);
    }
    for (int i = 0; i < UI_ANIM_BLOCK_LEN; ++i) {
        int idx = (pos + i) % count;
        ui_ws2812_set_pixel(idx, dr, dg, db);
    }
    (void)led_strip_refresh(s_strip);
}

static void ui_update_state_outputs(filler_state_t st, filler_fault_t flt)
{
    // LED1/LED2 indicate active filling only.
    bool filling = (st == FILLER_FILL);
    gpio_set_level(CONFIG_LED1_GPIO, filling ? 1 : 0);
    gpio_set_level(CONFIG_LED2_GPIO, filling ? 1 : 0);

    uint8_t r = 0, g = 0, b = 0;
    bool animated = false;
    ui_get_state_color(st, flt, &r, &g, &b, &animated);
    ui_ws2812_set_all(r, g, b);
}

static void ui_handle_button(filler_state_t st)
{
    static int last_btn = 1; // active-low
    int cur = gpio_get_level(CONFIG_BUTTON_1_GPIO);

    if (last_btn != 0 && cur == 0) {
        vTaskDelay(pdMS_TO_TICKS(UI_BTN_DEBOUNCE_MS));
        if (gpio_get_level(CONFIG_BUTTON_1_GPIO) == 0) {
            if (st == FILLER_IDLE || st == FILLER_FAULT) {
                ESP_LOGI(TAG, "button: start");
                filler_request_start();
            } else {
                ESP_LOGI(TAG, "button: abort");
                filler_request_abort();
            }
            while (gpio_get_level(CONFIG_BUTTON_1_GPIO) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
    last_btn = cur;
}

static void ui_handle_encoder(filler_state_t st, int32_t enc_delta, int64_t *last_beep_us)
{
    // Only apply changes when idle or fault; always drain queue elsewhere.
    if (enc_delta == 0) return;
    if (st != FILLER_IDLE && st != FILLER_FAULT) return;

    app_params_t p;
    app_params_get(&p);
    p.target_grams += ((float)enc_delta * UI_TARGET_STEP_G);
    if (p.target_grams < 0.0f) p.target_grams = 0.0f;
    (void)app_params_set(&p);
    ESP_LOGI(TAG, "encoder: target=%.1f g", (double)p.target_grams);

    if (last_beep_us) {
        int64_t now_us = esp_timer_get_time();
        if (now_us - *last_beep_us >= (UI_BEEP_MIN_INTERVAL_MS * 1000)) {
            *last_beep_us = now_us;
            buzzer_beep_ms(20);
        }
    }
}

static void ui_render(ssd1306_handle_t disp,
                      const scale_latest_t *s,
                      const app_params_t *p,
                      filler_state_t st,
                      uint8_t slot,
                      filler_fault_t flt,
                      bool has_tare,
                      float tare_g)
{
    char line1[24];
    char line2[32];
    char line3[32];

    ssd1306_clear(disp);

    // Line 1: big current weight.
    if (s && s->valid) {
        float shown_g = s->grams;
        char prefix = 'A';
        if (has_tare) {
            shown_g -= tare_g;
            prefix = 'R';
        }
        snprintf(line1, sizeof(line1), "%c%.1f g", prefix, (double)shown_g);
    } else {
        snprintf(line1, sizeof(line1), "--.- g");
    }
    ssd1306_draw_text_scaled(disp, 0, LINE2PIXEL(0), line1, true, 3);

    // Line 2: state + slot.
    if (st == FILLER_FAULT) {
        snprintf(line2, sizeof(line2), "FAULT:%s", filler_fault_name(flt));
    } else {
        snprintf(line2, sizeof(line2), "%s %u/%u", filler_state_name(st), (unsigned)(slot + 1), (unsigned)p->slots_total);
    }
    ssd1306_draw_text(disp, 0, LINE2PIXEL(4), line2, true);

    // Line 3: action hint + target.
    snprintf(line3, sizeof(line3), "Btn:%s  Tgt:%3.0fg",
             (st == FILLER_IDLE || st == FILLER_FAULT) ? "START" : "STOP",
             (double)p->target_grams);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(6), line3, true);

    ssd1306_display(disp);
}

static void task_ui(void *arg)
{
    ui_args_t cfg = *(ui_args_t*)arg;
    vPortFree(arg);

    filler_state_t last_state = FILLER_DONE;
    filler_fault_t last_fault = FLT_NONE;
    int anim_pos = 0;
    int64_t last_anim_us = 0;

    TickType_t next_refresh = xTaskGetTickCount() + pdMS_TO_TICKS(UI_PERIOD_MS);
    const TickType_t period = pdMS_TO_TICKS(UI_PERIOD_MS);
    int64_t last_beep_us = 0;

    while (1) {
        // Wait for encoder events or for the next display refresh tick.
        TickType_t now_ticks = xTaskGetTickCount();
        int64_t now_us = esp_timer_get_time();
        int64_t next_anim_us = (last_anim_us == 0)
            ? now_us
            : (last_anim_us + (int64_t)UI_ANIM_PERIOD_MS * 1000);
        TickType_t wait_refresh = (next_refresh > now_ticks) ? (next_refresh - now_ticks) : 0;
        TickType_t wait_anim = 0;
        if (now_us < next_anim_us) {
            wait_anim = pdMS_TO_TICKS((next_anim_us - now_us) / 1000);
        }
        TickType_t wait = (wait_refresh < wait_anim) ? wait_refresh : wait_anim;
        int32_t enc_delta = 0;

        if (s_enc_q) {
            rotary_encoder_event_t ev;
            if (xQueueReceive(s_enc_q, &ev, wait) == pdPASS) {
                if (ev.type == RE_ET_CHANGED) enc_delta += ev.diff;
                // Drain any queued events quickly.
                while (xQueueReceive(s_enc_q, &ev, 0) == pdPASS) {
                    if (ev.type == RE_ET_CHANGED) enc_delta += ev.diff;
                }
            }
        } else {
            vTaskDelay(wait);
        }

        filler_state_t st = filler_get_state();
        filler_fault_t flt = filler_get_fault();

        if (st != last_state) {
            ESP_LOGI(TAG, "state -> %s", filler_state_name(st));
            last_state = st;
            ui_update_state_outputs(st, flt);
            anim_pos = 0;
            last_anim_us = 0;
        }
        if (flt != last_fault) {
            ESP_LOGI(TAG, "fault -> %s", filler_fault_name(flt));
            last_fault = flt;
            ui_update_state_outputs(st, flt);
        }

        ui_handle_button(st);
        ui_handle_encoder(st, enc_delta, &last_beep_us);

        {
            uint8_t r = 0, g = 0, b = 0;
            bool animated = false;
            ui_get_state_color(st, flt, &r, &g, &b, &animated);
            now_us = esp_timer_get_time();
            if (animated && (last_anim_us == 0 || (now_us - last_anim_us) >= (UI_ANIM_PERIOD_MS * 1000))) {
                ui_render_anim_block(r, g, b, anim_pos);
                if (CONFIG_WS2812_LED_COUNT > 0) {
                    anim_pos = (anim_pos + 1) % CONFIG_WS2812_LED_COUNT;
                }
                last_anim_us = now_us;
            }
        }

        now_ticks = xTaskGetTickCount();
        if (now_ticks >= next_refresh) {
            scale_latest_t latest = {0};
            scale_latest_get(&latest);

            app_params_t params;
            app_params_get(&params);

            uint8_t slot = filler_get_slot_idx();
            float tare_g = 0.0f;
            bool has_tare = filler_get_jar_tare(&tare_g);
            ui_render(cfg.disp, &latest, &params, st, slot, flt, has_tare, tare_g);
            next_refresh = now_ticks + period;
        }
    }
}

void ui_task_set_encoder_queue(QueueHandle_t q)
{
    s_enc_q = q;
}

void ui_task_set_led_strip(led_strip_handle_t strip)
{
    s_strip = strip;
}

void ui_task_start(ssd1306_handle_t disp, UBaseType_t prio, BaseType_t core)
{
    if (s_task) return;
    ui_args_t *args = pvPortMalloc(sizeof(*args));
    if (!args) return;
    args->disp = disp;
    xTaskCreatePinnedToCore(task_ui, "ui_task", 4096, args, prio, &s_task, core);
}
