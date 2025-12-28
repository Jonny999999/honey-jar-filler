#include "ui_task.h"

#include <stdio.h>
#include <stdbool.h>

#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "encoder.h"
#include "app.h"
#include "config.h"
#include "filler_fsm.h"
#include "scale_hx711.h"

// UI refresh rate.
#define UI_PERIOD_MS 180

// Button debounce.
#define UI_BTN_DEBOUNCE_MS 30

// Encoder step for target grams.
#define UI_TARGET_STEP_G 5.0f

#define LINE2PIXEL(n) ((n) * 8)

typedef struct {
    ssd1306_handle_t disp;
} ui_args_t;

static QueueHandle_t s_enc_q;
static TaskHandle_t s_task;
static const char *TAG = "ui_task";

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

static void ui_handle_encoder(filler_state_t st)
{
    if (st != FILLER_IDLE || !s_enc_q) return;

    rotary_encoder_event_t ev;
    while (xQueueReceive(s_enc_q, &ev, 0) == pdPASS) {
        if (ev.type != RE_ET_CHANGED) continue;

        app_params_t p;
        app_params_get(&p);
        p.target_grams += ((float)ev.diff * UI_TARGET_STEP_G);
        if (p.target_grams < 0.0f) p.target_grams = 0.0f;
        (void)app_params_set(&p);
        ESP_LOGI(TAG, "encoder: target=%.1f g", (double)p.target_grams);
    }
}

static void ui_render(ssd1306_handle_t disp,
                      const scale_latest_t *s,
                      const app_params_t *p,
                      filler_state_t st,
                      uint8_t slot,
                      filler_fault_t flt)
{
    char line1[24];
    char line2[32];
    char line3[32];

    ssd1306_clear(disp);

    // Line 1: big current weight.
    if (s && s->valid) snprintf(line1, sizeof(line1), "%.1f g", (double)s->grams);
    else snprintf(line1, sizeof(line1), "--.- g");
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

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(UI_PERIOD_MS);

    while (1) {
        scale_latest_t latest = {0};
        scale_latest_get(&latest);

        app_params_t params;
        app_params_get(&params);

        filler_state_t st = filler_get_state();
        uint8_t slot = filler_get_slot_idx();
        filler_fault_t flt = filler_get_fault();

        if (st != last_state) {
            ESP_LOGI(TAG, "state -> %s", filler_state_name(st));
            last_state = st;
        }
        if (flt != last_fault) {
            ESP_LOGI(TAG, "fault -> %s", filler_fault_name(flt));
            last_fault = flt;
        }

        ui_handle_button(st);
        ui_handle_encoder(st);

        ui_render(cfg.disp, &latest, &params, st, slot, flt);

        vTaskDelayUntil(&last, period);
    }
}

void ui_task_set_encoder_queue(QueueHandle_t q)
{
    s_enc_q = q;
}

void ui_task_start(ssd1306_handle_t disp, UBaseType_t prio, BaseType_t core)
{
    if (s_task) return;
    ui_args_t *args = pvPortMalloc(sizeof(*args));
    if (!args) return;
    args->disp = disp;
    xTaskCreatePinnedToCore(task_ui, "ui_task", 4096, args, prio, &s_task, core);
}
