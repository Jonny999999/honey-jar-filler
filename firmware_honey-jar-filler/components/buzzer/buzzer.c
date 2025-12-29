#include "buzzer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

// Default timings for convenience beeps.
#define BUZZER_SHORT_ON_MS  60
#define BUZZER_SHORT_OFF_MS 80
#define BUZZER_LONG_ON_MS   200
#define BUZZER_LONG_OFF_MS  120

#define BUZZER_QUEUE_LEN 8

typedef struct {
    uint8_t count;
    uint16_t on_ms;
    uint16_t off_ms;
    TaskHandle_t notify_task; // optional: notify when done
} buzzer_req_t;

static const char *TAG = "buzzer";

static gpio_num_t s_gpio = GPIO_NUM_NC;
static QueueHandle_t s_queue;
static TaskHandle_t s_task;
static bool s_enabled = true;

static void buzzer_play_req(const buzzer_req_t *r)
{
    if (!r) return;

    for (uint8_t i = 0; i < r->count; ++i) {
        gpio_set_level(s_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(r->on_ms));
        gpio_set_level(s_gpio, 0);
        if (i + 1 < r->count) {
            vTaskDelay(pdMS_TO_TICKS(r->off_ms));
        }
    }
}

static void buzzer_notify_done(const buzzer_req_t *r)
{
    if (r && r->notify_task) {
        xTaskNotifyGive(r->notify_task);
    }
}

static void buzzer_task(void *arg)
{
    (void)arg;
    buzzer_req_t req;

    while (1) {
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) != pdPASS) continue;

        if (s_enabled && s_gpio != GPIO_NUM_NC) {
            ESP_LOGD(TAG, "play: count=%u on=%u ms off=%u ms",
                     (unsigned)req.count, (unsigned)req.on_ms, (unsigned)req.off_ms);
            buzzer_play_req(&req);
        }
        buzzer_notify_done(&req);
    }
}

void buzzer_init(gpio_num_t gpio)
{
    s_gpio = gpio;

    ESP_LOGI(TAG, "init gpio=%d", (int)gpio);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) {
        ESP_LOGW(TAG, "gpio_config failed");
    }
    gpio_set_level(s_gpio, 0);
}

void buzzer_task_start(UBaseType_t prio)
{
    if (s_task) return;
    if (!s_queue) {
        s_queue = xQueueCreate(BUZZER_QUEUE_LEN, sizeof(buzzer_req_t));
        if (!s_queue) {
            ESP_LOGW(TAG, "queue create failed");
            return;
        }
    }

    if (xTaskCreate(buzzer_task, "buzzer", 2048, NULL, prio, &s_task) != pdPASS) {
        ESP_LOGW(TAG, "task create failed");
        s_task = NULL;
    } else {
        ESP_LOGI(TAG, "task started (prio=%u)", (unsigned)prio);
    }
}

void buzzer_set_enabled(bool enable)
{
    s_enabled = enable;
    if (!enable && s_gpio != GPIO_NUM_NC) {
        gpio_set_level(s_gpio, 0);
    }
}

bool buzzer_is_enabled(void)
{
    return s_enabled;
}

static void buzzer_enqueue(const buzzer_req_t *req)
{
    if (!req || !s_queue) return;

    if (!s_enabled) {
        buzzer_notify_done(req);
        return;
    }

    ESP_LOGI(TAG, "enqueue: count=%u on=%u ms off=%u ms",
             (unsigned)req->count, (unsigned)req->on_ms, (unsigned)req->off_ms);
    if (xQueueSend(s_queue, req, 0) != pdPASS) {
        ESP_LOGW(TAG, "queue full, dropping beep");
        buzzer_notify_done(req);
    }
}

void buzzer_beep_ms(uint16_t on_ms)
{
    ESP_LOGI(TAG, "beep_ms(%u)", (unsigned)on_ms);
    buzzer_req_t r = {
        .count = 1,
        .on_ms = on_ms,
        .off_ms = BUZZER_SHORT_OFF_MS,
        .notify_task = NULL,
    };
    buzzer_enqueue(&r);
}

void buzzer_beep_count(uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
    ESP_LOGI(TAG, "beep_count(%u, %u, %u)", (unsigned)count, (unsigned)on_ms, (unsigned)off_ms);
    buzzer_req_t r = {
        .count = count,
        .on_ms = on_ms,
        .off_ms = off_ms,
        .notify_task = NULL,
    };
    buzzer_enqueue(&r);
}

void buzzer_beep_short(uint8_t count)
{
    ESP_LOGI(TAG, "beep_short(%u)", (unsigned)count);
    buzzer_beep_count(count, BUZZER_SHORT_ON_MS, BUZZER_SHORT_OFF_MS);
}

void buzzer_beep_long(uint8_t count)
{
    ESP_LOGI(TAG, "beep_long(%u)", (unsigned)count);
    buzzer_beep_count(count, BUZZER_LONG_ON_MS, BUZZER_LONG_OFF_MS);
}

void buzzer_beep_blocking(uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
    if (!s_enabled) return;
    if (!s_queue) return;

    ESP_LOGI(TAG, "beep_blocking(%u, %u, %u)", (unsigned)count, (unsigned)on_ms, (unsigned)off_ms);
    buzzer_req_t r = {
        .count = count,
        .on_ms = on_ms,
        .off_ms = off_ms,
        .notify_task = xTaskGetCurrentTaskHandle(),
    };

    if (xQueueSend(s_queue, &r, 0) != pdPASS) {
        ESP_LOGW(TAG, "queue full, dropping blocking beep");
        return;
    }

    uint32_t total_ms = (uint32_t)count * (on_ms + off_ms);
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(total_ms + 500));
}
