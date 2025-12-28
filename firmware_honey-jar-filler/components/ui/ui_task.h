#ifndef UI_TASK_H
#define UI_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ssd1306.h"

#ifdef __cplusplus
extern "C" {
#endif

// Provide the encoder event queue before starting the UI task.
void ui_task_set_encoder_queue(QueueHandle_t q);

// Start the UI task (OLED + buttons + encoder).
void ui_task_start(ssd1306_handle_t disp, UBaseType_t prio, BaseType_t core);

#ifdef __cplusplus
}
#endif

#endif // UI_TASK_H
