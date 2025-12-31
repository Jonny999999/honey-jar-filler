#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the buzzer GPIO (active HIGH). Must be called before any beeps.
void buzzer_init(gpio_num_t gpio);

// Start the buzzer task which plays queued beep requests.
void buzzer_task_start(UBaseType_t prio);

// Enable/disable buzzer output. When disabled, all requests are ignored.
void buzzer_set_enabled(bool enable);
bool buzzer_is_enabled(void);

// Queue a single beep (non-blocking). Uses default off-time between beeps.
void buzzer_beep_ms(uint16_t on_ms);

// Queue a beep pattern (non-blocking): count beeps with on/off durations.
void buzzer_beep_count(uint8_t count, uint16_t on_ms, uint16_t off_ms);

// Convenience patterns with built-in timings.
void buzzer_beep_short(uint8_t count);
void buzzer_beep_long(uint8_t count);

// Blocking version: waits until the queued pattern is finished.
// Returns immediately if the buzzer is disabled or queue is full.
void buzzer_beep_blocking(uint8_t count, uint16_t on_ms, uint16_t off_ms);

#ifdef __cplusplus
}
#endif

#endif // BUZZER_H
