// Host-simulation shim: minimal FreeRTOS types so firmware headers compile.
#pragma once
#include <stdint.h>
typedef unsigned int UBaseType_t;
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *SemaphoreHandle_t;
typedef void *QueueHandle_t;
typedef void *TaskHandle_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
