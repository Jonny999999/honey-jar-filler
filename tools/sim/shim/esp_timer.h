// Host-simulation shim: time is driven by the simulation harness.
#pragma once
#include <stdint.h>
int64_t esp_timer_get_time(void);
