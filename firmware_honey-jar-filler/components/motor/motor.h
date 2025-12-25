#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Relay-driven motor control (HIGH=drive, LOW=brake).
void motor_init(gpio_num_t relay_gpio); // default OFF (brake)
void motor_set(bool on);               // true=drive, false=brake

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H
