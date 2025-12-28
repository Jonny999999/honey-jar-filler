#include "motor.h"

// Local storage for the relay GPIO.
static gpio_num_t s_relay_gpio = GPIO_NUM_NC;

void motor_init(gpio_num_t relay_gpio)
{
    s_relay_gpio = relay_gpio;

    // Configure relay GPIO as push-pull output.
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << relay_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    // Default safe state: brake (relay OFF).
    gpio_set_level(s_relay_gpio, 0);
}

void motor_set(bool on)
{
    if (s_relay_gpio == GPIO_NUM_NC) return;
    gpio_set_level(s_relay_gpio, on ? 1 : 0);
}
