#ifndef LEDS_H
#define LEDS_H

#include "boards.h"
#include "gpio_pins.h"

#define LED_RED 0
#define LED_GREEN 1
#define LED_BLUE 2

#define SET_LED(x) SET_PIN(LED_START + x)
#define CLEAR_LED(x) CLEAR_PIN(LED_START + x)
#define TOGGLE_LED(x) TOGGLE_PIN(LED_START + x)

#endif // LEDS_H
