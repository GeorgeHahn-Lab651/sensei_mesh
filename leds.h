#ifndef LEDS_H
#define LEDS_H

#include "boards.h"

#define LED_RED 0
#define LED_GREEN 1
#define LED_BLUE 2

#ifdef __cplusplus
extern "C" {
#endif

void init_leds();
void toggle_led(unsigned char led);
void led_config(uint8_t led, uint8_t conf);

#ifdef __cplusplus
}
#endif

#endif // LEDS_H
