#ifndef LEDS_H
#define LEDS_H

#include "boards.h"

#define LED_BLUE LED_1
#define LED_GREEN LED_2
#define LED_RED LED_3

void init_leds();
void toggle_led(unsigned char led);

#endif // LEDS_H
