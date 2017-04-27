#ifndef LEDS_H
#define LEDS_H

#include "boards.h"

#define LED_RED 0
#define LED_GREEN 1
#define LED_BLUE 2

#define DBG_TICK_PIN(x) NRF_GPIO->OUTSET = (1 << (x)); \
                    __NOP();\
                    __NOP();\
                    __NOP();\
                    __NOP();\
                    NRF_GPIO->OUTCLR = (1 << (x))

#define SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
#define CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))

#define TOGGLE_PIN(x) do { uint32_t gpio_state = NRF_GPIO->OUT;      \
                              NRF_GPIO->OUTSET = ((1<<(x)) & ~gpio_state); \
                              NRF_GPIO->OUTCLR = ((1<<(x)) & gpio_state); } while (0)


#ifdef __cplusplus
extern "C" {
#endif

void toggle_led(unsigned char led);
void led_config(uint8_t led, uint8_t conf);

#ifdef __cplusplus
}
#endif

#endif // LEDS_H
