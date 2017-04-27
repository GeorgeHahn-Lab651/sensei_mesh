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

#define SET_LED(x) SET_PIN(LED_START + x)
#define CLEAR_LED(x) CLEAR_PIN(LED_START + x)
#define TOGGLE_LED(x) TOGGLE_PIN(LED_START + x)

#endif // LEDS_H
