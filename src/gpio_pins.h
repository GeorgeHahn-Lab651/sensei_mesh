#ifndef GPIO_PINS_H
#define GPIO_PINS_H

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

#endif //GPIO_PINS_H
