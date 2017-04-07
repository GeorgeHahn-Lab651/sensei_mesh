
#include "leds.h"

void toggle_led(unsigned char led) {
  LEDS_INVERT(1 << (led + LED_START));
}

void led_config(uint8_t led, uint8_t conf)
{
  if (conf)
  {
    NRF_GPIO->OUTSET = (1 << (led + LED_START));
  }
  else
  {
    NRF_GPIO->OUTCLR = (1 << (led + LED_START));
  }
}
