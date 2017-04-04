
#include "leds.h"

void init_leds() {
  LEDS_CONFIGURE(LEDS_MASK);
}

void toggle_led(unsigned char led) {
  LEDS_INVERT(1 << led);
}
