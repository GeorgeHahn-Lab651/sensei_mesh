
#include "app_cmd.h"
#include "nrf_error.h"
#include "leds.h"

uint16_t app_cmd_handler(uint8_t *data, uint16_t len, uint8_t *response, uint16_t *response_length) {
  toggle_led(LED_RED);
  data[0] = 0xfa;
  *response_length = 1;
  return NRF_SUCCESS;
}
