
#include "app_cmd.h"
#include "nrf_error.h"
#include "leds.h"
#include "epoch.h"

uint16_t app_cmd_handler(uint8_t *data, uint8_t len, uint8_t *response, uint8_t *response_length) {

  app_cmd_t *cmd = (app_cmd_t*) data;

  uint16_t error_code;

  switch(cmd->opcode) {
    case APP_CMD_OPCODE_SET_EPOCH_TIME:
      {
        set_epoch_time(cmd->params.set_epoch_time.epoch, cmd->params.set_epoch_time.ms);
        error_code = NRF_SUCCESS;
      }
      break;
    default:
      error_code = NRF_ERROR_NOT_SUPPORTED;
  }

  toggle_led(LED_RED);
  response[0] = 0xfa;
  response[1] = 0xba;
  *response_length = 2;
  return error_code;
}
