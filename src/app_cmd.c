
#include "app_cmd.h"
#include "nrf_error.h"
#include "leds.h"
#include "time_sync.h"
#include "config.h"

uint16_t app_cmd_handler(uint8_t *data, uint8_t len, uint8_t *response, uint8_t *response_length) {

  app_cmd_t *cmd = (app_cmd_t*) data;

  uint16_t error_code;

  switch(cmd->opcode) {
    case APP_CMD_OPCODE_SET_TIME:
      {
        set_clock_time(cmd->params.set_clock_time.epoch, cmd->params.set_clock_time.ms, CLOCK_SOURCE_SERIAL);
        error_code = NRF_SUCCESS;
        *response_length = 0;
      }
      break;
    case APP_CMD_OPCODE_SET_CONFIG:
      {
        error_code = set_config(&cmd->params.set_config.config);
        *response_length = 0;
      }
      break;
    case APP_CMD_OPCODE_GET_CONFIG:
      {
        get_config((app_config_t*)response);
        *response_length = sizeof(app_config_t);
        error_code = NRF_SUCCESS;
      }
      break;
    default:
      error_code = NRF_ERROR_NOT_SUPPORTED;
  }
  return error_code;
}
