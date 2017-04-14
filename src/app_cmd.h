#ifndef APP_CMD_H
#define APP_CMD_H

#include <stdint.h>
#include "toolchain.h"

typedef enum
{
  APP_CMD_OPCODE_SET_EPOCH_TIME = 0x02,
} app_cmd_opcode_t;

typedef __packed_armcc struct
{
	int32_t epoch;
  uint16_t ms;
} __packed_gcc app_cmd_params_set_clock_time_t;

typedef __packed_armcc struct
{
    app_cmd_opcode_t opcode;
    __packed_armcc union
    {
        app_cmd_params_set_clock_time_t         set_clock_time;
    } __packed_gcc params;
} __packed_gcc  app_cmd_t;


uint16_t app_cmd_handler(uint8_t *data, uint8_t len, uint8_t *response, uint8_t *response_length);

#endif // APP_CMD_H
