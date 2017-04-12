#ifndef APP_CMD_H
#define APP_CMD_H

#include <stdint.h>
#include "toolchain.h"

typedef enum
{
  APP_CMD_OPCODE_SET_TIME = 0x02,
} app_cmd_opcode_t;

typedef __packed_armcc struct
{
	uint8_t epoch[4];
} __packed_gcc app_cmd_params_set_time_t;


typedef __packed_armcc struct
{
    app_cmd_opcode_t opcode;
    __packed_armcc union
    {
        app_cmd_params_set_time_t         set_time;
    } __packed_gcc params;
} __packed_gcc  app_cmd_t;


uint16_t app_cmd_handler(uint8_t *data, uint16_t len, uint8_t *response, uint16_t *response_length);

#endif // APP_CMD_H
