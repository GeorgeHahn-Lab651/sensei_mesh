#ifndef APP_EVT_H
#define APP_EVT_H

#include "toolchain.h"
#include <stdint.h>
#include "heartbeat.h"

typedef enum
{
  APP_EVT_OPCODE_HEARTBEAT = 0x01
} app_evt_opcode_t;

typedef __packed_armcc struct
{
    app_evt_opcode_t opcode;
    __packed_armcc union
    {
        heartbeat_ad_t heartbeat;
    } __packed_gcc params;
} __packed_gcc  app_evt_t;


void app_event_send(app_evt_t *evt);

#endif // APP_EVT_H
