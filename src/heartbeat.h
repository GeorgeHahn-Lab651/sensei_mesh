#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <stdint.h>
#include "toolchain.h"


#define HEARTBEAT_ADV_DATA_TYPE (0x48)

typedef __packed_armcc struct
{
  uint8_t sensor_id;
  uint32_t epoch_time;
  uint16_t clock_version;
} __packed_gcc heartbeat_ad_t;


void heartbeat_init();
void send_heartbeat_packet();
void received_heartbeat(heartbeat_ad_t *p_heartbeat_ad);

#endif // HEARTBEAT_H
