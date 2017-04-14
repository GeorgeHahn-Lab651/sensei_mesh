#ifndef __TIME_SYNC_H_
#define __TIME_SYNC_H_

#include <stdint.h>
#include "toolchain.h"

typedef enum {
  CLOCK_SOURCE_UNSET,
  CLOCK_SOURCE_RF,
  CLOCK_SOURCE_SERIAL,
} clock_source_t;

typedef __packed_armcc struct
{
  uint8_t sensor_id;
  uint32_t epoch_time;
  uint16_t clock_version;
} __packed_gcc heartbeat_ad_t;

void time_sync_init();
void set_clock_time(int32_t epoch, uint16_t ms, clock_source_t clock_source);

#endif //__TIME_SYNC_H_
