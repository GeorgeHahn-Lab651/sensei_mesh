#ifndef __TIME_SYNC_H_
#define __TIME_SYNC_H_

#include <stdint.h>
#include "toolchain.h"

typedef enum {
  CLOCK_SOURCE_UNSET,
  CLOCK_SOURCE_RF,
  CLOCK_SOURCE_SERIAL,
} clock_source_t;

void time_sync_init();
void set_clock_time(int32_t epoch, uint16_t ms, clock_source_t clock_source, int16_t clock_version);

// Returns unix epoch
int32_t get_clock_time();
// Clock version increments from a master source and is distributed by all
// in the heartbeat message
int16_t get_clock_version();

// Callbacks
void main_timer_cb();
void offset_timer_cb();

#endif //__TIME_SYNC_H_
