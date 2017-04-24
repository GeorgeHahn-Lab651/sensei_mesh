#ifndef __SCHEDULER_H_
#define __SCHEDULER_H_

#include <stdint.h>
#include "toolchain.h"

typedef enum {
  CLOCK_SOURCE_UNSET,
  CLOCK_SOURCE_RF,
  CLOCK_SOURCE_SERIAL,
} clock_source_t;

typedef enum {
  SCHEDULER_STATE_STOPPED,
  SCHEDULER_STATE_SLEEP,
  SCHEDULER_STATE_BEFORE_HB,
  SCHEDULER_STATE_AFTER_HB,
  SCHEDULER_STATE_REPORTING,
} scheduler_state_t;

#define MAX_EXPECTED_CLOCK_SKEW_MS (10)
#define HEARTBEAT_WINDOW_MS (100)
#define TOTAL_RADIO_WINDOW_MS (900)

void scheduler_init();
void set_clock_time(int32_t epoch, uint16_t ms, clock_source_t clock_source, int16_t clock_version);

// Returns unix epoch
int32_t get_clock_time();

// Clock version increments from a master source and is distributed by all
// in the heartbeat message
int16_t get_clock_version();

// Seconds since the sensor started
int32_t get_uptime();

#endif //__SCHEDULER_H_
