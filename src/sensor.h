#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "proximity.h"
#include "toolchain.h"
#include <stdbool.h>
#include <stdint.h>

#define MAX_SENSOR_ID = 64

#define STATUS_FLAG_JOSTLE_DETECTED (1 << 0)

typedef __packed_armcc struct {
  // MAX_PROXIMITY_TRACKING_COUNT is currently 5 and should never be much larger
  uint8_t proximity_ids[MAX_PROXIMITY_TRACKING_COUNT];
  uint8_t proximity_rssi[MAX_PROXIMITY_TRACKING_COUNT];
  uint8_t battery;
  int8_t accel_x;
  int8_t accel_y;
  int8_t accel_z;
  uint8_t status;
  int32_t valid_time;
} __packed_gcc sensor_value_t;

void sensor_init();
void sensor_warmup_event();
void report_sensor_data();

#endif
