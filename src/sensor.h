#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include <stdbool.h>
#include "toolchain.h"

#define MAX_SENSOR_ID = 64

typedef __packed_armcc struct
{
    uint8_t proximity_ids[MAX_PROXIMITY_TRACKING_COUNT];
    uint8_t proximity_rss[MAX_PROXIMITY_TRACKING_COUNT];
    uint8_t battery;
    int8_t accel_x;
    int8_t accel_y;
    int8_t accel_z;
    uint8_t status;
    uint32_t uptime;
} __packed_gcc sensor_value_t;

void sensor_init();
void sensor_warmup_event();
void report_sensor_data();

#endif
