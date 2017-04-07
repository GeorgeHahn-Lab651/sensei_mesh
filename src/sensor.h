#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include <stdbool.h>
#include "toolchain.h"

#define MAX_PROXIMITY_TRACKING_COUNT 5

typedef __packed_armcc struct
{
    uint8_t proximity_ids[MAX_PROXIMITY_TRACKING_COUNT];
    uint8_t proximity_rssis[MAX_PROXIMITY_TRACKING_COUNT];
    uint8_t battery;
    uint8_t accel_x;
    uint8_t accel_y;
    uint8_t accel_z;
    uint8_t status;
    uint32_t uptime;
} __packed_gcc sensor_value_t;

void sensor_init();

void sensor_update();

#endif
