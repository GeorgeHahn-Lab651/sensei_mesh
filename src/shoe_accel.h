#ifndef SHOE_ACCEL_H
#define SHOE_ACCEL_H

#include "stdint.h"


// Init must be called before any other functions in this module.
void shoe_accel_init();

// The accelerometer should be enabled for at least 10ms before
// calling read_shoe_accel()
void enable_shoe_accel();

// The accelerometer can be disabled (power will be cut off) for
// power savings.
void disable_shoe_accel();

// Values range from -127 to 127.  127 = 2g
void read_shoe_accel(int8_t *x, int8_t *y, int8_t *z);

#endif // SHOE_ACCEL_H
