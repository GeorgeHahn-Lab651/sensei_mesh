#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"

void battery_init();
uint8_t get_battery_adc();

#ifdef __cplusplus
}
#endif
#endif // BATTERY_H
