#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdint.h>
#include <stdbool.h>
#include "toolchain.h"

typedef __packed_armcc struct
{
    uint8_t sensor_id;
    uint8_t serial_enabled;
    uint8_t mesh_channel;
    uint8_t sleep_enabled;
} __packed_gcc app_config_t;

// Returns true on success
bool config_init();

// Copies config to passed storage; reads from flash on first time, from
// ram on subsequent calls.
// Returns true on success
bool get_config(app_config_t *);

// Stores config in flash, and updates ram copy for subsequent use
// Returns true on success
uint32_t set_config(app_config_t *);

// Shortcut for getting sensor id.  Returns 0 on failure to load sensor id
uint8_t get_sensor_id();


#endif //__CONFIG_H__
