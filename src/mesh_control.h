#ifndef MESH_CONTROL_H
#define MESH_CONTROL_H

#include <stdint.h>
#include "toolchain.h"

// Default wake interval is 10 seconds
#define DEFAULT_WAKE_INTERVAL (10)

typedef __packed_armcc struct
{
  uint16_t wake_interval;
  uint8_t enable_ble;  // Not used yet...
} __packed_gcc mesh_control_t;


void mesh_control_init();

// Wake interval controls the cycle of waking and sleeping
// current_epoch % wake_interval == 0 indicates the start of a wake period
uint16_t mesh_control_get_wake_interval();

void mesh_control_update_config(mesh_control_t *new_config);

#endif // MESH_CONTROL_H
