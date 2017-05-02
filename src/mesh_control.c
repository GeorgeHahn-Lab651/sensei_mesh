
#include "mesh_control.h"

static mesh_control_t m_config;

void mesh_control_init() {
  m_config.wake_interval = DEFAULT_WAKE_INTERVAL;
  m_config.enable_ble = 0;
}

uint16_t mesh_control_get_wake_interval() {
  return m_config.wake_interval;
}

void mesh_control_update_config(mesh_control_t *new_config) {
  m_config = *new_config;
}
