
#include "sensor.h"
#include "config.h"
#include "rbc_mesh.h"
#include <app_error.h>

#define SENSOR_HANDLE (0x0100 + get_sensor_id())

static sensor_value_t m_value;

void sensor_init() {
  uint32_t error_code;
  error_code = rbc_mesh_value_enable(SENSOR_HANDLE);
  APP_ERROR_CHECK(error_code);
}

void sensor_update() {
  uint32_t error_code;
  error_code = rbc_mesh_value_set(SENSOR_HANDLE, (uint8_t*)&m_value, sizeof(sensor_value_t));
  APP_ERROR_CHECK(error_code);
}
