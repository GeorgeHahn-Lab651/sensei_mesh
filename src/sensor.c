
#include "sensor.h"
#include "config.h"
#include "rbc_mesh.h"
#include "scheduler.h"
#include "battery.h"
#include "handles.h"
#include <app_error.h>

static sensor_value_t m_value;

void sensor_init() {
  uint32_t error_code;
  error_code = rbc_mesh_value_enable(SENSOR_HANDLE);
  APP_ERROR_CHECK(error_code);
}

void gather_sensor_data() {
  m_value.uptime = get_uptime();
  m_value.battery = get_battery_adc();
}

void report_sensor_data() {
  uint32_t error_code;

  if (get_sensor_id() > 0) {
    gather_sensor_data();
    error_code = rbc_mesh_value_set(SENSOR_HANDLE, (uint8_t*)&m_value, sizeof(sensor_value_t));
    APP_ERROR_CHECK(error_code);
  } else {
    // Would be nice to report this somewhere.
  }
}
