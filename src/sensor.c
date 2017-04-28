
#include "sensor.h"
#include "config.h"
#include "rbc_mesh.h"
#include "scheduler.h"
#include "battery.h"
#include "handles.h"
#include "shoe_accel.h"
#include <app_error.h>

static sensor_value_t m_value;

void sensor_init() {
  shoe_accel_init();

  uint32_t error_code;
  error_code = rbc_mesh_value_enable(SENSOR_HANDLE);
  APP_ERROR_CHECK(error_code);
}

void sensor_warmup_event() {
  enable_shoe_accel();
}

void gather_sensor_data() {
  m_value.valid_time = get_clock_time();
  m_value.battery = get_battery_adc();
  //m_value.status = ??

  read_shoe_accel(&m_value.accel_x, &m_value.accel_y, &m_value.accel_z);

  proximity_get_strongest_signals(m_value.proximity_ids, m_value.proximity_rssi, MAX_PROXIMITY_TRACKING_COUNT);
  proximity_values_reset();

  disable_shoe_accel();
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
