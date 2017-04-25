#include "proximity.h"
#include <string.h>

static uint8_t rssi_sorted[MAX_PROXIMITY_TRACKING_COUNT];
static uint8_t sensor_ids_sorted[MAX_PROXIMITY_TRACKING_COUNT];

void proximity_add_entry(uint8_t sensor_id, uint8_t rssi) {
  uint8_t i,j;
  for (i=0; i < MAX_PROXIMITY_TRACKING_COUNT; i++) {
    if (rssi < rssi_sorted[i] || rssi_sorted[i] == 0) {
      j = MAX_PROXIMITY_TRACKING_COUNT-1;
      while (j > i) {
        rssi_sorted[j] = rssi_sorted[j-1];
        sensor_ids_sorted[j] = sensor_ids_sorted[j-1];
        j--;
      }
      rssi_sorted[i] = rssi;
      sensor_ids_sorted[i] = sensor_id;
      break;
    }
  }
}

void proximity_values_reset() {
  memset(rssi_sorted, 0, sizeof(uint8_t) * MAX_PROXIMITY_TRACKING_COUNT);
  memset(sensor_ids_sorted, 0, sizeof(uint8_t) * MAX_PROXIMITY_TRACKING_COUNT);
}

uint8_t proximity_get_strongest_signals(uint8_t *sensor_ids, uint8_t *rssi_values, uint8_t output_array_size) {
  uint8_t n = 0;
  while(n<MAX_PROXIMITY_TRACKING_COUNT && n<output_array_size && rssi_sorted[n] > 0) {
    n++;
  }
  memcpy(sensor_ids, sensor_ids_sorted, sizeof(uint8_t) * n);
  memcpy(rssi_values, rssi_sorted, sizeof(uint8_t) * n);
  return n;
}
