#include "pstorage.h"
#include "config.h"
#include "nrf_error.h"
#include <string.h>

// Should be bigger than app_config_t
#define APP_CONFIG_BLOCK_SIZE 64
#define APP_CONFIG_OFFSET 0

static pstorage_handle_t m_storage_handle;
static app_config_t m_config;
static bool m_loaded = false;

static void m_storage_callback(pstorage_handle_t *p_handle, uint8_t op_code, uint32_t result, uint8_t *p_data, uint32_t data_len) {
  // Nothing for now, with our memory cache, we don't care how long it takes to store.
}

bool config_init() {
  if (pstorage_init() != NRF_SUCCESS) {
    return false;
  }

  pstorage_module_param_t params = {
    m_storage_callback,
    APP_CONFIG_BLOCK_SIZE,  // Block size
    1                       // Block count
  };

  if (pstorage_register(&params, &m_storage_handle) != NRF_SUCCESS) {
    return false;
  }

  return true;
}

static bool ensure_config_loaded() {
  if (!m_loaded) {
    if (pstorage_load((uint8_t*)&m_config, &m_storage_handle, sizeof(app_config_t), APP_CONFIG_OFFSET) != NRF_SUCCESS) {
      return false;
    }
    m_loaded = true;
  }
  return true;
}

bool get_config(app_config_t *config) {
  if (!ensure_config_loaded()) {
    return false;
  }
  memcpy(config, &m_config, sizeof(app_config_t));
  return true;
}

bool set_config(app_config_t *config) {
  memcpy(&m_config, config, sizeof(app_config_t));
  return pstorage_store(&m_storage_handle, (uint8_t*)&m_config, sizeof(app_config_t), APP_CONFIG_OFFSET) == NRF_SUCCESS;
}

uint8_t get_sensor_id() {
  if (!ensure_config_loaded()) {
    return 0;
  }
  return m_config.sensor_id;
}
