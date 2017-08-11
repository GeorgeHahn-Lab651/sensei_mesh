#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "mesh_aci.h"
#include "rbc_mesh.h"

#include "app_cmd.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_util.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "config.h"
#include "handles.h"
#include "heartbeat.h"
#include "leds.h"
#include "mesh_control.h"
#include "nrf_gpio.h"
#include "pstorage_platform.h"
#include "scheduler.h"
#include "sensor.h"
#include "serial_handler.h"
#include "softdevice_handler.h"
#include "transport_control.h"

#include "SEGGER_RTT.h"

#if NORDIC_SDK_VERSION >= 11
#include "nrf_nvic.h"
#endif

#define MESH_ACCESS_ADDR (0xA555410C)
#define MESH_INTERVAL_MIN_MS (100)

#if NORDIC_SDK_VERSION >= 11
nrf_nvic_state_t nrf_nvic_state = {0};

/**< Clock source used by the                 \
     Softdevice.For calibrating timeslot      \
     time. */
static nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
#define MESH_CLOCK_SOURCE (clock_lf_cfg)
#else
#define MESH_CLOCK_SOURCE (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)
#endif

/** @brief General error handler. */
static inline void error_loop(void) {
  TOGGLE_LED(LED_RED);
  __disable_irq();
  while (true) {
    __WFE();
  }
}

/**
* @brief Softdevice crash handler, never returns
*
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num,
                       const uint8_t *p_file_name) {
  SEGGER_RTT_WriteString(0, "sd_assert_handler (looping forever)\n");
  error_loop();
}

/** @brief Hardware fault handler. */
void HardFault_Handler(void) {
  SEGGER_RTT_WriteString(0, "HardFault_Handler (looping forever)\n");
  error_loop();
}

/**
* @brief Mesh framework event handler.
*
* @param[in] p_evt Mesh event propagated from framework.
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t *p_evt) {

  switch (p_evt->type) {
  case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
  case RBC_MESH_EVENT_TYPE_NEW_VAL:
  case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
    if (p_evt->params.rx.value_handle == MESH_CONTROL_HANDLE &&
        p_evt->params.rx.data_len == sizeof(mesh_control_t)) {
      mesh_control_update_config((mesh_control_t *)p_evt->params.rx.p_data);
    }
    break;
  case RBC_MESH_EVENT_TYPE_TX:
  case RBC_MESH_EVENT_TYPE_INITIALIZED:
  case RBC_MESH_EVENT_TYPE_DFU_NEW_FW_AVAILABLE:
  case RBC_MESH_EVENT_TYPE_DFU_RELAY_REQ:
  case RBC_MESH_EVENT_TYPE_DFU_SOURCE_REQ:
  case RBC_MESH_EVENT_TYPE_DFU_START:
  case RBC_MESH_EVENT_TYPE_DFU_END:
  case RBC_MESH_EVENT_TYPE_DFU_BANK_AVAILABLE:
    break;
  }
}

/* dispatch system events to interested modules. */
static void sys_evt_dispatch(uint32_t sys_evt) {
  // pstorage_sys_event_handler(sys_evt);
  // ble_advertising_on_sys_evt(sys_evt);
}

void clock_initialization() {
  NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;

  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {
    // Do nothing.
  }

  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}

static void packet_peek_cb(rbc_mesh_packet_peek_params_t *params) {
  if (params->packet_type == BLE_PACKET_TYPE_ADV_NONCONN_IND &&
      params->p_payload[1] == HEARTBEAT_ADV_DATA_TYPE) {
    received_heartbeat((heartbeat_ad_t *)&params->p_payload[2], params->rssi);
  }
}

int main(void) {
  SEGGER_RTT_WriteString(0, "\nmain()\n");

  bsp_init(BSP_INIT_BUTTONS & BSP_INIT_LED, 0, 0);

  SEGGER_RTT_WriteString(0, "Mesh control init\n");
  mesh_control_init();

#if LEDS_NUMBER > 0
  SEGGER_RTT_WriteString(0, "Some LEDs\n");
  nrf_gpio_cfg_output(LED_START + LED_GREEN);
  nrf_gpio_cfg_output(LED_START + LED_RED);
  nrf_gpio_cfg_output(LED_START + LED_BLUE);
#endif

  SEGGER_RTT_WriteString(0, "Enabling sd handler\n");
/* Enable Softdevice (including sd_ble before framework */

// Initialize the SoftDevice handler module.
#if (NORDIC_SDK_VERSION >= 11)
  SOFTDEVICE_HANDLER_INIT(&MESH_CLOCK_SOURCE, NULL);
#else
  SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SOURCE, NULL);
#endif
  SEGGER_RTT_WriteString(0, "Done enabling sd handler\n");
  softdevice_ble_evt_handler_set(rbc_mesh_ble_evt_handler);

  // clock_initialization();
  // sd_softdevice_disable();

  // Register with the SoftDevice handler module for system events.
  softdevice_sys_evt_handler_set(sys_evt_dispatch);

// Debug pins
// nrf_gpio_cfg_output(5);
// nrf_gpio_cfg_output(6);

// Disable simblee's proximityMode
#ifdef SIMBLEE
  nrf_gpio_cfg_output(31);
  CLEAR_PIN(31);
#endif

  // if (NRF_CLOCK->LFCLKSRC == (CLOCK_LFCLKSRC_SRC_Xtal <<
  // CLOCK_LFCLKSRC_SRC_Pos)) {
  //   TOGGLE_LED(LED_GREEN);
  // }

  /* Initialize mesh. */
  rbc_mesh_init_params_t init_params;
  init_params.access_addr = MESH_ACCESS_ADDR;
  init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
  init_params.channel = Config.DEFAULT_MESH_CHANNEL;
  init_params.lfclksrc = clock_lf_cfg; // MESH_CLOCK_SOURCE;
  init_params.tx_power = RBC_MESH_TXPOWER_0dBm;

  uint32_t error_code;
  error_code = rbc_mesh_init(init_params);
  APP_ERROR_CHECK(error_code);
  // led_config(LED_GREEN, 1);

  // This has to come after rbc_mesh_init for some reason.  Otherwise we
  // get a HardFault when rbc_mesh_init is called
  Config.Init();

  // Setup handler for watching for heartbeat messages
  rbc_mesh_packet_peek_cb_set(packet_peek_cb);

  // Change channel if needed
  // if (app_config.mesh_channel != DEFAULT_MESH_CHANNEL) {
  //   tc_radio_params_set(MESH_ACCESS_ADDR, app_config.mesh_channel);
  // }

  // Initializes, but does not start, clock
  scheduler_init(Config.GetSleepEnabled());
  // Inits structures for sending heartbeat
  heartbeat_init(Config.DEFAULT_MESH_CHANNEL);

  /* Initialize mesh ACI */
  mesh_aci_init();
  mesh_aci_app_cmd_handler_set(app_cmd_handler);

  // Stop serial if serial is disabled
  if (!Config.GetSerialEnabled()) {
    serial_handler_stop();
  }

  /* Enable our handle */
  if (Config.GetSensorID() > 0) {
    sensor_init();
  }

  error_code = rbc_mesh_value_enable(MESH_CONTROL_HANDLE);
  APP_ERROR_CHECK(error_code);

  // error_code = rbc_mesh_value_enable(TEST_LED_HANDLE);
  // APP_ERROR_CHECK(error_code);

  // Start clock
  start_clock(0);

  // rbc_mesh_stop();
  // rbc_mesh_start();
  // sd_app_evt_wait();
  // while(1) {
  //   __WFE();
  //   __SEV();
  //   __WFE();
  // }

  SEGGER_RTT_WriteString(0, "Main loop\n");
  rbc_mesh_event_t evt;
  while (true) {
    if (rbc_mesh_event_get(&evt) == NRF_SUCCESS) {
      rbc_mesh_event_handler(&evt);
      rbc_mesh_event_release(&evt);
    }
    sd_app_evt_wait();
  }
}