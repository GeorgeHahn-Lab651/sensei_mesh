

#include "rbc_mesh.h"
#include "mesh_aci.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "leds.h"
#include "pstorage_platform.h"
#include "app_cmd.h"
#include "config.h"
#include "sensor.h"
#include "transport_control.h"
#include "scheduler.h"
#include "heartbeat.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>


#define MESH_ACCESS_ADDR        (0xA555410C)
#define MESH_INTERVAL_MIN_MS    (100)
#define MESH_CHANNEL            (38)
#define MESH_CLOCK_SRC          (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)
#define TEST_LED_HANDLE         (0xfe01)

/** @brief General error handler. */
static inline void error_loop(void)
{
  toggle_led(LED_RED);
  __disable_irq();
  while (true)
  {
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
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
  error_loop();
}

/** @brief Hardware fault handler. */
void HardFault_Handler(void)
{
  error_loop();
}

/**
* @brief Mesh framework event handler.
*
* @param[in] p_evt Mesh event propagated from framework.
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* p_evt)
{

  switch (p_evt->type)
  {
    case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
    case RBC_MESH_EVENT_TYPE_NEW_VAL:
    case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
      if (p_evt->params.rx.value_handle == TEST_LED_HANDLE) {
        led_config(LED_BLUE, p_evt->params.rx.p_data[0]);
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
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    //ble_advertising_on_sys_evt(sys_evt);
}


void clock_initialization()
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}

static void packet_peek_cb(rbc_mesh_packet_peek_params_t *params) {
  if (params->packet_type == BLE_PACKET_TYPE_ADV_NONCONN_IND &&
      params->p_payload[1] == HEARTBEAT_ADV_DATA_TYPE) {
    received_heartbeat((heartbeat_ad_t*)&params->p_payload[2]);
  }
}

int main(void)
{
  //clock_initialization();

  nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLDOWN);
  nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLDOWN);

  /* Enable Softdevice (including sd_ble before framework */
  SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SRC, NULL);
  softdevice_ble_evt_handler_set(rbc_mesh_ble_evt_handler);

  // Register with the SoftDevice handler module for system events.
  softdevice_sys_evt_handler_set(sys_evt_dispatch);

  LEDS_CONFIGURE(LEDS_MASK);


  // if (NRF_CLOCK->LFCLKSRC == (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos)) {
  //   toggle_led(LED_GREEN);
  // }

  /* Initialize mesh. */
  rbc_mesh_init_params_t init_params;
  init_params.access_addr = MESH_ACCESS_ADDR;
  init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
  init_params.channel = MESH_CHANNEL;
  init_params.lfclksrc = MESH_CLOCK_SRC;
  init_params.tx_power = RBC_MESH_TXPOWER_0dBm;

  uint32_t error_code;
  error_code = rbc_mesh_init(init_params);
  APP_ERROR_CHECK(error_code);

  // Setup handler for watching for heartbeat messages
  rbc_mesh_packet_peek_cb_set(packet_peek_cb);

  // This has to come after rbc_mesh_init for some reason.  Otherwise we
  // get a HardFault when rbc_mesh_init is called
  app_config_t app_config;
  config_init();
  get_config(&app_config);

  // Change channel if needed
  if (app_config.mesh_channel != 38) {
    tc_radio_params_set(MESH_ACCESS_ADDR, app_config.mesh_channel);
  }

  scheduler_init(); // Initializes, but does not start, timer
  heartbeat_init(); // Inits structures for sending heartbeat

  /* Initialize serial ACI */
  if (app_config.serial_enabled) {
    mesh_aci_init();
    mesh_aci_app_cmd_handler_set(app_cmd_handler);
  }

  /* Enable test led handle */
  error_code = rbc_mesh_value_enable(TEST_LED_HANDLE);
  APP_ERROR_CHECK(error_code);

  /* Enable our handle */
  if (app_config.sensor_id > 0) {
    sensor_init();
  }

  /* Main event loop */
  rbc_mesh_event_t evt;
  while (true)
  {
    for (uint32_t pin = BUTTON_START; pin <= BUTTON_STOP; ++pin)
    {
      if(nrf_gpio_pin_read(pin) == 1)
      {
        while(nrf_gpio_pin_read(pin) == 1);
        uint8_t mesh_data[1];
        uint32_t led_status = !!((pin - BUTTON_START) & 0x01); /* even buttons are OFF, odd buttons are ON */

        mesh_data[0] = led_status;
        led_config(LED_BLUE, led_status);
        error_code = rbc_mesh_value_set(TEST_LED_HANDLE, mesh_data, 1);
        APP_ERROR_CHECK(error_code);
      }
    }

    if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
    {
      rbc_mesh_event_handler(&evt);
      rbc_mesh_event_release(&evt);
    }

    sd_app_evt_wait();
  }
}
