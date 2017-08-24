#include "battery.h"
#include "assert.h"

#ifdef NRF52
#include "app_timer.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "power_manage.h"
#include "softdevice_handler.h"
#include <stdint.h>
#include <string.h>

void saadc_event_handler(nrf_drv_saadc_evt_t const *p_event) {
  log("saadc event");
}

uint8_t get_battery_adc() {
  // Initialize ADC
  nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_8BIT;
  ret_code_t err_code = nrf_drv_saadc_init(&saadc_config, saadc_event_handler);
  APP_ERROR_CHECK(err_code);

  // Initialize ADC channel
  nrf_saadc_channel_config_t config =
      NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
  err_code = nrf_drv_saadc_channel_init(0, &config);
  APP_ERROR_CHECK(err_code);

  // Get an ADC reading
  nrf_saadc_value_t val;
  err_code = nrf_drv_saadc_sample_convert(0, &val);
  APP_ERROR_CHECK(err_code);

  // Uninitialize channel (Not strictly necessary; the call to
  // nrf_drv_saadc_uninit below should handle this)
  err_code = nrf_drv_saadc_channel_uninit(0);
  APP_ERROR_CHECK(err_code);

  // Uninitialize ADC
  nrf_drv_saadc_uninit();

  return val;
}

void battery_init() {}
#endif

#ifdef NRF51
#include "nrf_adc.h"

void battery_init() {}
uint8_t get_battery_adc() {
  uint8_t res;
  // Configure ADC
  NRF_ADC->CONFIG =
      (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->ENABLE = (ADC_ENABLE_ENABLE_Enabled << ADC_ENABLE_ENABLE_Pos);

  NRF_ADC->EVENTS_END = 0; // Stop any running conversions.
  NRF_ADC->TASKS_START = 1;

  while (!NRF_ADC->EVENTS_END)
    ; // TODO: Should probably power_manage() here

  res = NRF_ADC->RESULT;

  NRF_ADC->ENABLE = (ADC_ENABLE_ENABLE_Disabled << ADC_ENABLE_ENABLE_Pos);
  NRF_ADC->TASKS_STOP = 1;

  // GPIOs release regarding PAN028
  NRF_ADC->CONFIG =
      (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
      (ADC_CONFIG_INPSEL_SupplyTwoThirdsPrescaling << ADC_CONFIG_INPSEL_Pos) |
      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);

  return res;
}
#endif
