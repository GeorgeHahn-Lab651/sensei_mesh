#include "battery.h"

#if NORDIC_SDK_VERSION >= 11
/*
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

volatile uint8_t battery_level = 0;
volatile bool battery_level_done = false;

#define SAADC_SAMPLES_IN_BUFFER         4
#define SAADC_SAMPLE_RATE_DIVIDER	0x2000;
#define SAADC_CONFIG_RESOLUTION      NRF_SAADC_RESOLUTION_10BIT
#define SAADC_CONFIG_OVERSAMPLE      NRF_SAADC_OVERSAMPLE_DISABLED
#define SAADC_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_ppi_channel_t m_ppi_channel;

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_timer_init(&m_timer, NULL, timer_handler);
    APP_ERROR_CHECK(err_code);

    // setup m_timer for compare event
    uint32_t ticks = SAADC_SAMPLE_RATE_DIVIDER;
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    // setup ppi channel so that timer compare event is triggering sample task in SAADC
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        uint16_t adc_value;

        // set buffers
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
						
        // print samples on hardware UART and parse data for BLE transmission
        printf("ADC event number: %d\r\n",(int)m_adc_evt_counter);
        for (int i = 0; i < SAADC_SAMPLES_IN_BUFFER; i++)
        {
            adc_value = p_event->data.done.p_buffer[i];
            
            printf("%d\r\n", adc_value);
        }
						
        m_adc_evt_counter++;
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
	
    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
	
    nrf_saadc_channel_config_t channel_0_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
    channel_0_config.gain = NRF_SAADC_GAIN1_4;
    channel_0_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_1_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
    channel_1_config.gain = NRF_SAADC_GAIN1_4;
    channel_1_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_2_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
    channel_2_config.gain = NRF_SAADC_GAIN1_4;
    channel_2_config.reference = NRF_SAADC_REFERENCE_VDD4;
	
    nrf_saadc_channel_config_t channel_3_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);
    channel_3_config.gain = NRF_SAADC_GAIN1_4;
    channel_3_config.reference = NRF_SAADC_REFERENCE_VDD4;				
	
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
    APP_ERROR_CHECK(err_code);	

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);   
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}


uint8_t get_battery_adc() {
    saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();

    while(!battery_level_done) {
        uint32_t err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
    return battery_level;
}
*/

uint8_t get_battery_adc() { return 0; }

#else

#include "nrf_adc.h"
uint8_t get_battery_adc() {
    uint8_t res;
    // Configure ADC
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
                      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE = (ADC_ENABLE_ENABLE_Enabled << ADC_ENABLE_ENABLE_Pos);

    NRF_ADC->EVENTS_END = 0; // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;

    while (!NRF_ADC->EVENTS_END);

    res = NRF_ADC->RESULT;

    NRF_ADC->ENABLE = (ADC_ENABLE_ENABLE_Disabled << ADC_ENABLE_ENABLE_Pos);
    NRF_ADC->TASKS_STOP = 1;

    // GPIOs release regarding PAN028
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
                      (ADC_CONFIG_INPSEL_SupplyTwoThirdsPrescaling << ADC_CONFIG_INPSEL_Pos) |
                      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);

    return res;
}
#endif
