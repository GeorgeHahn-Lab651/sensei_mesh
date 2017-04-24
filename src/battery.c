#include "battery.h"
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
