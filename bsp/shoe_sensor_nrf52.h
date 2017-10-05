

#ifndef SHOE_SENSOR_H
#define SHOE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#define BMX055

// LEDs definitions for Sensei shoe sensor board
#define LEDS_NUMBER 0
#define BUTTONS_NUMBER 0

#define ARDUINO_SCL_PIN 18
#define ARDUINO_SDA_PIN 02

#define INT1_GPIO_PIN 20
#define INT2_GPIO_PIN 00

#define BUTTONS_LIST                                                           \
  { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0 BUTTON_1
#define BSP_BUTTON_1 BUTTON_2

#define BSP_BUTTON_0_MASK (1 << BSP_BUTTON_0)
#define BSP_BUTTON_1_MASK (1 << BSP_BUTTON_1)

#define BUTTONS_MASK (BSP_BUTTON_0_MASK | BSP_BUTTON_1_MASK)

#define RX_PIN_NUMBER 0
#define TX_PIN_NUMBER 1
//#define CTS_PIN_NUMBER 0xff
//#define RTS_PIN_NUMBER 0xff
#define HWFC false

// Low frequency clock source to be used by the SoftDevice & mesh

// If you have LFXTAL available
#define NRF_CLOCK_LFCLKSRC                                                     \
  {                                                                            \
    .source = NRF_CLOCK_LF_SRC_XTAL, .rc_ctiv = 0, .rc_temp_ctiv = 0,          \
    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM                         \
  }

// Otherwise, synthesize it internally
// #define NRF_CLOCK_LFCLKSRC      {.source       = NRF_CLOCK_LF_SRC_RC,               \
//                                 .rc_ctiv       = 16,                                \
//                                 .rc_temp_ctiv  = 2,                                 \
//                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif // SHOE_SENSOR_H
