

#ifndef SHOE_SENSOR_H
#define SHOE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#define ACCEL_ADXL337
#define SIMBLEE

// LEDs definitions for Sensei shoe sensor board
#define LEDS_NUMBER    0
#define BUTTONS_NUMBER 0


#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)

#define BUTTONS_MASK   (BSP_BUTTON_0_MASK | BSP_BUTTON_1_MASK )

#define RX_PIN_NUMBER  0
#define TX_PIN_NUMBER  1
//#define CTS_PIN_NUMBER 0xff
//#define RTS_PIN_NUMBER 0xff
#define HWFC           false

#ifdef __cplusplus
}
#endif

#endif // SHOE_SENSOR_H
