/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

// LEDs definitions for PCA10028
#define LEDS_NUMBER    3

#define LED_START      2
#define LED_1          2
#define LED_2          3
#define LED_3          4
#define LED_STOP       4

#define LEDS_LIST { LED_1, LED_2, LED_3 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK )
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 2

#define BUTTON_START   5
#define BUTTON_1       5
#define BUTTON_2       6
#define BUTTON_STOP    6
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

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

#endif // CUSTOM_BOARD_H
