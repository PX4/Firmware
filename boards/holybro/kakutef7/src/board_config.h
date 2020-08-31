/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * KakuteF7 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* GPIOs ***********************************************************************************/

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#define GPIO_nLED_BLUE       /* PA2 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN2)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_ARMED_STATE_LED  LED_BLUE

#define  FLASH_BASED_PARAMS


/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */
#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n)                GPIO_ADC1_IN##n

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */
#define PX4_ADC_GPIO  \
	/* PC2 */  ADC1_GPIO(12),  \
	/* PC3 */  ADC1_GPIO(13),  \
	/* PC5 */  ADC1_GPIO(15)

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_RSSI_IN_CHANNEL                 /* PC5 */  ADC1_CH(15)
#define ADC_BATTERY_VOLTAGE_CHANNEL         /* PC3 */  ADC1_CH(13)
#define ADC_BATTERY_CURRENT_CHANNEL         /* PC2 */  ADC1_CH(12)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)       | \
	 (1 << ADC_RSSI_IN_CHANNEL))

/* Define Battery 1 Voltage Divider and A per V
 */
#define BOARD_BATTERY1_V_DIV         (10.9f)
#define BOARD_BATTERY1_A_PER_V       (17.f)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  6
#define DIRECT_INPUT_TIMER_CHANNELS  6

/* Tone alarm output */
#define GPIO_TONE_ALARM_IDLE    /* PD15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)
#define GPIO_TONE_ALARM_GPIO    /* PD15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN15)

/* USB OTG FS
 *
 * PA8  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA8 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN8)

/* High-resolution timer */
#define HRT_TIMER               2  /* use timer 2 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS4"

#define GPIO_RSSI_IN                       /* PC5  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN5)

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_RSSI_IN,                \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 4

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
