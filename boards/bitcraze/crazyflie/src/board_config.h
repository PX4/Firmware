/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * PX4-CRAZYFLIE internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* Crazyflie GPIOs **********************************************************************************/
/* LEDs */


/* Radio TX indicator */
#define GPIO_LED_RED_L       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
			      GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN0)
/* Radio RX indicator */
#define GPIO_LED_GREEN_L       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
				GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)
#define GPIO_LED_GREEN_R       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
				GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN2)
#define GPIO_LED_RED_R       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
			      GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN3)

/* PX4: armed state indicator ; Stock FW: Blinking while charging */
#define GPIO_LED_BLUE_L		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN2)

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_BLUE
#define BOARD_ARMED_STATE_LED  LED_GREEN

#define LED_TX 4
#define LED_RX 5

#define GPIO_FSYNC_MPU9250		(GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN14) // Needs to be set low
#define GPIO_DRDY_MPU9250		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)


#define GPIO_NRF_TXEN			(GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTA|GPIO_PIN4)


/*
 * I2C busses
 */
#define PX4_I2C_BUS_ONBOARD_HZ      400000
#define PX4_I2C_BUS_EXPANSION_HZ      400000

#define PX4_I2C_BUS_MTD	1

#define BOARD_NUMBER_I2C_BUSES  3
#define BOARD_I2C_BUS_CLOCK_INIT {PX4_I2C_BUS_ONBOARD_HZ, 100000, PX4_I2C_BUS_EXPANSION_HZ}


/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_MPU9250	0x69

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS 0

/* Tone alarm output : These are only applicable when the buzzer deck is attached */
#define TONE_ALARM_TIMER	5	/* timer 5 */
#define TONE_ALARM_CHANNEL 3	/* channel 3 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF2|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN2)
#define GPIO_TONE_ALARM_NEG (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)

/* PWM
*/

#define DIRECT_PWM_OUTPUT_CHANNELS	4

/* This board overrides the defaults by providing
 * PX4_PWM_ALTERNATE_RANGES and a replacement set of
 * constants
 */


/* PWM directly wired to transistor. Duty cycle directly corresponds to power
 * So we need to override the defaults
 */

#define PX4_PWM_ALTERNATE_RANGES
#define PWM_LOWEST_MIN 0
#define PWM_MOTOR_OFF	0
#define PWM_DEFAULT_MIN 20
#define PWM_HIGHEST_MIN 0
#define PWM_HIGHEST_MAX 255
#define PWM_DEFAULT_MAX 255
#define PWM_LOWEST_MAX 255
#define PWM_DEFAULT_TRIM 1500


/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE (1024*3)

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
 * Name: board_peripheral_reset
 *
 * Description:
 *   Called to reset the periferal bus
 *
 ****************************************************************************************************/

#define board_peripheral_reset(ms)

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void stm32_usbinitialize(void);

/****************************************************************************
 * Name: board_i2c_initialize
 *
 * Description:
 *   Called to set I2C bus frequencies.
 *
 ****************************************************************************/

int board_i2c_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

extern int stm32_spi_bus_initialize(void);


#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
