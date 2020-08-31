/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * PX4FMUv4Pro internal definitions
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

/* PX4IO connection configuration */
#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE     "/dev/ttyS4"
#define PX4IO_SERIAL_TX_GPIO    GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO    GPIO_USART6_RX
#define PX4IO_SERIAL_BASE       STM32_USART6_BASE	/* hardwired on the board */
#define PX4IO_SERIAL_VECTOR     STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP  DMAMAP_USART6_TX
#define PX4IO_SERIAL_RX_DMAMAP  DMAMAP_USART6_RX
#define PX4IO_SERIAL_RCC_REG	STM32_RCC_APB2ENR
#define PX4IO_SERIAL_RCC_EN	RCC_APB2ENR_USART6EN
#define PX4IO_SERIAL_CLOCK      STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE    1500000			/* 1.5Mbps -> max rate for IO */

/* Configuration ************************************************************************************/
//{GPIO_RSSI_IN,           0,                       0}, - pio Analog used as PWM
//{0,                      GPIO_LED_SAFETY,         0},	pio replacement
//{GPIO_SAFETY_SWITCH_IN,  0,                       0},   pio replacement
//{0,                      GPIO_PERIPH_3V3_EN,      0},	Owned by the 8266 driver
//{0,                      GPIO_SBUS_INV,           0},	https://github.com/PX4/Firmware/blob/master/src/modules/px4iofirmware/sbus.c
//{GPIO_8266_GPIO0,        0,                       0},   Owned by the 8266 driver
//{0,                      GPIO_SPEKTRUM_PWR_EN,     0},	Owned Spektum driver input to auto pilot
//{0,                      GPIO_8266_PD,            0},	Owned by the 8266 driver
//{0,                      GPIO_8266_RST,           0},	Owned by the 8266 driver

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED1		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN11)
#define GPIO_LED2		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_LED3		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN3)

#define GPIO_LED_RED 	GPIO_LED1
#define GPIO_LED_GREEN 	GPIO_LED2
#define GPIO_LED_BLUE   GPIO_LED3

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_BLUE
#define BOARD_ARMED_STATE_LED  LED_GREEN

/* SPI */
#define GPIO_SPI_CS_TEMPCAL_EEPROM	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN5)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
// ADC defines to be used in sensors.cpp to read from a particular channel

#define ADC_BATTERY1_VOLTAGE_CHANNEL	2   // PA2
#define ADC_BATTERY1_CURRENT_CHANNEL	3   // PA3
#define ADC_5V_RAIL_SENSE				4   // PA4
#define ADC_BATTERY2_VOLTAGE_CHANNEL	11  // PC1
#define ADC_BATTERY2_CURRENT_CHANNEL	13  // PC3

#define ADC_CHANNELS (1 << ADC_BATTERY1_VOLTAGE_CHANNEL) | (1 << ADC_BATTERY1_CURRENT_CHANNEL) | \
	(1 << ADC_5V_RAIL_SENSE) | \
	(1 << ADC_BATTERY2_VOLTAGE_CHANNEL) | (1 << ADC_BATTERY2_CURRENT_CHANNEL)

/* Define Battery 1 Voltage Divider and A per V
 */

#define BOARD_BATTERY1_V_DIV (6.490196078f)
#define BOARD_BATTERY1_A_PER_V (26.4f)

/* Define Battery 2 Voltage Divider and A per V
 */

#define BOARD_BATTERY2_V_DIV (6.490196078f)
#define BOARD_BATTERY2_A_PER_V (26.4f)

/* Define LTC4417 UV set by resistors on the board that are different than FMUv2 3.7V */

#define BOARD_VALID_UV (4.01f)

/* Power supply control and monitoring GPIOs */
#define GPIO_nVDD_BRICK1_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_nVDD_BRICK2_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN5)
#define BOARD_NUMBER_BRICKS     2
#define GPIO_nVDD_USB_VALID		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)
#define GPIO_VDD_3V3_PERIPH_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)
#define GPIO_VDD_5V_PERIPH_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN10)
#define GPIO_VDD_5V_HIPOWER_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN4)
#define GPIO_VDD_5V_PERIPH_OC	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTG|GPIO_PIN4)
#define GPIO_VDD_5V_HIPOWER_OC	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN3)

/* Tone alarm output */
#define TONE_ALARM_TIMER	2	/* timer 2 */
#define TONE_ALARM_CHANNEL	1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN15)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS	6
#define DIRECT_INPUT_TIMER_CHANNELS  6

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER           3 /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL   4 /* use capture/compare channel 4 */

#define HRT_PPM_CHANNEL     3	/* use capture/compare channel 3 */
#define GPIO_PPM_IN         (GPIO_ALT|GPIO_AF2|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER         4
#define PWMIN_TIMER_CHANNEL 2
#define GPIO_PWM_IN			GPIO_TIM4_CH2IN_2

/* N.B. By the virtue of not being named GPIO_BTN_SAFETY, this pin is
 * initialized but not used (compiled for) in the FMU module
 */
#define GPIO_BTN_SAFETY_FMU		(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN4)
#define GPIO_SBUS_INV			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)

#define GPIO_8266_GPIO0			(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN2)
//TODO: fo not see on schematic #define GPIO_SPEKTRUM_PWR_EN		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_8266_PD			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)
#define GPIO_8266_RST			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN6)

/* No Power switch controls or binding control *********************************************
 * V4 Pro does not have control to bind SPEKTRUM - there is only 5V VCC on
 * the connector interface and Spektrum requires VDD 3v3 to be controllable
 */

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_OC))

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define BOARD_HAS_ON_RESET 1

#define BOARD_HAS_STATIC_MANIFEST 1

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
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
