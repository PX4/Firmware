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
 * mRo X2.1 internal definitions
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

/* PX4IO connection configuration */
#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE	"/dev/ttyS4"
#define PX4IO_SERIAL_TX_GPIO	GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO	GPIO_USART6_RX
#define PX4IO_SERIAL_BASE	STM32_USART6_BASE	/* hardwired on the board */
#define PX4IO_SERIAL_VECTOR	STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP	DMAMAP_USART6_TX
#define PX4IO_SERIAL_RX_DMAMAP	DMAMAP_USART6_RX
#define PX4IO_SERIAL_RCC_REG	STM32_RCC_APB2ENR
#define PX4IO_SERIAL_RCC_EN	RCC_APB2ENR_USART6EN
#define PX4IO_SERIAL_CLOCK	STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE	1500000			/* 1.5Mbps -> max rate for IO */


/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED1		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)
#define BOARD_OVERLOAD_LED     LED_RED

/* External interrupts but on board*/
#define GPIO_EXTI_MPU_DRDY              (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)

/* Data ready pins but on board */

#define GPIO_EXTI_ICM_2060X_DRDY      (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN14)

/* Data ready pins off */
#define GPIO_EXTI_MPU_DRDY_OFF          (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)
#define GPIO_EXTI_ICM_2060X_DRDY_OFF  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTC|GPIO_PIN14)

/* SPI1 off */
#define GPIO_SPI1_SCK_OFF   (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN5)
#define GPIO_SPI1_MISO_OFF  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN6)
#define GPIO_SPI1_MOSI_OFF  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN7)

/* SPI1 chip selects off */
#define GPIO_SPI_CS_ICM_2060X_OFF (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI_CS_BARO_OFF        (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN7)
#define GPIO_SPI_CS_MPU_OFF		    (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTC|GPIO_PIN2)

/* SPI chip selects */
#define GPIO_SPI_CS_ICM_2060X	 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI_CS_BARO         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)
#define GPIO_SPI_CS_FRAM         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
#define GPIO_SPI_CS_MPU          (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN2)

#define PX4_SPI_BUS_SENSORS	1
#define PX4_SPI_BUS_RAMTRON	2
#define PX4_SPI_BUS_BARO	PX4_SPI_BUS_SENSORS

/* Use these in place of the uint32_t enumeration to select a specific SPI device on SPI1 */
#include <drivers/drv_sensor.h>
#define PX4_SPIDEV_ICM_20602 PX4_MK_SPI_SEL(0, DRV_IMU_DEVTYPE_ICM20602)
#define PX4_SPIDEV_ICM_20608 PX4_MK_SPI_SEL(0, DRV_IMU_DEVTYPE_ICM20608)
#define PX4_SPIDEV_BARO      PX4_MK_SPI_SEL(0, DRV_BARO_DEVTYPE_MS5611)
#define PX4_SPIDEV_MPU       PX4_MK_SPI_SEL(0, DRV_IMU_DEVTYPE_MPU9250)

/* I2C busses */

/* There is no I2C2 so there is not notion of internal / external*/
#define PX4_I2C_BUS_EXPANSION 1
#define PX4_I2C_BUS_LED		PX4_I2C_BUS_EXPANSION

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 2) | (1 << 3) | (1 << 4) | (1 << 13) | (1 << 14) | (1 << 15)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL  2
#define ADC_BATTERY_CURRENT_CHANNEL  3
#define ADC_5V_RAIL_SENSE            4
#define ADC_AIRSPEED_VOLTAGE_CHANNEL 15

/* Power supply control and monitoring GPIOs */
// Signal is not connected
#define GPIO_VDD_5V_PERIPH_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define GPIO_VDD_BRICK_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_VDD_3V3_SENSORS_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_VDD_5V_PERIPH_OC	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)

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
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER		4
#define PWMIN_TIMER_CHANNEL	2
#define GPIO_PWM_IN		GPIO_TIM4_CH2IN_2

/* By Providing BOARD_ADC_USB_CONNECTED this board support the ADC
 * system_power interface, and therefore provides the true logic
 * GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK_VALID   (!px4_arch_gpioread(GPIO_VDD_BRICK_VALID))
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (0)

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE (5120 + 512 + 512)	// 5120 fat + 512 + 512 spi

/* This board provides the board_on_reset interface */

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

extern void board_peripheral_reset(int ms);

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void stm32_usbinitialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */
__END_DECLS
