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
 * @author David Sidrane <david_s5@nscdg.com>
 * @author Lorenz Meier <lorenz@px4.io>
 *
 * AEROFC_V1 internal definitions
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

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */
#define GPIO_LED0		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN9)
#define GPIO_LED1		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)
#define GPIO_LED2		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN11)
#define GPIO_LED3		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)

#define BOARD_HAS_CONTROL_STATUS_LEDS	1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_BLUE
#define BOARD_ARMED_STATE_LED  LED_GREEN

#define GPIO_VDD_5V_SENSORS_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN13)

#define GPIO_FORCE_BOOTLOADER	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN11|GPIO_EXTI)

/*
 * I2C busses
 *
 * Peripheral   Port
 * I2C1_SDA     PB9
 * I2C1_SCL     PB8
 *
 * I2C3_SDA     PC9
 * I2C3_SCL     PA8
 *
 */

#define PX4_I2C_BUS_EXPANSION   1
#define PX4_I2C_BUS_EXPANSION1  2
#define PX4_I2C_BUS_ONBOARD	3

#include <drivers/drv_sensor.h>
#define GPIO_SPI_CS_MPU6500	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define PX4_SPI_BUS_SENSORS	1
#define PX4_SPIDEV_MPU		PX4_MK_SPI_SEL(0, DRV_IMU_DEVTYPE_MPU9250)

/*
 * STM32 ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can
 * be used by the PX4 Firmware in the adc driver
 */
#define ADC_CHANNELS 0

/*
 * ADC channels: these use the ADC block implemented in the FPGA. There are 5
 * channels in the FPGA, but only 1 is phisically connected, that's dedicated
 * for voltage reading.
 */
#define ADC_BATTERY_VOLTAGE_CHANNEL    1
#define ADC_BATTERY_CURRENT_CHANNEL    ((uint8_t)(-1))

/*
 * Define Battery 1 Voltage Divider, using default for A/V
 */
#define BOARD_BATTERY1_V_DIV (9.0f)

#define DIRECT_PWM_OUTPUT_CHANNELS	1
#define BOARD_HAS_PWM			0

/*
 * USB OTG FS
 */

/*
 * RC Serial port
 */
#define RC_SERIAL_PORT		"/dev/ttyS2" /* No HW invert support */
/*
 * High-resolution timer
 */
#define HRT_TIMER		3	/* use timer3 for the HRT */
#define HRT_TIMER_CHANNEL	4	/* use capture/compare channel */

#define  FLASH_BASED_PARAMS
#define  FLASH_BASED_DATAMAN

/*
 * ESCs do not respond
 */
#define BOARD_TAP_ESC_MODE 1

#define MEMORY_CONSTRAINED_SYSTEM

#define BOARD_CRASHDUMP_RESET_ONLY

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

#define board_peripheral_reset(ms)

/************************************************************************************
 * Name: board_sdio_initialize
 *
 * Description:
 *   Called to configure SDIO.
 *
 ************************************************************************************/

extern int board_sdio_initialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
