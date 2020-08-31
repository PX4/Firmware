/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * av_x-v1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/* Configuration ************************************************************************************/

#define BOARD_HAS_NBAT_V              1 // Only one Vbat to ADC
#define BOARD_HAS_NBAT_I              0 // No Ibat ADC

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                      (n)
#define ADC1_GPIO(n)                    GPIO_ADC1_IN##n

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */

#define PX4_ADC_GPIO  \
	/* PA0 */  ADC1_GPIO(0),  \
	/* PA1 */  ADC1_GPIO(1),  \
	/* PA2 */  ADC1_GPIO(2),  \
	/* PA3 */  ADC1_GPIO(3),  \
	/* PA4 */  ADC1_GPIO(4),  \
	/* PB8 */  ADC1_GPIO(8),  \
	/* PC0 */  ADC1_GPIO(10), \
	/* PC1 */  ADC1_GPIO(11), \
	/* PC2 */  ADC1_GPIO(12), \
	/* PC3 */  ADC1_GPIO(13), \
	/* PC4 */  ADC1_GPIO(14)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY1_VOLTAGE_CHANNEL        /* PA0 */  ADC1_CH(0)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL))

/* Define Battery 1 Voltage Divider and A per V
 */

#define BOARD_BATTERY1_V_DIV         (10.133333333f)
#define BOARD_BATTERY1_A_PER_V       (36.367515152f)

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V               (5.6f)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  9
#define DIRECT_INPUT_TIMER_CHANNELS  9

#define BOARD_CAPTURE_GPIO /* PD14 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN14)

/* High-resolution timer */
#define HRT_TIMER		     5  /* use timer5 for the HRT */
#define HRT_TIMER_CHANNEL    1  /* use capture/compare channel 3 */

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS4"

/* Power switch controls ******************************************************/

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* AV-X_V1 never powers off the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#define ADC_BATTERY_VOLTAGE_CHANNEL  0
#define ADC_BATTERY_CURRENT_CHANNEL  1 // TODO: review

#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN1_TX,                     \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 6

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

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

#define board_peripheral_reset(ms)


#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
