/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file px4iov2_init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * stm32_boardinitialize() function that is called during cpu startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>

#include <stm32.h>
#include "board_config.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{

	/* configure GPIOs */

	/* Set up for sensing HW */

	stm32_configgpio(GPIO_SENSE_PC14_DN);
	stm32_configgpio(GPIO_SENSE_PC15_UP);

	/* LEDS - default to off */
	stm32_configgpio(GPIO_LED1);
	stm32_configgpio(GPIO_LED2);
	stm32_configgpio(GPIO_LED3);
	stm32_configgpio(GPIO_LED4);

	/*  PixHawk 1:
	 *      PC14 Floating
	 *      PC15 Floating
	 *
	 *  PixHawk 2:
	 *      PC14 3.3v
	 *      PC15 GND
	 */

	uint8_t sense = stm32_gpioread(GPIO_SENSE_PC15_UP) << 1  | stm32_gpioread(GPIO_SENSE_PC14_DN);

	if (sense == SENSE_PH2) {
		stm32_configgpio(GPIO_HEATER_OFF);
	}

	stm32_configgpio(GPIO_PC14);
	stm32_configgpio(GPIO_PC15);


	stm32_configgpio(GPIO_BTN_SAFETY);

	/* spektrum power enable is active high - enable it by default */
	stm32_configgpio(GPIO_SPEKTRUM_PWR_EN);

	stm32_configgpio(GPIO_SERVO_FAULT_DETECT);

	/* RSSI inputs */
	stm32_configgpio(GPIO_TIM_RSSI); /* xxx alternate function */
	stm32_configgpio(GPIO_ADC_RSSI);

	/* servo rail voltage */
	stm32_configgpio(GPIO_ADC_VSERVO);

	stm32_configgpio(GPIO_SBUS_INPUT); /* xxx alternate function */
	stm32_configgpio(GPIO_SBUS_OUTPUT);

	/* sbus output enable is active low - disable it by default */
	stm32_gpiowrite(GPIO_SBUS_OENABLE, true);
	stm32_configgpio(GPIO_SBUS_OENABLE);

	stm32_configgpio(GPIO_PPM); /* xxx alternate function */

	stm32_gpiowrite(GPIO_PWM1, true);
	stm32_configgpio(GPIO_PWM1);

	stm32_gpiowrite(GPIO_PWM2, true);
	stm32_configgpio(GPIO_PWM2);

	stm32_gpiowrite(GPIO_PWM3, true);
	stm32_configgpio(GPIO_PWM3);

	stm32_gpiowrite(GPIO_PWM4, true);
	stm32_configgpio(GPIO_PWM4);

	stm32_gpiowrite(GPIO_PWM5, true);
	stm32_configgpio(GPIO_PWM5);

	stm32_gpiowrite(GPIO_PWM6, true);
	stm32_configgpio(GPIO_PWM6);

	stm32_gpiowrite(GPIO_PWM7, true);
	stm32_configgpio(GPIO_PWM7);

	stm32_gpiowrite(GPIO_PWM8, true);
	stm32_configgpio(GPIO_PWM8);
}
