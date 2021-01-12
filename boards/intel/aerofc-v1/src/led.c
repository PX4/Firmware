/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *         Author: David Sidrane <david_s5@nscdg.com>
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
 * @file led.c
 *
 * AEROFC_V1 LED backend.
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>

#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

/*
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

__EXPORT void led_init(void)
{
	/* Configure LED0-3 GPIOs for output */

	stm32_configgpio(GPIO_LED0);
	stm32_configgpio(GPIO_LED1);
	stm32_configgpio(GPIO_LED2);
	stm32_configgpio(GPIO_LED3);
}

static uint32_t _led_param_get(int led)
{
	switch (led) {
	case 0:
		return GPIO_LED0; // LED_BLUE

	case 1:
		return GPIO_LED1; // LED_RED

	case 2:
		return GPIO_LED2; // LED SAFETY

	case 3:
		return GPIO_LED3; // LED GREEN

	}

	return 0;
}

__EXPORT void led_on(int led)
{
	const uint32_t param = _led_param_get(led);

	if (param) {
		stm32_gpiowrite(param, false);
	}
}

__EXPORT void led_off(int led)
{
	const uint32_t param = _led_param_get(led);

	if (param) {
		stm32_gpiowrite(param, true);
	}
}

__EXPORT void led_toggle(int led)
{
	const uint32_t param = _led_param_get(led);

	if (param) {
		stm32_gpiowrite(param, !stm32_gpioread(param));
	}
}
