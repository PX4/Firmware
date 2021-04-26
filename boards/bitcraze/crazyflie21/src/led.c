/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file crazyflie_led.c
 *
 * Crazyflie LED backend.
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

static uint32_t g_ledmap[] = {
	GPIO_LED_BLUE_L,  // Indexed by LED_BLUE
	GPIO_LED_RED_R,   // Indexed by LED_RED, LED_AMBER
	0,				  // Indexed by LED_SAFETY
	GPIO_LED_GREEN_R, // Indexed by LED_GREEN
	GPIO_LED_RED_L,   // Indexed by LED_TX
	GPIO_LED_GREEN_L  // Indexed by LED_RX
};


__EXPORT void led_init()
{
	/* Configure LED1 GPIO for output */
	for (size_t l = 0; l < (sizeof(g_ledmap) / sizeof(g_ledmap[0])); l++) {
		if (g_ledmap[l]) {
			px4_arch_configgpio(g_ledmap[l]);
		}
	}
}

__EXPORT void led_on(int led)
{
	/* Pull down to switch on */
	px4_arch_gpiowrite(g_ledmap[led], g_ledmap[led] & GPIO_OUTPUT_SET ? true : false);
}

__EXPORT void led_off(int led)
{
	/* Pull up to switch off */
	px4_arch_gpiowrite(g_ledmap[led], g_ledmap[led] & GPIO_OUTPUT_SET ? false : true);
}

__EXPORT void led_toggle(int led)
{
	if (px4_arch_gpioread(g_ledmap[led])) {
		px4_arch_gpiowrite(g_ledmap[led], false);

	} else {
		px4_arch_gpiowrite(g_ledmap[led], true);
	}
}
