/****************************************************************************
 * boards/arm/s32k1xx/ucans32k146/src/s32k1xx_userleds.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "s32k1xx_pin.h"
#include "board_config.h"

#include <arch/board/board.h>

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

void board_userled_initialize(void)
{
	/* Configure LED GPIOs for output */

	s32k1xx_pinconfig(GPIO_LED_R);
	s32k1xx_pinconfig(GPIO_LED_G);
	s32k1xx_pinconfig(GPIO_LED_B);
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
	uint32_t ledcfg;

	if (led == BOARD_LED_R) {
		ledcfg = GPIO_LED_R;

	} else if (led == BOARD_LED_G) {
		ledcfg = GPIO_LED_G;

	} else if (led == BOARD_LED_B) {
		ledcfg = GPIO_LED_B;

	} else {
		return;
	}

	s32k1xx_gpiowrite(ledcfg, ledon); /* High illuminates */
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint8_t ledset)
{
	/* Low illuminates */

	s32k1xx_gpiowrite(GPIO_LED_R, (ledset & BOARD_LED_R_BIT) != 0);
	s32k1xx_gpiowrite(GPIO_LED_G, (ledset & BOARD_LED_G_BIT) != 0);
	s32k1xx_gpiowrite(GPIO_LED_B, (ledset & BOARD_LED_B_BIT) != 0);
}

#endif /* !CONFIG_ARCH_LEDS */
