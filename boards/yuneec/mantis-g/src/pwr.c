/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file pwr.c
 *
 * Board-specific power button functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_platform_common/px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>

#include <uORB/uORB.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/tune_control.h>

#include <arm_arch.h>
#include "board_config.h"
#include <stm32_pwr.h>

extern void led_on(int led);
extern void led_off(int led);

static struct timespec time_down;


static int default_power_button_state_notification(board_power_button_state_notification_e request)
{
//	syslog(0,"%d\n", request);
	return PWR_BUTTON_RESPONSE_SHUT_DOWN_NOW;
}


static power_button_state_notification_t power_state_notification = default_power_button_state_notification;

int board_register_power_state_notification_cb(power_button_state_notification_t cb)
{
	power_state_notification = cb;

	if (board_pwr_button_down() && (time_down.tv_nsec != 0 || time_down.tv_sec != 0)) {
		// make sure we don't miss the first event
		power_state_notification(PWR_BUTTON_DOWN);
	}

	return OK;
}

static int board_shutdown(void)
{
	led_on(BOARD_LED_BLUE);

	// disable the interrups
	px4_enter_critical_section();

	stm32_configgpio(POWER_OFF_GPIO);

	while (1);

	return 0;
}

static int board_button_irq(int irq, FAR void *context, FAR void *args)
{
	if (board_pwr_button_down()) {

		led_on(BOARD_LED_RED);
		clock_gettime(CLOCK_REALTIME, &time_down);
		power_state_notification(PWR_BUTTON_DOWN);

	} else {
		power_state_notification(PWR_BUTTON_UP);

		led_off(BOARD_LED_RED);

		struct timespec now;

		clock_gettime(CLOCK_REALTIME, &now);

		uint64_t tdown_ms = time_down.tv_sec * 1000 + time_down.tv_nsec / 1000000;

		uint64_t tnow_ms  = now.tv_sec * 1000 + now.tv_nsec / 1000000;

		if (tdown_ms != 0 && (tnow_ms - tdown_ms) >= MS_PWR_BUTTON_DOWN) {

			led_on(BOARD_LED_BLUE);

			if (power_state_notification(PWR_BUTTON_REQUEST_SHUT_DOWN) == PWR_BUTTON_RESPONSE_SHUT_DOWN_NOW) {
				up_mdelay(200);
				board_shutdown();
			}

		} else {
			power_state_notification(PWR_BUTTON_IDEL);
		}

	}

	return OK;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: board_pwr_init()
 *
 * Description:
 *   Called to configure power control
 *
 * Input Parameters:
 *   stage- 0 for boot, 1 for board init
 *
 ************************************************************************************/

void board_pwr_init(int stage)
{
	if (stage == 0) {
		stm32_configgpio(POWER_ON_GPIO);
		stm32_configgpio(KEY_AD_GPIO);
		stm32_configgpio(POWER_CHECK_GPIO);
	}

	if (stage == 1) {
		stm32_gpiosetevent(KEY_AD_GPIO, true, true, true, board_button_irq, NULL);
	}
}

/****************************************************************************
 * Name: board_pwr_button_down
 *
 * Description:
 *   Called to Read the logical state of the active low power button.
 *
 ****************************************************************************/

bool board_pwr_button_down(void)
{
	return 0 == stm32_gpioread(KEY_AD_GPIO);
}
