/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file safety.c
 * Safety button logic.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>

#include <drivers/drv_hrt.h>

#include "px4io.h"

static struct hrt_call arming_call;
static struct hrt_call failsafe_call;

/*
 * Count the number of times in a row that we see the arming button
 * held down.
 */
static unsigned counter = 0;

/*
 * Define the various LED flash sequences for each system state.
 */
#define LED_PATTERN_FMU_OK_TO_ARM		0x0003			/**< slow blinking			*/
#define LED_PATTERN_FMU_REFUSE_TO_ARM		0x5555		/**< fast blinking			*/
#define LED_PATTERN_IO_ARMED			0x5050			/**< long off, then double blink 	*/
#define LED_PATTERN_FMU_ARMED			0x5500			/**< long off, then quad blink 		*/
#define LED_PATTERN_IO_FMU_ARMED		0xffff			/**< constantly on			*/

static unsigned blink_counter = 0;

/*
 * IMPORTANT: The arming state machine critically
 * 	      depends on using the same threshold
 *            for arming and disarming. Since disarming
 *            is quite deadly for the system, a similar
 *            length can be justified.
 */
#define ARM_COUNTER_THRESHOLD	10

static void safety_check_button(void *arg);
static void failsafe_blink(void *arg);

void
safety_init(void)
{
	/* arrange for the button handler to be called at 10Hz */
	hrt_call_every(&arming_call, 1000, 100000, safety_check_button, NULL);
}

void
failsafe_led_init(void)
{
	/* arrange for the failsafe blinker to be called at 8Hz */
	hrt_call_every(&failsafe_call, 1000, 125000, failsafe_blink, NULL);
}

static void
safety_check_button(void *arg)
{
	const bool safety_button_pressed = BUTTON_SAFETY;

	/* Keep safety button pressed for one second to turn off safety
	 *
	 * Note that safety cannot be turned on again by button because a button
	 * hardware problem could accidentally disable it in flight.
	 */
	if (safety_button_pressed && !(r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) &&
	    (r_setup_arming & PX4IO_P_SETUP_ARMING_IO_ARM_OK)) {

		if (counter <= ARM_COUNTER_THRESHOLD) {
			counter++;

		}

		if (counter == ARM_COUNTER_THRESHOLD) {
			// switch safety off -> ready to arm state
			atomic_modify_or(&r_status_flags, PX4IO_P_STATUS_FLAGS_SAFETY_OFF);
		}

	} else {
		counter = 0;
	}

	/* Select the appropriate LED flash pattern depending on the current IO/FMU arm state */
	uint16_t pattern = LED_PATTERN_FMU_REFUSE_TO_ARM;

	if (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
		if (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED) {
			pattern = LED_PATTERN_IO_FMU_ARMED;

		} else {
			pattern = LED_PATTERN_IO_ARMED;
		}

	} else if (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED) {
		pattern = LED_PATTERN_FMU_ARMED;

	} else if (r_setup_arming & PX4IO_P_SETUP_ARMING_IO_ARM_OK) {
		pattern = LED_PATTERN_FMU_OK_TO_ARM;

	}

	/* Turn the LED on if we have a 1 at the current bit position */
	LED_SAFETY(pattern & (1 << blink_counter++));

	if (blink_counter > 15) {
		blink_counter = 0;
	}
}

static void
failsafe_blink(void *arg)
{
	/* indicate that a serious initialisation error occured */
	if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_INIT_OK)) {
		LED_AMBER(true);
		return;
	}

	static bool failsafe = false;

	/* blink the failsafe LED if we don't have FMU input */
	if (!(r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)) {
		failsafe = !failsafe;

	} else {
		failsafe = false;
	}

	LED_AMBER(failsafe);
}
