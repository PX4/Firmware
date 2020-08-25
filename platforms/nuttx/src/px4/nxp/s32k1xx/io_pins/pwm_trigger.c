/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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

/*
 * @file drv_pwm_trigger.c
 *
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_trigger.h>

#include <px4_arch/io_timer.h>

int up_pwm_trigger_set(unsigned channel, uint16_t value)
{
	return io_timer_set_ccr(channel, value);
}

int up_pwm_trigger_init(uint32_t channel_mask)
{
	/* Init channels */
	for (unsigned channel = 0; channel_mask != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {

			// First free any that were not trigger mode before
			if (-EBUSY == io_timer_is_channel_free(channel)) {
				io_timer_free_channel(channel);
			}

			io_timer_channel_init(channel, IOTimerChanMode_Trigger, NULL, NULL);
			channel_mask &= ~(1 << channel);
		}
	}

	/* Enable the timers */
	up_pwm_trigger_arm(true);

	return OK;
}

void up_pwm_trigger_deinit()
{
	/* Disable the timers */
	up_pwm_trigger_arm(false);

	/* Deinit channels */
	uint32_t current = io_timer_get_mode_channels(IOTimerChanMode_Trigger);

	for (unsigned channel = 0; current != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (current & (1 << channel)) {

			io_timer_channel_init(channel, IOTimerChanMode_NotUsed, NULL, NULL);
			current &= ~(1 << channel);
		}
	}
}

void
up_pwm_trigger_arm(bool armed)
{
	io_timer_set_enable(armed, IOTimerChanMode_Trigger, IO_TIMER_ALL_MODES_CHANNELS);
}
