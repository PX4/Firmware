/****************************************************************************
 *
 *   Copyright (C) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file drv_input_capture.c
 *
 * Servo driver supporting input capture connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have input pins.
 *
 * Require an interrupt.
 *
 * The use of thie interface is mutually exclusive with the pwm
 * because the same timers are used and there is a resource contention
 * with the ARR as it sets the pwm rate and in this driver needs to match
 * that of the hrt to back calculate the actual point in time the edge
 * was detected.
 *
 * This  is accomplished by taking the difference between the current
 * count rCNT snapped at the time interrupt and the rCCRx captured on the
 * edge transition. This delta is applied to hrt time and the resulting
 * value is the absolute system time the edge occured.
 *
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
#include <drivers/drv_input_capture.h>
#include <px4_arch/io_timer.h>

#include <kinetis.h>
#include "hardware/kinetis_sim.h"
#include "hardware/kinetis_ftm.h"


#define _REG(_addr)	(*(volatile uint32_t *)(_addr))
#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(io_timers[_tmr].base, _reg)


/* Timer register accessors */

#define rFILTER(_tmr)     REG(_tmr,KINETIS_FTM_FILTER_OFFSET)

static input_capture_stats_t channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	capture_callback_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];



static void input_capture_chan_handler(void *context, const io_timers_t *timer, uint32_t chan_index,
				       const timer_io_channels_t *chan,
				       hrt_abstime isrs_time, uint16_t isrs_rcnt,
				       uint16_t capture)
{
	channel_stats[chan_index].last_edge = px4_arch_gpioread(chan->gpio_in);

	if ((isrs_rcnt - capture) > channel_stats[chan_index].latnecy) {
		channel_stats[chan_index].latnecy = (isrs_rcnt - capture);
	}

	channel_stats[chan_index].chan_in_edges_out++;
	channel_stats[chan_index].last_time = isrs_time - (isrs_rcnt - capture);
	uint32_t overflow = _REG32(timer, KINETIS_FTM_CSC_OFFSET(chan->timer_channel - 1)) & FTM_CSC_CHF;

	if (overflow) {

		/* Error we has a second edge before we cleared CCxR */

		channel_stats[chan_index].overflows++;
	}

	if (channel_handlers[chan_index].callback)  {
		channel_handlers[chan_index].callback(channel_handlers[chan_index].context, chan_index,
						      channel_stats[chan_index].last_time,
						      channel_stats[chan_index].last_edge, overflow);
	}
}

static void input_capture_bind(unsigned channel, capture_callback_t callback, void *context)
{
	irqstate_t flags = px4_enter_critical_section();
	channel_handlers[channel].callback = callback;
	channel_handlers[channel].context = context;
	px4_leave_critical_section(flags);
}

static void input_capture_unbind(unsigned channel)
{
	input_capture_bind(channel, NULL, NULL);
}

int up_input_capture_set(unsigned channel, input_capture_edge edge, capture_filter_t filter,
			 capture_callback_t callback, void *context)
{
	if (filter > FTM_FILTER_CH0FVAL_MASK) {
		return -EINVAL;
	}

	if (edge > Both) {
		return -EINVAL;
	}

	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		/* This register selects the filter value for the inputs of channels.
		   Channels 4, 5, 6 and 7 do not have an input filter.
		*/
		if (filter && timer_io_channels[channel].timer_channel - 1 > 3) {
			return -EINVAL;
		}


		if (edge == Disabled) {

			io_timer_set_enable(false, IOTimerChanMode_Capture, 1 << channel);
			input_capture_unbind(channel);

		} else {

			if (-EBUSY == io_timer_is_channel_free(channel)) {
				io_timer_free_channel(channel);
			}

			input_capture_bind(channel, callback, context);

			rv = io_timer_channel_init(channel, IOTimerChanMode_Capture, input_capture_chan_handler, context);

			if (rv != 0) {
				return rv;
			}

			rv = up_input_capture_set_filter(channel, filter);

			if (rv == 0) {
				rv = up_input_capture_set_trigger(channel, edge);

				if (rv == 0) {
					rv = io_timer_set_enable(true, IOTimerChanMode_Capture, 1 << channel);
				}
			}
		}
	}

	return rv;
}

int up_input_capture_get_filter(unsigned channel, capture_filter_t *filter)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -EINVAL;

		if (timer_io_channels[channel].timer_channel - 1 <= 3) {
			rv = -ENXIO;

			/* Any pins in capture mode */

			if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

				uint32_t timer = timer_io_channels[channel].timer_index;
				uint16_t rvalue;
				rv = OK;

				switch (timer_io_channels[channel].timer_channel) {

				case 1:
					rvalue = rFILTER(timer) & FTM_FILTER_CH0FVAL_MASK;
					*filter = (rvalue >> FTM_FILTER_CH0FVAL_SHIFT);
					break;

				case 2:
					rvalue = rFILTER(timer) & FTM_FILTER_CH1FVAL_MASK;
					*filter = (rvalue >> FTM_FILTER_CH1FVAL_SHIFT);
					break;

				case 3:
					rvalue = rFILTER(timer) & FTM_FILTER_CH2FVAL_MASK;
					*filter = (rvalue >> FTM_FILTER_CH2FVAL_SHIFT);
					break;

				case 4:
					rvalue = rFILTER(timer) & FTM_FILTER_CH3FVAL_MASK;
					*filter = (rvalue >> FTM_FILTER_CH3FVAL_SHIFT);
					break;

				default:
					rv = -EIO;
				}
			}
		}
	}

	return rv;
}
int up_input_capture_set_filter(unsigned channel,  capture_filter_t filter)
{
	if (filter > FTM_FILTER_CH0FVAL_MASK) {
		return -EINVAL;
	}

	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			rv = OK;
			uint32_t timer = timer_io_channels[channel].timer_index;
			uint16_t rvalue;

			irqstate_t flags = px4_enter_critical_section();

			switch (timer_io_channels[channel].timer_channel) {

			case 1:
				rvalue = rFILTER(timer) & ~FTM_FILTER_CH0FVAL_MASK;
				rvalue |= (filter << FTM_FILTER_CH0FVAL_SHIFT);
				rFILTER(timer) = rvalue;
				break;

			case 2:
				rvalue = rFILTER(timer) & ~FTM_FILTER_CH1FVAL_MASK;
				rvalue |= (filter << FTM_FILTER_CH1FVAL_SHIFT);
				rFILTER(timer) = rvalue;
				break;

			case 3:
				rvalue = rFILTER(timer) & ~FTM_FILTER_CH2FVAL_MASK;
				rvalue |= (filter << FTM_FILTER_CH2FVAL_SHIFT);
				rFILTER(timer) = rvalue;
				break;

			case 4:
				rvalue = rFILTER(timer) & ~FTM_FILTER_CH2FVAL_MASK;
				rvalue |= (filter << FTM_FILTER_CH2FVAL_SHIFT);
				rFILTER(timer) = rvalue;
				break;

			default:
				rv = -EIO;
			}

			px4_leave_critical_section(flags);
		}
	}

	return rv;
}

int up_input_capture_get_trigger(unsigned channel,  input_capture_edge *edge)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			rv = OK;

			uint32_t timer = timer_io_channels[channel].timer_index;
			uint16_t rvalue = _REG32(timer, KINETIS_FTM_CSC_OFFSET(timer_io_channels[channel].timer_channel - 1));
			rvalue &= (FTM_CSC_MSB | FTM_CSC_MSA);

			switch (rvalue) {

			case (FTM_CSC_MSA):
				*edge = Rising;
				break;

			case (FTM_CSC_MSB):
				*edge = Falling;
				break;

			case (FTM_CSC_MSB|FTM_CSC_MSA):
				*edge = Both;
				break;

			default:
				rv = -EIO;
			}
		}
	}

	return rv;
}
int up_input_capture_set_trigger(unsigned channel,  input_capture_edge edge)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			uint16_t edge_bits = 0;

			switch (edge) {
			case Disabled:
				break;

			case Rising:
				edge_bits = FTM_CSC_MSA;
				break;

			case Falling:
				edge_bits = FTM_CSC_MSB;
				break;

			case Both:
				edge_bits = (FTM_CSC_MSB | FTM_CSC_MSA);
				break;

			default:
				return -EINVAL;;
			}

			uint32_t timer = timer_io_channels[channel].timer_index;
			irqstate_t flags = px4_enter_critical_section();
			uint32_t rvalue = _REG32(timer, KINETIS_FTM_CSC_OFFSET(timer_io_channels[channel].timer_channel - 1));
			rvalue &= (FTM_CSC_MSB | FTM_CSC_MSA);
			rvalue |=  edge_bits;
			_REG32(timer, KINETIS_FTM_CSC_OFFSET(timer_io_channels[channel].timer_channel - 1)) = rvalue;
			px4_leave_critical_section(flags);
			rv = OK;
		}
	}

	return rv;
}

int up_input_capture_get_callback(unsigned channel, capture_callback_t *callback, void **context)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			irqstate_t flags = px4_enter_critical_section();
			*callback = channel_handlers[channel].callback;
			*context = channel_handlers[channel].context;
			px4_leave_critical_section(flags);
			rv = OK;
		}
	}

	return rv;

}

int up_input_capture_set_callback(unsigned channel, capture_callback_t callback, void *context)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {
			input_capture_bind(channel, callback, context);
			rv = 0;
		}
	}

	return rv;
}

int up_input_capture_get_stats(unsigned channel, input_capture_stats_t *stats, bool clear)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {
		irqstate_t flags = px4_enter_critical_section();
		*stats =  channel_stats[channel];

		if (clear) {
			memset(&channel_stats[channel], 0, sizeof(*stats));
		}

		px4_leave_critical_section(flags);
	}

	return rv;
}
