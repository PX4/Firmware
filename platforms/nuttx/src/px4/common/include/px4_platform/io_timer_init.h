/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
#pragma once


#include <stdint.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_arch/io_timer.h>
#include <board_config.h>


/**
 * Initialize the IO channel mapping from timer and channel configurations.
 * @param io_timers_conf configured timers
 * @param timer_io_channels_conf configured channels
 */
static inline constexpr io_timers_channel_mapping_t initIOTimerChannelMapping(const io_timers_t
		io_timers_conf[MAX_IO_TIMERS],
		const timer_io_channels_t timer_io_channels_conf[MAX_TIMER_IO_CHANNELS])
{
	io_timers_channel_mapping_t ret{};

	// requirement: channels of the same timer must be grouped together, but the ordering does not matter

	for (unsigned i = 0; i < MAX_IO_TIMERS; ++i) {
		if (io_timers_conf[i].base == 0) {
			break;
		}

		uint32_t first_channel = UINT32_MAX;
		uint32_t channel_count = 0;

		for (uint32_t channel = 0; channel < MAX_TIMER_IO_CHANNELS; ++channel) {
			if (timer_io_channels_conf[channel].gpio_in == 0 && timer_io_channels_conf[channel].gpio_out == 0) {
				break;
			}

			if (timer_io_channels_conf[channel].timer_index == i) {
				if (first_channel == UINT32_MAX) {
					first_channel = channel;

				} else {
					constexpr_assert(timer_io_channels_conf[channel - 1].timer_index == i, "Timers are not grouped together");
				}

				++channel_count;
			}
		}

		if (first_channel == UINT32_MAX) { //unused timer, channel_count is 0
			first_channel = 0;
		}

		ret.element[i].first_channel_index = first_channel;
		ret.element[i].channel_count = channel_count;
	}

	// validate that the number of configured channels matches DIRECT_PWM_OUTPUT_CHANNELS
	uint32_t num_channels = 0;

	while (num_channels < MAX_TIMER_IO_CHANNELS &&
	       (timer_io_channels_conf[num_channels].gpio_in != 0 || timer_io_channels_conf[num_channels].gpio_out != 0)) {
		++num_channels;
	}

	constexpr_assert(DIRECT_PWM_OUTPUT_CHANNELS == num_channels, "DIRECT_PWM_OUTPUT_CHANNELS misconfigured");
	constexpr_assert(DIRECT_PWM_OUTPUT_CHANNELS <= MAX_TIMER_IO_CHANNELS,
			 "DIRECT_PWM_OUTPUT_CHANNELS > MAX_TIMER_IO_CHANNELS");

	return ret;
}
