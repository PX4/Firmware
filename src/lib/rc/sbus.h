/****************************************************************************
 *
 *	Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file sbus.h
 *
 * RC protocol definition for S.BUS
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

__BEGIN_DECLS

#define SBUS_FRAME_SIZE			25
#define SBUS_BUFFER_SIZE		(SBUS_FRAME_SIZE + SBUS_FRAME_SIZE / 2)

typedef   uint8_t sbus_frame_t[SBUS_FRAME_SIZE + (SBUS_FRAME_SIZE / 2)];


__EXPORT int	sbus_init(const char *device, bool singlewire);

/**
 * Parse serial bytes on the S.BUS bus
 *
 * The S.bus protocol doesn't provide reliable framing,
 * so we detect frame boundaries by the inter-frame delay.
 *
 * The minimum frame spacing is 7ms; with 25 bytes at 100000bps
 * frame transmission time is 3ms.
 *
 * If an interval of more than 4ms (7 ms frame spacing plus margin)
 * passes between calls, the first byte we read will be the first
 * byte of a frame.
 *
 * In the case where byte(s) are dropped from a frame, this also
 * provides a degree of protection. Of course, it would be better
 * if we didn't drop bytes...
 */
__EXPORT int	sbus_config(int sbus_fd, bool singlewire);
__EXPORT bool	sbus_input(int sbus_fd, uint16_t *values, uint16_t *num_values, bool *sbus_failsafe,
			   bool *sbus_frame_drop,
			   uint16_t max_channels);
__EXPORT bool	sbus_parse(uint64_t now, uint8_t *frame, unsigned len, uint16_t *values,
			   uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, unsigned *frame_drops, uint16_t max_channels);
__EXPORT void	sbus1_output(int sbus_fd, uint16_t *values, uint16_t num_values);
__EXPORT void	sbus2_output(int sbus_fd, uint16_t *values, uint16_t num_values);
__EXPORT void	sbus1_set_output_rate_hz(uint16_t rate_hz);

/**
 * The number of incomplete frames we encountered
 */
__EXPORT unsigned	sbus_dropped_frames(void);

__END_DECLS
