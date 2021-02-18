/****************************************************************************
 *
 *	Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file dsm.h
 *
 * RC protocol definition for Spektrum RC
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <stdint.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

__BEGIN_DECLS

#define DSM_FRAME_SIZE		16		/**< DSM frame size in bytes */
#define DSM_FRAME_CHANNELS	7		/**< Max supported DSM channels per frame */
#define DSM_MAX_CHANNEL_COUNT   18  /**< Max channel count of any DSM RC */
#define DSM_BUFFER_SIZE		(DSM_FRAME_SIZE + DSM_FRAME_SIZE / 2)


#pragma pack(push, 1)
typedef   uint8_t dsm_frame_t[DSM_BUFFER_SIZE]; /**< DSM dsm frame receive buffer */
typedef   uint8_t dsm_buf_t[DSM_FRAME_SIZE * 2]; // Define working buffer

typedef  struct dsm_decode_t {
	dsm_frame_t frame;
	dsm_buf_t buf;
} dsm_decode_t;

#pragma pack(pop)

__EXPORT int	dsm_init(const char *device);
__EXPORT void	dsm_deinit(void);
__EXPORT void	dsm_proto_init(void);
__EXPORT int	dsm_config(int dsm_fd);
__EXPORT bool	dsm_input(int dsm_fd, uint16_t *values, uint16_t *num_values, bool *dsm_11_bit, uint8_t *n_bytes,
			  uint8_t **bytes, uint8_t *rssi, unsigned *frame_drops, unsigned max_values);

__EXPORT bool	dsm_parse(const uint64_t now, const uint8_t *frame, const unsigned len, uint16_t *values,
			  uint16_t *num_values, bool *dsm_11_bit, unsigned *frame_drops, uint8_t *rssi_percent, uint16_t max_channels);

#ifdef SPEKTRUM_POWER
__EXPORT void	dsm_bind(uint16_t cmd, int pulses);
#endif

enum DSM_CMD {							/* DSM bind states */
	DSM_CMD_BIND_POWER_DOWN = 0,
	DSM_CMD_BIND_POWER_UP,
	DSM_CMD_BIND_SET_RX_OUT,
	DSM_CMD_BIND_SEND_PULSES,
	DSM_CMD_BIND_REINIT_UART
};

#define DSM2_BIND_PULSES 3	/* DSM_BIND_START parameter, pulses required to start pairing DSM2 22ms */
#define DSMX_BIND_PULSES 7	/* DSM_BIND_START parameter, pulses required to start pairing DSMx 22ms */
#define DSMX8_BIND_PULSES 9     /* DSM_BIND_START parameter, pulses required to start pairing DSMx 11ms */

__END_DECLS
