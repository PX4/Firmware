/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file ghst.cpp
 *
 * RC protocol definition for IRC Ghost (Immersion RC Ghost).
 *
 * @author Igor Misic <igy1000mb@gmail.com>
 * @author Juraj Ciberlin <jciberlin1@gmail.com>
 */

#if 0 // enable non-verbose debugging
#define GHST_DEBUG PX4_WARN
#else
#define GHST_DEBUG(...)
#endif

#if 0 // verbose debugging. Careful when enabling: it leads to too much output, causing dropped bytes
#define GHST_VERBOSE PX4_WARN
#else
#define GHST_VERBOSE(...)
#endif

#include <drivers/drv_hrt.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>

// TODO: include RSSI dBm to percentage conversion for ghost receiver
#include "spektrum_rssi.h"

#include "ghst.hpp"
#include "common_rc.h"

#define MIN(a,b) (((a)<(b))?(a):(b))

#define GHST_FRAME_PAYLOAD_SIZE_TELEMETRY	(10U)
#define GHST_FRAME_CRC_SIZE			(1U)
#define GHST_FRAME_TYPE_SIZE			(1U)
#define GHST_TYPE_DATA_CRC_SIZE			(12U)
#define GHST_MAX_NUM_CHANNELS			(16)

enum class ghst_parser_state_t : uint8_t {
	unsynced = 0U,
	synced
};

// only RSSI frame contains value of RSSI, if it is not received, send last received RSSI
static int8_t ghst_rssi = -1;

static ghst_frame_t &ghst_frame = rc_decode_buf.ghst_frame;
static uint32_t current_frame_position = 0U;
static ghst_parser_state_t parser_state = ghst_parser_state_t::unsynced;

static uint16_t prev_rc_vals[GHST_MAX_NUM_CHANNELS];

/**
 * parse the current ghst_frame buffer
 */
static bool ghst_parse_buffer(uint16_t *values, int8_t *rssi, uint16_t *num_values, uint16_t max_channels);

int ghst_config(int uart_fd)
{
	struct termios t;
	int ret_val;

	// no parity, one stop bit
	tcgetattr(uart_fd, &t);
	cfsetspeed(&t, GHST_BAUDRATE);
	t.c_cflag &= ~(CSTOPB | PARENB);
	memset(prev_rc_vals, static_cast<int>(UINT16_MAX), sizeof(uint16_t) * GHST_MAX_NUM_CHANNELS);
	ret_val = tcsetattr(uart_fd, TCSANOW, &t);
	return ret_val;
}

/**
 * Convert from RC to PWM value
 * @param chan_value channel value in [172, 1811]
 * @return PWM channel value in [1000, 2000]
 */
static uint16_t convert_channel_value(unsigned chan_value);


bool ghst_parse(const uint64_t now, const uint8_t *frame, unsigned len, uint16_t *values,
		int8_t *rssi, uint16_t *num_values, uint16_t max_channels)
{
	bool success = false;
	uint8_t *ghst_frame_ptr = (uint8_t *)&ghst_frame;

	memcpy(values, prev_rc_vals, sizeof(uint16_t) * GHST_MAX_NUM_CHANNELS);

	while (len > 0U) {

		// fill in the ghst_buffer, as much as we can
		const uint32_t current_len = MIN(len, sizeof(ghst_frame_t) - current_frame_position);
		memcpy(ghst_frame_ptr + current_frame_position, frame, current_len);
		current_frame_position += current_len;

		// protection to guarantee parsing progress
		if (current_len == 0U) {
			GHST_DEBUG("========== parser bug: no progress (%u) ===========", len);

			for (uint32_t i = 0U; i < current_frame_position; ++i) {
				GHST_DEBUG("ghst_frame_ptr[%u]: 0x%x", i, ghst_frame_ptr[i]);
			}

			// reset the parser
			current_frame_position = 0U;
			parser_state = ghst_parser_state_t::unsynced;
			success = false;

		} else {
			len -= current_len;
			frame += current_len;

			if (ghst_parse_buffer(values, rssi, num_values, max_channels)) {
				success = true;
			}
		}
	}

	return success;
}

uint8_t ghst_frame_CRC(const ghst_frame_t &frame)
{
	uint8_t crc = crc8_dvb_s2(0U, frame.type);

	for (uint32_t i = 0U; i < frame.header.length - GHST_FRAME_CRC_SIZE - GHST_FRAME_TYPE_SIZE; ++i) {
		crc = crc8_dvb_s2(crc, frame.payload[i]);
	}

	return crc;
}

static uint16_t convert_channel_value(unsigned int chan_value)
{
	/*
	 *       RC     PWM
	 * min  172 ->  988us
	 * mid  992 -> 1500us
	 * max 1811 -> 2012us
	 */
	static constexpr float scale = (2012.0F - 988.0F) / (1811.0F - 172.0F);
	static constexpr float offset = 988.0F - (172.0F * scale);
	float scaled_chan_value = scale * static_cast<float>(chan_value);
	float scaled_chan_value_with_offset = scaled_chan_value + offset;
	uint16_t converted_chan_value = static_cast<uint16_t>(scaled_chan_value_with_offset);

	return converted_chan_value;
}

static bool ghst_parse_buffer(uint16_t *values, int8_t *rssi, uint16_t *num_values, uint16_t max_channels)
{
	uint8_t *ghst_frame_ptr = (uint8_t *)&ghst_frame;

	if (parser_state == ghst_parser_state_t::unsynced) {
		// there is no sync yet, try to find an RC packet by searching for a matching frame length and type
		for (uint32_t i = 1U; i < current_frame_position - 1U; ++i) {
			if ((ghst_frame_ptr[i + 1U] >= static_cast<uint8_t>(ghstFrameType::frameTypeFirst)) &&
			    (ghst_frame_ptr[i + 1U] <= static_cast<uint8_t>(ghstFrameType::frameTypeLast))) {
				if (ghst_frame_ptr[i] == GHST_TYPE_DATA_CRC_SIZE) {
					parser_state = ghst_parser_state_t::synced;
					uint32_t frame_offset = i - 1U;
					GHST_VERBOSE("RC channels found at offset %u", frame_offset);

					// move the rest of the buffer to the beginning
					if (frame_offset != 0U) {
						memmove(ghst_frame_ptr, ghst_frame_ptr + frame_offset, current_frame_position - frame_offset);
						current_frame_position -= frame_offset;
					}

					break;
				}
			}
		}
	}

	if (parser_state != ghst_parser_state_t::synced) {
		if (current_frame_position >= sizeof(ghst_frame_t)) {
			// discard most of the data, but keep the last 3 bytes (otherwise we could miss the frame start)
			current_frame_position = 3U;

			memcpy(ghst_frame_ptr, ghst_frame_ptr + sizeof(ghst_frame_t) - current_frame_position, current_frame_position);

			GHST_VERBOSE("Discarding buffer");
		}

		return false;
	}


	if (current_frame_position < 3U) {
		// wait until we have the address, length and type
		return false;
	}

	// now we have at least the header and the type

	const unsigned current_frame_length = ghst_frame.header.length + sizeof(ghst_frame_header_t);

	if ((current_frame_length > sizeof(ghst_frame_t)) || (current_frame_length < 4U)) {
		// frame too long or bogus (frame length should be longer than 4, at least 1 address, 1 length, 1 type, 1 data, 1 crc)
		// discard everything and go into unsynced state
		current_frame_position = 0U;
		parser_state = ghst_parser_state_t::unsynced;
		GHST_DEBUG("Frame too long/bogus (%u, type=%u) -> unsync", current_frame_length, ghst_frame.type);
		return false;
	}

	if (current_frame_position < current_frame_length) {
		// we do not have the full frame yet -> wait for more data
		GHST_VERBOSE("waiting for more data (%u < %u)", current_frame_position, current_frame_length);
		return false;
	}

	bool ret = false;

	// now we have the full frame

	if ((ghst_frame.type >= static_cast<uint8_t>(ghstFrameType::frameTypeFirst)) &&
	    (ghst_frame.type <= static_cast<uint8_t>(ghstFrameType::frameTypeLast)) &&
	    (ghst_frame.header.length == GHST_TYPE_DATA_CRC_SIZE)) {
		const uint8_t crc = ghst_frame.payload[ghst_frame.header.length - 2U];

		if (crc == ghst_frame_CRC(ghst_frame)) {
			const ghstPayloadData_t *const rcChannels = (ghstPayloadData_t *)&ghst_frame.payload;
			*num_values = MIN(max_channels, GHST_MAX_NUM_CHANNELS);

			// all frames contain data from chan1to4
			if (max_channels > 0U) { values[0] = convert_channel_value(rcChannels->chan1to4.chan1 >> 1U); }

			if (max_channels > 1U) { values[1] = convert_channel_value(rcChannels->chan1to4.chan2 >> 1U); }

			if (max_channels > 2U) { values[2] = convert_channel_value(rcChannels->chan1to4.chan3 >> 1U); }

			if (max_channels > 3U) { values[3] = convert_channel_value(rcChannels->chan1to4.chan4 >> 1U); }

			if (ghst_frame.type == static_cast<uint8_t>(ghstFrameType::frameType5to8)) {
				if (max_channels > 4U) { values[4] = convert_channel_value(rcChannels->chanA << 3U); }

				if (max_channels > 5U) { values[5] = convert_channel_value(rcChannels->chanB << 3U); }

				if (max_channels > 6U) { values[6] = convert_channel_value(rcChannels->chanC << 3U); }

				if (max_channels > 7U) { values[7] = convert_channel_value(rcChannels->chanD << 3U); }

			} else if (ghst_frame.type == static_cast<uint8_t>(ghstFrameType::frameType9to12)) {
				if (max_channels > 8U) { values[8] = convert_channel_value(rcChannels->chanA << 3U); }

				if (max_channels > 9U) { values[9] = convert_channel_value(rcChannels->chanB << 3U); }

				if (max_channels > 10U) { values[10] = convert_channel_value(rcChannels->chanC << 3U); }

				if (max_channels > 11U) { values[11] = convert_channel_value(rcChannels->chanD << 3U); }

			} else if (ghst_frame.type == static_cast<uint8_t>(ghstFrameType::frameType13to16)) {
				if (max_channels > 12U) { values[12] = convert_channel_value(rcChannels->chanA << 3U); }

				if (max_channels > 13U) { values[13] = convert_channel_value(rcChannels->chanB << 3U); }

				if (max_channels > 14U) { values[14] = convert_channel_value(rcChannels->chanC << 3U); }

				if (max_channels > 15U) { values[15] = convert_channel_value(rcChannels->chanD << 3U); }

			} else if (ghst_frame.type == static_cast<uint8_t>(ghstFrameType::frameTypeRssi)) {
				const ghstPayloadRssi_t *const rssiValues = (ghstPayloadRssi_t *)&ghst_frame.payload;
				// TODO: call function for RSSI dBm to percentage conversion for ghost receiver
				ghst_rssi = spek_dbm_to_percent(static_cast<int8_t>(rssiValues->rssidBm));

			} else {
				GHST_DEBUG("Frame type: %u", ghst_frame.type);
			}

			*rssi = ghst_rssi;

			memcpy(prev_rc_vals, values, sizeof(uint16_t) * GHST_MAX_NUM_CHANNELS);

			GHST_VERBOSE("Got Channels");

			ret = true;

		} else {
			GHST_DEBUG("CRC check failed");
		}

	} else {
		GHST_DEBUG("Got Non-RC frame (len=%u, type=%u)", current_frame_length, ghst_frame.type);
	}

	// either reset or move the rest of the buffer
	if (current_frame_position > current_frame_length) {
		GHST_VERBOSE("Moving buffer (%u > %u)", current_frame_position, current_frame_length);
		memmove(ghst_frame_ptr, ghst_frame_ptr + current_frame_length, current_frame_position - current_frame_length);
		current_frame_position -= current_frame_length;

	} else {
		current_frame_position = 0U;
	}

	return ret;
}

/**
 * write an uint8_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint8_t(uint8_t *buf, int &offset, uint8_t value)
{
	buf[offset++] = value;
}

/**
 * write an uint16_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint16_t(uint8_t *buf, int &offset, uint16_t value)
{
	buf[offset] = value & 0xFFU;
	buf[offset + 1] = value >> 8U;
	offset += 2;
}

/**
 * write frame header
 */
static inline void write_frame_header(uint8_t *buf, int &offset, ghstTelemetryType type, uint8_t payload_size)
{
	write_uint8_t(buf, offset, static_cast<uint8_t>(ghstAddress::rxAddress));
	write_uint8_t(buf, offset, payload_size + GHST_FRAME_CRC_SIZE + GHST_FRAME_TYPE_SIZE);
	write_uint8_t(buf, offset, static_cast<uint8_t>(type));
}

/**
 * write frame CRC
 */
static inline void write_frame_crc(uint8_t *buf, int &offset, int buf_size)
{
	write_uint8_t(buf, offset, crc8_dvb_s2_buf(buf + 2U, buf_size - 3));
}

bool ghst_send_telemetry_battery_status(int uart_fd, uint16_t voltage_in_10mV,
					uint16_t current_in_10mA, uint16_t fuel_in_10mAh)
{
	uint8_t buf[GHST_FRAME_PAYLOAD_SIZE_TELEMETRY + 4U]; // address, frame length, type, crc
	int offset = 0;
	write_frame_header(buf, offset, ghstTelemetryType::batteryPack, GHST_FRAME_PAYLOAD_SIZE_TELEMETRY);
	write_uint16_t(buf, offset, voltage_in_10mV);
	write_uint16_t(buf, offset, current_in_10mA);
	write_uint16_t(buf, offset, fuel_in_10mAh);
	write_uint8_t(buf, offset, 0x00U); // empty
	write_uint8_t(buf, offset, 0x00U); // empty
	write_uint8_t(buf, offset, 0x00U); // empty
	write_uint8_t(buf, offset, 0x00U); // empty
	write_frame_crc(buf, offset, sizeof(buf));

	return write(uart_fd, buf, offset) == offset;
}
