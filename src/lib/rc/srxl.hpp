/****************************************************************************
 *
 *	Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * Serial protocol definition for Spektrum SRXL
 *
 * @author Kurt Kiefer <kekiefer@gmail.com>
 * @author Daniel Williams <equipoise@gmail.com>
 */

#pragma once

#include <cstdint>
#include <px4_platform_common/defines.h>

class SRXLCodec
{
public:
	SRXLCodec() = default;
	~SRXLCodec() = default;

	void configure(int fd);

	/// \brief retrieve frame drops count, as reported by the receiver
	uint16_t frame_drops() const;

	/// \brief parse and process, the frame in the given buffer
	bool parse(const uint64_t now, const uint8_t *source, const uint8_t source_length);

	/// \brief retrieve control channel data
	uint16_t *channels();
	int channel_count() const;

	/// \brief request that the currently-connected receiver place itself into bind-mode
	int request_bind_receiver(const uint8_t bind_mode);

	/// \brief get the last reported signal strength value.
	/// \return reports signal RSSI in percentage points: 0-100
	int rssi_percentage() const;

	/// \brief check if the receiver currently accepts telemetry downlink
	bool supports_telemetry() const;


private:
	// Version 1 specific
	constexpr static const uint8_t SRXL1_FRAME_HEADER = 	0xA5;
	constexpr static const uint8_t SRXL1_MAX_LENGTH =	64;

	// Version 2 specific
	constexpr const static uint8_t SRXL2_FRAME_HEADER = 	0xA6;
	constexpr const static uint8_t SRXL2_MIN_LENGTH =	5;
	constexpr const static uint8_t SRXL2_MAX_LENGTH =	80;

	// Packet Types
	enum frame_type_t {
		FRAME_TYPE_HANDSHAKE = 0x21,
		FRAME_TYPE_BIND = 0x41,
		FRAME_TYPE_PARAMETER = 0x50,
		FRAME_TYPE_SIGNAL_QUALITY = 0x55,
		FRAME_TYPE_TELEMETRY = 0x80,
		FRAME_TYPE_CONTROL = 0xCD,
	};

	// Frame Sizes
	constexpr static uint8_t FRAME_LENGTH_HANDSHAKE = 14;
	constexpr static uint8_t FRAME_LENGTH_BIND = 21;
	constexpr static uint8_t FRAME_LENGTH_PARAMETER = 14;
	constexpr static uint8_t FRAME_LENGTH_SIGNAL_QUALITY = 10;
	constexpr static uint8_t FRAME_LENGTH_TELEMETRY = 22;

	// apply to all versions
	constexpr static const uint8_t SRXL_FRAME_HEADER_SIZE = 3;
	constexpr static const uint8_t SRXL_MIN_FRAME_LENGTH = SRXL2_MIN_LENGTH;
	constexpr static const uint8_t SRXL_MAX_FRAME_LENGTH = SRXL2_MAX_LENGTH;
	constexpr static const uint8_t SRXL_MAX_PAYLOAD_LENGTH = SRXL2_MAX_LENGTH - SRXL_FRAME_HEADER_SIZE;

	constexpr static const uint8_t SRXL_RECIEVER_MIN_ID = 0x10;
	constexpr static const uint8_t SRXL_RECIEVER_MAX_ID = 0x2F;
	/// The PX4 flight controller will advertise this SRXL bus ID:
	constexpr static const uint8_t FLIGHT_CONTROLLER_DEFAULT_ID = 0x30;

	constexpr static const uint32_t FLIGHT_CONTROLLER_UUID = 0x12345678;

	// for complete reference, see SRXL2 Specification, Rev K, page 10
	//     the other bind types are not implemented in this driver
	enum bind_type_t {
		SRXL_BIND_NONE = 0x00,
		SRXL_BIND_MODE_DSMX_22MS =  0xA2,   // Enforce 22ms ONLY
		SRXL_BIND_MODE_DSMX_11MS =  0xB2,   // auto-selects 11ms or 22ms based on best available speed.
	};

private:

	static uint16_t srxlCrc16(const uint8_t *const packet);

	uint16_t decode_channel(const uint16_t raw_value);

	void handle_bind_frame(const uint8_t *const frame);

	void handle_control_frame(const uint8_t *const frame);

	void handle_handshake_frame(const uint8_t *const frame);

	void handle_signal_quality(const uint8_t *const frame);

	void pack(uint8_t *const frame);

	void print_frame(const char *const preamble, const uint8_t *frame);

	void reset();

	// adapted from frsky_telemetry module.  Seems to work ok.
	bool set_single_wire(bool single_wire = true);

	// void set_payload( uint8_t *payload, size_t length, srxl_frame_t* dest);

	bool validate_checksum(const uint8_t *const frame);

private:
	// size_t _bytes_received;

	// size_t _bytes_to_transmit;

	// file handle for internal serial port
	// this is singleton-enforced
	int _fd;

	hrt_abstime _last_rx_time;
	hrt_abstime _last_print_time = 0;

	uint16_t _frame_drops;

	// uint32_t _partial_frame_count;

	uint8_t _rssi_percentage;
	static constexpr int max_control_channel_count = 32;
	uint16_t _control_channels[max_control_channel_count];
	int _updated_channel_count = 0;

	// id for this px4-flight-controller  ... on the srxl bus
	constexpr static uint8_t _fc_id = FLIGHT_CONTROLLER_DEFAULT_ID;

	// id of the paired spektrum receiver
	uint8_t _receiver_id = 0;
	uint8_t _receiver_bind_type = 0;  // 0 == not bound

	// Options: A byte with 3 RF enable bits:
	//	bit 0:	enable Telemetry transmission over RF for the device
	//	bit 1:	allow Bind reply over RF
	//	bit 2:	request US power level for RF transmits
	uint8_t _receiver_options = 0;

};
