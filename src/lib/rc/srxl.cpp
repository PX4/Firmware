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

#include <cstring>
#include <cmath>

#include <termios.h>


#include "srxl.hpp"

#define SRXL_DEBUG_LEVEL 2

// enable debugging output
#if SRXL_DEBUG_LEVEL > 0
#define SRXL_WARN(...) printf(__VA_ARGS__)
#else
#define SRXL_WARN(...)
#endif

#if SRXL_DEBUG_LEVEL > 1
#define SRXL_DEBUG(...) printf(__VA_ARGS__)
#else
#define SRXL_DEBUG(...)
#endif

// verbose debugging--Careful when enabling: it leads to too much output, causing dropped bytes
#if SRXL_DEBUG_LEVEL > 2
#define SRXL_TRACE(...) printf(__VA_ARGS__)
#else
#define SRXL_TRACE(...)
#endif

void SRXLCodec::configure(const int new_file_despcriptor)
{
	SRXL_TRACE(">> SRXL: initializing...\n");

	_fd = new_file_despcriptor;

#ifdef SPEKTRUM_POWER_CONFIG
	// Enable power controls for Spektrum receiver
	SPEKTRUM_POWER_CONFIG();
#endif
#ifdef SPEKTRUM_POWER
	// enable power on Spektrum connector
	SPEKTRUM_POWER(true);
#endif

	if (_fd > 0) {
		struct termios t;

		/* 115200bps, no parity, one stop bit */
		tcgetattr(_fd, &t);
		cfsetspeed(&t, 115200);
		t.c_cflag &= ~(CSTOPB | PARENB);
		tcsetattr(_fd, TCSANOW, &t);

		set_single_wire(true);

		reset();
	}
}

uint16_t *SRXLCodec::channels()
{
	return _control_channels;
}

int SRXLCodec::channel_count() const
{
	return _updated_channel_count;
}


// Compute SRXL CRC over packet buffer (assumes length is correctly set)
uint16_t SRXLCodec::srxlCrc16(const uint8_t *const packet)
{
	uint16_t crc = 0;                // Seed with 0
	const uint8_t data_length = packet[2] - 2;  // Exclude 2 CRC bytes at end of packet from the length

	if (data_length <= SRXL2_MAX_LENGTH - 2) {
		// Use bitwise method
		for (uint8_t i = 0; i < data_length; ++i) {
			crc = crc ^ ((uint16_t)packet[i] << 8);

			for (int b = 0; b < 8; b++) {
				if (crc & 0x8000) {
					crc = (crc << 1) ^ 0x1021;

				} else {
					crc = crc << 1;
				}
			}
		}
	}

	return crc;
}

uint16_t SRXLCodec::decode_channel(const uint16_t raw_value)
{
	// The data for the channels is as follows:
	//     CH 1 = 0x2AA0 = 10912     // (approx -100% on Spektrum transmitter)
	//     CH 2 = 0x8000 = 32768     // (center position)
	//     CH 3 = 0x8004 = 32772     // (1 tick above center at 14-bit resolution)
	//     CH 5 = 0x7FFC = 32764     // (1 tick below center at 14-bit resolution)
	//     CH 6 = 0xD554 = 54612     // (approx. 100% on Spektrum transmitter)
	if (raw_value < 0x1000 || raw_value > 0xF000) {
		// if the value is unrealistic, fail the parsing entirely
		SRXL_DEBUG("Invalid SRXL-Channel range: %u\n", raw_value);
		return 0;
	}

	// The channel value must be bit-shifted to the right to match the application’s accepted resolution.
	// For example, to match DSMX 2048 data (11-bit), shift each value 5 bits to the right. -- SXRL2 Specification p15
	auto shifted_value = raw_value >> 6;

	auto offset_value = shifted_value + 1000;

	return offset_value;
}

uint16_t SRXLCodec::frame_drops() const
{
	return _frame_drops;
}

void SRXLCodec::handle_bind_frame(const uint8_t *const frame)
{
	// <0xA6><0x41><Length><Request><DeviceID><Type><Options><GUID><UID><CRC>

	const uint8_t request_type = frame[3];

	if (0xEB == request_type) {
		printf("        ::Bind-Mode-Entered::\n");
		// Enter Bind Mode -- for receivers; ignore.
		return;

	} else if (0xB5 == request_type) {
		// Request Bind Status -- for receivers; ignore.
		printf("        ::Request-Bind-Status::\n");
		return;

	} else if (0xDB == request_type) {
		// Bind Data Report
		print_frame("    ::Bind-Report: ", frame);

		const uint8_t src_id = frame[4];
		printf("        ::Bind::  from: %02x(=?=%02x)", src_id, _receiver_id);

		_receiver_bind_type = frame[5];
		printf("  btype: %02x", _receiver_bind_type);

		_receiver_options = frame[6];
		printf("  telemetry: %s\n", (supports_telemetry() ? "enabled" : "disabled"));

	} else if (0xB5 == request_type) {
		// Set Bind Info -- for receivers; ignore.
		printf("        ::Set-Bind-Info::\n");

		const uint8_t src_id = frame[4];
		printf("        ::Bind::  from: %02x(%02x)", src_id, _receiver_id);

		_receiver_bind_type = frame[5];
		printf("  btype: %02x", _receiver_bind_type);

		_receiver_options = frame[6];
		printf("  telemetry: %s\n", (supports_telemetry() ? "enabled" : "disabled"));

		return;

	} else {
		// invalid value; ignore.
		return;
	}
}

void SRXLCodec::handle_control_frame(const uint8_t *const frame)
{
	// if a valid frame, start extracting fields
	// const uint8_t frame_length = frame[2];
	// const uint8_t frame_command = frame[3];
	// const uint8_t reply_id = frame[4];

	// if( 0 == frame_command ){
	// }

	// If no reply is desired, set this to 0x00 (no device).
	// const bool should_reply = ( _fc_id == reply_id );

	const int8_t rssi_report = frame[5];

	if (0 < rssi_report) {
		// this is a percentage RSSI report:
		_rssi_percentage = rssi_report;

	} else {
		// this is a dBm RSSI report; NYI
	}

	_frame_drops = (frame[6] << 8) | frame[7];

	// channel mask is small-endian:
	const uint32_t channel_mask = frame[8] | (frame[9] << 8) | (frame[10] << 16) | (frame[11] << 24);

	if (0 == channel_mask) {
		// common-case-optimization: if channel-mask is zero, there is no data to interpret.
		return;
	}

	int receive_channel_index = 12;  ///< byte index in the received frame
	int max_channel_count = 0;


	for (int receive_channel_number = 0; receive_channel_number < max_control_channel_count; ++receive_channel_number) {
		// only extract channels which are actually received
		const bool channel_present = static_cast<uint32_t>(1 << receive_channel_number) & channel_mask;

		if (channel_present) {
			const uint16_t receive_channel_value = frame[receive_channel_index] | (frame[receive_channel_index + 1] << 8);
			const uint16_t px4_channel_value = decode_channel(receive_channel_value);
			max_channel_count = receive_channel_number + 1;

			if (0 != px4_channel_value) {
				_control_channels[receive_channel_number] = px4_channel_value;
			}

			receive_channel_index += 2;
		}
	}

#ifdef SRXL_DEBUG_LEVEL
	constexpr static uint64_t print_interval = 1e6;

	if (print_interval < (_last_rx_time - _last_print_time)) {
		_last_print_time = _last_rx_time;
		printf("    >>Control-Frame::Channel-Mask:  %08x \n", channel_mask);

		for (int i = 0; i < 4; ++i) {   // 4 channels are the troublesome ones-- pitch roll, throttle, yaw
			printf("            [%2d]: %04x \n", i, _control_channels[i]);
		}
	}

#endif

	if (max_channel_count > _updated_channel_count) {
		_updated_channel_count = max_channel_count;
	}
}

void SRXLCodec::handle_handshake_frame(const uint8_t *const request)
{
	// Note: currently ignore, and assume this is an active handshake request
	const uint8_t dest_id = request[4];

	if (0xFF == dest_id) {
		// This is a broadcast id:
		//   - This coincides with a successful receiver handshake
		//   - Used to set baud rate; we only support 115200 baud, so make no changes
		SRXL_DEBUG("    >> Handshake complete: 0xFF\n");
		return;
	}

	const uint8_t source_id = request[3];

	if (_fc_id == source_id) {
		// we detected our own transmission packet.. ignore
		return;
	}

	printf("    >> Handshake complete:  %d => %d\n", source_id, dest_id);
	_receiver_id = source_id;

	// ==== Construct Reply ====
	uint8_t reply[FRAME_LENGTH_HANDSHAKE];
	reply[0] = SRXL2_FRAME_HEADER;
	reply[1] = FRAME_TYPE_HANDSHAKE;
	reply[2] = FRAME_LENGTH_HANDSHAKE;
	reply[3] = _fc_id; ///< SrcId
	reply[4] = _receiver_id;  ///< DestID
	reply[5] = 10;  ///< Priority; 10 == 0x0A = default
	reply[6] = 0;   ///< BaudRate; 0 => 115200 baud
	reply[7] = 0;   ///< Info

	// store as big-endian
	reply[8] = static_cast<uint8_t>(FLIGHT_CONTROLLER_UUID >> 24);
	reply[9] = static_cast<uint8_t>(FLIGHT_CONTROLLER_UUID >> 16);
	reply[10] = static_cast<uint8_t>(FLIGHT_CONTROLLER_UUID >> 8);
	reply[11] = static_cast<uint8_t>(FLIGHT_CONTROLLER_UUID);

	pack(reply);

	// send reply
	::write(_fd, &reply, FRAME_LENGTH_HANDSHAKE);
}

void SRXLCodec::handle_signal_quality(const uint8_t *const request)
{
}

void SRXLCodec::pack(uint8_t *const frame)
{
	const uint8_t frame_length = frame[2] - 2;
	const uint16_t crc = srxlCrc16(frame);
	frame[frame_length + 0] = static_cast<uint8_t>(crc >> 8);   // MSB
	frame[frame_length + 1] = static_cast<uint8_t>(crc & 0xFF); // LSB
}


bool SRXLCodec::parse(const uint64_t now, const uint8_t *source, const uint8_t source_length)
{

	// check 1: look for sync byte:
	const uint8_t *cursor = source;   // < movable pointer to constant data

	//     Note: This parser ignores SRXL version 1 messages
	if (*cursor != SRXL2_FRAME_HEADER) {
		// printf("        XX Not an SRXL frame.\n");
		return false;
	}

	// if a valid frame, start extracting fields
	const uint8_t frame_type = cursor[1];
	const uint8_t frame_length = cursor[2];

	// check 2: have we received enough bytes to be a frame, at all?
	if (source_length < SRXL_MIN_FRAME_LENGTH) {
		// printf("        XX incomplete frame: %d < %d\n", source_length, frame->length );
		return false;
	}

	// check 3: have we received the whole frame?
	if (source_length < frame_length) {
		// printf("        XX incomplete frame: %d < %d\n", source_length, frame->length );
		return false;
	}

	// check 4: verify checksum
	if (! validate_checksum(cursor)) {
		return false;
	}

	_last_rx_time = now;

	// check 5: is this a known frame type => route appropriately
	// printf("    ::frame-type:  %02x: \n", frame_type );
	switch (frame_type) {
	case FRAME_TYPE_BIND:
		handle_bind_frame(cursor);
		return false;

	case FRAME_TYPE_CONTROL:
		handle_control_frame(cursor);
		return true;

	case FRAME_TYPE_HANDSHAKE:
		handle_handshake_frame(cursor);
		_updated_channel_count = 0;
		return true;

	case FRAME_TYPE_PARAMETER:
		// not supported
		return false;

	case FRAME_TYPE_SIGNAL_QUALITY:
		// NYI
		// handle_signal_quality( *frame );
		return false;

	case FRAME_TYPE_TELEMETRY:
		// receiving telemetry not supported; only sending telemetry
		return false;

	default:  // failure path
		return false;
	}
}

#if SRXL_DEBUG_LEVEL > 1
void SRXLCodec::print_frame(const char *const preamble, const uint8_t *cursor)
{
	const uint8_t frame_length = cursor[2];
	printf("%s %d: [", preamble, frame_length);

	for (int i = 0; i < frame_length; ++i, ++cursor) {
		printf(" %02X", *cursor);
	}

	printf("]\n");
}
#endif

void SRXLCodec::reset()
{
	// _partial_frame_count = 0;

	// initializes all channels to the invalid-flag-value
	for (int i = 0; i < max_control_channel_count; ++i) {
		_control_channels[i] = UINT16_MAX;
	}

	_last_rx_time = hrt_absolute_time();

	_frame_drops = 0;

	_rssi_percentage = 0;

	_updated_channel_count = 0;

	// reset connection state
	_receiver_id = 0;
	_receiver_bind_type = 0;
	_receiver_options = 0;
}

int SRXLCodec::rssi_percentage() const
{
	return _rssi_percentage;
}

int SRXLCodec::request_bind_receiver(uint8_t bind_mode)
{
	// explicitly override bind_mode -- upstream code does not correctly populate this value
	bind_mode = SRXLCodec::SRXL_BIND_MODE_DSMX_11MS;

	printf("    >> Request Bind as: %02x\n", bind_mode);

	// ==== Construct Reply ====
	// <0xA6><0x41><Length> <Request><DeviceID><Type><Options><GUID><UID><CRC>
	uint8_t reply[FRAME_LENGTH_BIND];
	reply[0] = SRXL2_FRAME_HEADER;
	reply[1] = FRAME_TYPE_BIND;
	reply[2] = FRAME_LENGTH_BIND;
	reply[3] = 0xEB;  ///< request-type
	reply[4] = _receiver_id;  ///< DeviceId === DestID === default receiver address
	reply[5] = bind_mode;  ///< bind-type
	///< bind type (1==telemetry, 2==bind-reply, 3=request US power level for transmits )
	reply[6] = 3; // = 1 & 2;
	// bit #2 ( == 0x03) requests US Power Levels -- FCC-Compliant Power Levels)

	// Zero out remaining fields
	// // GUID field:
	// memset(reply+7, 0, 8);
	// // UID field:
	// memset(reply+15, 0, 4);

	pack(reply);

	// send reply
	return ::write(_fd, &reply, FRAME_LENGTH_BIND);
}

bool SRXLCodec::set_single_wire(bool single_wire)
{
#ifdef SER_SINGLEWIRE_ENABLED
	auto flags  = single_wire ? (SER_SINGLEWIRE_ENABLED | SER_SINGLEWIRE_PUSHPULL | SER_SINGLEWIRE_PULLDOWN) : 0 ;

	if (0 > ioctl(_fd, TIOCSSINGLEWIRE, flags)) {
		perror("!! Could not set TIOCSSINGLEWIRE:");
		return false;
	}

	return true;
#else
	return false;
#endif
}

bool SRXLCodec::supports_telemetry() const
{
	return (1 & _receiver_options);
}

bool SRXLCodec::validate_checksum(const uint8_t *const frame)
{
	const uint8_t frame_length = frame[2];
	const uint16_t found_checksum = (frame[frame_length - 2] << 8) | frame[frame_length - 1];

	if (0 == found_checksum) {
		printf("    XX Peer has returned the frame with checksum == 0 !?\n");
		return false;
	}

	const uint16_t calculated_checksum = srxlCrc16(frame);

	if (found_checksum == calculated_checksum) {
		return true;

	} else {
		printf("    XX invalid checksum:  %04X != %04X\n", found_checksum, calculated_checksum);
		return false;
	}

}
