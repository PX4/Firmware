/****************************************************************************
 *
 *	Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file st24.h
 *
 * RC protocol definition for Yuneec ST24 transmitter
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#pragma once

#include <stdint.h>

__BEGIN_DECLS

#define ST24_DATA_LEN_MAX	64
#define ST24_STX1		0x55
#define ST24_STX2		0x55

enum ST24_PACKET_TYPE {
	ST24_PACKET_TYPE_CHANNELDATA12 = 0,
	ST24_PACKET_TYPE_CHANNELDATA24,
	ST24_PACKET_TYPE_TRANSMITTERGPSDATA
};

enum ST24_DECODE_STATE {
	ST24_DECODE_STATE_UNSYNCED = 0,
	ST24_DECODE_STATE_GOT_STX1,
	ST24_DECODE_STATE_GOT_STX2,
	ST24_DECODE_STATE_GOT_LEN,
	ST24_DECODE_STATE_GOT_TYPE,
	ST24_DECODE_STATE_GOT_DATA
};

#pragma pack(push, 1)
typedef struct {
	uint8_t	header1;			///< 0x55 for a valid packet
	uint8_t	header2;			///< 0x55 for a valid packet
	uint8_t	length;				///< length includes type, data, and crc = sizeof(type)+sizeof(data[payload_len])+sizeof(crc8)
	uint8_t	type;				///< from enum ST24_PACKET_TYPE
	uint8_t	st24_data[ST24_DATA_LEN_MAX];
	uint8_t	crc8;				///< crc8 checksum, calculated by st24_common_crc8 and including fields length, type and st24_data
} ReceiverFcPacket;

/**
 * RC Channel data (12 channels).
 *
 * This is incoming from the ST24
 */
typedef struct {
	uint16_t t;			///< packet counter or clock
	uint8_t	rssi;			///< signal strength
	uint8_t	lost_count;		///< Number of UART packets sent since reception of last RF frame (100 frame means RC timeout of 1s)
	uint8_t	channel[18];		///< channel data, 12 channels (12 bit numbers)
} ChannelData12;

/**
 * RC Channel data (12 channels).
 *
 */
typedef struct {
	uint16_t t;			///< packet counter or clock
	uint8_t	rssi;			///< signal strength
	uint8_t	lost_count;		///< Number of UART packets sent since reception of last RF frame (100 frame means RC timeout of 1s)
	uint8_t	channel[36];		///< channel data, 24 channels (12 bit numbers)
} ChannelData24;

/**
 * Telemetry packet
 *
 * This is outgoing to the ST24
 *
 * imuStatus:
 * 8 bit total
 * bits 0-2 for status
 * - value 0 is FAILED
 * - value 1 is INITIALIZING
 * - value 2 is RUNNING
 * - values 3 through 7 are reserved
 * bits 3-7 are status for sensors (0 or 1)
 * - mpu6050
 * - accelerometer
 * - primary gyro x
 * - primary gyro y
 * - primary gyro z
 *
 * pressCompassStatus
 * 8 bit total
 * bits 0-3 for compass status
 * - value 0 is FAILED
 * - value 1 is INITIALIZING
 * - value 2 is RUNNING
 * - value 3 - 15 are reserved
 * bits 4-7 for pressure status
 * - value 0 is FAILED
 * - value 1 is INITIALIZING
 * - value 2 is RUNNING
 * - value 3 - 15 are reserved
 *
 */
typedef struct {
	uint16_t t;			///< packet counter or clock
	int32_t	lat;			///< lattitude (degrees)	+/- 90 deg
	int32_t	lon;			///< longitude (degrees)	+/- 180 deg
	int32_t	alt;			///< 0.01m resolution, altitude (meters)
	int16_t	vx, vy, vz; 		///< velocity 0.01m res, +/-320.00 North-East- Down
	uint8_t	nsat;			///<number of satellites
	uint8_t	voltage; 		///< 25.4V	voltage = 5 + 255*0.1 = 30.5V, min=5V
	uint8_t	current; 		///< 0.5A resolution
	int16_t	roll, pitch, yaw;	///< 0.01 degree resolution
	uint8_t	motorStatus;		///< 1 bit per motor for status 1=good, 0= fail
	uint8_t	imuStatus;		///< inertial measurement unit status
	uint8_t	pressCompassStatus;	///< baro / compass status
} TelemetryData;

#pragma pack(pop)

/**
 * CRC8 implementation for ST24 protocol
 *
 * @param prt Pointer to the data to CRC
 * @param len number of bytes to accumulate in the checksum
 * @return the checksum of these bytes over len
 */
uint8_t st24_common_crc8(uint8_t *ptr, uint8_t len);

__EXPORT void st24_reset(void);

/**
 * Decoder for ST24 protocol
 *
 * @param byte current char to read
 * @param rssi pointer to a byte where the RSSI value is written back to
 * @param lost_count pointer to a byte where the receive count of packets since last wireless frame is written back to ( > 0 if RC is lost)
 * @param channels pointer to a datastructure of size max_chan_count where channel values (12 bit) are written back to
 * @param max_chan_count maximum channels to decode - if more channels are decoded, the last n are skipped and success (0) is returned
 * @return 0 for success (a decoded packet), 1 for no packet yet (accumulating), 2 for unknown packet, 3 for out of sync, 4 for checksum error
 */
__EXPORT int st24_decode(uint8_t byte, uint8_t *rssi, uint8_t *lost_count, uint16_t *channel_count,
			 uint16_t *channels, uint16_t max_chan_count);

__END_DECLS
