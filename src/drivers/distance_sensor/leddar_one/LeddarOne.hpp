/****************************************************************************
 *
 *   Copyright (C) 2017-2019 Intel Corporation. All rights reserved.
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

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

#define LEDDAR_ONE_FIELD_OF_VIEW        (0.105f) // 6 deg cone angle.

#define LEDDAR_ONE_MAX_DISTANCE         40.0f
#define LEDDAR_ONE_MIN_DISTANCE         0.01f

#define LEDDAR_ONE_MEASURE_INTERVAL     100_ms // 10Hz

#define MODBUS_SLAVE_ADDRESS            0x01
#define MODBUS_READING_FUNCTION         0x04
#define READING_START_ADDR              0x14
#define READING_LEN                     0xA

static const uint8_t request_reading_msg[] = {
	MODBUS_SLAVE_ADDRESS,
	MODBUS_READING_FUNCTION,
	0, /* starting addr high byte */
	READING_START_ADDR,
	0, /* number of bytes to read high byte */
	READING_LEN,
	0x30, /* CRC low */
	0x09 /* CRC high */
};

struct __attribute__((__packed__)) reading_msg {
	uint8_t slave_addr;
	uint8_t function;
	uint8_t len;
	uint8_t low_timestamp_high_byte;
	uint8_t low_timestamp_low_byte;
	uint8_t high_timestamp_high_byte;
	uint8_t high_timestamp_low_byte;
	uint8_t temp_high;
	uint8_t temp_low;
	uint8_t num_detections_high_byte;
	uint8_t num_detections_low_byte;
	uint8_t first_dist_high_byte;
	uint8_t first_dist_low_byte;
	uint8_t first_amplitude_high_byte;
	uint8_t first_amplitude_low_byte;
	uint8_t second_dist_high_byte;
	uint8_t second_dist_low_byte;
	uint8_t second_amplitude_high_byte;
	uint8_t second_amplitude_low_byte;
	uint8_t third_dist_high_byte;
	uint8_t third_dist_low_byte;
	uint8_t third_amplitude_high_byte;
	uint8_t third_amplitude_low_byte;
	uint16_t crc; /* little-endian */
};

class LeddarOne : public px4::ScheduledWorkItem
{
public:
	LeddarOne(const char *serial_port, const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~LeddarOne() override;

	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

private:

	/**
	 * Calculates the 16 byte crc value for the data frame.
	 * @param data_frame The data frame to compute a checksum for.
	 * @param crc16_length The length of the data frame.
	 */
	uint16_t crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length);

	/**
	 * Reads the data measrurement from serial UART.
	 */
	int collect();

	/**
	 * Sends a data request message to the sensor.
	 */
	int measure();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	void Run() override;

	const char *_serial_port{nullptr};

	PX4Rangefinder _px4_rangefinder;

	int _file_descriptor{-1};

	uint8_t _buffer[sizeof(reading_msg)];
	uint8_t _buffer_len{0};

	hrt_abstime _measurement_time{0};

	perf_counter_t _comms_error{perf_alloc(PC_COUNT, MODULE_NAME": comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sample")};
};
