/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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

#include "CM8JL65.hpp"

#include <fcntl.h>

#include <lib/drivers/device/Device.hpp>

static constexpr unsigned char crc_msb_vector[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40
};

static constexpr unsigned char crc_lsb_vector[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
	0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
	0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
	0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
	0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
	0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
	0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
	0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
	0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
	0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
	0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
	0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
	0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
	0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
	0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
	0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
	0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
	0x41, 0x81, 0x80, 0x40
};

CM8JL65::CM8JL65(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	// Store the port name.
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);

	// Use conservative distance bounds, to make sure we don't fuse garbage data
	_px4_rangefinder.set_min_distance(0.2f);	// Datasheet: 0.17m
	_px4_rangefinder.set_max_distance(7.9f);	// Datasheet: 8.0m
	_px4_rangefinder.set_fov(0.0488692f);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_CM8JL65);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
}

CM8JL65::~CM8JL65()
{
	// Ensure we are truly inactive.
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

uint16_t
CM8JL65::crc16_calc(const unsigned char *data_frame, uint8_t crc16_length)
{
	// compute CRC16 IBM 8005
	unsigned char crc_high_byte = 0xFF;
	unsigned char crc_low_byte = 0xFF;
	size_t index = 0;

	while (crc16_length--) {
		index = crc_low_byte ^ *(data_frame++);
		crc_low_byte = (unsigned char)(crc_high_byte ^ crc_msb_vector[index]);
		crc_high_byte = crc_lsb_vector[index];
	}

	uint16_t crc16 = (crc_high_byte << 8 | crc_low_byte);
	crc16 = (crc16 >> 8) | (crc16 << 8); // Convert endian

	return crc16;
}

int
CM8JL65::collect()
{
	perf_begin(_sample_perf);

	int bytes_processed = 0;
	int distance_mm = -1;
	int index = 0;

	bool crc_valid = false;

	// Read from the sensor UART buffer.
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int bytes_read = ::read(_file_descriptor, &_linebuf[0], sizeof(_linebuf));

	if (bytes_read > 0) {
		index = bytes_read - 6 ;

		while (index >= 0 && !crc_valid) {
			if (_linebuf[index] == START_FRAME_DIGIT1) {
				bytes_processed = index;

				while (bytes_processed < bytes_read && !crc_valid) {
					if (data_parser(_linebuf[bytes_processed], _frame_data, _parse_state, _crc16, distance_mm) == PX4_OK) {
						crc_valid = true;
					}

					bytes_processed++;
				}

				_parse_state = PARSE_STATE::WAITING_FRAME;
			}

			index--;
		}

	} else if (bytes_read == -1 && errno == EAGAIN) {
		return -EAGAIN;

	} else {

		PX4_ERR("read error: %i, errno: %i", bytes_read, errno);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	if (!crc_valid) {
		return -EAGAIN;
	}

	bytes_read = OK;

	const float current_distance = static_cast<float>(distance_mm) / 1000.0f;

	_px4_rangefinder.update(timestamp_sample, current_distance);

	perf_end(_sample_perf);

	return PX4_OK;
}

int
CM8JL65::data_parser(const uint8_t check_byte, uint8_t parserbuf[PARSER_BUF_LENGTH],
		     PARSE_STATE &state, uint16_t &crc16, int &distance)
{
	switch (state) {
	case PARSE_STATE::WAITING_FRAME:
		if (check_byte == START_FRAME_DIGIT1) {
			state = PARSE_STATE::DIGIT_1;
		}

		break;

	case PARSE_STATE::DIGIT_1:
		if (check_byte == START_FRAME_DIGIT1) {
			state = PARSE_STATE::DIGIT_1;

		} else if (check_byte == START_FRAME_DIGIT2) {
			state = PARSE_STATE::DIGIT_2;

		} else {
			state = PARSE_STATE::WAITING_FRAME;
		}

		break;

	case PARSE_STATE::DIGIT_2:
		state = PARSE_STATE::MSB_DATA;
		parserbuf[DISTANCE_MSB_POS] = check_byte; // MSB Data
		break;

	case PARSE_STATE::MSB_DATA:
		state = PARSE_STATE::LSB_DATA;
		parserbuf[DISTANCE_LSB_POS] = check_byte; // LSB Data

		// Calculate CRC.
		crc16 = crc16_calc(parserbuf, PARSER_BUF_LENGTH);
		break;

	case PARSE_STATE::LSB_DATA:
		if (check_byte == (crc16 >> 8)) {
			state = PARSE_STATE::CHECKSUM;

		} else {
			state = PARSE_STATE::WAITING_FRAME;
		}

		break;

	case PARSE_STATE::CHECKSUM:
		// Here, reset state to `NOT-STARTED` no matter crc ok or not
		state = PARSE_STATE::WAITING_FRAME;

		if (check_byte == (crc16 & 0xFF)) {
			// printf("Checksum verified \n");
			distance = (parserbuf[DISTANCE_MSB_POS] << 8) | parserbuf[DISTANCE_LSB_POS];
			return PX4_OK;
		}

		break;
	}

	return PX4_ERROR;
}

int
CM8JL65::init()
{
	start();

	return PX4_OK;
}

int
CM8JL65::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit.
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void
CM8JL65::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void
CM8JL65::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	// Perform collection.
	collect();
}

void
CM8JL65::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(CM8JL65_MEASURE_INTERVAL);

	PX4_INFO("driver started");
}

void
CM8JL65::stop()
{
	// Clear the work queue schedule.
	ScheduleClear();

	// Ensure the serial port is closed.
	::close(_file_descriptor);
}
