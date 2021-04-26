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

#include "LeddarOne.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>

LeddarOne::LeddarOne(const char *serial_port, uint8_t device_orientation):
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
	_px4_rangefinder(0, device_orientation)
{
	_serial_port = strdup(serial_port);

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_serial_port[strlen(_serial_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_LEDDARONE);

	_px4_rangefinder.set_max_distance(LEDDAR_ONE_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(LEDDAR_ONE_MIN_DISTANCE);
	_px4_rangefinder.set_fov(LEDDAR_ONE_FIELD_OF_VIEW);
}

LeddarOne::~LeddarOne()
{
	stop();

	free((char *)_serial_port);
	perf_free(_comms_error);
	perf_free(_sample_perf);
}

uint16_t
LeddarOne::crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length)
{
	uint16_t crc = 0xFFFF;

	for (uint8_t i = 0; i < crc16_length; i++) {
		crc ^= data_frame[i];

		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 1) {
				crc = (crc >> 1) ^ 0xA001;

			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

int
LeddarOne::collect()
{
	perf_begin(_sample_perf);

	const int buffer_size = sizeof(_buffer);
	const int message_size = sizeof(reading_msg);

	int bytes_read = ::read(_file_descriptor, _buffer + _buffer_len, buffer_size - _buffer_len);

	if (bytes_read < 1) {
		// Trigger a new measurement.
		return measure();
	}

	_buffer_len += bytes_read;

	if (_buffer_len < message_size) {
		// Return on next scheduled cycle to collect remaining data.
		return PX4_OK;
	}

	reading_msg *msg {nullptr};
	msg = (reading_msg *)_buffer;

	if (msg->slave_addr != MODBUS_SLAVE_ADDRESS ||
	    msg->function != MODBUS_READING_FUNCTION) {

		PX4_ERR("slave address or function read error");
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return measure();
	}

	const uint16_t crc16 = crc16_calc(_buffer, buffer_size - 2);

	if (crc16 != msg->crc) {
		PX4_ERR("crc error");
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return measure();
	}

	// NOTE: little-endian support only.
	uint16_t distance_mm = (msg->first_dist_high_byte << 8 | msg->first_dist_low_byte);
	float distance_m = static_cast<float>(distance_mm) / 1000.0f;

	// @TODO - implement a meaningful signal quality value.
	int8_t signal_quality = -1;

	_px4_rangefinder.update(_measurement_time, distance_m, signal_quality);

	perf_end(_sample_perf);

	// Trigger the next measurement.
	return measure();
}

int
LeddarOne::init()
{
	if (open_serial_port() != PX4_OK) {
		return PX4_ERROR;
	}

	hrt_abstime time_now = hrt_absolute_time();

	const hrt_abstime timeout_usec = time_now + 500000_us; // 0.5sec

	while (time_now < timeout_usec) {
		if (measure() == PX4_OK) {
			px4_usleep(LEDDAR_ONE_MEASURE_INTERVAL);

			if (collect() == PX4_OK) {
				// The file descriptor can only be accessed by the process that opened it,
				// so closing here allows the port to be opened from scheduled work queue.
				stop();
				return PX4_OK;
			}
		}

		px4_usleep(1000);
		time_now = hrt_absolute_time();
	}

	PX4_ERR("No readings from LeddarOne");
	return PX4_ERROR;
}

int
LeddarOne::measure()
{
	// Flush the receive buffer.
	tcflush(_file_descriptor, TCIFLUSH);

	int num_bytes = ::write(_file_descriptor, request_reading_msg, sizeof(request_reading_msg));

	if (num_bytes != sizeof(request_reading_msg)) {
		PX4_INFO("measurement error: %i, errno: %i", num_bytes, errno);
		return PX4_ERROR;
	}

	_measurement_time = hrt_absolute_time();
	_buffer_len = 0;
	return PX4_OK;
}

int
LeddarOne::open_serial_port(const speed_t speed)
{
	// File descriptor already initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_serial_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	if (tcgetattr(_file_descriptor, &uart_config)) {
		PX4_ERR("Unable to get termios from %s.", _serial_port);
		::close(_file_descriptor);
		_file_descriptor = -1;
		return PX4_ERROR;
	}

	// Clear: data bit size, two stop bits, parity, hardware flow control.
	uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);

	// Set: 8 data bits, enable receiver, ignore modem status lines.
	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	// Clear: echo, echo new line, canonical input and extended input.
	uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

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

	// Flush the hardware buffers.
	tcflush(_file_descriptor, TCIOFLUSH);

	PX4_DEBUG("opened UART port %s", _serial_port);

	return PX4_OK;
}

void
LeddarOne::print_info()
{
	perf_print_counter(_comms_error);
	perf_print_counter(_sample_perf);
}

void
LeddarOne::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
}

void
LeddarOne::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(LEDDAR_ONE_MEASURE_INTERVAL, LEDDAR_ONE_MEASURE_INTERVAL);
}

void
LeddarOne::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);
	_file_descriptor = -1;

	// Clear the work queue schedule.
	ScheduleClear();
}
