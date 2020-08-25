/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "TERARANGER.hpp"

#include <lib/parameters/param.h>

static constexpr uint8_t crc_table[] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
	0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
	0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
	0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
	0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
	0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
	0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
	0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
	0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
	0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
	0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
	0xfa, 0xfd, 0xf4, 0xf3
};

static uint8_t crc8(uint8_t *p, uint8_t len)
{
	uint16_t i;
	uint16_t crc = 0x0;

	while (len--) {
		i = (crc ^ *p++) & 0xFF;
		crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
	}

	return crc & 0xFF;
}

TERARANGER::TERARANGER(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency) :
	I2C("TERARANGER", nullptr, bus, TERARANGER_ONE_BASEADDR, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_rangefinder(0 /* device id not yet used */, ORB_PRIO_DEFAULT, rotation)
{
	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;
}

TERARANGER::~TERARANGER()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TERARANGER::collect()
{
	if (!_collect_phase) {
		return measure();
	}

	perf_begin(_sample_perf);

	// Transfer data from the bus.
	uint8_t val[3] {};
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret_val = transfer(nullptr, 0, &val[0], 3);

	if (ret_val < 0) {
		PX4_ERR("error reading from sensor: %d", ret_val);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret_val;
	}

	uint16_t distance_mm = (val[0] << 8) | val[1];
	float distance_m = static_cast<float>(distance_mm) * 1e-3f;

	if (crc8(val, 2) == val[2]) {
		_px4_rangefinder.update(timestamp_sample, distance_m);

	}

	// Next phase is measurement.
	_collect_phase = false;

	perf_count(_sample_perf);
	perf_end(_sample_perf);

	return PX4_OK;
}

int TERARANGER::init()
{
	int32_t hw_model = 0;
	param_get(param_find("SENS_EN_TRANGER"), &hw_model);

	switch (hw_model) {
	case 0: // Disabled
		PX4_WARN("Disabled");
		return PX4_ERROR;

	case 1: // Autodetect - assume default Teraranger One
		set_device_address(TERARANGER_ONE_BASEADDR);

		if (I2C::init() != OK) {
			set_device_address(TERARANGER_EVO_BASEADDR);

			if (I2C::init() != OK) {
				return PX4_ERROR;

			} else {
				// Assume minimum and maximum possible distances acros Evo family
				_px4_rangefinder.set_min_distance(TERARANGER_EVO_3M_MIN_DISTANCE);
				_px4_rangefinder.set_max_distance(TERARANGER_EVO_60M_MAX_DISTANCE);
			}

		} else {
			_px4_rangefinder.set_min_distance(TERARANGER_ONE_MIN_DISTANCE);
			_px4_rangefinder.set_max_distance(TERARANGER_ONE_MAX_DISTANCE);
		}

		break;

	case 2: // Teraranger One.
		set_device_address(TERARANGER_ONE_BASEADDR);

		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

		_px4_rangefinder.set_min_distance(TERARANGER_ONE_MIN_DISTANCE);
		_px4_rangefinder.set_max_distance(TERARANGER_ONE_MAX_DISTANCE);
		break;

	case 3: // Teraranger Evo60m.
		set_device_address(TERARANGER_EVO_BASEADDR);

		// I2C init (and probe) first.
		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

		_px4_rangefinder.set_min_distance(TERARANGER_EVO_60M_MIN_DISTANCE);
		_px4_rangefinder.set_max_distance(TERARANGER_EVO_60M_MAX_DISTANCE);
		break;

	case 4: // Teraranger Evo600Hz.
		set_device_address(TERARANGER_EVO_BASEADDR);

		// I2C init (and probe) first.
		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

		_px4_rangefinder.set_min_distance(TERARANGER_EVO_600HZ_MIN_DISTANCE);
		_px4_rangefinder.set_max_distance(TERARANGER_EVO_600HZ_MAX_DISTANCE);
		break;

	case 5: // Teraranger Evo3m.
		set_device_address(TERARANGER_EVO_BASEADDR);

		// I2C init (and probe) first.
		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

		_px4_rangefinder.set_min_distance(TERARANGER_EVO_3M_MIN_DISTANCE);
		_px4_rangefinder.set_max_distance(TERARANGER_EVO_3M_MAX_DISTANCE);
		break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int TERARANGER::measure()
{
	// Send the command to begin a measurement.
	const uint8_t cmd = TERARANGER_MEASURE_REG;
	int ret_val = transfer(&cmd, sizeof(cmd), nullptr, 0);

	if (ret_val != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret_val);
		return ret_val;
	}

	_collect_phase = true;
	return PX4_OK;
}

int TERARANGER::probe()
{
	uint8_t who_am_i = 0;

	const uint8_t cmd = TERARANGER_WHO_AM_I_REG;

	// Can't use a single transfer as Teraranger needs a bit of time for internal processing.
	if (transfer(&cmd, 1, nullptr, 0) == OK) {
		if (transfer(nullptr, 0, &who_am_i, 1) == OK && who_am_i == TERARANGER_WHO_AM_I_REG_VAL) {
			return measure();
		}
	}

	PX4_DEBUG("WHO_AM_I byte mismatch 0x%02x should be 0x%02x\n",
		  (unsigned)who_am_i,
		  TERARANGER_WHO_AM_I_REG_VAL);

	// Not found on any address.
	return -EIO;
}

void TERARANGER::RunImpl()
{
	// Perform data collection.
	collect();
}

void TERARANGER::start()
{
	_collect_phase = false;

	// Schedule the driver to run on a set interval
	ScheduleOnInterval(TERARANGER_MEASUREMENT_INTERVAL);
}

void TERARANGER::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	_px4_rangefinder.print_status();
}
