/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file SDP3X.hpp
 *
 * Driver for Sensirion SDP3X Differential Pressure Sensor
 *
 */

#include "SDP3X.hpp"

using namespace time_literals;

int
SDP3X::probe()
{
	bool require_initialization = !init_sdp3x();

	if (require_initialization && _keep_retrying) {
		PX4_INFO("no sensor found, but will keep retrying");
		return 0;
	}

	return require_initialization ? -1 : 0;
}

int SDP3X::write_command(uint16_t command)
{
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(command >> 8);
	cmd[1] = static_cast<uint8_t>(command & 0xff);
	return transfer(&cmd[0], 2, nullptr, 0);
}

bool
SDP3X::init_sdp3x()
{
	return configure() == 0;
}

int
SDP3X::configure()
{
	int ret = write_command(SDP3X_CONT_MEAS_AVG_MODE);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("config failed");
		_state = State::RequireConfig;
		return ret;
	}

	_state = State::Configuring;

	return ret;
}

int
SDP3X::read_scale()
{
	// get scale
	uint8_t val[9];
	int ret = transfer(nullptr, 0, &val[0], sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_ERR("get scale failed");
		return ret;
	}

	// Check the CRC
	if (!crc(&val[0], 2, val[2]) || !crc(&val[3], 2, val[5]) || !crc(&val[6], 2, val[8])) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	_scale = (((uint16_t)val[6]) << 8) | val[7];

	switch (_scale) {
	case SDP3X_SCALE_PRESSURE_SDP31:
		_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SDP31;
		break;

	case SDP3X_SCALE_PRESSURE_SDP32:
		_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SDP32;
		break;

	case SDP3X_SCALE_PRESSURE_SDP33:
		_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SDP33;
		break;
	}

	return PX4_OK;
}

void SDP3X::start()
{
	// make sure to wait 10ms after configuring the measurement mode
	ScheduleDelayed(10_ms);
}

int
SDP3X::collect()
{
	perf_begin(_sample_perf);

	// read 6 bytes from the sensor
	uint8_t val[6];
	int ret = transfer(nullptr, 0, &val[0], sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Check the CRC
	if (!crc(&val[0], 2, val[2]) || !crc(&val[3], 2, val[5])) {
		perf_count(_comms_errors);
		return -EAGAIN;
	}

	int16_t P = (((int16_t)val[0]) << 8) | val[1];
	int16_t temp = (((int16_t)val[3]) << 8) | val[4];

	float diff_press_pa_raw = static_cast<float>(P) / static_cast<float>(_scale);
	float temperature_c = temp / static_cast<float>(SDP3X_SCALE_TEMPERATURE);

	differential_pressure_s report{};

	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.temperature = temperature_c;
	report.differential_pressure_filtered_pa = _filter.apply(diff_press_pa_raw) - _diff_pres_offset;
	report.differential_pressure_raw_pa = diff_press_pa_raw - _diff_pres_offset;
	report.device_id = _device_id.devid;

	_airspeed_pub.publish(report);

	perf_end(_sample_perf);

	return ret;
}

void
SDP3X::RunImpl()
{
	switch (_state) {
	case State::RequireConfig:
		if (configure() == PX4_OK) {
			ScheduleDelayed(10_ms);

		} else {
			// periodically retry to configure
			ScheduleDelayed(300_ms);
		}

		break;

	case State::Configuring:
		if (read_scale() == 0) {
			_state = State::Running;

		} else {
			_state = State::RequireConfig;
		}

		ScheduleDelayed(10_ms);
		break;

	case State::Running:
		int ret = collect();

		if (ret != 0 && ret != -EAGAIN) {
			_sensor_ok = false;
			DEVICE_DEBUG("measure error");
			_state = State::RequireConfig;
		}

		ScheduleDelayed(CONVERSION_INTERVAL);
		break;
	}
}

bool SDP3X::crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
	uint8_t crc_value = 0xff;

	// calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
	for (unsigned i = 0; i < size; i++) {
		crc_value ^= (data[i]);

		for (int bit = 8; bit > 0; --bit) {
			if (crc_value & 0x80) {
				crc_value = (crc_value << 1) ^ 0x31;

			} else {
				crc_value = (crc_value << 1);
			}
		}
	}

	// verify checksum
	return (crc_value == checksum);
}
