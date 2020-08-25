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

int
SDP3X::probe()
{
	return !init_sdp3x();
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
	int ret;

	if (get_device_address() == I2C_ADDRESS_1_SDP3X) { // since we are broadcasting, only do it for the first device address
		// step 1 - reset on broadcast
		uint16_t prev_addr = get_device_address();
		set_device_address(SDP3X_RESET_ADDR);
		uint8_t reset_cmd = SDP3X_RESET_CMD;
		ret = transfer(&reset_cmd, 1, nullptr, 0);
		set_device_address(prev_addr);

		if (ret != PX4_OK) {
			perf_count(_comms_errors);
			return false;
		}

		// wait until sensor is ready
		px4_usleep(20000);
	}

	// step 2 - configure
	ret = write_command(SDP3X_CONT_MEAS_AVG_MODE);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_ERR("config failed");
		return false;
	}

	px4_usleep(10000);

	// step 3 - get scale
	uint8_t val[9];
	ret = transfer(nullptr, 0, &val[0], sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_ERR("get scale failed");
		return false;
	}

	// Check the CRC
	if (!crc(&val[0], 2, val[2]) || !crc(&val[3], 2, val[5]) || !crc(&val[6], 2, val[8])) {
		perf_count(_comms_errors);
		return false;
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

	return true;
}

int
SDP3X::collect()
{
	perf_begin(_sample_perf);

	// read 9 bytes from the sensor
	uint8_t val[6];
	int ret = transfer(nullptr, 0, &val[0], sizeof(val));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	// Check the CRC
	if (!crc(&val[0], 2, val[2]) || !crc(&val[3], 2, val[5])) {
		perf_count(_comms_errors);
		return EAGAIN;

	} else {
		ret = 0;
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

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

void
SDP3X::RunImpl()
{
	int ret = PX4_ERROR;

	// measurement phase
	ret = collect();

	if (PX4_OK != ret) {
		_sensor_ok = false;
		DEVICE_DEBUG("measure error");
	}

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(CONVERSION_INTERVAL);
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
