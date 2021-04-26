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

#include "SRF02.hpp"

SRF02::SRF02(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency, int address) :
	I2C(DRV_DIST_DEVTYPE_SRF02, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_rangefinder(get_device_id(), rotation)
{
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_SRF02);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND);
	_px4_rangefinder.set_max_distance(SRF02_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(SRF02_MIN_DISTANCE);
}

SRF02::~SRF02()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

void SRF02::start()
{
	// Reset the report ring and state machine.
	_collect_phase = false;

	// Schedule a cycle to start things.
	ScheduleDelayed(5);
}

int SRF02::init()
{
	// I2C init (and probe) first.
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// XXX we should find out why we need to wait 200 ms here
	px4_usleep(200000);

	return measure();
}

int SRF02::collect()
{
	// Read from the sensor.
	uint8_t val[2] {};
	uint8_t cmd = 0x02;
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = transfer(&cmd, 1, nullptr, 0);
	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = float(distance_cm) * 1e-2f;

	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);
	return PX4_OK;
}

int SRF02::measure()
{
	uint8_t cmd[2];
	cmd[0] = 0x00;
	cmd[1] = SRF02_TAKE_RANGE_REG;

	// Send the command to begin a measurement.
	int ret = transfer(cmd, 2, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

void SRF02::RunImpl()
{
	if (_collect_phase) {
		// Perform collection.
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			// If error restart the measurement state machine.
			start();
			return;
		}

		// Next phase is measurement.
		_collect_phase = false;
	}

	// Perform measurement.
	if (OK != measure()) {
		PX4_DEBUG("measure error sonar adress");
	}

	// Next phase is collection.
	_collect_phase = true;

	// Schedule a fresh cycle call when the measurement is done.
	ScheduleDelayed(_interval);
}

void SRF02::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
