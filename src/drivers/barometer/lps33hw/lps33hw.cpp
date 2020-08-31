/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "lps33hw.hpp"

using namespace ST_LPS33HW;

namespace lps33hw
{

template<typename T>
static void getTwosComplement(T &raw, uint8_t length)
{
	if (raw & ((T)1 << (length - 1))) {
		raw -= (T)1 << length;
	}
}

LPS33HW::LPS33HW(I2CSPIBusOption bus_option, int bus, device::Device *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus,
		     interface->get_device_address()),
	_px4_barometer(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors"))
{
}

LPS33HW::~LPS33HW()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
LPS33HW::init()
{
	uint8_t who_am_i;
	int ret = RegisterRead(Register::WHO_AM_I, who_am_i);

	if (ret != 0) {
		PX4_DEBUG("read failed (%i)", ret);
		return ret;
	}

	if (who_am_i != WHO_AM_I_VALUE) {
		PX4_DEBUG("WHO_AM_I mismatch (%i)", who_am_i);
		return PX4_ERROR;
	}

	ret = reset();

	if (ret != OK) {
		PX4_DEBUG("reset failed");
		return ret;
	}

	start();

	return PX4_OK;
}

int
LPS33HW::reset()
{
	// Soft Reset
	int ret = RegisterWrite(Register::CTRL_REG2, SWRESET);

	// wait until SWRESET goes back to 0
	int i;

	for (i = 0; i < 10; ++i) {
		uint8_t val;
		ret = RegisterRead(Register::CTRL_REG2, val);

		if (ret == 0 && (val & SWRESET) == 0) {
			break;
		}

		usleep(1000);
	}

	if (i >= 10) {
		PX4_ERR("reset failed");
		return PX4_ERROR;
	}

	// Configure sampling rate
	ret = RegisterWrite(Register::CTRL_REG1, ODR_75HZ | BDU);

	return ret;
}

void
LPS33HW::start()
{
	ScheduleOnInterval(1000000 / SAMPLE_RATE);
}

void
LPS33HW::RunImpl()
{
	perf_begin(_sample_perf);

	uint8_t data[6];

	if (_interface->read((uint8_t)Register::STATUS, data, sizeof(data)) != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return;
	}

	uint8_t status = data[0];

	if ((status & P_DA) == 0) { // check if pressure data is available
		perf_end(_sample_perf);
		return;
	}

	hrt_abstime timestamp_sample = hrt_absolute_time();
	float temp = ((int16_t)data[4] | (data[5] << 8)) / 100.f;

	int32_t Praw = (int32_t)data[1] | (data[2] << 8) | (data[3] << 16);
	getTwosComplement(Praw, 24);
	float pressure_hPa = Praw / 4096.f;

	_px4_barometer.set_error_count(perf_event_count(_comms_errors));
	_px4_barometer.set_temperature(temp);
	_px4_barometer.update(timestamp_sample, pressure_hPa); // hPascals -> Millibar

	perf_end(_sample_perf);
}

int
LPS33HW::RegisterRead(Register reg, uint8_t &val)
{
	return _interface->read((uint8_t)reg, &val, 1);
}

int
LPS33HW::RegisterWrite(Register reg, uint8_t value)
{
	return _interface->write((uint8_t)reg, &value, 1);
}

void
LPS33HW::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

} // namespace lps33hw
