/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file lis3mdl.cpp
 *
 * Driver for the LIS3MDL magnetometer connected via I2C or SPI.
 *
 * Based on the hmc5883 driver.
 */

#include <px4_platform_common/time.h>
#include "lis3mdl.h"

LIS3MDL::LIS3MDL(device::Device *interface, enum Rotation rotation, I2CSPIBusOption bus_option, int bus) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_px4_mag(interface->get_device_id(), rotation),
	_interface(interface),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms_errors")),
	_conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_errors")),
	_range_errors(perf_alloc(PC_COUNT, MODULE_NAME": range_errors")),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_continuous_mode_set(false),
	_mode(CONTINUOUS),
	_measure_interval(0),
	_range_ga(4.0f),
	_check_state_cnt(0),
	_cntl_reg1(
		CNTL_REG1_DEFAULT),  // 1 11 111 0 0 | temp-en, ultra high performance (XY), fast_odr disabled, self test disabled
	_cntl_reg2(CNTL_REG2_DEFAULT),  // 4 gauss FS range, reboot settings default
	_cntl_reg3(CNTL_REG3_DEFAULT),  // operating mode CONTINUOUS!
	_cntl_reg4(CNTL_REG4_DEFAULT),  // Z-axis ultra high performance mode
	_cntl_reg5(CNTL_REG5_DEFAULT),  // fast read disabled, continious update disabled (block data update)
	_range_bits(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{
	_px4_mag.set_external(_interface->external());
}

LIS3MDL::~LIS3MDL()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);

	delete _interface;
}

int LIS3MDL::collect()
{
	struct {
		uint8_t x[2];
		uint8_t y[2];
		uint8_t z[2];
	} lis_report{};

	struct {
		int16_t x;
		int16_t y;
		int16_t z;
		int16_t t;
	} report{};

	uint8_t buf_rx[2] {};

	_px4_mag.set_error_count(perf_event_count(_comms_errors));

	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	_interface->read(ADDR_OUT_X_L, (uint8_t *)&lis_report, sizeof(lis_report));

	/**
	 * Silicon Bug: the X axis will be read instead of the temperature registers if you do a sequential read through XYZ.
	 * The temperature registers must be addressed directly.
	 */

	int ret = _interface->read(ADDR_OUT_T_L, (uint8_t *)&buf_rx, sizeof(buf_rx));

	if (ret != OK) {
		perf_end(_sample_perf);
		perf_count(_comms_errors);
		return ret;
	}

	perf_end(_sample_perf);

	report.x = (int16_t)((lis_report.x[1] << 8) | lis_report.x[0]);
	report.y = (int16_t)((lis_report.y[1] << 8) | lis_report.y[0]);
	report.z = (int16_t)((lis_report.z[1] << 8) | lis_report.z[0]);

	report.t = (int16_t)((buf_rx[1] << 8) | buf_rx[0]);

	float temperature = 25.0f + (report.t / 8.0f);
	_px4_mag.set_temperature(temperature);

	_px4_mag.update(timestamp_sample, report.x, report.y, report.z);

	return PX4_OK;
}

void LIS3MDL::RunImpl()
{
	/* _measure_interval == 0  is used as _task_should_exit */
	if (_measure_interval == 0) {
		return;
	}

	/* Collect last measurement at the start of every cycle */
	if (collect() != OK) {
		PX4_DEBUG("collection error");
		/* restart the measurement state machine */
		start();
		return;
	}

	if (measure() != OK) {
		PX4_DEBUG("measure error");
	}

	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(LIS3MDL_CONVERSION_INTERVAL);
	}
}

int LIS3MDL::init()
{
	/* reset the device configuration */
	reset();

	_measure_interval = LIS3MDL_CONVERSION_INTERVAL;
	start();

	return PX4_OK;
}

int LIS3MDL::measure()
{
	int ret = 0;

	/* Send the command to begin a measurement. */
	if ((_mode == CONTINUOUS) && !_continuous_mode_set) {
		ret = write_reg(ADDR_CTRL_REG3, MODE_REG_CONTINOUS_MODE);
		_continuous_mode_set = true;

	} else if (_mode == SINGLE) {
		ret = write_reg(ADDR_CTRL_REG3, MODE_REG_SINGLE_MODE);
		_continuous_mode_set = false;
	}

	if (ret != OK) {
		perf_count(_comms_errors);
	}

	return ret;
}

void LIS3MDL::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u", _measure_interval);
}

int LIS3MDL::reset()
{
	int ret = set_default_register_values();

	if (ret != OK) {
		return PX4_ERROR;
	}

	ret = set_range(_range_ga);

	if (ret != OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
LIS3MDL::set_default_register_values()
{
	write_reg(ADDR_CTRL_REG1, CNTL_REG1_DEFAULT);
	write_reg(ADDR_CTRL_REG2, CNTL_REG2_DEFAULT);
	write_reg(ADDR_CTRL_REG3, CNTL_REG3_DEFAULT);
	write_reg(ADDR_CTRL_REG4, CNTL_REG4_DEFAULT);
	write_reg(ADDR_CTRL_REG5, CNTL_REG5_DEFAULT);

	return PX4_OK;
}

int LIS3MDL::set_range(unsigned range)
{
	if (range <= 4) {
		_range_bits = 0x00;
		_px4_mag.set_scale(1.0f / 6842.0f);
		_range_ga = 4.0f;

	} else if (range <= 8) {
		_range_bits = 0x01;
		_px4_mag.set_scale(1.0f / 3421.0f);
		_range_ga = 8.0f;

	} else if (range <= 12) {
		_range_bits = 0x02;
		_px4_mag.set_scale(1.0f / 2281.0f);
		_range_ga = 12.0f;

	} else {
		_range_bits = 0x03;
		_px4_mag.set_scale(1.0f / 1711.0f);
		_range_ga = 16.0f;
	}

	/*
	 * Send the command to set the range
	 */
	int ret = write_reg(ADDR_CTRL_REG2, (_range_bits << 5));

	if (ret != OK) {
		perf_count(_comms_errors);
	}

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CTRL_REG2, range_bits_in);

	if (ret != OK) {
		perf_count(_comms_errors);
	}

	if (range_bits_in == (_range_bits << 5)) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

void LIS3MDL::start()
{
	set_default_register_values();

	/* schedule a cycle to start things */
	ScheduleNow();
}

int LIS3MDL::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

int LIS3MDL::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}
