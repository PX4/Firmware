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

/**
 * @file lis2mdl.cpp
 *
 * Driver for the LIS2MDL magnetometer connected via I2C or SPI.
 *
 * Based on the LIS2MDL driver.
 */

#include <px4_platform_common/time.h>
#include "lis2mdl.h"

LIS2MDL::LIS2MDL(device::Device *interface, enum Rotation rotation, I2CSPIBusOption bus_option, int bus) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_px4_mag(interface->get_device_id(), rotation),
	_interface(interface),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms_errors")),
	_conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_errors")),
	_range_errors(perf_alloc(PC_COUNT, MODULE_NAME": range_errors")),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_interval(0)
{
	_px4_mag.set_external(_interface->external());
	_px4_mag.set_scale(0.0015f); /* 49.152f / (2^15) */
}

LIS2MDL::~LIS2MDL()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);

	delete _interface;
}

int
LIS2MDL::measure()
{
	struct {
		uint8_t status;
		uint8_t x[2];
		uint8_t y[2];
		uint8_t z[2];
	}       lis_report;

	struct {
		int16_t x;
		int16_t y;
		int16_t z;
		int16_t t;
	} report;

	uint8_t buf_rx[2] = {};

	_px4_mag.set_error_count(perf_event_count(_comms_errors));

	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _interface->read(ADDR_STATUS_REG, (uint8_t *)&lis_report, sizeof(lis_report));

	/**
	 * Silicon Bug: the X axis will be read instead of the temperature registers if you do a sequential read through XYZ.
	 * The temperature registers must be addressed directly.
	 */
	ret = _interface->read(ADDR_OUT_T_L, (uint8_t *)&buf_rx, sizeof(buf_rx));

	if (ret != OK) {
		perf_end(_sample_perf);
		perf_count(_comms_errors);
		return ret;
	}

	perf_end(_sample_perf);

	if ((lis_report.status & (1 << 3)) == 0) { // check data ready
		return 0;
	}

	report.x = (int16_t)((lis_report.x[1] << 8) | lis_report.x[0]);
	report.y = (int16_t)((lis_report.y[1] << 8) | lis_report.y[0]);
	report.z = (int16_t)((lis_report.z[1] << 8) | lis_report.z[0]);

	report.t = (int16_t)((buf_rx[1] << 8) | buf_rx[0]);

	float temperature = report.t;
	_px4_mag.set_temperature(25.0f + (temperature / 8.0f));

	/* swap x and y axis */
	_px4_mag.update(timestamp_sample, report.y, report.x, report.z);

	return PX4_OK;
}

void
LIS2MDL::RunImpl()
{
	/* _measure_interval == 0  is used as _task_should_exit */
	if (_measure_interval == 0) {
		return;
	}

	if (measure() != OK) {
		PX4_DEBUG("measure error");
	}

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(LIS2MDL_CONVERSION_INTERVAL);
}

int
LIS2MDL::init()
{

	int ret = write_reg(ADDR_CFG_REG_A, CFG_REG_A_ODR | CFG_REG_A_MD | CFG_REG_A_TEMP_COMP_EN);
	ret = write_reg(ADDR_CFG_REG_B, 0);
	ret = write_reg(ADDR_CFG_REG_C, CFG_REG_C_BDU);

	_measure_interval = LIS2MDL_CONVERSION_INTERVAL;
	start();

	return ret;
}

void
LIS2MDL::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u", _measure_interval);
}

void
LIS2MDL::start()
{
	/* schedule a cycle to start things */
	ScheduleNow();
}

int
LIS2MDL::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

int
LIS2MDL::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}
