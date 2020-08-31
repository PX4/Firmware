/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file rm3100.cpp
 *
 * Driver for the RM3100 magnetometer connected via I2C or SPI.
 *
 * Based on the lis3mdl driver.
 */

#include "rm3100.h"

RM3100::RM3100(device::Device *interface, enum Rotation rotation, I2CSPIBusOption bus_option, int bus) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_px4_mag(interface->get_device_id(), rotation),
	_interface(interface),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms_errors")),
	_conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_errors")),
	_range_errors(perf_alloc(PC_COUNT, MODULE_NAME": range_errors")),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_continuous_mode_set(false),
	_mode(SINGLE),
	_measure_interval(0),
	_check_state_cnt(0)
{
	_px4_mag.set_external(_interface->external());

	_px4_mag.set_scale(1.f / (RM3100_SENSITIVITY * UTESLA_TO_GAUSS));
}

RM3100::~RM3100()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);

	delete _interface;
}

int RM3100::self_test()
{
	/* Chances are that a poll event was triggered, so wait for conversion and read registers in order to clear DRDY bit */
	usleep(RM3100_CONVERSION_INTERVAL);
	collect();

	/* Fail if calibration is not good */
	int ret = 0;
	uint8_t cmd = 0;

	/* Configure mag into self test mode */
	cmd = BIST_SELFTEST;
	ret = _interface->write(ADDR_BIST, &cmd, 1);

	if (ret != PX4_OK) {
		return ret;
	}

	/* Now we need to write to POLL to launch self test */
	cmd = POLL_XYZ;
	ret = _interface->write(ADDR_POLL, &cmd, 1);

	if (ret != PX4_OK) {
		return ret;
	}

	/* Now wait for status register */
	usleep(RM3100_CONVERSION_INTERVAL);

	if (check_measurement() != PX4_OK) {
		return -1;;
	}

	/* Now check BIST register to see whether self test is ok or not*/
	ret = _interface->read(ADDR_BIST, &cmd, 1);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = !((cmd & BIST_XYZ_OK) == BIST_XYZ_OK);

	return ret;
}

int RM3100::check_measurement()
{
	uint8_t status = 0;
	int ret = _interface->read(ADDR_STATUS, &status, 1);

	if (ret != 0) {
		return ret;
	}

	return !((status & STATUS_DRDY) == STATUS_DRDY) ;
}

int RM3100::collect()
{
	/* Check whether a measurement is available or not, otherwise return immediately */
	if (check_measurement() != 0) {
		PX4_DEBUG("No measurement available");
		return 0;
	}

	struct {
		uint8_t x[3];
		uint8_t y[3];
		uint8_t z[3];
	} rm_report{};

	_px4_mag.set_error_count(perf_event_count(_comms_errors));

	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _interface->read(ADDR_MX, (uint8_t *)&rm_report, sizeof(rm_report));

	if (ret != OK) {
		perf_end(_sample_perf);
		perf_count(_comms_errors);
		return ret;
	}

	perf_end(_sample_perf);

	/* Rearrange mag data */
	int32_t xraw = ((rm_report.x[0] << 16) | (rm_report.x[1] << 8) | rm_report.x[2]);
	int32_t yraw = ((rm_report.y[0] << 16) | (rm_report.y[1] << 8) | rm_report.y[2]);
	int32_t zraw = ((rm_report.z[0] << 16) | (rm_report.z[1] << 8) | rm_report.z[2]);

	/* Convert 24 bit signed values to 32 bit signed values */
	convert_signed(&xraw);
	convert_signed(&yraw);
	convert_signed(&zraw);

	_px4_mag.update(timestamp_sample, xraw, yraw, zraw);

	ret = OK;


	return ret;
}

void RM3100::convert_signed(int32_t *n)
{
	/* Sensor returns values as 24 bit signed values, so we need to manually convert to 32 bit signed values */
	if ((*n & (1 << 23)) == (1 << 23)) {
		*n |= 0xFF000000;
	}
}

void RM3100::RunImpl()
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
		ScheduleDelayed(_measure_interval);
	}
}

int RM3100::init()
{
	/* reset the device configuration */
	reset();

	int ret = self_test();

	if (ret != PX4_OK) {
		PX4_ERR("self test failed");
	}

	_measure_interval = RM3100_CONVERSION_INTERVAL;
	start();

	return ret;
}

int RM3100::measure()
{
	int ret = 0;
	uint8_t cmd = 0;

	/* Send the command to begin a measurement. */
	if ((_mode == CONTINUOUS) && !_continuous_mode_set) {
		cmd = (CMM_DEFAULT | CONTINUOUS_MODE);
		ret = _interface->write(ADDR_CMM, &cmd, 1);
		_continuous_mode_set = true;

	} else if (_mode == SINGLE) {
		if (_continuous_mode_set) {
			/* This is needed for polling mode */
			cmd = (CMM_DEFAULT | POLLING_MODE);
			ret = _interface->write(ADDR_CMM, &cmd, 1);

			if (ret != OK) {
				perf_count(_comms_errors);
				return ret;
			}

			_continuous_mode_set = false;
		}

		cmd = POLL_XYZ;
		ret = _interface->write(ADDR_POLL, &cmd, 1);
	}


	if (ret != OK) {
		perf_count(_comms_errors);
	}

	return ret;
}

void RM3100::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u", _measure_interval);
}

int RM3100::reset()
{
	int ret = set_default_register_values();

	if (ret != OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int RM3100::set_default_register_values()
{
	uint8_t cmd[2] = {0, 0};

	cmd[0] = CCX_DEFAULT_MSB;
	cmd[1] = CCX_DEFAULT_LSB;
	_interface->write(ADDR_CCX, cmd, 2);

	cmd[0] = CCY_DEFAULT_MSB;
	cmd[1] = CCY_DEFAULT_LSB;
	_interface->write(ADDR_CCY, cmd, 2);

	cmd[0] = CCZ_DEFAULT_MSB;
	cmd[1] = CCZ_DEFAULT_LSB;
	_interface->write(ADDR_CCZ, cmd, 2);

	cmd[0] = CMM_DEFAULT;
	_interface->write(ADDR_CMM, cmd, 1);

	cmd[0] = TMRC_DEFAULT;
	_interface->write(ADDR_TMRC, cmd, 1);

	cmd[0] = BIST_DEFAULT;
	_interface->write(ADDR_BIST, cmd, 1);

	return PX4_OK;
}

void RM3100::start()
{
	set_default_register_values();

	/* schedule a cycle to start things */
	ScheduleNow();
}
