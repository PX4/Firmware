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
 * @file lps22hb.cpp
 *
 * Driver for the LPS22HB barometer connected via I2C or SPI.
 */

#include "LPS22HB.hpp"

/* Max measurement rate is 25Hz */
#define LPS22HB_CONVERSION_INTERVAL	(1000000 / 25)	/* microseconds */

LPS22HB::LPS22HB(I2CSPIBusOption bus_option, int bus, device::Device *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

LPS22HB::~LPS22HB()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int LPS22HB::init()
{
	if (reset() != OK) {
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

void LPS22HB::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

int LPS22HB::reset()
{
	return write_reg(CTRL_REG2, BOOT | SWRESET);
}

void LPS22HB::RunImpl()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(LPS22HB_CONVERSION_INTERVAL);
}

int LPS22HB::measure()
{
	// Send the command to begin a 16-bit measurement.
	int ret = write_reg(CTRL_REG2, IF_ADD_INC | ONE_SHOT);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int LPS22HB::collect()
{
	perf_begin(_sample_perf);

	/* get measurements from the device : MSB enables register address auto-increment */
	struct {
		uint8_t		STATUS;
		uint8_t		PRESS_OUT_XL;
		uint8_t		PRESS_OUT_L;
		uint8_t		PRESS_OUT_H;
		uint8_t		TEMP_OUT_L;
		uint8_t		TEMP_OUT_H;
	} report{};

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _interface->read(STATUS, (uint8_t *)&report, sizeof(report));

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint32_t TEMP_OUT = report.TEMP_OUT_L + (report.TEMP_OUT_H << 8);
	float temperature = 42.5f + (TEMP_OUT / 480.0f);
	_px4_baro.set_temperature(temperature);

	// To obtain the pressure in hPa, take the two’s complement of the complete word and then divide by 4096 LSB/hPa.
	uint32_t P = report.PRESS_OUT_XL + (report.PRESS_OUT_L << 8) + (report.PRESS_OUT_H << 16);

	/* Pressure and MSL in mBar */
	float pressure = P / 4096.0f;
	_px4_baro.update(timestamp_sample, pressure);

	perf_end(_sample_perf);
	return PX4_OK;
}

int LPS22HB::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int LPS22HB::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void LPS22HB::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
