/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

#include "LPS25H.hpp"

LPS25H::LPS25H(I2CSPIBusOption bus_option, int bus, device::Device *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_px4_barometer(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms_errors"))
{
	_interface->set_device_type(DRV_BARO_DEVTYPE_LPS25H);
	_px4_barometer.set_device_type(DRV_BARO_DEVTYPE_LPS25H);
}

LPS25H::~LPS25H()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int LPS25H::init()
{
	if (reset() != OK) {
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

void LPS25H::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

int LPS25H::reset()
{
	// Power on
	int ret = write_reg(ADDR_CTRL_REG1, CTRL_REG1_PD);
	usleep(1000);

	// Reset
	ret = write_reg(ADDR_CTRL_REG2, CTRL_REG2_BOOT | CTRL_REG2_SWRESET);
	usleep(5000);

	// Power on
	ret = write_reg(ADDR_CTRL_REG1, CTRL_REG1_PD);
	usleep(1000);

	return ret;
}

void LPS25H::RunImpl()
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

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_interval > LPS25H_CONVERSION_INTERVAL) {
			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - LPS25H_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(LPS25H_CONVERSION_INTERVAL);
}

int LPS25H::measure()
{
	/*
	 * Send the command to begin a 16-bit measurement.
	 */
	int ret = write_reg(ADDR_CTRL_REG2, CTRL_REG2_ONE_SHOT);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int LPS25H::collect()
{
	perf_begin(_sample_perf);

	struct {
		uint8_t		status;
		uint8_t		p_xl;
		uint8_t		p_l;
		uint8_t		p_h;
		int16_t		t;
	} report{};

	/* get measurements from the device : MSB enables register address auto-increment */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = _interface->read(ADDR_STATUS_REG | (1 << 7), (uint8_t *)&report, sizeof(report));

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	_px4_barometer.set_error_count(perf_event_count(_comms_errors));

	/* get measurements from the device */
	float temperature = 42.5f + (report.t / 480);
	_px4_barometer.set_temperature(temperature);

	/* raw pressure */
	uint32_t raw = report.p_xl + (report.p_l << 8) + (report.p_h << 16);

	/* Pressure and MSL in mBar */
	float pressure = raw / 4096.0f;

	_px4_barometer.update(timestamp_sample, pressure);

	perf_end(_sample_perf);
	return PX4_OK;
}

int LPS25H::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int LPS25H::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void LPS25H::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	_px4_barometer.print_status();
}
