/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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

#include "BMP280.hpp"

BMP280::BMP280(I2CSPIBusOption bus_option, int bus, bmp280::IBMP280 *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus,
		     interface->get_device_address()),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

BMP280::~BMP280()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
BMP280::init()
{
	// reset sensor
	_interface->set_reg(BMP280_VALUE_RESET, BMP280_ADDR_RESET);
	usleep(10000);

	// check id
	if (_interface->get_reg(BMP280_ADDR_ID) != BMP280_VALUE_ID) {
		PX4_DEBUG("id of your baro is not: 0x%02x", BMP280_VALUE_ID);
		return -EIO;
	}

	// set config, recommended settings
	_interface->set_reg(_curr_ctrl, BMP280_ADDR_CTRL);
	_interface->set_reg(BMP280_CONFIG_F16, BMP280_ADDR_CONFIG);

	// get calibration and pre process them
	_cal = _interface->get_calibration(BMP280_ADDR_CAL);

	_fcal.t1 = _cal->t1 * powf(2,  4);
	_fcal.t2 = _cal->t2 * powf(2, -14);
	_fcal.t3 = _cal->t3 * powf(2, -34);

	_fcal.p1 = _cal->p1            * (powf(2,  4) / -100000.0f);
	_fcal.p2 = _cal->p1 * _cal->p2 * (powf(2, -31) / -100000.0f);
	_fcal.p3 = _cal->p1 * _cal->p3 * (powf(2, -51) / -100000.0f);

	_fcal.p4 = _cal->p4 * powf(2,  4) - powf(2, 20);
	_fcal.p5 = _cal->p5 * powf(2, -14);
	_fcal.p6 = _cal->p6 * powf(2, -31);

	_fcal.p7 = _cal->p7 * powf(2, -4);
	_fcal.p8 = _cal->p8 * powf(2, -19) + 1.0f;
	_fcal.p9 = _cal->p9 * powf(2, -35);

	Start();

	return OK;
}

void
BMP280::Start()
{
	// reset the report ring and state machine
	_collect_phase = false;

	// schedule a cycle to start things
	ScheduleNow();
}

void
BMP280::RunImpl()
{
	if (_collect_phase) {
		collect();

	} else {
		measure();
	}

	ScheduleDelayed(_measure_interval);
}

int
BMP280::measure()
{
	perf_begin(_measure_perf);

	_collect_phase = true;

	// start measure
	int ret = _interface->set_reg(_curr_ctrl | BMP280_CTRL_MODE_FORCE, BMP280_ADDR_CTRL);

	if (ret != OK) {
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int
BMP280::collect()
{
	perf_begin(_sample_perf);

	_collect_phase = false;

	// this should be fairly close to the end of the conversion, so the best approximation of the time
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	bmp280::data_s *data = _interface->get_data(BMP280_ADDR_DATA);

	if (data == nullptr) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	// convert data to number 20 bit
	uint32_t p_raw = data->p_msb << 12 | data->p_lsb << 4 | data->p_xlsb >> 4;
	uint32_t t_raw = data->t_msb << 12 | data->t_lsb << 4 | data->t_xlsb >> 4;

	// Temperature
	float ofs = (float) t_raw - _fcal.t1;
	float t_fine = (ofs * _fcal.t3 + _fcal.t2) * ofs;
	const float T = t_fine * (1.0f / 5120.0f);

	// Pressure
	float tf = t_fine - 128000.0f;
	float x1 = (tf * _fcal.p6 + _fcal.p5) * tf + _fcal.p4;
	float x2 = (tf * _fcal.p3 + _fcal.p2) * tf + _fcal.p1;

	float pf = ((float) p_raw + x1) / x2;
	const float P = (pf * _fcal.p9 + _fcal.p8) * pf + _fcal.p7;

	_px4_baro.set_error_count(perf_event_count(_comms_errors));
	_px4_baro.set_temperature(T);

	float pressure = P / 100.0f; // to mbar
	_px4_baro.update(timestamp_sample, pressure);

	perf_end(_sample_perf);

	return OK;
}

void
BMP280::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_perf);
	perf_print_counter(_comms_errors);
}
