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
 * @file tcbp001ta.cpp
 *
 * Driver for TCBP001TA barometer.
 *
 * @author Xiaowei Zhao <xiaowei_zhao1013@163.com>
 * @author Stone White <stone@thone.io>
 *
 */

#include "tcbp001ta.hpp"

TCBP001TA::TCBP001TA(tcbp001ta::ITCBP001TA *interface) :
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id())),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
	_px4_baro.set_device_type(DRV_BARO_DEVTYPE_TCBP001TA);
}

TCBP001TA::~TCBP001TA()
{
	// make sure we are truly inactive
	Stop();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
TCBP001TA::init()
{
	// reset sensor
	_interface->set_reg(TCBP001TA_VALUE_RESET, TCBP001TA_ADDR_RESET);
	usleep(10000);

	// check id
	if (_interface->get_reg(TCBP001TA_ADDR_ID) != TCBP001TA_VALUE_ID) {
		PX4_WARN("id of your baro is not: 0x%02x", TCBP001TA_VALUE_ID);
		return -EIO;
	}

	// Presure  write [0x6 | (0x3 << 4)] to 0x06
	_interface->set_reg(TCBP001TA_PRS_TMP_PRC_64 | TCBP001TA_PRS_TMP_RATE_8, TCBP001TA_ADDR_PRS_CFG);

	// Temperature  write [0x80 | 0x1 | (0x2 << 4)] to 0x07
	_interface->set_reg(TCBP001TA_TMP_EXT_MEMS | TCBP001TA_PRS_TMP_PRC_2 | TCBP001TA_PRS_TMP_RATE_4,
			    TCBP001TA_ADDR_TMP_CFG);

	// write 0x00 to 0x09
	_interface->set_reg(0x00, TCBP001TA_ADDR_CFG_REG);

	tmp_osr_scale_coeff = TCBP001TA_get_scaling_coef(0);
	prs_osr_scale_coeff = TCBP001TA_get_scaling_coef(0);

	// get calibration and pre process them
	_cal = _interface->get_calibration(TCBP001TA_ADDR_CAL);

	if (_cal == nullptr) {
		PX4_INFO("init failed");
		return -EIO;
	}

	_fcal.c0 = (_cal->c0_h << 4) + ((_cal->c0l_1h >> 4) & 0x0F);

	if (_fcal.c0 > POW_2_11_MINUS_1) {
		_fcal.c0 = _fcal.c0 - POW_2_12;
	}

	_fcal.c1 = (_cal->c1l + ((_cal->c0l_1h & 0x0F) << 8));

	if (_fcal.c1 > POW_2_11_MINUS_1) {
		_fcal.c1 = _fcal.c1 - POW_2_12;
	}

	_fcal.c00 = ((_cal->c00m << 4) + (_cal->c00h << 12)) + ((_cal->c00l_10h >> 4) & 0x0F);

	if (_fcal.c00 > POW_2_19_MINUS_1) {
		_fcal.c00 = _fcal.c00 - POW_2_20;
	}

	_fcal.c10 = ((_cal->c00l_10h & 0x0F) << 16) + _cal->c10l + (_cal->c10m << 8);

	if (_fcal.c10 > POW_2_19_MINUS_1) {
		_fcal.c10 = _fcal.c10 - POW_2_20;
	}

	_fcal.c01 = (_cal->c01l + (_cal->c01h << 8));
	//if(_fcal.c01 > POW_2_15_MINUS_1)
	//_fcal.c01 = _fcal.c01 - POW_2_16;

	_fcal.c11 = (_cal->c11l + (_cal->c11h << 8));
	//if(_fcal.c11 > POW_2_15_MINUS_1)
	//_fcal.c11 = _fcal.c11 - POW_2_16;

	_fcal.c20 = (_cal->c20l + (_cal->c20h << 8));
	//if(_fcal.c20 > POW_2_15_MINUS_1)
	//_fcal.c20 = _fcal.c20 - POW_2_16;

	_fcal.c21 = (_cal->c21l + (_cal->c21h << 8));
	//if(_fcal.c21 > POW_2_15_MINUS_1)
	//_fcal.c21 = _fcal.c21 - POW_2_16;

	_fcal.c30 = (_cal->c30l + (_cal->c30h << 8));
	//if(_fcal.c30 > POW_2_15_MINUS_1)
	//_fcal.c30 = _fcal.c30 - POW_2_16;

	Start();

	return OK;
}

void
TCBP001TA::Start()
{
	// reset the report ring and state machine
	_collect_phase = false;

	// schedule a cycle to start things
	ScheduleNow();
}

void
TCBP001TA::Stop()
{
	ScheduleClear();
}

void
TCBP001TA::Run()
{
	if (_collect_phase) {
		collect();

	} else {
		measure();
	}

	ScheduleDelayed(_measure_interval);
}

int
TCBP001TA::measure()
{
	perf_begin(_measure_perf);

	_collect_phase = true;

	// start measure
	//int ret = _interface->set_reg(_curr_ctrl | TCBP001TA_CTRL_MODE_FORCE, TCBP001TA_ADDR_CTRL);

	//if (ret != OK) {
	//perf_count(_comms_errors);
	//perf_cancel(_measure_perf);
	//return -EIO;
	//}
	perf_end(_measure_perf);

	return OK;
}

int
TCBP001TA::collect()
{
	perf_begin(_sample_perf);

	_collect_phase = false;
	// this should be fairly close to the end of the conversion, so the best approximation of the time
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	//write 0x02 to reg 0x08  Temperature
	_interface->set_reg(0x02, TCBP001TA_ADDR_MEAS_CFG);

	tcbp001ta::data_s *data_temp = _interface->get_data(TCBP001TA_ADDR_TMP_DATA);

	if (data_temp == nullptr) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	uint32_t p_raw = (data_temp->p_msb << 16 | data_temp->p_lsb << 8) + (data_temp->p_xlsb);

	//write 0x01 to reg 0x08  Pressure
	_interface->set_reg(0x01, TCBP001TA_ADDR_MEAS_CFG);

	tcbp001ta::data_s *data_pres = _interface->get_data(TCBP001TA_ADDR_PRS_DATA);

	if (data_pres == nullptr) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	uint32_t t_raw = (data_pres->t_msb << 16 | data_pres->t_lsb << 8) + (data_pres->t_xlsb);

	// PX4_INFO("p raw %f", p_raw);
	// PX4_INFO("t raw %f", t_raw);
	// PX4_INFO("data 0 %d data 1 %d data2 %d", data_temp->p_msb, data_temp->p_lsb, data_temp->p_xlsb);
	// PX4_INFO("data 3 %d data 4 %d data5 %d", data_pres->t_msb, data_pres->t_lsb, data_pres->t_xlsb);

	// Temperature
	float Traw_sc = (float)t_raw / (float)(tmp_osr_scale_coeff);

	const float T = (_fcal.c0 / 2.0f) + (float)(_fcal.c1 * Traw_sc);

	// Pressure
	float Praw_sc = (float) p_raw / (float)(prs_osr_scale_coeff);

	const float P = _fcal.c00 +
			Praw_sc * (_fcal.c10 + Praw_sc * (_fcal.c20 + Praw_sc * _fcal.c30)) +
			Praw_sc * _fcal.c01 +
			Praw_sc * Praw_sc * (_fcal.c11 + Praw_sc * _fcal.c21);

	_px4_baro.set_error_count(perf_event_count(_comms_errors));
	_px4_baro.set_temperature(T);

	float pressure = P / 100.0f; // to mbar
	// PX4_INFO("press %f", double(pressure));
	_px4_baro.update(timestamp_sample, pressure);

	perf_end(_sample_perf);

	return OK;
}

void
TCBP001TA::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_perf);
	perf_print_counter(_comms_errors);

	_px4_baro.print_status();
}

uint32_t
TCBP001TA::TCBP001TA_get_scaling_coef(uint8_t osr)
{
	uint32_t scaling_coeff;

	switch (osr) {

	case 0:
		scaling_coeff = 524288;
		break;

	case 1:
		scaling_coeff = 1572864;
		break;

	case 2:
		scaling_coeff = 3670016;
		break;

	case 3:
		scaling_coeff = 7864320;
		break;

	case 4:
		scaling_coeff = 253952;
		break;

	case 5:
		scaling_coeff = 516096;
		break;

	case 6:
		scaling_coeff = 1040384;
		break;

	case 7:
		scaling_coeff = 2088960;
		break;

	default:
		scaling_coeff = 524288;
		break;
	}

	return scaling_coeff;
}
