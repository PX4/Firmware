/****************************************************************************
 *
 *   Copyright (C) 2016 Intel Corporation. All rights reserved.
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

#include "AEROFC_ADC.hpp"

using namespace time_literals;

AEROFC_ADC::AEROFC_ADC(I2CSPIBusOption bus_option, int bus_number, int bus_frequency) :
	I2C(DRV_ADC_DEVTYPE_AEROFC, MODULE_NAME, bus_number, SLAVE_ADDR, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus_number),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample"))
{
}

AEROFC_ADC::~AEROFC_ADC()
{
	perf_free(_sample_perf);
}

int AEROFC_ADC::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	ScheduleOnInterval(100_ms); // 10 Hz

	return PX4_OK;
}

int AEROFC_ADC::probe()
{
	uint8_t buffer[2];
	int ret;

	_retries = 3;

	/* Enable ADC */
	buffer[0] = ADC_ENABLE_REG;
	buffer[1] = 0x01;
	ret = transfer(buffer, 2, NULL, 0);

	if (ret != PX4_OK) {
		goto error;
	}

	usleep(10000);

	/* Read ADC value */
	buffer[0] = ADC_CHANNEL_REG;
	ret = transfer(buffer, 1, buffer, 2);

	if (ret != PX4_OK) {
		goto error;
	}

	return PX4_OK;

error:
	return -EIO;
}

void AEROFC_ADC::RunImpl()
{
	/*
	 * https://github.com/intel-aero/intel-aero-fpga/blob/master/aero_sample/adc/adc.html
	 * https://github.com/intel-aero/meta-intel-aero/wiki/95-(References)-FPGA
	 * https://github.com/intel-aero/intel-aero-fpga/blob/master/aero_rtf_kit/RTL/adc.v
	 */
	perf_begin(_sample_perf);

	uint8_t buffer[10] {};
	buffer[0] = ADC_CHANNEL_REG;
	int ret = transfer(buffer, 1, buffer, sizeof(buffer));

	if (ret != PX4_OK) {
		PX4_ERR("Error reading sample");
		return;
	}

	adc_report_s adc_report{};
	adc_report.device_id = get_device_id();
	adc_report.timestamp = hrt_absolute_time();
	adc_report.v_ref = 3.0f;
	adc_report.resolution = 1 << 12;

	unsigned i;

	for (i = 0; i < MAX_CHANNEL; ++i) {
		adc_report.channel_id[i] = i;
		adc_report.raw_data[i] = (buffer[i * 2] | (buffer[i * 2 + 1] << 8));
	}

	for (; i < PX4_MAX_ADC_CHANNELS; ++i) {	// set unused channel id to -1
		adc_report.channel_id[i] = -1;
	}

	_to_adc_report.publish(adc_report);

	perf_end(_sample_perf);
}
