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

AEROFC_ADC::AEROFC_ADC(uint8_t bus) :
	I2C("AEROFC_ADC", ADC0_DEVICE_PATH, bus, SLAVE_ADDR, 400000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample"))
{
	_sample.am_channel = 1;
	pthread_mutex_init(&_sample_mutex, nullptr);
}

AEROFC_ADC::~AEROFC_ADC()
{
	ScheduleClear();
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

ssize_t AEROFC_ADC::read(file *filp, char *buffer, size_t len)
{
	if (len < sizeof(_sample)) {
		return -ENOSPC;
	}

	if (len > sizeof(_sample)) {
		len = sizeof(_sample);
	}

	pthread_mutex_lock(&_sample_mutex);
	memcpy(buffer, &_sample, len);
	pthread_mutex_unlock(&_sample_mutex);

	return len;
}

void AEROFC_ADC::Run()
{
	uint8_t buffer[2] {};
	buffer[0] = ADC_CHANNEL_REG;
	int ret = transfer(buffer, 1, buffer, sizeof(buffer));

	if (ret != PX4_OK) {
		PX4_ERR("Error reading sample");
		return;
	}

	pthread_mutex_lock(&_sample_mutex);
	_sample.am_data = (buffer[0] | (buffer[1] << 8));
	pthread_mutex_unlock(&_sample_mutex);
}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 12; // 12 bit ADC
}
