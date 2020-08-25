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

#include "PCF8583.hpp"

PCF8583::PCF8583(I2CSPIBusOption bus_option, const int bus, int bus_frequency) :
	I2C("PCF8583", nullptr, bus, PCF8583_BASEADDR_DEFAULT, bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus)
{
}

int PCF8583::init()
{
	set_device_address(_param_pcf8583_addr.get());

	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	PX4_DEBUG("addr: %d, pool: %d, reset: %d, magenet: %d", get_device_address(), _param_pcf8583_pool.get(),
		  _param_pcf8583_reset.get(),
		  _param_pcf8583_magnet.get());

	// set counter mode
	setRegister(0x00, 0b00100000);

	// start measurement
	resetCounter();

	ScheduleOnInterval(_param_pcf8583_pool.get());

	return PX4_OK;
}

int PCF8583::probe()
{
	return PX4_OK;
}

int PCF8583::getCounter()
{
	uint8_t a = readRegister(0x01);
	uint8_t b = readRegister(0x02);
	uint8_t c = readRegister(0x03);

	return int((a & 0x0f) * 1 + ((a & 0xf0) >> 4) * 10 + (b & 0x0f) * 100 + ((b & 0xf0) >> 4) * 1000 +
		   (c & 0x0f) * 10000 + ((c & 0xf0) >> 4) * 1000000);
}

void PCF8583::resetCounter()
{
	setRegister(0x01, 0x00);
	setRegister(0x02, 0x00);
	setRegister(0x03, 0x00);
}

void PCF8583::setRegister(uint8_t reg, uint8_t value)
{
	uint8_t buff[2];
	buff[0] = reg;
	buff[1] = value;
	int ret = transfer(buff, 2, nullptr, 0);

	if (PX4_OK != ret) {
		PX4_DEBUG("setRegister : i2c::transfer returned %d", ret);
	}
}

uint8_t PCF8583::readRegister(uint8_t reg)
{
	uint8_t rcv{};
	int ret = transfer(&reg, 1, &rcv, 1);

	if (PX4_OK != ret) {
		PX4_DEBUG("readRegister : i2c::transfer returned %d", ret);
	}

	return rcv;
}

void PCF8583::RunImpl()
{
	// read sensor and compute frequency
	int oldcount = _count;
	uint64_t oldtime = _last_measurement_time;

	_count = getCounter();
	_last_measurement_time = hrt_absolute_time();

	int diffCount = _count - oldcount;
	uint64_t diffTime = _last_measurement_time - oldtime;

	if (_param_pcf8583_reset.get() < _count + diffCount) {
		resetCounter();
		_last_measurement_time = hrt_absolute_time();
		_count = 0;
	}

	float indicated_frequency = (float)diffCount / _param_pcf8583_magnet.get() / ((float)diffTime / 1000000);
	float estimated_accurancy = 1 / (float)_param_pcf8583_magnet.get() / ((float)diffTime / 1000000);

	// publish
	rpm_s msg{};
	msg.indicated_frequency_hz = indicated_frequency;
	msg.indicated_frequency_rpm = indicated_frequency * 60.f;
	msg.estimated_accurancy_hz = estimated_accurancy;
	msg.estimated_accurancy_rpm = estimated_accurancy * 60.f;
	msg.timestamp = hrt_absolute_time();
	_rpm_pub.publish(msg);
}

void PCF8583::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("poll interval:  %d us", _param_pcf8583_pool.get());
}
