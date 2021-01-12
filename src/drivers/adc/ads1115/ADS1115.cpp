/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include "ADS1115.h"
#include <cassert>

int ADS1115::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	uint8_t config[2] = {};
	config[0] = CONFIG_HIGH_OS_NOACT | CONFIG_HIGH_MUX_P0NG | CONFIG_HIGH_PGA_6144 | CONFIG_HIGH_MODE_SS;
	config[1] = CONFIG_LOW_DR_250SPS | CONFIG_LOW_COMP_MODE_TRADITIONAL | CONFIG_LOW_COMP_POL_RESET |
		    CONFIG_LOW_COMP_LAT_NONE | CONFIG_LOW_COMP_QU_DISABLE;
	return writeReg(ADDRESSPOINTER_REG_CONFIG, config, 2);
}

int ADS1115::setChannel(ADS1115::ChannelSelection ch)
{
	uint8_t buf[2] = {};
	uint8_t next_mux_reg = CONFIG_HIGH_MUX_P0NG;

	switch (ch) {
	case A0:
		next_mux_reg = CONFIG_HIGH_MUX_P0NG;
		break;

	case A1:
		next_mux_reg = CONFIG_HIGH_MUX_P1NG;
		break;

	case A2:
		next_mux_reg = CONFIG_HIGH_MUX_P2NG;
		break;

	case A3:
		next_mux_reg = CONFIG_HIGH_MUX_P3NG;
		break;

	default:
		assert(false);
		break;
	}

	buf[0] = CONFIG_HIGH_OS_START_SINGLE | next_mux_reg | CONFIG_HIGH_PGA_6144 | CONFIG_HIGH_MODE_SS;
	buf[1] = CONFIG_LOW_DR_250SPS | CONFIG_LOW_COMP_MODE_TRADITIONAL | CONFIG_LOW_COMP_POL_RESET |
		 CONFIG_LOW_COMP_LAT_NONE | CONFIG_LOW_COMP_QU_DISABLE;
	return writeReg(ADDRESSPOINTER_REG_CONFIG, buf, 2);    // must write whole register to take effect
}

bool ADS1115::isSampleReady()
{
	uint8_t buf[1] = {0x00};

	if (readReg(ADDRESSPOINTER_REG_CONFIG, buf, 1) != 0) { return false; } // Pull config register

	return (buf[0] & (uint8_t) 0x80);
}

ADS1115::ChannelSelection ADS1115::getMeasurement(int16_t *value)
{
	uint8_t buf[2] = {0x00};
	readReg(ADDRESSPOINTER_REG_CONFIG, buf, 1); // Pull config register
	ChannelSelection channel;

	switch ((buf[0] & (uint8_t) 0x70) >> 4) {
	case 0x04:
		channel = A0;
		break;

	case 0x05:
		channel = A1;
		break;

	case 0x06:
		channel = A2;
		break;

	case 0x07:
		channel = A3;
		break;

	default:
		return Invalid;
	}

	readReg(ADDRESSPOINTER_REG_CONVERSATION, buf, 2);
	uint16_t raw_adc_val = buf[0] * 256 + buf[1];

	if (raw_adc_val & (uint16_t) 0x8000) {     // Negetive value
		raw_adc_val = ~raw_adc_val + 1;     // 2's complement
		*value = -raw_adc_val;

	} else {
		*value = raw_adc_val;
	}

	return channel;
}

ADS1115::ChannelSelection ADS1115::cycleMeasure(int16_t *value)
{
	uint8_t buf[2] = {0x00};
	readReg(ADDRESSPOINTER_REG_CONFIG, buf, 1); // Pull config register
	ChannelSelection channel;
	uint8_t next_mux_reg = CONFIG_HIGH_MUX_P0NG;

	switch ((buf[0] & (uint8_t) 0x70) >> 4) {
	case 0x04:
		channel = A0;
		next_mux_reg = CONFIG_HIGH_MUX_P1NG;
		break;

	case 0x05:
		channel = A1;
		next_mux_reg = CONFIG_HIGH_MUX_P2NG;
		break;

	case 0x06:
		channel = A2;
		next_mux_reg = CONFIG_HIGH_MUX_P3NG;
		break;

	case 0x07:
		channel = A3;
		next_mux_reg = CONFIG_HIGH_MUX_P0NG;
		break;

	default:
		return Invalid;
	}

	readReg(ADDRESSPOINTER_REG_CONVERSATION, buf, 2);
	uint16_t raw_adc_val = buf[0] * 256 + buf[1];

	if (raw_adc_val & (uint16_t) 0x8000) {     // Negetive value
		raw_adc_val = ~raw_adc_val + 1;     // 2's complement
		*value = -raw_adc_val;

	} else {
		*value = raw_adc_val;
	}

	buf[0] = CONFIG_HIGH_OS_START_SINGLE | next_mux_reg | CONFIG_HIGH_PGA_6144 | CONFIG_HIGH_MODE_SS;
	buf[1] = CONFIG_LOW_DR_250SPS | CONFIG_LOW_COMP_MODE_TRADITIONAL | CONFIG_LOW_COMP_POL_RESET |
		 CONFIG_LOW_COMP_LAT_NONE | CONFIG_LOW_COMP_QU_DISABLE;
	writeReg(ADDRESSPOINTER_REG_CONFIG, buf, 2);    // must write whole register to take effect
	return channel;
}

int ADS1115::readReg(uint8_t addr, uint8_t *buf, size_t len)
{
	return transfer(&addr, 1, buf, len);
}

int ADS1115::writeReg(uint8_t addr, uint8_t *buf, size_t len)
{
	uint8_t buffer[len + 1];
	buffer[0] = addr;
	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
	return transfer(buffer, len + 1, nullptr, 0);
}