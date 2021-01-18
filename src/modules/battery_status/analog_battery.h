/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <battery/battery.h>
#include <parameters/param.h>

class AnalogBattery : public Battery
{
public:
	AnalogBattery(int index, ModuleParams *parent, const int sample_interval_us);

	/**
	 * Update current battery status message.
	 *
	 * @param voltage_raw Battery voltage read from ADC, volts
	 * @param current_raw Voltage of current sense resistor, volts
	 */
	void updateBatteryStatusADC(float voltage_raw, float current_raw);

	/**
	 * Whether the ADC channel for the voltage of this battery is valid.
	 * Corresponds to BOARD_BRICK_VALID_LIST
	 */
	bool is_valid();

	/**
	 * Which ADC channel is used for voltage reading of this battery
	 */
	int get_voltage_channel();

	/**
	 * Which ADC channel is used for current reading of this battery
	 */
	int get_current_channel();

protected:

	struct {
		param_t v_offs_cur;
		param_t v_div;
		param_t a_per_v;
		param_t v_channel;
		param_t i_channel;

		param_t v_div_old;
		param_t a_per_v_old;
		param_t adc_channel_old;
	} _analog_param_handles;

	struct {
		float v_offs_cur;
		float v_div;
		float a_per_v;
		int v_channel;
		int i_channel;

		float v_div_old;
		float a_per_v_old;
		int adc_channel_old;
	} _analog_params;

	virtual void updateParams() override;
};
