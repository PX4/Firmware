/****************************************************************************
 *
 *   Copyright (c) 2016, 2017 PX4 Development Team. All rights reserved.
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
 * @file battery.h
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <drivers/drv_hrt.h>
#include <px4_module_params.h>
#include <drivers/drv_adc.h>
#include <board_config.h>

class BatteryBase : ModuleParams
{
public:
	BatteryBase();

	/**
	 * Reset all battery stats and report invalid/nothing.
	 */
	void reset();

	/**
	 * Get the battery cell count
	 */
	int cell_count() { return _get_bat_n_cells(); }

	/**
	 * Get the empty voltage per cell
	 */
	float empty_cell_voltage() { return _get_bat_v_empty(); }

	/**
	 * Get the full voltage per cell
	 */
	float full_cell_voltage() { return _get_bat_v_charged(); }

	/**
	 * Update current battery status message.
	 *
	 * @param voltage_v: current voltage in V
	 * @param current_a: current current in A
	 * @param connected: Battery is connected
	 * @param selected_source: This battery is on the brick that the selected source for selected_source
	 * @param priority: The brick number -1. The term priority refers to the Vn connection on the LTC4417
	 * @param throttle_normalized: throttle from 0 to 1
	 */
	void updateBatteryStatus(int32_t voltage_raw, int32_t current_raw, hrt_abstime timestamp,
				 bool selected_source, int priority,
				 float throttle_normalized,
				 bool armed);

	int vChannel{-1};
	int iChannel{-1};

protected:
	static constexpr int   DEFAULT_V_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_V_LIST;
	static constexpr int   DEFAULT_I_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_I_LIST;

	virtual float _get_bat_v_empty() = 0;
	virtual float _get_bat_v_charged() = 0;
	virtual int _get_bat_n_cells() = 0;
	virtual float _get_bat_capacity() = 0;
	virtual float _get_bat_v_load_drop() = 0;
	virtual float _get_bat_r_internal() = 0;
	virtual float _get_bat_low_thr() = 0;
	virtual float _get_bat_crit_thr() = 0;
	virtual float _get_bat_emergen_thr() = 0;
	virtual float _get_cnt_v_volt() = 0;
	virtual float _get_cnt_v_curr() = 0;
	virtual float _get_v_offs_cur() = 0;
	virtual float _get_v_div() = 0;
	virtual float _get_a_per_v() = 0;
	virtual int _get_source() = 0;
	virtual int _get_adc_channel() = 0;

	virtual int _get_brick_index() = 0;
	virtual bool _is_valid()
	{
		bool valid[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;
		return valid[_get_brick_index()];
	}

private:
	void filterVoltage(float voltage_v);
	void filterThrottle(float throttle);
	void filterCurrent(float current_a);
	void sumDischarged(hrt_abstime timestamp, float current_a);
	void estimateRemaining(float voltage_v, float current_a, float throttle, bool armed);
	void determineWarning(bool connected);
	void computeScale();

	bool _battery_initialized = false;
	float _voltage_filtered_v = -1.f;
	float _throttle_filtered = -1.f;
	float _current_filtered_a = -1.f;
	float _discharged_mah = 0.f;
	float _discharged_mah_loop = 0.f;
	float _remaining_voltage = -1.f;		///< normalized battery charge level remaining based on voltage
	float _remaining = -1.f;			///< normalized battery charge level, selected based on config param
	float _scale = 1.f;
	uint8_t _warning;
	hrt_abstime _last_timestamp;

	orb_advert_t _orbAdvert{nullptr};
	int _orbInstance;
	battery_status_s _battery_status;
};
