/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
 * @file battery.cpp
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Timothy Scott <timothy@auterion.com>
 */

#include "battery.h"
#include <mathlib/mathlib.h>
#include <cstring>
#include <px4_platform_common/defines.h>

using namespace time_literals;

Battery::Battery(int index, ModuleParams *parent, const int sample_interval_us) :
	ModuleParams(parent),
	_index(index < 1 || index > 9 ? 1 : index)
{
	const float expected_filter_dt = static_cast<float>(sample_interval_us) / 1_s;
	_voltage_filter_v.setParameters(expected_filter_dt, 1.f);
	_current_filter_a.setParameters(expected_filter_dt, .5f);
	_current_average_filter_a.setParameters(expected_filter_dt, 25.f);
	_throttle_filter.setParameters(expected_filter_dt, 1.f);

	if (index > 9 || index < 1) {
		PX4_ERR("Battery index must be between 1 and 9 (inclusive). Received %d. Defaulting to 1.", index);
	}

	// 16 chars for parameter name + null terminator
	char param_name[17];

	snprintf(param_name, sizeof(param_name), "BAT%d_V_EMPTY", _index);
	_param_handles.v_empty = param_find(param_name);

	if (_param_handles.v_empty == PARAM_INVALID) {
		PX4_ERR("Could not find parameter with name %s", param_name);
	}

	snprintf(param_name, sizeof(param_name), "BAT%d_V_CHARGED", _index);
	_param_handles.v_charged = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_N_CELLS", _index);
	_param_handles.n_cells = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_CAPACITY", _index);
	_param_handles.capacity = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_V_LOAD_DROP", _index);
	_param_handles.v_load_drop = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_R_INTERNAL", _index);
	_param_handles.r_internal = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_SOURCE", _index);
	_param_handles.source = param_find(param_name);

	_param_handles.low_thr = param_find("BAT_LOW_THR");
	_param_handles.crit_thr = param_find("BAT_CRIT_THR");
	_param_handles.emergen_thr = param_find("BAT_EMERGEN_THR");

	_param_handles.v_empty_old = param_find("BAT_V_EMPTY");
	_param_handles.v_charged_old = param_find("BAT_V_CHARGED");
	_param_handles.n_cells_old = param_find("BAT_N_CELLS");
	_param_handles.capacity_old = param_find("BAT_CAPACITY");
	_param_handles.v_load_drop_old = param_find("BAT_V_LOAD_DROP");
	_param_handles.r_internal_old = param_find("BAT_R_INTERNAL");
	_param_handles.source_old = param_find("BAT_SOURCE");

	updateParams();
}

void Battery::updateBatteryStatus(const hrt_abstime &timestamp, float voltage_v, float current_a, bool connected,
				  int source, int priority, float throttle_normalized)
{
	if (!_battery_initialized) {
		_voltage_filter_v.reset(voltage_v);
		_current_filter_a.reset(current_a);
		_throttle_filter.reset(throttle_normalized);
	}

	_voltage_filter_v.update(voltage_v);
	_current_filter_a.update(current_a);
	_throttle_filter.update(throttle_normalized);
	sumDischarged(timestamp, current_a);
	estimateStateOfCharge(_voltage_filter_v.getState(), _current_filter_a.getState(), _throttle_filter.getState());
	computeScale();

	if (connected && _battery_initialized) {
		_warning = determineWarning(_state_of_charge);
	}

	if (_voltage_filter_v.getState() > 2.1f) {
		_battery_initialized = true;

	} else {
		connected = false;
	}

	battery_status_s battery_status{};
	battery_status.voltage_v = voltage_v;
	battery_status.voltage_filtered_v = _voltage_filter_v.getState();
	battery_status.current_a = current_a;
	battery_status.current_filtered_a = _current_filter_a.getState();
	battery_status.current_average_a = _current_average_filter_a.getState();
	battery_status.discharged_mah = _discharged_mah;
	battery_status.remaining = _state_of_charge;
	battery_status.scale = _scale;
	battery_status.time_remaining_s = computeRemainingTime(current_a);;
	battery_status.temperature = NAN;
	// Publish at least one cell such that the total voltage gets into MAVLink BATTERY_STATUS
	battery_status.cell_count = math::max(_params.n_cells, static_cast<int32_t>(1));
	battery_status.connected = connected;
	battery_status.source = source;
	battery_status.priority = priority;
	battery_status.capacity = _params.capacity > 0.f ? static_cast<uint16_t>(_params.capacity) : 0;
	battery_status.id = static_cast<uint8_t>(_index);
	battery_status.warning = _warning;

	static constexpr int32_t uorb_max_cells = sizeof(battery_status.voltage_cell_v) / sizeof(
				battery_status.voltage_cell_v[0]);

	int max_cells = math::min(battery_status.cell_count, uorb_max_cells);

	// Fill cell voltages with average values to work around MAVLink BATTERY_STATUS not allowing to report just total voltage
	for (int i = 0; i < max_cells; i++) {
		battery_status.voltage_cell_v[i] = battery_status.voltage_filtered_v / max_cells;
	}

	if (source == _params.source) {
		battery_status.timestamp = hrt_absolute_time();
		_battery_status_pub.publish(battery_status);
	}
}

void Battery::sumDischarged(const hrt_abstime &timestamp, float current_a)
{
	// Not a valid measurement
	if (current_a < 0.f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dt.
	if (_last_timestamp != 0) {
		const float dt = (timestamp - _last_timestamp) / 1e6;
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		_discharged_mah_loop = (current_a * 1e3f) * (dt / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	_last_timestamp = timestamp;
}

void Battery::estimateStateOfCharge(const float voltage_v, const float current_a, const float throttle)
{
	// remaining battery capacity based on voltage
	float cell_voltage = voltage_v / _params.n_cells;

	// correct battery voltage locally for load drop to avoid estimation fluctuations
	if (_params.r_internal >= 0.f) {
		cell_voltage += _params.r_internal * current_a;

	} else {
		// assume linear relation between throttle and voltage drop
		cell_voltage += throttle * _params.v_load_drop;
	}

	_state_of_charge_volt_based = math::gradual(cell_voltage, _params.v_empty, _params.v_charged, 0.f, 1.f);

	// choose which quantity we're using for final reporting
	if (_params.capacity > 0.f && _battery_initialized) {
		// if battery capacity is known, fuse voltage measurement with used capacity
		// The lower the voltage the more adjust the estimate with it to avoid deep discharge
		const float weight_v = 3e-4f * (1 - _state_of_charge_volt_based);
		_state_of_charge = (1 - weight_v) * _state_of_charge + weight_v * _state_of_charge_volt_based;
		// directly apply current capacity slope calculated using current
		_state_of_charge -= _discharged_mah_loop / _params.capacity;
		_state_of_charge = math::max(_state_of_charge, 0.f);

		const float state_of_charge_current_based = math::max(1.f - _discharged_mah / _params.capacity, 0.f);
		_state_of_charge = math::min(state_of_charge_current_based, _state_of_charge);

	} else {
		_state_of_charge = _state_of_charge_volt_based;
	}
}

uint8_t Battery::determineWarning(float state_of_charge)
{
	if (state_of_charge < _params.emergen_thr) {
		return battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (state_of_charge < _params.crit_thr) {
		return battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (state_of_charge < _params.low_thr) {
		return battery_status_s::BATTERY_WARNING_LOW;

	} else {
		return battery_status_s::BATTERY_WARNING_NONE;
	}
}

void Battery::computeScale()
{
	const float voltage_range = (_params.v_charged - _params.v_empty);

	// reusing capacity calculation to get single cell voltage before drop
	const float bat_v = _params.v_empty + (voltage_range * _state_of_charge_volt_based);

	_scale = _params.v_charged / bat_v;

	if (_scale > 1.3f) { // Allow at most 30% compensation
		_scale = 1.3f;

	} else if (!PX4_ISFINITE(_scale) || _scale < 1.f) { // Shouldn't ever be more than the power at full battery
		_scale = 1.f;
	}
}

float Battery::computeRemainingTime(float current_a)
{
	float time_remaining_s = -1.f;

	// Only estimate remaining time with useful in flight current measurements
	if (_current_filter_a.getState() > 1.f) {
		// Initialize strongly filtered current to an estimated average consumption
		if (_current_average_filter_a.getState() < 0.f) {
			// TODO: better initial value based on "average current" from last flight
			_current_average_filter_a.reset(15.f);
		}

		// Filter current very strong, we basically want the average consumption
		_current_average_filter_a.update(current_a);

		// Remaining time estimation only possible with capacity
		if (_params.capacity > 0.f) {
			const float remaining_capacity_mah = _state_of_charge * _params.capacity;
			const float current_ma = _current_average_filter_a.getState() * 1e3f;
			time_remaining_s = remaining_capacity_mah / current_ma * 3600.f;
		}
	}

	return time_remaining_s;
}

void Battery::updateParams()
{
	if (_index == 1) {
		migrateParam<float>(_param_handles.v_empty_old, _param_handles.v_empty, &_params.v_empty_old, &_params.v_empty,
				    _first_parameter_update);
		migrateParam<float>(_param_handles.v_charged_old, _param_handles.v_charged, &_params.v_charged_old, &_params.v_charged,
				    _first_parameter_update);
		migrateParam<int32_t>(_param_handles.n_cells_old, _param_handles.n_cells, &_params.n_cells_old, &_params.n_cells,
				      _first_parameter_update);
		migrateParam<float>(_param_handles.capacity_old, _param_handles.capacity, &_params.capacity_old, &_params.capacity,
				    _first_parameter_update);
		migrateParam<float>(_param_handles.v_load_drop_old, _param_handles.v_load_drop, &_params.v_load_drop_old,
				    &_params.v_load_drop, _first_parameter_update);
		migrateParam<float>(_param_handles.r_internal_old, _param_handles.r_internal, &_params.r_internal_old,
				    &_params.r_internal, _first_parameter_update);
		migrateParam<int32_t>(_param_handles.source_old, _param_handles.source, &_params.source_old, &_params.source,
				      _first_parameter_update);

	} else {
		param_get(_param_handles.v_empty, &_params.v_empty);
		param_get(_param_handles.v_charged, &_params.v_charged);
		param_get(_param_handles.n_cells, &_params.n_cells);
		param_get(_param_handles.capacity, &_params.capacity);
		param_get(_param_handles.v_load_drop, &_params.v_load_drop);
		param_get(_param_handles.r_internal, &_params.r_internal);
		param_get(_param_handles.source, &_params.source);
	}

	param_get(_param_handles.low_thr, &_params.low_thr);
	param_get(_param_handles.crit_thr, &_params.crit_thr);
	param_get(_param_handles.emergen_thr, &_params.emergen_thr);

	ModuleParams::updateParams();

	_first_parameter_update = false;
}
