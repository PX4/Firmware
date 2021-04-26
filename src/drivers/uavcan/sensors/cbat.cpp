/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "cbat.hpp"

#include <lib/ecl/geo/geo.h>
#include <px4_defines.h>

const char *const UavcanCBATBridge::NAME = "cbat";

UavcanCBATBridge::UavcanCBATBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_cbat", ORB_ID(battery_status)),
	ModuleParams(nullptr),
	_sub_battery(node),
	_warning(battery_status_s::BATTERY_WARNING_NONE)
{
}

int UavcanCBATBridge::init()
{
	int res = _sub_battery.start(CBATCbBinder(this, &UavcanCBATBridge::battery_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanCBATBridge::battery_sub_cb(const uavcan::ReceivedDataStructure<cuav::equipment::power::CBAT> &msg)
{
	battery_status_s battery{};

	battery.timestamp = hrt_absolute_time();
	battery.voltage_v = msg.voltage;
	battery.voltage_filtered_v = battery.voltage_v;
	battery.current_a = msg.current;
	battery.current_filtered_a = battery.current_a;
	battery.average_current_a = msg.average_current;
	battery.discharged_mah = msg.passed_charge * 1000;
	battery.remaining = msg.state_of_charge / 100.0f;
	// battery.scale = msg.; // Power scaling factor, >= 1, or -1 if unknown
	battery.temperature = msg.temperature + CONSTANTS_ABSOLUTE_NULL_CELSIUS;
	battery.cell_count = msg.cell_count;
	battery.connected = true;
	battery.source = msg.status_flags & cuav::equipment::power::CBAT::STATUS_FLAG_IN_USE; // BATTERY_SOURCE_EXTERNAL
	// battery.priority = msg.;
	battery.capacity = msg.full_charge_capacity * 1000;
	battery.cycle_count = msg.cycle_count;
	battery.run_time_to_empty = msg.average_time_to_empty;
	battery.average_time_to_empty = msg.average_time_to_empty;
	battery.serial_number = msg.serial_number;
	battery.manufacture_date = msg.manufacture_date;
	battery.state_of_health = msg.state_of_health;
	battery.max_error = msg.max_error;
	battery.id = msg.getSrcNodeID().get();
	battery.interface_error = msg.interface_error;

	for (uint8_t i = 0; i < msg.cell_count; i++) {
		battery.voltage_cell_v[i] = msg.voltage_cell[i];
	}

	//Calculate max cell delta
	float min_cell_voltage = msg.voltage_cell[0];
	float max_cell_voltage = msg.voltage_cell[0];

	for (uint8_t i = 1; i < msg.cell_count; i++) {
		min_cell_voltage = math::min(min_cell_voltage, msg.voltage_cell[i]);
		max_cell_voltage = math::max(max_cell_voltage, msg.voltage_cell[i]);
	}

	// Calculate the max difference between the min and max cells with complementary filter.
	battery.max_cell_voltage_delta = (0.5f * (max_cell_voltage - min_cell_voltage)) +
					 (0.5f * _max_cell_voltage_delta);
	_max_cell_voltage_delta = battery.max_cell_voltage_delta;

	battery.is_powering_off = msg.is_powering_off;

	determineWarning(battery.remaining);
	battery.warning = _warning;

	// Expand the information
	battery.average_power = msg.average_power;
	battery.available_energy = msg.available_energy;
	battery.remaining_capacity = msg.remaining_capacity;
	battery.design_capacity = msg.design_capacity;
	battery.average_time_to_full = msg.average_time_to_full;
	battery.over_discharge_count = msg.over_discharge_count;
	battery.nominal_voltage = msg.nominal_voltage;

	publish(msg.getSrcNodeID().get(), &battery);
}

void
UavcanCBATBridge::determineWarning(float remaining)
{
	// propagate warning state only if the state is higher, otherwise remain in current warning state
	if (remaining < _param_bat_emergen_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_EMERGENCY)) {
		_warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (remaining < _param_bat_crit_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
		_warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (remaining < _param_bat_low_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_LOW)) {
		_warning = battery_status_s::BATTERY_WARNING_LOW;
	}
}
