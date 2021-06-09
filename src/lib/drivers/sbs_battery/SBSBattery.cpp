/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file batt_smbus.h
 *
 * This is a basic, SBS v1.1 compliant implementation of
 * an SMBUS Smart Battery. This driver is to be used as a default,
 * or as a base-class for more specific implementations such as
 * the Rotoye Batmon or TI BQ40z50/80
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 * @author Eohan George <eg@.skymul.com>
 * @author Alex Mikhalev <alex@corvus-robotics.com>
 * @author Mohammed Kabir <kabir@corvus-robotics.com>
 *
 */

#include <lib/drivers/sbs_battery/SBSBattery.hpp>

// Convert a uint16_t to int16_t without relying on type punning pointers
inline static int16_t u2int16(uint16_t x)
{
	return static_cast<int16_t>(x);
}

SBSBatteryBase::SBSBatteryBase(SMBus *interface) :
	ModuleParams(nullptr),
	_interface(interface)
{
	_interface->init();
}

SBSBatteryBase::~SBSBatteryBase()
{
	perf_free(_cycle);

	delete _interface;
}

int SBSBatteryBase::populate_startup_data()
{
	int ret = PX4_OK;

	// There's no convenient way to auto-detect the number of cells via the SMBus Battery specification, so we just read it from a parameter for now
	_cell_count = uint8_t(_param_sbs_bat_n_cells.get());

	ret |= _interface->block_read(SBS_REG_MANUFACTURER_NAME, _manufacturer_name, MANUFACTURER_NAME_SIZE,
				      true);
	_manufacturer_name[sizeof(_manufacturer_name) - 1] = '\0';

	ret |= _interface->read_word(SBS_REG_MANUFACTURE_DATE, _manufacture_date);

	ret |= _interface->read_word(SBS_REG_SERIAL_NUMBER, _serial_number);

	ret |= _interface->read_word(SBS_REG_CYCLE_COUNT, _cycle_count);

	uint16_t design_voltage_mv;
	ret |= _interface->read_word(SBS_REG_DESIGN_VOLTAGE, design_voltage_mv);
	_design_voltage = float(design_voltage_mv) * 0.001f;

	ret |= _interface->read_word(SBS_REG_DESIGN_CAPACITY, _design_capacity);
	_design_capacity *= _param_sbs_bat_c_mult.get();

	ret |= _interface->read_word(SBS_REG_FULL_CHARGE_CAPACITY, _actual_capacity);
	_actual_capacity *= _param_sbs_bat_c_mult.get();

	return ret;
}

int SBSBatteryBase::populate_runtime_data(battery_status_s &data)
{
	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	// Voltage (mV -> V)
	int ret = _interface->read_word(SBS_REG_VOLTAGE, result);
	data.voltage_v = float(result) * 0.001f;
	data.voltage_filtered_v = data.voltage_v;

	// Current (mA -> A, scaling)
	ret |= _interface->read_word(SBS_REG_CURRENT, result);
	data.current_a = float(u2int16(result)) * -0.001f * _param_sbs_bat_c_mult.get();
	data.current_filtered_a = data.current_a;

	// Average current (mA -> A, scaling)
	ret |= _interface->read_word(SBS_REG_AVERAGE_CURRENT, result);
	data.average_current_a = float(u2int16(result)) * -0.001f * _param_sbs_bat_c_mult.get();

	// Time to empty (minutes).
	ret |= _interface->read_word(SBS_REG_RUN_TIME_TO_EMPTY, result);
	data.run_time_to_empty = result;

	// Average time to empty (minutes).
	ret |= _interface->read_word(SBS_REG_AVERAGE_TIME_TO_EMPTY, result);
	data.average_time_to_empty = result;

	// Remaining amount
	ret |= _interface->read_word(SBS_REG_RELATIVE_SOC, result);
	data.remaining = float(result) / 100.0f;

	// Max Error
	ret |= _interface->read_word(SBS_REG_MAX_ERROR, result);
	data.max_error = result;

	// Battery temperature
	ret |= _interface->read_word(SBS_REG_TEMP, result);
	data.temperature = (float(result) / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	// Static data
	// TODO: move to a new battery_info message?
	data.capacity = _actual_capacity;
	data.cycle_count = _cycle_count;
	data.serial_number = _serial_number;
	data.manufacture_date = _manufacture_date;

	return ret;
}

void SBSBatteryBase::run()
{
	perf_begin(_cycle);

	// Get the current time.
	uint64_t now = hrt_absolute_time();

	battery_status_s new_report = {};

	// TODO(hyonlim): this driver should support multiple SMBUS going forward.
	new_report.id = 1;
	new_report.timestamp = now;
	new_report.connected = true;
	new_report.cell_count = _cell_count;
	new_report.warning = battery_status_s::BATTERY_WARNING_NONE;

	// Read data from sensor.
	int ret = populate_runtime_data(new_report);

	if (new_report.remaining > _param_bat_low_thr.get()) {
		// Leave as battery_status_s::BATTERY_WARNING_NONE

	} else if (new_report.remaining > _param_bat_crit_thr.get()) {
		new_report.warning = battery_status_s::BATTERY_WARNING_LOW;

	} else if (new_report.remaining > _param_bat_emergen_thr.get()) {
		new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else {
		new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
	}

	new_report.interface_error = perf_event_count(_interface->_interface_errors);

	// Only publish if no errors.
	if (!ret) {
		_battery_status_pub.publish(new_report);
	}

	perf_end(_cycle);
}
