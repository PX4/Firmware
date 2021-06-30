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

/**
 * @file batmon.cpp
 *
 * BatMon module for Smart Battery utilizing SBS 1.1 specifications
 * Setup/usage information: https://rotoye.com/batmon-tutorial/
 *
 * @author Eohan George <eohan@rotoye.com>
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 */

#include "batmon.h"
#include <mathlib/mathlib.h>

extern "C" __EXPORT int batmon_main(int argc, char *argv[]);

Batmon::Batmon(I2CSPIBusOption bus_option, const int bus, SMBus *interface):
	SMBUS_SBS_BaseClass(bus_option, bus, interface)
{

}

Batmon::~Batmon()
{
	// Needn't change the parameter since there are other modules that may use it.
	//int battsource = 0;
	//param_set(param_find("BAT_SOURCE"), &battsource);
}

I2CSPIDriverBase *Batmon::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)

{
	//TODO there may be a better way to pass the driver ID, since iterator object should have it
	SMBus *interface = new SMBus(DRV_BAT_DEVTYPE_BATMON_SMBUS, iterator.bus(), cli.i2c_address);

	int32_t batmon_en_param = 0;
	param_get(param_find("BATMON_DRIVER_EN"), &batmon_en_param);

	if (batmon_en_param == 0) {	// BATMON_DRIVER_EN is set to disabled. Do not start driver
		return nullptr;        // TODO: add option for autodetect I2C address
	}

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	Batmon *instance = new Batmon(iterator.configuredBusOption(), iterator.bus(), interface);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	int ret = instance->get_startup_info();
	ret |= instance->get_batmon_startup_info();

	if (ret != PX4_OK) {
		delete instance;
		return nullptr;
	}


	// Setting the BAT_SOURCE to "external"
	int32_t battsource = 1;
	param_set(param_find("BAT_SOURCE"), &battsource);

	instance->ScheduleOnInterval(SBS_MEASUREMENT_INTERVAL_US);

	return instance;
}

void Batmon::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for SMBUS Communication with BatMon enabled smart-battery
Setup/usage information: https://rotoye.com/batmon-tutorial/
### Examples
To start at address 0x0B, on bus 4
$ batmon start -X -a 11 -b 4

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("batmon", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x0B);

	PRINT_MODULE_USAGE_COMMAND_DESCR("man_info", "Prints manufacturer info.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("suspend", "Suspends the driver from rescheduling the cycle.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("resume", "Resumes the driver from suspension.");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void Batmon::RunImpl()
{
	int ret = PX4_OK;

	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	// Read data from sensor.
	battery_status_s new_report = {};

	new_report.id = 1;

	// Set time of reading.
	new_report.timestamp = hrt_absolute_time();

	new_report.connected = true;

	ret |= _interface->read_word(BATT_SMBUS_VOLTAGE, result);

	ret |= get_cell_voltages();

	for (int i = 0; i < _cell_count; i++) {
		new_report.voltage_cell_v[i] = _cell_voltages[i];
	}

	// Convert millivolts to volts.
	new_report.voltage_v = ((float)result) / 1000.0f;
	new_report.voltage_filtered_v = new_report.voltage_v;

	// Read current.
	ret |= _interface->read_word(BATT_SMBUS_CURRENT, result);

	new_report.current_a = (-1.0f * ((float)(*(int16_t *)&result)) / 1000.0f);
	new_report.current_filtered_a = new_report.current_a;

	// Read average current.
	ret |= _interface->read_word(BATT_SMBUS_AVERAGE_CURRENT, result);

	float average_current = (-1.0f * ((float)(*(int16_t *)&result)) / 1000.0f);

	new_report.current_average_a = average_current;

	// Read run time to empty (minutes).
	ret |= _interface->read_word(BATT_SMBUS_RUN_TIME_TO_EMPTY, result);
	new_report.time_remaining_s = result * 60;

	// Read average time to empty (minutes).
	ret |= _interface->read_word(BATT_SMBUS_AVERAGE_TIME_TO_EMPTY, result);
	new_report.average_time_to_empty = result;

	// Read remaining capacity.
	ret |= _interface->read_word(BATT_SMBUS_REMAINING_CAPACITY, result);

	// Calculate total discharged amount in mah.
	new_report.discharged_mah = _batt_startup_capacity - (float)result;

	// Read Relative SOC.
	ret |= _interface->read_word(BATT_SMBUS_RELATIVE_SOC, result);

	// Normalize 0.0 to 1.0
	new_report.remaining = (float)result / 100.0f;

	// Read Max Error
	//ret |= _interface->read_word(BATT_SMBUS_MAX_ERROR, result); //TODO: to be implemented
	//new_report.max_error = result;

	// Read battery temperature and covert to Celsius.
	ret |= _interface->read_word(BATT_SMBUS_TEMP, result);
	new_report.temperature = ((float)result / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	// Only publish if no errors.
	if (ret == PX4_OK) {
		new_report.capacity = _batt_capacity;
		new_report.cycle_count = _cycle_count;
		new_report.serial_number = _serial_number;
		new_report.max_cell_voltage_delta = _max_cell_voltage_delta;
		new_report.cell_count = _cell_count;
		new_report.state_of_health = _state_of_health;

		// TODO: This critical setting should be set with BMS info or through a paramter
		// Setting a hard coded BATT_CELL_VOLTAGE_THRESHOLD_FAILED may not be appropriate
		//if (_lifetime_max_delta_cell_voltage > BATT_CELL_VOLTAGE_THRESHOLD_FAILED) {
		//	new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		if (new_report.remaining > _low_thr) {
			new_report.warning = battery_status_s::BATTERY_WARNING_NONE;

		} else if (new_report.remaining > _crit_thr) {
			new_report.warning = battery_status_s::BATTERY_WARNING_LOW;

		} else if (new_report.remaining > _emergency_thr) {
			new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		} else {
			new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
		}

		new_report.interface_error = perf_event_count(_interface->_interface_errors);

		int instance = 0;
		orb_publish_auto(ORB_ID(battery_status), &_batt_topic, &new_report, &instance);

		_last_report = new_report;
	}
}

int Batmon::get_batmon_startup_info()
{
	int ret = PX4_OK;

	// Read battery threshold params on startup.
	param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	param_get(param_find("BAT_LOW_THR"), &_low_thr);
	param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);

	// Read BatMon specific data
	uint16_t num_cells;
	ret = _interface->read_word(BATT_SMBUS_CELL_COUNT, num_cells);
	_cell_count = math::min((uint8_t)num_cells, (uint8_t)MAX_CELL_COUNT);

	int32_t _num_cells = num_cells;
	param_set(param_find("BAT_N_CELLS"), &_num_cells);

	return ret;
}

void Batmon::custom_method(const BusCLIArguments &cli)
{
	switch(cli.custom1) {
		case 1:
			// TODO: analyze why these statements are not printed
			PX4_INFO("The manufacturer name: %s", _manufacturer_name);
			PX4_INFO("The manufacturer date: %d", _manufacture_date);
			PX4_INFO("The serial number: %d", _serial_number);
			break;
		case 4:
			suspend();
			break;
		case 5:
			resume();
			break;
	}
}


int Batmon::get_cell_voltages()
{
	// Temporary variable for storing SMBUS reads.
	uint16_t result = 0;
	uint8_t ret = 0;

	// Making the assumption that the register value of BATT_SMBUS_CELL_1_VOLTAGE and BATT_SMBUS_CELL_10_VOLTAGE are sequential and decreasing order.
	for (int i = 0 ; i < _cell_count; i++) {
		ret |= _interface->read_word(BATT_SMBUS_CELL_1_VOLTAGE - i, result);
		// Convert millivolts to volts.
		_cell_voltages[i] = ((float)result) * 0.001f;
	}

	//Calculate max cell delta
	_min_cell_voltage = _cell_voltages[0];
	float max_cell_voltage = _cell_voltages[0];

	for (uint8_t i = 1; (i < _cell_count && i < (sizeof(_cell_voltages) / sizeof(_cell_voltages[0]))); i++) {
		_min_cell_voltage = math::min(_min_cell_voltage, _cell_voltages[i]);
		max_cell_voltage = math::max(max_cell_voltage, _cell_voltages[i]);
	}

	// Calculate the max difference between the min and max cells with complementary filter.
	_max_cell_voltage_delta = (0.5f * (max_cell_voltage - _min_cell_voltage)) +
				  (0.5f * _last_report.max_cell_voltage_delta);

	return ret;
}

extern "C" __EXPORT int batmon_main(int argc, char *argv[])
{
	using ThisDriver = Batmon;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;

	int32_t batmon_addr_batt1 = BATMON_DEFAULT_SMBUS_ADDR;
	param_get(param_find("BATMON_ADDR_DFLT"), &batmon_addr_batt1);
	cli.i2c_address = batmon_addr_batt1;

	const char *verb = cli.parseDefaultArguments(argc, argv);
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BAT_DEVTYPE_BATMON_SMBUS);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "man_info")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "suspend")) {
		cli.custom1 = 4;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "resume")) {
		cli.custom1 = 5;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
