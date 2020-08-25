/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 */

#include "batt_smbus.h"

#include <lib/parameters/param.h>

extern "C" __EXPORT int batt_smbus_main(int argc, char *argv[]);

BATT_SMBUS::BATT_SMBUS(I2CSPIBusOption bus_option, const int bus, SMBus *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_interface(interface)
{
	battery_status_s new_report = {};
	_batt_topic = orb_advertise(ORB_ID(battery_status), &new_report);

	int battsource = 1;
	param_set(param_find("BAT_SOURCE"), &battsource);

	_interface->init();
	// unseal() here to allow an external config script to write to protected flash.
	// This is neccessary to avoid bus errors due to using standard i2c mode instead of SMbus mode.
	// The external config script should then seal() the device.
	unseal();
}

BATT_SMBUS::~BATT_SMBUS()
{
	orb_unadvertise(_batt_topic);
	perf_free(_cycle);

	if (_manufacturer_name != nullptr) {
		delete[] _manufacturer_name;
	}

	if (_interface != nullptr) {
		delete _interface;
	}

	int battsource = 0;
	param_set(param_find("BAT_SOURCE"), &battsource);
}

void BATT_SMBUS::RunImpl()
{
	// Get the current time.
	uint64_t now = hrt_absolute_time();

	// Read data from sensor.
	battery_status_s new_report = {};

	// Set time of reading.
	new_report.timestamp = now;

	new_report.connected = true;

	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	int ret = _interface->read_word(BATT_SMBUS_VOLTAGE, result);

	ret |= get_cell_voltages();

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

	new_report.average_current_a = average_current;

	// If current is high, turn under voltage protection off. This is neccessary to prevent
	// a battery from cutting off while flying with high current near the end of the packs capacity.
	set_undervoltage_protection(average_current);

	// Read run time to empty.
	ret |= _interface->read_word(BATT_SMBUS_RUN_TIME_TO_EMPTY, result);
	new_report.run_time_to_empty = result;

	// Read average time to empty.
	ret |= _interface->read_word(BATT_SMBUS_AVERAGE_TIME_TO_EMPTY, result);
	new_report.average_time_to_empty = result;

	// Read remaining capacity.
	ret |= _interface->read_word(BATT_SMBUS_REMAINING_CAPACITY, result);

	// Calculate remaining capacity percent with complementary filter.
	new_report.remaining = 0.8f * _last_report.remaining + 0.2f * (1.0f - (float)((float)(_batt_capacity - result) /
			       (float)_batt_capacity));

	// Calculate total discharged amount.
	new_report.discharged_mah = _batt_startup_capacity - result;

	// Check if max lifetime voltage delta is greater than allowed.
	if (_lifetime_max_delta_cell_voltage > BATT_CELL_VOLTAGE_THRESHOLD_FAILED) {
		new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (new_report.remaining > _low_thr) {
		new_report.warning = battery_status_s::BATTERY_WARNING_NONE;

	} else if (new_report.remaining > _crit_thr) {
		new_report.warning = battery_status_s::BATTERY_WARNING_LOW;

	} else if (new_report.remaining > _emergency_thr) {
		new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else {
		new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
	}

	// Read battery temperature and covert to Celsius.
	ret |= _interface->read_word(BATT_SMBUS_TEMP, result);
	new_report.temperature = ((float)result / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	new_report.capacity = _batt_capacity;
	new_report.cycle_count = _cycle_count;
	new_report.serial_number = _serial_number;
	new_report.max_cell_voltage_delta = _max_cell_voltage_delta;
	new_report.cell_count = _cell_count;
	new_report.voltage_cell_v[0] = _cell_voltages[0];
	new_report.voltage_cell_v[1] = _cell_voltages[1];
	new_report.voltage_cell_v[2] = _cell_voltages[2];
	new_report.voltage_cell_v[3] = _cell_voltages[3];

	// Only publish if no errors.
	if (!ret) {
		orb_publish(ORB_ID(battery_status), _batt_topic, &new_report);

		_last_report = new_report;
	}
}

void BATT_SMBUS::suspend()
{
	ScheduleClear();
}

void BATT_SMBUS::resume()
{
	ScheduleOnInterval(BATT_SMBUS_MEASUREMENT_INTERVAL_US);
}

int BATT_SMBUS::get_cell_voltages()
{
	// Temporary variable for storing SMBUS reads.
	uint16_t result = 0;

	int ret = _interface->read_word(BATT_SMBUS_CELL_1_VOLTAGE, result);
	// Convert millivolts to volts.
	_cell_voltages[0] = ((float)result) / 1000.0f;

	ret = _interface->read_word(BATT_SMBUS_CELL_2_VOLTAGE, result);
	// Convert millivolts to volts.
	_cell_voltages[1] = ((float)result) / 1000.0f;

	ret = _interface->read_word(BATT_SMBUS_CELL_3_VOLTAGE, result);
	// Convert millivolts to volts.
	_cell_voltages[2] = ((float)result) / 1000.0f;

	ret = _interface->read_word(BATT_SMBUS_CELL_4_VOLTAGE, result);
	// Convert millivolts to volts.
	_cell_voltages[3] = ((float)result) / 1000.0f;

	//Calculate max cell delta
	_min_cell_voltage = _cell_voltages[0];
	float max_cell_voltage = _cell_voltages[0];

	for (uint8_t i = 1; i < (sizeof(_cell_voltages) / sizeof(_cell_voltages[0])); i++) {
		_min_cell_voltage = math::min(_min_cell_voltage, _cell_voltages[i]);
		max_cell_voltage = math::max(max_cell_voltage, _cell_voltages[i]);
	}

	// Calculate the max difference between the min and max cells with complementary filter.
	_max_cell_voltage_delta = (0.5f * (max_cell_voltage - _min_cell_voltage)) +
				  (0.5f * _last_report.max_cell_voltage_delta);

	return ret;
}

void BATT_SMBUS::set_undervoltage_protection(float average_current)
{
	// Disable undervoltage protection if armed. Enable if disarmed and cell voltage is above limit.
	if (average_current > BATT_CURRENT_UNDERVOLTAGE_THRESHOLD) {
		if (_cell_undervoltage_protection_status != 0) {
			// Disable undervoltage protection
			uint8_t protections_a_tmp = BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED;
			uint16_t address = BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS;

			if (dataflash_write(address, &protections_a_tmp, 1) == PX4_OK) {
				_cell_undervoltage_protection_status = 0;
				PX4_WARN("Disabled CUV");

			} else {
				PX4_WARN("Failed to disable CUV");
			}
		}

	} else {
		if (_cell_undervoltage_protection_status == 0) {
			if (_min_cell_voltage > BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD) {
				// Enable undervoltage protection
				uint8_t protections_a_tmp = BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT;
				uint16_t address = BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS;

				if (dataflash_write(address, &protections_a_tmp, 1) == PX4_OK) {
					_cell_undervoltage_protection_status = 1;
					PX4_WARN("Enabled CUV");

				} else {
					PX4_WARN("Failed to enable CUV");
				}
			}
		}
	}

}

//@NOTE: Currently unused, could be helpful for debugging a parameter set though.
int BATT_SMBUS::dataflash_read(uint16_t &address, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	int result = _interface->block_write(code, &address, 2, true);

	if (result != PX4_OK) {
		return result;
	}

	result = _interface->block_read(code, data, length, true);

	return result;
}

int BATT_SMBUS::dataflash_write(uint16_t address, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};

	tx_buf[0] = ((uint8_t *)&address)[0];
	tx_buf[1] = ((uint8_t *)&address)[1];

	if (length > MAC_DATA_BUFFER_SIZE) {
		return PX4_ERROR;
	}

	memcpy(&tx_buf[2], data, length);

	// code (1), byte_count (1), addr(2), data(32) + pec
	int result = _interface->block_write(code, tx_buf, length + 2, false);

	return result;
}

int BATT_SMBUS::get_startup_info()
{
	int result = 0;
	// The name field is 21 characters, add one for null terminator.
	const unsigned name_length = 22;

	// Try and get battery SBS info.
	if (_manufacturer_name == nullptr) {
		char man_name[name_length] = {};
		result = manufacturer_name((uint8_t *)man_name, sizeof(man_name));

		if (result != PX4_OK) {
			PX4_DEBUG("Failed to get manufacturer name");
			return PX4_ERROR;
		}

		_manufacturer_name = new char[sizeof(man_name)];
	}

	uint16_t serial_num;
	result = _interface->read_word(BATT_SMBUS_SERIAL_NUMBER, serial_num);

	uint16_t remaining_cap;
	result |= _interface->read_word(BATT_SMBUS_REMAINING_CAPACITY, remaining_cap);

	uint16_t cycle_count;
	result |= _interface->read_word(BATT_SMBUS_CYCLE_COUNT, cycle_count);

	uint16_t full_cap;
	result |= _interface->read_word(BATT_SMBUS_FULL_CHARGE_CAPACITY, full_cap);

	if (!result) {
		_serial_number = serial_num;
		_batt_startup_capacity = remaining_cap;
		_cycle_count = cycle_count;
		_batt_capacity = full_cap;
	}

	if (lifetime_data_flush() == PX4_OK) {
		// Flush needs time to complete, otherwise device is busy. 100ms not enough, 200ms works.
		px4_usleep(200000);

		if (lifetime_read_block_one() == PX4_OK) {
			if (_lifetime_max_delta_cell_voltage > BATT_CELL_VOLTAGE_THRESHOLD_FAILED) {
				PX4_WARN("Battery Damaged Will Not Fly. Lifetime max voltage difference: %4.2f",
					 (double)_lifetime_max_delta_cell_voltage);
			}
		}

	} else {
		PX4_WARN("Failed to flush lifetime data");
	}

	// Read battery threshold params on startup.
	param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	param_get(param_find("BAT_LOW_THR"), &_low_thr);
	param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);

	return result;
}

uint16_t BATT_SMBUS::get_serial_number()
{
	uint16_t serial_num = 0;

	if (_interface->read_word(BATT_SMBUS_SERIAL_NUMBER, serial_num) == PX4_OK) {
		return serial_num;
	}

	return PX4_ERROR;
}

int BATT_SMBUS::manufacture_date()
{
	uint16_t date;
	int result = _interface->read_word(BATT_SMBUS_MANUFACTURE_DATE, date);

	if (result != PX4_OK) {
		return result;
	}

	return date;
}

int BATT_SMBUS::manufacturer_name(uint8_t *man_name, const uint8_t length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_NAME;
	uint8_t rx_buf[21] = {};

	// Returns 21 bytes, add 1 byte for null terminator.
	int result = _interface->block_read(code, rx_buf, length - 1, true);

	memcpy(man_name, rx_buf, sizeof(rx_buf));

	man_name[21] = '\0';

	return result;
}

int BATT_SMBUS::manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	int result = _interface->block_write(code, address, 2, false);

	if (result != PX4_OK) {
		return result;
	}

	result = _interface->block_read(code, data, length, true);
	memmove(data, &((uint8_t *)data)[2], length - 2); // remove the address bytes

	return result;
}

int BATT_SMBUS::manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};
	memcpy(tx_buf, address, 2);

	if (data != nullptr) {
		memcpy(&tx_buf[2], data, length);
	}

	int result = _interface->block_write(code, tx_buf, length + 2, false);

	return result;
}

int BATT_SMBUS::unseal()
{
	// See bq40z50 technical reference.
	uint16_t keys[2] = {0x0414, 0x3672};

	int ret = _interface->write_word(BATT_SMBUS_MANUFACTURER_ACCESS, keys[0]);

	ret |= _interface->write_word(BATT_SMBUS_MANUFACTURER_ACCESS, keys[1]);

	return ret;
}

int BATT_SMBUS::seal()
{
	// See bq40z50 technical reference.
	uint16_t reg = BATT_SMBUS_SEAL;

	return manufacturer_write(reg, nullptr, 0);
}

int BATT_SMBUS::lifetime_data_flush()
{
	uint16_t flush = BATT_SMBUS_LIFETIME_FLUSH;

	return manufacturer_write(flush, nullptr, 0);
}

int BATT_SMBUS::lifetime_read_block_one()
{
	const int buffer_size = 32 + 2; // 32 bytes of data and 2 bytes of address
	uint8_t lifetime_block_one[buffer_size] = {};

	if (PX4_OK != manufacturer_read(BATT_SMBUS_LIFETIME_BLOCK_ONE, lifetime_block_one, buffer_size)) {
		PX4_INFO("Failed to read lifetime block 1.");
		return PX4_ERROR;
	}

	//Get max cell voltage delta and convert from mV to V.
	_lifetime_max_delta_cell_voltage = (float)(lifetime_block_one[17] << 8 | lifetime_block_one[16]) / 1000.0f;

	PX4_INFO("Max Cell Delta: %4.2f", (double)_lifetime_max_delta_cell_voltage);

	return PX4_OK;
}

void BATT_SMBUS::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Smart battery driver for the BQ40Z50 fuel gauge IC.

### Examples
To write to flash to set parameters. address, number_of_bytes, byte0, ... , byteN
$ batt_smbus -X write_flash 19069 2 27 0

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("batt_smbus", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x0B);

	PRINT_MODULE_USAGE_COMMAND_DESCR("man_info", "Prints manufacturer info.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("unseal", "Unseals the devices flash memory to enable write_flash commands.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("seal", "Seals the devices flash memory to disbale write_flash commands.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("suspend", "Suspends the driver from rescheduling the cycle.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("resume", "Resumes the driver from suspension.");

	PRINT_MODULE_USAGE_COMMAND_DESCR("write_flash", "Writes to flash. The device must first be unsealed with the unseal command.");
	PRINT_MODULE_USAGE_ARG("address", "The address to start writing.", true);
	PRINT_MODULE_USAGE_ARG("number of bytes", "Number of bytes to send.", true);
	PRINT_MODULE_USAGE_ARG("data[0]...data[n]", "One byte of data at a time separated by spaces.", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *BATT_SMBUS::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	SMBus *interface = new SMBus(iterator.bus(), cli.i2c_address);
	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}
	BATT_SMBUS *instance = new BATT_SMBUS(iterator.configuredBusOption(), iterator.bus(), interface);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	int result = instance->get_startup_info();

	if (result != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->ScheduleOnInterval(BATT_SMBUS_MEASUREMENT_INTERVAL_US);

	return instance;
}

void
BATT_SMBUS::custom_method(const BusCLIArguments &cli)
{
	switch(cli.custom1) {
		case 1: {
			uint8_t man_name[22];
			int result = manufacturer_name(man_name, sizeof(man_name));
			PX4_INFO("The manufacturer name: %s", man_name);

			result = manufacture_date();
			PX4_INFO("The manufacturer date: %d", result);

			uint16_t serial_num = 0;
			serial_num = get_serial_number();
			PX4_INFO("The serial number: %d", serial_num);
		}
			break;
		case 2:
			unseal();
			break;
		case 3:
			seal();
			break;
		case 4:
			suspend();
			break;
		case 5:
			resume();
			break;
		case 6:
			if (cli.custom_data) {
				unsigned address = cli.custom2;
				uint8_t *tx_buf = (uint8_t*)cli.custom_data;
				unsigned length = tx_buf[0];

				if (PX4_OK != dataflash_write(address, tx_buf+1, length)) {
					PX4_ERR("Dataflash write failed: %d", address);
				}
				px4_usleep(100000);
			}
			break;
	}
}

extern "C" __EXPORT int batt_smbus_main(int argc, char *argv[])
{
	using ThisDriver = BATT_SMBUS;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = BATT_SMBUS_ADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BAT_DEVTYPE_SMBUS);

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
		return ThisDriver::module_custom_method(cli, iterator, false);
	}
	if (!strcmp(verb, "unseal")) {
		cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "seal")) {
		cli.custom1 = 3;
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
	if (!strcmp(verb, "write_flash")) {
		cli.custom1 = 6;
		if (argc >= 3) {
			uint16_t address = atoi(argv[1]);
			unsigned length = atoi(argv[2]);
			uint8_t tx_buf[33];
			cli.custom_data = &tx_buf;

			if (length > 32) {
				PX4_WARN("Data length out of range: Max 32 bytes");
				return 1;
			}

			tx_buf[0] = length;
			// Data needs to be fed in 1 byte (0x01) at a time.
			for (unsigned i = 0; i < length; i++) {
				if ((unsigned)argc <= 3 + i) {
					tx_buf[i+1] = atoi(argv[3 + i]);
				}
			}
			cli.custom2 = address;
			return ThisDriver::module_custom_method(cli, iterator);
		}
	}

	ThisDriver::print_usage();
	return -1;
}
