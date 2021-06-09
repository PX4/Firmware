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
 * @file bq40z80.cpp
 *
 * Driver for TI BQ40Z80 connected via SMBus (I2C). Loosely based on legacy
 * batt_smbus driver by Jacob Dahl <dahl.jakejacob@gmail.com>, Alex Klimaj
 * <alexklimaj@gmail.com> and Bazooka Joe <BazookaJoe1900@gmail.com>
 *
 * @author Alex Mikhalev <alex@corvus-robotics.com>
 * @author Mohammed Kabir <kabir@corvus-robotics.com>
 *
 */

#include "bq40z80.h"

extern "C" __EXPORT int bq40z80_main(int argc, char *argv[]);

BQ40Z80::BQ40Z80(I2CSPIBusOption bus_option, const int bus, SMBus *interface) :
	SBSBattery(bus_option, bus, interface)
{
	_cycle = perf_alloc(PC_ELAPSED, "bq40z80_cycle");
}

BQ40Z80::~BQ40Z80()
{
}

int BQ40Z80::populate_cell_voltages(battery_status_s &data)
{
	uint8_t DAstatus1[32] = {}; // 32 bytes of data

	if (PX4_OK != manufacturer_read(BQ40Z80_MAN_DASTATUS1, DAstatus1, sizeof(DAstatus1))) {
		return PX4_ERROR;
	}

	// Cells 1-4
	for (int i = 0; i < math::min(4, int(_cell_count)); i++) {
		// convert mV -> volts
		data.voltage_cell_v[i] = ((float)((DAstatus1[2 * i + 1] << 8) | DAstatus1[2 * i]) / 1000.0f);
	}

	uint8_t DAstatus3[18] = {}; // 18 bytes of data

	if (PX4_OK != manufacturer_read(BQ40Z80_MAN_DASTATUS3, DAstatus3, sizeof(DAstatus3))) {
		return PX4_ERROR;
	}

	// Cells 5-7
	if (_cell_count >= 5) {
		data.voltage_cell_v[4] = ((float)((DAstatus3[1] << 8) | DAstatus3[0]) / 1000.0f);
	}

	if (_cell_count >= 6) {
		data.voltage_cell_v[5] = ((float)((DAstatus3[7] << 8) | DAstatus3[6]) / 1000.0f);
	}

	if (_cell_count >= 7) {
		data.voltage_cell_v[6] = ((float)((DAstatus3[13] << 8) | DAstatus3[12]) / 1000.0f);
	}

	return PX4_OK;
}

int BQ40Z80::set_protection_cuv(bool enable)
{
	uint8_t protections_a_tmp = enable ?
				    BQ40Z80_ENABLED_PROTECTIONS_A_DEFAULT :
				    BQ40Z80_ENABLED_PROTECTIONS_A_CUV_DISABLED;

	return dataflash_write(BQ40Z80_MAN_ENABLED_PROTECTIONS_A_ADDRESS, &protections_a_tmp, 1);
}

int BQ40Z80::dataflash_read(const uint16_t address, void *data, const unsigned length)
{
	if (length > MAC_DATA_BUFFER_SIZE) {
		return -EINVAL;
	}

	uint8_t code = BQ40Z80_REG_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[2] = {};
	tx_buf[0] = ((uint8_t *)&address)[0];
	tx_buf[1] = ((uint8_t *)&address)[1];

	uint8_t rx_buf[MAC_DATA_BUFFER_SIZE + 2];

	int ret = _interface->block_write(code, tx_buf, 2, false);

	if (ret != PX4_OK) {
		return ret;
	}

	// Always returns 32 bytes of data
	ret = _interface->block_read(code, rx_buf, 32 + 2, true);
	memcpy(data, &rx_buf[2], length); // remove the address bytes

	return ret;
}

int BQ40Z80::dataflash_write(const uint16_t address, void *data, const unsigned length)
{
	return manufacturer_write(address, data, length);
}

int BQ40Z80::manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length)
{
	if (length > MAC_DATA_BUFFER_SIZE) {
		return -EINVAL;
	}

	uint8_t code = BQ40Z80_REG_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	uint8_t rx_buf[MAC_DATA_BUFFER_SIZE + 2];

	int ret = _interface->block_write(code, address, 2, false);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = _interface->block_read(code, rx_buf, length + 2, true);
	memcpy(data, &rx_buf[2], length); // remove the address bytes

	return ret;
}

int BQ40Z80::manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BQ40Z80_REG_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};
	tx_buf[0] = cmd_code & 0xff;
	tx_buf[1] = (cmd_code >> 8) & 0xff;

	if (data != nullptr && length <= MAC_DATA_BUFFER_SIZE) {
		memcpy(&tx_buf[2], data, length);
	}

	int ret = _interface->block_write(code, tx_buf, length + 2, false);

	return ret;
}

int BQ40Z80::lifetime_flush()
{
	return manufacturer_write(BQ40Z80_MAN_LIFETIME_FLUSH, nullptr, 0);
}

int BQ40Z80::lifetime_read()
{
	uint8_t lifetime_block_one[32] = {}; // 32 bytes of data

	if (PX4_OK != manufacturer_read(BQ40Z80_MAN_LIFETIME_BLOCK_ONE, lifetime_block_one, sizeof(lifetime_block_one))) {
		return PX4_ERROR;
	}

	return PX4_OK;
}


int BQ40Z80::populate_startup_data()
{
	int ret = PX4_OK;

	uint16_t device_type;
	ret |= manufacturer_read(BQ40Z80_MAN_DEVICE_TYPE, &device_type, sizeof(device_type));

	if (ret || device_type != BQ40Z80_EXPECTED_DEVICE_TYPE) {
		PX4_DEBUG("BQ40Z80 failed probe (ret: %d, device_type: 0x%04x)", ret, device_type);
		return -EIO;
	}

	// Get generic SBS startup information
	ret |= BaseClass::populate_startup_data();

	uint8_t cell_configuration;
	ret |= dataflash_read(BQ40Z80_FLASH_CELL_CONFIGURATION, &cell_configuration, sizeof(cell_configuration));

	uint16_t state_of_health;
	ret |= _interface->read_word(BQ40Z80_REG_STATE_OF_HEALTH, state_of_health);

	if (ret) {
		PX4_WARN("Failed to read startup info: %d", ret);
		return ret;
	}

	_state_of_health = state_of_health;
	_cell_count = cell_configuration & 0x07;

	return ret;
}

int BQ40Z80::populate_runtime_data(battery_status_s &data)
{
	int ret = PX4_OK;

	ret |= BaseClass::populate_runtime_data(data);

	ret |= populate_cell_voltages(data);

	data.state_of_health = _state_of_health;

	return ret;
}

void BQ40Z80::print_module_description()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Smart battery driver for the TI BQ40Z80 fuel gauge IC.

)DESCR_STR");
}

extern "C" __EXPORT int bq40z80_main(int argc, char *argv[])
{
	return BQ40Z80::main(argc, argv);
}
