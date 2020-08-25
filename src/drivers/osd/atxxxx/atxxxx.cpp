/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file atxxxx.cpp
 * @author Daniele Pettenuzzo
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 * Driver for the ATXXXX chip (e.g. MAX7456) on the omnibus f4 fcu connected via SPI.
 */

#include "atxxxx.h"
#include "symbols.h"

using namespace time_literals;

static constexpr uint32_t OSD_UPDATE_RATE{50_ms};	// 20 Hz

OSDatxxxx::OSDatxxxx(I2CSPIBusOption bus_option, int bus, int devid, int bus_frequency, spi_mode_e spi_mode) :
	SPI("OSD", nullptr, bus, devid, spi_mode, bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus)
{
}

int
OSDatxxxx::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	if (ret != PX4_OK) {
		return ret;
	}

	ret = reset();

	if (ret != PX4_OK) {
		return ret;
	}

	ret = init_osd();

	if (ret != PX4_OK) {
		return ret;
	}

	// clear the screen
	int num_rows = (_param_osd_atxxxx_cfg.get() == 1 ? OSD_NUM_ROWS_NTSC : OSD_NUM_ROWS_PAL);

	for (int i = 0; i < OSD_CHARS_PER_ROW; i++) {
		for (int j = 0; j < num_rows; j++) {
			add_character_to_screen(' ', i, j);
		}
	}

	return ret;
}

int
OSDatxxxx::start()
{
	ScheduleOnInterval(OSD_UPDATE_RATE, 10000);

	return PX4_OK;
}

int
OSDatxxxx::probe()
{
	uint8_t data = 0;
	int ret = PX4_OK;

	ret |= writeRegister(0x00, 0x01); //disable video output
	ret |= readRegister(0x00, &data, 1);

	if (data != 1 || ret != PX4_OK) {
		PX4_ERR("probe failed (%i %i)", ret, data);
	}

	return ret;
}

int
OSDatxxxx::init_osd()
{
	int ret = PX4_OK;
	uint8_t data = OSD_ZERO_BYTE;

	if (_param_osd_atxxxx_cfg.get() == 2) {
		data |= OSD_PAL_TX_MODE;
	}

	ret |= writeRegister(0x00, data);
	ret |= writeRegister(0x04, OSD_ZERO_BYTE);

	enable_screen();

	return ret;
}

int
OSDatxxxx::readRegister(unsigned reg, uint8_t *data, unsigned count)
{
	uint8_t cmd[5] {}; // read up to 4 bytes

	cmd[0] = DIR_READ(reg);

	int ret = transfer(&cmd[0], &cmd[0], count + 1);

	if (ret != PX4_OK) {
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	memcpy(&data[0], &cmd[1], count);

	return ret;
}

int
OSDatxxxx::writeRegister(unsigned reg, uint8_t data)
{
	uint8_t cmd[2] {}; // write 1 byte

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	int ret = transfer(&cmd[0], nullptr, 2);

	if (OK != ret) {
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	return ret;
}

int
OSDatxxxx::add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y)
{
	uint16_t position = (OSD_CHARS_PER_ROW * pos_y) + pos_x;
	uint8_t position_lsb = 0;
	int ret = PX4_ERROR;

	if (position > 0xFF) {
		position_lsb = static_cast<uint8_t>(position) - 0xFF;
		ret = writeRegister(0x05, 0x01); //DMAH

	} else {
		position_lsb = static_cast<uint8_t>(position);
		ret = writeRegister(0x05, 0x00); //DMAH
	}

	if (ret != 0) {
		return ret;
	}

	ret = writeRegister(0x06, position_lsb); //DMAL

	if (ret != 0) {
		return ret;
	}

	ret = writeRegister(0x07, c);

	return ret;
}

void
OSDatxxxx::add_string_to_screen_centered(const char *str, uint8_t pos_y, int max_length)
{
	int len = strlen(str);

	if (len > max_length) {
		len = max_length;
	}

	int pos = (OSD_CHARS_PER_ROW - max_length) / 2;
	int before = (max_length - len) / 2;

	for (int i = 0; i < before; ++i) {
		add_character_to_screen(' ', pos++, pos_y);
	}

	for (int i = 0; i < len; ++i) {
		add_character_to_screen(str[i], pos++, pos_y);
	}

	while (pos < (OSD_CHARS_PER_ROW + max_length) / 2) {
		add_character_to_screen(' ', pos++, pos_y);
	}
}

void
OSDatxxxx::clear_line(uint8_t pos_x, uint8_t pos_y, int length)
{
	for (int i = 0; i < length; ++i) {
		add_character_to_screen(' ', pos_x + i, pos_y);
	}
}

int
OSDatxxxx::add_battery_info(uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];
	int ret = PX4_OK;

	// TODO: show battery symbol based on battery fill level
	snprintf(buf, sizeof(buf), "%c%5.2f", OSD_SYMBOL_BATT_3, (double)_battery_voltage_filtered_v);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen('V', pos_x + 5, pos_y);

	pos_y++;
	pos_x++;

	snprintf(buf, sizeof(buf), "%5d", (int)_battery_discharge_mah);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen(OSD_SYMBOL_MAH, pos_x + 5, pos_y);

	return ret;
}

int
OSDatxxxx::add_altitude(uint8_t pos_x, uint8_t pos_y)
{
	char buf[16];
	int ret = PX4_OK;

	snprintf(buf, sizeof(buf), "%c%5.2f%c", OSD_SYMBOL_ARROW_NORTH, (double)_local_position_z, OSD_SYMBOL_M);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	return ret;
}

int
OSDatxxxx::add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];
	int ret = PX4_OK;

	snprintf(buf, sizeof(buf), "%c%5.1f", OSD_SYMBOL_FLIGHT_TIME, (double)flight_time);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	return ret;
}

int
OSDatxxxx::enable_screen()
{
	uint8_t data = 0;
	int ret = PX4_OK;

	ret |= readRegister(0x00, &data, 1);
	ret |= writeRegister(0x00, data | 0x48);

	return ret;
}

int
OSDatxxxx::disable_screen()
{
	uint8_t data = 0;
	int ret = PX4_OK;

	ret |= readRegister(0x00, &data, 1);
	ret |= writeRegister(0x00, data & 0xF7);

	return ret;
}

int
OSDatxxxx::update_topics()
{
	/* update battery subscription */
	if (_battery_sub.updated()) {
		battery_status_s battery{};
		_battery_sub.copy(&battery);

		if (battery.connected) {
			_battery_voltage_filtered_v = battery.voltage_filtered_v;
			_battery_discharge_mah = battery.discharged_mah;
			_battery_valid = true;

		} else {
			_battery_valid = false;
		}
	}

	/* update vehicle local position subscription */
	if (_local_position_sub.updated()) {
		vehicle_local_position_s local_position{};
		_local_position_sub.copy(&local_position);

		_local_position_valid = local_position.z_valid;

		if (_local_position_valid) {
			_local_position_z = -local_position.z;
		}
	}

	/* update vehicle status subscription */
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
		    _arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			// arming
			_arming_timestamp = hrt_absolute_time();

		} else if (vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED &&
			   _arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			// disarming
		}

		_arming_state = vehicle_status.arming_state;
		_nav_state = vehicle_status.nav_state;
	}

	return PX4_OK;
}

const char *
OSDatxxxx::get_flight_mode(uint8_t nav_state)
{
	const char *flight_mode = "UNKNOWN";

	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		flight_mode = "MANUAL";
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		flight_mode = "ALTITUDE";
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		flight_mode = "POSITION";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		flight_mode = "RETURN";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		flight_mode = "MISSION";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		flight_mode = "AUTO";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
		flight_mode = "FAILURE";
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		flight_mode = "ACRO";
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		flight_mode = "TERMINATE";
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		flight_mode = "OFFBOARD";
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		flight_mode = "STABILIZED";
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		flight_mode = "RATTITUDE";
		break;
	}

	return flight_mode;
}

int
OSDatxxxx::update_screen()
{
	int ret = PX4_OK;

	if (_battery_valid) {
		ret |= add_battery_info(1, 1);

	} else {
		clear_line(1, 1, 10);
		clear_line(1, 2, 10);
	}

	if (_local_position_valid) {
		ret |= add_altitude(1, 3);

	} else {
		clear_line(1, 3, 10);
	}

	const char *flight_mode = "";

	if (_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		float flight_time_sec = static_cast<float>((hrt_absolute_time() - _arming_timestamp) / (1e6f));
		ret |= add_flighttime(flight_time_sec, 1, 14);

	} else {
		flight_mode = get_flight_mode(_nav_state);
	}

	add_string_to_screen_centered(flight_mode, 12, 10);

	return ret;
}

int
OSDatxxxx::reset()
{
	int ret = writeRegister(0x00, 0x02);
	usleep(100);

	return ret;
}

void
OSDatxxxx::RunImpl()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	update_topics();

	update_screen();
}

void
OSDatxxxx::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
OSD driver for the ATXXXX chip that is mounted on the OmnibusF4SD board for example.

It can be enabled with the OSD_ATXXXX_CFG parameter.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("atxxxx", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *OSDatxxxx::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				       int runtime_instance)
{
	OSDatxxxx *instance = new OSDatxxxx(iterator.configuredBusOption(), iterator.bus(), iterator.devid(),
					cli.bus_frequency, cli.spi_mode);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	instance->start();

	return instance;
}

int
atxxxx_main(int argc, char *argv[])
{
	using ThisDriver = OSDatxxxx;
	BusCLIArguments cli{false, true};
	cli.spi_mode = SPIDEV_MODE0;
	cli.default_spi_frequency = OSD_SPI_BUS_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_OSD_DEVTYPE_ATXXXX);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
