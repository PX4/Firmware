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

#include "BMI055_accel.hpp"
#include "BMI055_gyro.hpp"
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

/** driver 'main' command */
extern "C" { __EXPORT int bmi055_main(int argc, char *argv[]); }

void
BMI055::print_usage()
{
	PRINT_MODULE_USAGE_NAME("bmi055", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('A', "Accel", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('G', "Gyro", true);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);

	PRINT_MODULE_USAGE_COMMAND("regdump");
	PRINT_MODULE_USAGE_COMMAND("testerror");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *BMI055::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	BMI055 *instance = nullptr;

	if (cli.type == DRV_ACC_DEVTYPE_BMI055) {
		instance = new BMI055_accel(iterator.configuredBusOption(), iterator.bus(), BMI055_DEVICE_PATH_ACCEL, iterator.devid(),
					    cli.rotation, cli.bus_frequency, cli.spi_mode);

	} else if (cli.type == DRV_GYR_DEVTYPE_BMI055) {
		instance = new BMI055_gyro(iterator.configuredBusOption(), iterator.bus(), BMI055_DEVICE_PATH_GYRO, iterator.devid(),
					   cli.rotation, cli.bus_frequency, cli.spi_mode);
	}

	if (instance == nullptr) {
		return nullptr;
	}

	if (OK != instance->init()) {
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		delete instance;
		return nullptr;
	}

	instance->start();

	return instance;
}

void
BMI055::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case 0: print_registers();
		break;

	case 1: test_error();
		break;
	}
}

BMI055::BMI055(const char *name, const char *devname, I2CSPIBusOption bus_option, int bus, int type, uint32_t device,
	       enum spi_mode_e mode,
	       uint32_t frequency, enum Rotation rotation):
	SPI(name, devname, bus, device, mode, frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, type),
	_whoami(0),
	_register_wait(0),
	_reset_wait(0),
	_rotation(rotation),
	_checked_next(0)
{
}

uint8_t
BMI055::read_reg(unsigned reg)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

uint16_t
BMI055::read_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

void
BMI055::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

int
bmi055_main(int argc, char *argv[])
{
	using ThisDriver = BMI055;
	int ch;
	BusCLIArguments cli{false, true};
	cli.type = 0;
	cli.default_spi_frequency = BMI055_BUS_SPEED;

	while ((ch = cli.getopt(argc, argv, "AGR:")) != EOF) {
		switch (ch) {
		case 'A':
			cli.type = DRV_ACC_DEVTYPE_BMI055;
			break;

		case 'G':
			cli.type = DRV_GYR_DEVTYPE_BMI055;
			break;

		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, cli.type);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "regdump")) {
		cli.custom1 = 0;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "testerror")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
