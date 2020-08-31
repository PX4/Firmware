/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include "FXAS21002C.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

void
FXAS21002C::print_usage()
{
	PRINT_MODULE_USAGE_NAME("fxas21002c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, true);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x20);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_COMMAND("regdump");
	PRINT_MODULE_USAGE_COMMAND("testerror");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *FXAS21002C::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	device::Device *interface = nullptr;

	if (iterator.busType() == BOARD_I2C_BUS) {
		interface = FXAS21002C_I2C_interface(iterator.bus(), cli.bus_frequency, cli.i2c_address);

	} else if (iterator.busType() == BOARD_SPI_BUS) {
		interface = FXAS21002C_SPI_interface(iterator.bus(), iterator.devid(), cli.bus_frequency, cli.spi_mode);
	}

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	FXAS21002C *dev = new FXAS21002C(interface, iterator.configuredBusOption(), iterator.bus(), cli.rotation,
					 cli.i2c_address);

	if (dev == nullptr) {
		delete interface;
		return nullptr;
	}

	if (OK != dev->init()) {
		delete dev;
		return nullptr;
	}

	return dev;
}

void FXAS21002C::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case 0: print_registers(); break;

	case 1: test_error(); break;
	}

}

extern "C" int fxas21002c_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = FXAS21002C;
	BusCLIArguments cli{true, true};
	cli.default_i2c_frequency = 400 * 1000;
	cli.default_spi_frequency = 2 * 1000 * 1000;
	cli.spi_mode = SPIDEV_MODE0;
	cli.i2c_address = 0x20;

	while ((ch = cli.getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
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

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_GYR_DEVTYPE_FXAS2100C);

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
	return PX4_ERROR;
}
