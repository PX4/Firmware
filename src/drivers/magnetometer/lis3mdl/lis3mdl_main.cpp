/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file lis3mdl_main.cpp
 *
 * Driver for the LIS3MDL magnetometer connected via I2C or SPI.
 */

#include "lis3mdl.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

I2CSPIDriverBase *LIS3MDL::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				       int runtime_instance)
{
	device::Device *interface = nullptr;

	if (iterator.busType() == BOARD_I2C_BUS) {
		interface = LIS3MDL_I2C_interface(iterator.bus(), cli.bus_frequency);

	} else if (iterator.busType() == BOARD_SPI_BUS) {
		interface = LIS3MDL_SPI_interface(iterator.bus(), iterator.devid(), cli.bus_frequency, cli.spi_mode);
	}

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	LIS3MDL *dev = new LIS3MDL(interface, cli.rotation, iterator.configuredBusOption(), iterator.bus());

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

void
LIS3MDL::custom_method(const BusCLIArguments &cli)
{
	reset();
}

void LIS3MDL::print_usage()
{
	PRINT_MODULE_USAGE_NAME("lis3mdl", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_COMMAND("reset");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int lis3mdl_main(int argc, char *argv[])
{
	using ThisDriver = LIS3MDL;
	int ch;
	BusCLIArguments cli{true, true};
	cli.default_i2c_frequency = 400000;
	cli.default_spi_frequency = 11 * 1000 * 1000;

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

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_MAG_DEVTYPE_LIS3MDL);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "reset")) {
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
