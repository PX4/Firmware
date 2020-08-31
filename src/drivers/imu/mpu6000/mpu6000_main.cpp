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

#include "MPU6000.hpp"
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

void
MPU6000::print_usage()
{
	PRINT_MODULE_USAGE_NAME("mpu6000", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);

	PRINT_MODULE_USAGE_PARAM_STRING('T', "6000", "6000|20608|20602|20689", "Device type", true);

	PRINT_MODULE_USAGE_COMMAND("reset");
	PRINT_MODULE_USAGE_COMMAND("regdump");
#ifndef CONSTRAINED_FLASH
	PRINT_MODULE_USAGE_COMMAND("factorytest");
#endif
	PRINT_MODULE_USAGE_COMMAND("testerror");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void
MPU6000::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case 0: reset();
		break;

	case 1: print_registers();
		break;
#ifndef CONSTRAINED_FLASH

	case 2: factory_self_test();
		break;
#endif

	case 3: test_error();
		break;
	}
}

I2CSPIDriverBase *MPU6000::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				       int runtime_instance)
{
	device::Device *interface = nullptr;
	int device_type = cli.type;

	if (iterator.busType() == BOARD_I2C_BUS) {
		interface = MPU6000_I2C_interface(iterator.bus(), iterator.devid(), device_type, iterator.external(),
							  cli.bus_frequency);

	} else if (iterator.busType() == BOARD_SPI_BUS) {
		interface = MPU6000_SPI_interface(iterator.bus(), iterator.devid(), device_type, iterator.external(),
							  cli.bus_frequency, cli.spi_mode);
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

	MPU6000 *dev = new MPU6000(interface, cli.rotation, device_type, iterator.configuredBusOption(), iterator.bus());

	if (dev == nullptr) {
		delete interface;
		return nullptr;
	}

	if (OK != dev->init()) {
		delete dev;
		return nullptr;
	}

	dev->start();
	return dev;
}

/** driver 'main' command */
extern "C" { __EXPORT int mpu6000_main(int argc, char *argv[]); }

int
mpu6000_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = MPU6000;
	BusCLIArguments cli{true, true};
	cli.type = MPU_DEVICE_TYPE_MPU6000;
	cli.default_i2c_frequency = 400000;
	cli.default_spi_frequency = 1000 * 1000; // low speed bus frequency

	while ((ch = cli.getopt(argc, argv, "T:R:")) != EOF) {
		switch (ch) {
		case 'T':
			cli.type = atoi(cli.optarg());
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

	uint16_t dev_type_driver = 0;

	switch (cli.type) {
	case MPU_DEVICE_TYPE_MPU6000:
		dev_type_driver = DRV_IMU_DEVTYPE_MPU6000;
		break;

	case MPU_DEVICE_TYPE_ICM20602:
		dev_type_driver = DRV_IMU_DEVTYPE_ICM20602;
		break;

	case MPU_DEVICE_TYPE_ICM20608:
		dev_type_driver = DRV_IMU_DEVTYPE_ICM20608G;
		break;

	case MPU_DEVICE_TYPE_ICM20689:
		dev_type_driver = DRV_IMU_DEVTYPE_ICM20689;
		break;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, dev_type_driver);

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
		cli.custom1 = 0;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "regdump")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

#ifndef CONSTRAINED_FLASH

	if (!strcmp(verb, "factorytest")) {
		cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator);
	}

#endif

	if (!strcmp(verb, "testerror")) {
		cli.custom1 = 3;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
