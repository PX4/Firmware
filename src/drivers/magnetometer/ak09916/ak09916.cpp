/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * Driver for the standalone AK09916 magnetometer.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "ak09916.hpp"


extern "C" __EXPORT int ak09916_main(int argc, char *argv[]);


AK09916::AK09916(I2CSPIBusOption bus_option, const int bus, int bus_frequency, enum Rotation rotation) :
	I2C("AK09916", nullptr, bus, AK09916_I2C_ADDR, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_mag(get_device_id(), ORB_PRIO_MAX, rotation),
	_mag_reads(perf_alloc(PC_COUNT, "ak09916_mag_reads")),
	_mag_errors(perf_alloc(PC_COUNT, "ak09916_mag_errors")),
	_mag_overruns(perf_alloc(PC_COUNT, "ak09916_mag_overruns")),
	_mag_overflows(perf_alloc(PC_COUNT, "ak09916_mag_overflows"))
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_AK09916);
	_px4_mag.set_scale(AK09916_MAG_RANGE_GA);
}

AK09916::~AK09916()
{
	perf_free(_mag_reads);
	perf_free(_mag_errors);
	perf_free(_mag_overruns);
	perf_free(_mag_overflows);
}

int
AK09916::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		DEVICE_DEBUG("AK09916 mag init failed (%i)", ret);
		return ret;
	}

	ret = reset();

	if (ret != PX4_OK) {
		return ret;
	}

	start();

	return PX4_OK;
}

void
AK09916::try_measure()
{
	if (!is_ready()) {
		return;
	}

	measure();
}

bool
AK09916::is_ready()
{
	uint8_t st1;
	const int ret = transfer(&AK09916REG_ST1, sizeof(AK09916REG_ST1), &st1, sizeof(st1));

	if (ret != OK) {
		return false;
	}

	// Monitor if data overrun flag is ever set.
	if (st1 & AK09916_ST1_DOR) {
		perf_count(_mag_overruns);
	}

	return (st1 & AK09916_ST1_DRDY);
}

void
AK09916::measure()
{
	ak09916_regs regs;

	const hrt_abstime now = hrt_absolute_time();

	const int ret = transfer(&AK09916REG_HXL, sizeof(AK09916REG_HXL),
				 reinterpret_cast<uint8_t *>(&regs), sizeof(regs));

	if (ret != OK) {
		_px4_mag.set_error_count(perf_event_count(_mag_errors));
		return;
	}

	// Monitor if magnetic sensor overflow flag is set.
	if (regs.st2 & AK09916_ST2_HOFL) {
		perf_count(_mag_overflows);
	}

	_px4_mag.set_external(external());
	_px4_mag.update(now, regs.x, regs.y, regs.z);
}

void
AK09916::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_mag_reads);
	perf_print_counter(_mag_errors);
	perf_print_counter(_mag_overruns);
	_px4_mag.print_status();
}

uint8_t
AK09916::read_reg(uint8_t reg)
{
	const uint8_t cmd = reg;
	uint8_t ret{};

	transfer(&cmd, 1, &ret, 1);

	return ret;
}

bool
AK09916::check_id()
{
	const uint8_t deviceid = read_reg(AK09916REG_WIA);

	return (AK09916_DEVICE_ID_A == deviceid);
}

void
AK09916::write_reg(uint8_t reg, uint8_t value)
{
	const uint8_t cmd[2] = { reg, value};
	transfer(cmd, 2, nullptr, 0);
}

int
AK09916::reset()
{
	int rv = probe();

	if (rv == OK) {
		// Now reset the mag.
		write_reg(AK09916REG_CNTL3, AK09916_RESET);

		// Then re-initialize the bus/mag.
		rv = setup();
	}

	return rv;
}

int
AK09916::probe()
{
	int retries = 10;

	do {
		write_reg(AK09916REG_CNTL3, AK09916_RESET);

		if (check_id()) {
			return OK;
		}

		retries--;
	} while (retries > 0);

	return PX4_ERROR;
}

int
AK09916::setup()
{
	write_reg(AK09916REG_CNTL2, AK09916_CNTL2_CONTINOUS_MODE_100HZ);

	return OK;
}

void
AK09916::start()
{
	ScheduleNow();
}

void
AK09916::RunImpl()
{
	try_measure();
	ScheduleDelayed(_cycle_interval);
}

I2CSPIDriverBase *
AK09916::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator, int runtime_instance)
{
	AK09916 *interface = new AK09916(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.rotation);

	if (interface == nullptr) {
		PX4_ERR("failed creating interface for bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	return interface;
}

void
AK09916::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ak09916", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int
ak09916_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = AK09916;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;

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

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_MAG_DEVTYPE_AK09916);

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
	return 1;
}
