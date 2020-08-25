/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include "L3GD20.hpp"

/**
 * Local functions in support of the shell command.
 */
namespace l3gd20
{

L3GD20 *g_dev{nullptr};

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */
static void start(bool external_bus, enum Rotation rotation)
{
	if (g_dev != nullptr) {
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_GYRO)
		g_dev = new L3GD20(PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_GYRO, rotation);
#else
		errx(0, "External SPI not available");
#endif

	} else {
		g_dev = new L3GD20(PX4_SPI_BUS_SENSORS, PX4_SPIDEV_GYRO, rotation);
	}

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Print a little info about the driver.
 */
static void info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running\n");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Dump the register information
 */
static void regdump()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();

	exit(0);
}

/**
 * trigger an error
 */
static void test_error()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->test_error();

	exit(0);
}

static void usage()
{
	warnx("missing command: try 'start', 'info', 'testerror' or 'regdump'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

extern "C" __EXPORT int l3gd20_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			l3gd20::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		l3gd20::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		l3gd20::start(external_bus, rotation);

	} else if (!strcmp(verb, "info")) {
		l3gd20::info();

	} else if (!strcmp(verb, "regdump")) {
		l3gd20::regdump();

	} else if (!strcmp(verb, "testerror")) {
		l3gd20::test_error();
	}

	l3gd20::usage();

	return -1;
}
