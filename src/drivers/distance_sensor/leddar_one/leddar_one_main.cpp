/****************************************************************************
 *
 *   Copyright (C) 2017-2019 Intel Corporation. All rights reserved.
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

#include "LeddarOne.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace leddar_one
{

LeddarOne *g_dev{nullptr};

static int start(const char *port, const uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	g_dev = new LeddarOne(port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	// Start the driver.
	g_dev->start();

	return PX4_OK;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	}

	PX4_INFO("driver stopped");
	return PX4_OK;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the LeddarOne LiDAR.

Most boards are configured to enable/start the driver on a specified UART using the SENS_LEDDAR1_CFG parameter.

Setup/usage information: https://docs.px4.io/master/en/sensor/leddar_one.html

### Examples

Attempt to start driver on a specified serial device.
$ leddar_one start -d /dev/ttyS1
Stop driver
$ leddar_one stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("leddar_one", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int leddar_one_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch = 0;
	int myoptind = 1;

	const char *port = nullptr;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "d:r", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			port = myoptarg;
			break;

		case 'r':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option");
			return leddar_one::usage();
		}
	}

	if (myoptind >= argc) {
		return leddar_one::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return leddar_one::start(port, rotation);

	} else if (!strcmp(argv[myoptind], "status")) {
		return leddar_one::status();

	} else if (!strcmp(argv[myoptind], "stop")) {
		return leddar_one::stop();
	}

	return leddar_one::usage();
}
