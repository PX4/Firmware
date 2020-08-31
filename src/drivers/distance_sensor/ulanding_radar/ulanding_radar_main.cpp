/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

#include "AerotennaULanding.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace ulanding_radar
{

AerotennaULanding *g_dev{nullptr};

static int start(const char *port, uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	if (port == nullptr) {
		PX4_ERR("serial port required");
		return -1;
	}

	// Instantiate the driver.
	g_dev = new AerotennaULanding(port, rotation);

	if (g_dev == nullptr) {
		return -1;
	}

	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	return 0;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		return -1;
	}

	return 0;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	g_dev->print_info();

	return 0;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the Aerotenna uLanding radar.

Setup/usage information: https://docs.px4.io/v1.9.0/en/sensor/ulanding_radar.html

### Examples

Attempt to start driver on a specified serial device.
$ ulanding_radar start -d /dev/ttyS1
Stop driver
$ ulanding_radar stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ulanding_radar", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	return PX4_OK;
}

} // namespace ulanding_radar

extern "C" __EXPORT int ulanding_radar_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			device_path = myoptarg;
			break;

		default:
			return ulanding_radar::usage();
		}
	}

	if (myoptind >= argc) {
		return ulanding_radar::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return ulanding_radar::start(device_path, rotation);

	} else if (!strcmp(argv[myoptind], "stop")) {
		return ulanding_radar::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return ulanding_radar::status();
	}

	return ulanding_radar::usage();
}
