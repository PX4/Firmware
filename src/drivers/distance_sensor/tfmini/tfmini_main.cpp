/****************************************************************************
 *
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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

#include "TFMINI.hpp"

#include <px4_platform_common/getopt.h>

/**
 * Local functions in support of the shell command.
 */
namespace tfmini
{

TFMINI	*g_dev{nullptr};

int start(const char *port, uint8_t rotation);
int status();
int stop();
int command(uint8_t *command, uint8_t framelen);
int usage();

int
start(const char *port, uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new TFMINI(port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

/**
 * Perform some commands the driver;
 * This can help to configure device.
 * Refer to your TFMINI documentation for the available commands.
 */
int
command(uint8_t *command, uint8_t framelen)
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	g_dev->write_command(command, framelen);

	int counter = 0;

	do {
		if (g_dev->get_command_result()) {
			uint8_t responselen;
			uint8_t *cresponse = g_dev->get_command_response(&responselen);
			char response[10];
			uint8_t idx;

			for (idx = 0; idx < responselen; idx++) {
				sprintf(response + idx * 2, "%02X", cresponse[idx]);
			}

			PX4_INFO("command confirmed [%ims] [%s]", 20 * counter, response);
			return PX4_OK;
		}

		px4_usleep(20000);
		counter++;
	} while (counter < 10); // wait 200ms for a command response - should be enough

	PX4_ERR("command not confirmed");
	return PX4_ERROR;
}

int stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		PX4_ERR("driver not running");
		return 1;
	}

	return PX4_OK;
}

int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the Benewake TFmini LiDAR.

Most boards are configured to enable/start the driver on a specified UART using the SENS_TFMINI_CFG parameter.

Setup/usage information: https://docs.px4.io/master/en/sensor/tfmini.html

### Examples

Attempt to start driver on a specified serial device.
$ tfmini start -d /dev/ttyS1
Stop driver
$ tfmini stop
Retreive the version on a TFMINI-Plus: (0x5A 0x04 0x01 0x5F)
$ tfmini command -c 5A04015F
Configure TFMINI-Plus in Standard 9 bytes (cm) mode: (0x5A 0x05 0x05 0x01 0x65)
$ tfmini command -c 5A05050165
Configure TFMINI-Plus Frame Rate: (0x5A 0x06 0x03 0xLL 0xHH 0xSU) - Example for 100Hz
$ tfmini command -c 5A06036400C7
Apply and Save Settings for TFMINI-Plus: (0x5A 0x04 0x11 0x6F)
$ tfmini command -c 5A04116F
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tfmini", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("command","Configure driver (require -c hexacommand as in -c 5A04015F)");
	PRINT_MODULE_USAGE_PARAM_STRING('c', nullptr, nullptr, "hexacommand", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int tfmini_main(int argc, char *argv[])
{
	int ch = 0;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = TFMINI_DEFAULT_PORT;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	const char *hexacommand = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:c:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			device_path = myoptarg;
			break;

		case 'c':
			hexacommand = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		PX4_ERR("unrecognized command");
		return tfmini::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return tfmini::start(device_path, rotation);

		} else {
			PX4_WARN("Please specify device path!");
			return tfmini::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return tfmini::stop();

	} else if (!strcmp(argv[myoptind], "command")) {
		uint8_t mycommandlen=0;
		uint8_t mycommand[8];

		if (hexacommand == nullptr)
			return tfmini::usage();

		uint8_t len = strlen(hexacommand);

		if (len>16) {
			PX4_ERR("invalid command - too long");
			return PX4_ERROR;
		}

		//convert hexacommand to a table of of uint8_t
		while (mycommandlen<len/2) {
			char val[3];
			val[0]=hexacommand[2*mycommandlen];
			val[1]=hexacommand[2*mycommandlen+1];
			val[2]='\0';
			mycommand[mycommandlen]=(uint8_t)strtol(val, NULL, 16);
			mycommandlen++;
		}
		return tfmini::command(mycommand, mycommandlen);

	} else if (!strcmp(argv[myoptind], "status")) {
		return tfmini::status();
	}

	return tfmini::usage();
}
