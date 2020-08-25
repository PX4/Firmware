/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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
 * @file ll40ls.cpp
 * @author Allyson Kreft
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author James Goppert <james.goppert@gmail.com>
 *
 * Interface for the PulsedLight Lidar-Lite range finders.
 */

#include <cstdlib>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <systemlib/err.h>

#include <board_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "LidarLitePWM.h"

#ifdef LIDAR_LITE_PWM_SUPPORTED

/**
 * @brief Local functions in support of the shell command.
 */
namespace ll40ls_pwm
{

LidarLitePWM *instance = nullptr;

int start_pwm(const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int usage();

/**
 * Start the pwm driver.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_pwm(const uint8_t rotation)
{
	if (instance != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	instance = new LidarLitePWM(rotation);

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate the driver");
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (instance->init() != PX4_OK) {
		PX4_ERR("Failed to initialize LidarLite pwm.");
		delete instance;
		instance = nullptr;
		return PX4_ERROR;
	}

	// Start the driver.
	instance->start();

	PX4_INFO("driver started");
	return PX4_OK;
}

/**
 * @brief Prints status info about the driver.
 */
int
status()
{
	if (instance == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	instance->print_info();
	return PX4_OK;
}

/**
 * @brief Stops the driver
 */
int
stop()
{
	if (instance != nullptr) {
		delete instance;
		instance = nullptr;
	}

	return PX4_OK;
}

/**
 * @brief Displays driver usage at the console.
 */
int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

PWM driver for LidarLite rangefinders.

The sensor/driver must be enabled using the parameter SENS_EN_LL40LS.

Setup/usage information: https://docs.px4.io/en/sensor/lidar_lite.html
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ll40ls", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status information");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	return PX4_OK;
}

} // namespace ll40ls


/**
 * @brief Driver 'main' command.
 */
extern "C" __EXPORT int ll40ls_pwm_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch = 0;
	int myoptind = 1;

	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting Lidar orientation to %d", (int)rotation);
			break;

		default:
			return ll40ls_pwm::usage();
		}
	}

	if (myoptind >= argc) {
		return ll40ls_pwm::usage();
	}

	// Start the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return ll40ls_pwm::start_pwm(rotation);
	}

	// Print the driver status.
	if (!strcmp(argv[myoptind], "status")) {
		return ll40ls_pwm::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return ll40ls_pwm::stop();
	}

	// Print driver usage information.
	return ll40ls_pwm::usage();
}

#else
extern "C" __EXPORT int ll40ls_pwm_main(int argc, char *argv[])
{
	PX4_ERR("PWM input not supported.");
	return -1;
}
#endif
