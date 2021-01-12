/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <parameters/param.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <string.h>

using namespace time_literals;

struct FailureUnit {
	char key[16];
	uint8_t value;
};

static constexpr FailureUnit failure_units[] = {
	{ "gyro", vehicle_command_s::FAILURE_UNIT_SENSOR_GYRO},
	{ "accel", vehicle_command_s::FAILURE_UNIT_SENSOR_ACCEL},
	{ "mag", vehicle_command_s::FAILURE_UNIT_SENSOR_MAG},
	{ "baro", vehicle_command_s::FAILURE_UNIT_SENSOR_BARO},
	{ "gps", vehicle_command_s::FAILURE_UNIT_SENSOR_GPS},
	{ "optical_flow", vehicle_command_s::FAILURE_UNIT_SENSOR_OPTICAL_FLOW},
	{ "vio", vehicle_command_s::FAILURE_UNIT_SENSOR_VIO},
	{ "distance_sensor", vehicle_command_s::FAILURE_UNIT_SENSOR_DISTANCE_SENSOR},
	{ "airspeed", vehicle_command_s::FAILURE_UNIT_SENSOR_AIRSPEED},
	{ "battery", vehicle_command_s::FAILURE_UNIT_SYSTEM_BATTERY},
	{ "motor", vehicle_command_s::FAILURE_UNIT_SYSTEM_MOTOR},
	{ "servo", vehicle_command_s::FAILURE_UNIT_SYSTEM_SERVO},
	{ "avoidance", vehicle_command_s::FAILURE_UNIT_SYSTEM_AVOIDANCE},
	{ "rc_signal", vehicle_command_s::FAILURE_UNIT_SYSTEM_RC_SIGNAL},
	{ "mavlink_signal", vehicle_command_s::FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL},
};

struct FailureType {
	char key[14];
	uint8_t value;
};

static constexpr FailureType failure_types[] = {
	{ "ok", vehicle_command_s::FAILURE_TYPE_OK},
	{ "off", vehicle_command_s::FAILURE_TYPE_OFF},
	{ "stuck", vehicle_command_s::FAILURE_TYPE_STUCK},
	{ "garbage", vehicle_command_s::FAILURE_TYPE_GARBAGE},
	{ "wrong", vehicle_command_s::FAILURE_TYPE_WRONG},
	{ "slow", vehicle_command_s::FAILURE_TYPE_SLOW},
	{ "delayed", vehicle_command_s::FAILURE_TYPE_DELAYED},
	{ "intermittent", vehicle_command_s::FAILURE_TYPE_INTERMITTENT},
};

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Inject failures into system.

### Implementation
This system command sends a vehicle command over uORB to trigger failure.

### Examples
Test the GPS failsafe by stopping GPS:

failure gps off
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME_SIMPLE("failure", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("help", "Show this help text");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gps|...", "Specify component");
	PRINT_MODULE_USAGE_COMMAND_DESCR("ok|off|...", "Specify failure type");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 4, "sensor instance (0=all)", true);

	PX4_INFO_RAW("\nComponents:\n");
	for (const auto &failure_unit : failure_units) {
		PX4_INFO_RAW("- %s\n", failure_unit.key);
	}

	PX4_INFO_RAW("\nFailure types:\n");
	for (const auto &failure_type : failure_types) {
		PX4_INFO_RAW("- %s\n", failure_type.key);
	}
}

int inject_failure(uint8_t unit, uint8_t type, uint8_t instance)
{
	PX4_WARN("inject failure unit: %s (%d), type: %s (%d), instance: %d", failure_units[unit].key, unit, failure_types[type].key, type, instance);

	uORB::Subscription command_ack_sub{ORB_ID(vehicle_command_ack)};

	uORB::Publication<vehicle_command_s> command_pub{ORB_ID(vehicle_command)};
	vehicle_command_s command{};

	command.command = vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE;
	command.param1 = static_cast<float>(unit);
	command.param2 = static_cast<float>(type);
	command.param3 = static_cast<float>(instance);
	command.timestamp = hrt_absolute_time();
	command_pub.publish(command);

	vehicle_command_ack_s ack;

	while (hrt_elapsed_time(&command.timestamp) < 1_s) {
		if (command_ack_sub.update(&ack)) {
			if (ack.command == command.command) {
				if (ack.result != vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED) {
					PX4_ERR("Result: %d", ack.result);
					return 1;

				} else {
					return 0;
				}
			}
		}

		px4_usleep(10000);
	}

	PX4_ERR("Timeout waiting for ack");
	return 1;
}

extern "C" __EXPORT int failure_main(int argc, char *argv[])
{
	int32_t param = 0;

	if (PX4_OK != param_get(param_find("SYS_FAILURE_EN"), &param)) {
		PX4_ERR("Could not get param SYS_FAILURE_EN");
		return 1;
	}

	if (param != 1) {
		PX4_ERR("Failure injection disabled by SYS_FAILURE_EN param.");
		return 1;
	}

	if (argc == 2 && strcmp(argv[1], "help") == 0) {
		print_usage();
		return 0;
	}

	if (argc < 3) {
		PX4_ERR("Not enough arguments.");
		print_usage();
		return 1;
	}

	const char *myoptarg = nullptr;
	int ch = 0;
	int myoptind = 1;

	uint8_t instance = 0;

	while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			instance = (uint8_t)atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option");
			print_usage();
			return 1;
		}
	}

	const char *requested_failure_unit = argv[myoptind];

	for (const auto &failure_unit : failure_units) {
		if (strncmp(failure_unit.key, requested_failure_unit, sizeof(failure_unit.key)) != 0) {
			continue;
		}

		const char *requested_failure_type = argv[myoptind + 1];

		for (const auto &failure_type : failure_types) {
			if (strncmp(failure_type.key, requested_failure_type, sizeof(failure_type.key)) != 0) {
				continue;
			}

			return inject_failure(failure_unit.value, failure_type.value, instance);
		}

		PX4_ERR("Failure type '%s' not found", requested_failure_type);
		return 1;
	}

	PX4_ERR("Component '%s' not found", requested_failure_unit);
	return 1;
}
