/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file land_detector_main.cpp
 * Land detection algorithm
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include "FixedwingLandDetector.h"
#include "MulticopterLandDetector.h"
#include "RoverLandDetector.h"
#include "VtolLandDetector.h"
#include "AirshipLandDetector.h"


namespace land_detector
{

static char _currentMode[12];

int LandDetector::task_spawn(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return PX4_ERROR;
	}

	LandDetector *obj = nullptr;

	if (strcmp(argv[1], "fixedwing") == 0) {
		obj = new FixedwingLandDetector();

	} else if (strcmp(argv[1], "multicopter") == 0) {
		obj = new MulticopterLandDetector();

	} else if (strcmp(argv[1], "vtol") == 0) {
		obj = new VtolLandDetector();

	} else if (strcmp(argv[1], "rover") == 0) {
		obj = new RoverLandDetector();

	} else if (strcmp(argv[1], "airship") == 0) {
		obj = new AirshipLandDetector();

	} else {
		print_usage("unknown mode");
		return PX4_ERROR;
	}

	if (obj == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	// Remember current active mode
	strncpy(_currentMode, argv[1], sizeof(_currentMode) - 1);
	_currentMode[sizeof(_currentMode) - 1] = '\0';

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	obj->start();

	return PX4_OK;
}

int LandDetector::print_status()
{
	PX4_INFO("running (%s)", _currentMode);
	return 0;
}
int LandDetector::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module to detect the freefall and landed state of the vehicle, and publishing the `vehicle_land_detected` topic.
Each vehicle type (multirotor, fixedwing, vtol, ...) provides its own algorithm, taking into account various
states, such as commanded thrust, arming state and vehicle motion.

### Implementation
Every type is implemented in its own class with a common base class. The base class maintains a state (landed,
maybe_landed, ground_contact). Each possible state is implemented in the derived classes. A hysteresis and a fixed
priority of each internal state determines the actual land_detector state.

#### Multicopter Land Detector
**ground_contact**: thrust setpoint and velocity in z-direction must be below a defined threshold for time
GROUND_CONTACT_TRIGGER_TIME_US. When ground_contact is detected, the position controller turns off the thrust setpoint
in body x and y.

**maybe_landed**: it requires ground_contact together with a tighter thrust setpoint threshold and no velocity in the
horizontal direction. The trigger time is defined by MAYBE_LAND_TRIGGER_TIME. When maybe_landed is detected, the
position controller sets the thrust setpoint to zero.

**landed**: it requires maybe_landed to be true for time LAND_DETECTOR_TRIGGER_TIME_US.

The module runs periodically on the HP work queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("land_detector", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_ARG("fixedwing|multicopter|vtol|rover|airship", "Select vehicle type", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int land_detector_main(int argc, char *argv[])
{
	return LandDetector::main(argc, argv);
}

} // namespace land_detector
