/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include "CameraFeedback.hpp"

CameraFeedback::CameraFeedback() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool
CameraFeedback::init()
{
	if (!_trigger_sub.registerCallback()) {
		PX4_ERR("camera_trigger callback registration failed!");
		return false;
	}

	return true;
}

void
CameraFeedback::Run()
{
	if (should_exit()) {
		_trigger_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	camera_trigger_s trig{};

	if (_trigger_sub.update(&trig)) {

		// update geotagging subscriptions
		vehicle_global_position_s gpos{};
		_gpos_sub.copy(&gpos);

		vehicle_attitude_s att{};
		_att_sub.copy(&att);

		if (trig.timestamp == 0 ||
		    gpos.timestamp == 0 ||
		    att.timestamp == 0) {

			// reject until we have valid data
			return;
		}

		camera_capture_s capture{};

		// Fill timestamps
		capture.timestamp = trig.timestamp;
		capture.timestamp_utc = trig.timestamp_utc;

		// Fill image sequence
		capture.seq = trig.seq;

		// Fill position data
		capture.lat = gpos.lat;
		capture.lon = gpos.lon;
		capture.alt = gpos.alt;

		if (gpos.terrain_alt_valid) {
			capture.ground_distance = gpos.alt - gpos.terrain_alt;

		} else {
			capture.ground_distance = -1.0f;
		}

		// Fill attitude data
		// TODO : this needs to be rotated by camera orientation or set to gimbal orientation when available
		capture.q[0] = att.q[0];
		capture.q[1] = att.q[1];
		capture.q[2] = att.q[2];
		capture.q[3] = att.q[3];
		capture.result = 1;

		_capture_pub.publish(capture);
	}
}

int
CameraFeedback::task_spawn(int argc, char *argv[])
{
	CameraFeedback *instance = new CameraFeedback();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
CameraFeedback::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
CameraFeedback::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("camera_feedback", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int camera_feedback_main(int argc, char *argv[])
{
	return CameraFeedback::main(argc, argv);
}
