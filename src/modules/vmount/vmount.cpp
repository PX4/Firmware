/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file vmount.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 * @author Julian Oes <julian@oes.ch>
 * @author Matthew Edwards (mje-nz)
 *
 * Driver for to control mounts such as gimbals or servos.
 * Inputs for the mounts can RC and/or mavlink commands.
 * Outputs to the mounts can be RC (PWM) output or mavlink.
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>

#include "input_mavlink.h"
#include "input_rc.h"
#include "input_test.h"
#include "output_rc.h"
#include "output_mavlink.h"

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>

using namespace vmount;

/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;

static constexpr int input_objs_len_max = 3;

struct ThreadData {
	InputBase *input_objs[input_objs_len_max] = {nullptr, nullptr, nullptr};
	int input_objs_len = 0;
	OutputBase *output_obj = nullptr;
};
static volatile ThreadData *g_thread_data = nullptr;

struct Parameters {
	int32_t mnt_mode_in;
	int32_t mnt_mode_out;
	int32_t mnt_mav_sysid;
	int32_t mnt_mav_compid;
	float mnt_ob_lock_mode;
	float mnt_ob_norm_mode;
	int32_t mnt_man_pitch;
	int32_t mnt_man_roll;
	int32_t mnt_man_yaw;
	int32_t mnt_do_stab;
	float mnt_range_pitch;
	float mnt_range_roll;
	float mnt_range_yaw;
	float mnt_off_pitch;
	float mnt_off_roll;
	float mnt_off_yaw;

	bool operator!=(const Parameters &p)
	{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
		return mnt_mode_in != p.mnt_mode_in ||
		       mnt_mode_out != p.mnt_mode_out ||
		       mnt_mav_sysid != p.mnt_mav_sysid ||
		       mnt_mav_compid != p.mnt_mav_compid ||
		       fabsf(mnt_ob_lock_mode - p.mnt_ob_lock_mode) > 1e-6f ||
		       fabsf(mnt_ob_norm_mode - p.mnt_ob_norm_mode) > 1e-6f ||
		       mnt_man_pitch != p.mnt_man_pitch ||
		       mnt_man_roll != p.mnt_man_roll ||
		       mnt_man_yaw != p.mnt_man_yaw ||
		       mnt_do_stab != p.mnt_do_stab ||
		       mnt_range_pitch != p.mnt_range_pitch ||
		       mnt_range_roll != p.mnt_range_roll ||
		       mnt_range_yaw != p.mnt_range_yaw ||
		       mnt_off_pitch != p.mnt_off_pitch ||
		       mnt_off_roll != p.mnt_off_roll ||
		       mnt_off_yaw != p.mnt_off_yaw;
#pragma GCC diagnostic pop

	}
};

struct ParameterHandles {
	param_t mnt_mode_in;
	param_t mnt_mode_out;
	param_t mnt_mav_sysid;
	param_t mnt_mav_compid;
	param_t mnt_ob_lock_mode;
	param_t mnt_ob_norm_mode;
	param_t mnt_man_pitch;
	param_t mnt_man_roll;
	param_t mnt_man_yaw;
	param_t mnt_do_stab;
	param_t mnt_range_pitch;
	param_t mnt_range_roll;
	param_t mnt_range_yaw;
	param_t mnt_off_pitch;
	param_t mnt_off_roll;
	param_t mnt_off_yaw;
};


/* functions */
static void usage();
static void update_params(ParameterHandles &param_handles, Parameters &params, bool &got_changes);
static bool get_params(ParameterHandles &param_handles, Parameters &params);

static int vmount_thread_main(int argc, char *argv[]);
extern "C" __EXPORT int vmount_main(int argc, char *argv[]);

static int vmount_thread_main(int argc, char *argv[])
{
	ParameterHandles param_handles;
	Parameters params {};
	OutputConfig output_config;
	ThreadData thread_data;

	InputTest *test_input = nullptr;

#ifdef __PX4_NUTTX
	/* the NuttX optarg handler does not
	 * ignore argv[0] like the POSIX handler
	 * does, nor does it deal with non-flag
	 * verbs well. So we Remove the application
	 * name and the verb.
	 */
	argc -= 1;
	argv += 1;
#endif

	if (argc > 0 && !strcmp(argv[0], "test")) {
		PX4_INFO("Starting in test mode");

		const char *axis_names[3] = {"roll", "pitch", "yaw"};
		float angles[3] = { 0.f, 0.f, 0.f };

		if (argc == 3) {
			bool found_axis = false;

			for (int i = 0 ; i < 3; ++i) {
				if (!strcmp(argv[1], axis_names[i])) {
					long angle_deg = strtol(argv[2], nullptr, 0);
					angles[i] = (float)angle_deg;
					found_axis = true;
				}
			}

			if (!found_axis) {
				usage();
				return -1;
			}

			test_input = new InputTest(angles[0], angles[1], angles[2]);

			if (!test_input) {
				PX4_ERR("memory allocation failed");
				return -1;
			}

		} else {
			usage();
			return -1;
		}
	}

	if (!get_params(param_handles, params)) {
		PX4_ERR("could not get mount parameters!");
		delete test_input;
		return -1;
	}

	uORB::Subscription parameter_update_sub{ORB_ID(parameter_update)};
	thread_running = true;
	ControlData *control_data = nullptr;
	g_thread_data = &thread_data;

	int last_active = -1;

	while (!thread_should_exit) {

		if (!thread_data.input_objs[0] && (params.mnt_mode_in >= 0 || test_input)) { //need to initialize

			output_config.gimbal_normal_mode_value = params.mnt_ob_norm_mode;
			output_config.gimbal_retracted_mode_value = params.mnt_ob_lock_mode;
			output_config.pitch_scale = 1.0f / ((params.mnt_range_pitch / 2.0f) * M_DEG_TO_RAD_F);
			output_config.roll_scale = 1.0f / ((params.mnt_range_roll / 2.0f) * M_DEG_TO_RAD_F);
			output_config.yaw_scale = 1.0f / ((params.mnt_range_yaw / 2.0f) * M_DEG_TO_RAD_F);
			output_config.pitch_offset = params.mnt_off_pitch * M_DEG_TO_RAD_F;
			output_config.roll_offset = params.mnt_off_roll * M_DEG_TO_RAD_F;
			output_config.yaw_offset = params.mnt_off_yaw * M_DEG_TO_RAD_F;
			output_config.mavlink_sys_id = params.mnt_mav_sysid;
			output_config.mavlink_comp_id = params.mnt_mav_compid;

			bool alloc_failed = false;
			thread_data.input_objs_len = 1;

			if (test_input) {
				thread_data.input_objs[0] = test_input;

			} else {
				switch (params.mnt_mode_in) {
				case 0:

					// Automatic
					thread_data.input_objs[0] = new InputMavlinkCmdMount(params.mnt_do_stab);
					thread_data.input_objs[1] = new InputMavlinkROI();

					// RC is on purpose last here so that if there are any mavlink
					// messages, they will take precedence over RC.
					// This logic is done further below while update() is called.
					thread_data.input_objs[2] = new InputRC(params.mnt_do_stab, params.mnt_man_roll, params.mnt_man_pitch,
										params.mnt_man_yaw);
					thread_data.input_objs_len = 3;

					break;

				case 1: //RC
					thread_data.input_objs[0] = new InputRC(params.mnt_do_stab, params.mnt_man_roll, params.mnt_man_pitch,
										params.mnt_man_yaw);
					break;

				case 2: //MAVLINK_ROI
					thread_data.input_objs[0] = new InputMavlinkROI();
					break;

				case 3: //MAVLINK_DO_MOUNT
					thread_data.input_objs[0] = new InputMavlinkCmdMount(params.mnt_do_stab);
					break;

				default:
					PX4_ERR("invalid input mode %i", params.mnt_mode_in);
					break;
				}
			}

			for (int i = 0; i < thread_data.input_objs_len; ++i) {
				if (!thread_data.input_objs[i]) {
					alloc_failed = true;
				}
			}

			switch (params.mnt_mode_out) {
			case 0: //AUX
				thread_data.output_obj = new OutputRC(output_config);

				if (!thread_data.output_obj) { alloc_failed = true; }

				break;

			case 1: //MAVLINK
				thread_data.output_obj = new OutputMavlink(output_config);

				if (!thread_data.output_obj) { alloc_failed = true; }

				break;

			default:
				PX4_ERR("invalid output mode %i", params.mnt_mode_out);
				thread_should_exit = true;
				break;
			}

			if (alloc_failed) {
				thread_data.input_objs_len = 0;
				PX4_ERR("memory allocation failed");
				thread_should_exit = true;
			}

			if (thread_should_exit) {
				break;
			}

			int ret = thread_data.output_obj->initialize();

			if (ret) {
				PX4_ERR("failed to initialize output mode (%i)", ret);
				thread_should_exit = true;
				break;
			}
		}

		if (thread_data.input_objs_len > 0) {

			//get input: we cannot make the timeout too large, because the output needs to update
			//periodically for stabilization and angle updates.

			for (int i = 0; i < thread_data.input_objs_len; ++i) {

				bool already_active = (last_active == i);

				ControlData *control_data_to_check = nullptr;
				unsigned int poll_timeout = already_active ? 50 : 0; // poll only on active input to reduce latency
				int ret = thread_data.input_objs[i]->update(poll_timeout, &control_data_to_check, already_active);

				if (ret) {
					PX4_ERR("failed to read input %i (ret: %i)", i, ret);
					continue;
				}

				if (control_data_to_check != nullptr || already_active) {
					control_data = control_data_to_check;
					last_active = i;
				}
			}

			//update output
			int ret = thread_data.output_obj->update(control_data);

			if (ret) {
				PX4_ERR("failed to write output (%i)", ret);
				break;
			}

			//only publish the mount orientation if the mode is not mavlink
			//if the gimbal speaks mavlink it publishes its own orientation
			if (params.mnt_mode_out != 1) { // 1 = MAVLINK
				thread_data.output_obj->publish();
			}


		} else {
			//wait for parameter changes. We still need to wake up regularily to check for thread exit requests
			px4_usleep(1e6);
		}

		if (test_input && test_input->finished()) {
			thread_should_exit = true;
			break;
		}

		// check for parameter updates
		if (parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			bool updated = false;
			update_params(param_handles, params, updated);

			if (updated) {
				//re-init objects
				for (int i = 0; i < input_objs_len_max; ++i) {
					if (thread_data.input_objs[i]) {
						delete (thread_data.input_objs[i]);
						thread_data.input_objs[i] = nullptr;
					}
				}

				thread_data.input_objs_len = 0;

				last_active = -1;

				if (thread_data.output_obj) {
					delete (thread_data.output_obj);
					thread_data.output_obj = nullptr;
				}
			}
		}
	}

	g_thread_data = nullptr;

	for (int i = 0; i < input_objs_len_max; ++i) {
		if (thread_data.input_objs[i]) {
			delete (thread_data.input_objs[i]);
			thread_data.input_objs[i] = nullptr;
		}
	}

	thread_data.input_objs_len = 0;

	if (thread_data.output_obj) {
		delete (thread_data.output_obj);
		thread_data.output_obj = nullptr;
	}

	thread_running = false;
	return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int vmount_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_ERR("missing command");
		usage();
		return -1;
	}

	const bool found_start = !strcmp(argv[1], "start");
	const bool found_test = !strcmp(argv[1], "test");

	if (found_start || found_test) {

		/* this is not an error */
		if (thread_running) {
			if (found_start) {
				PX4_WARN("mount driver already running");
				return 0;

			} else {
				PX4_WARN("mount driver already running, run vmount stop before 'vmount test'");
				return 1;
			}
		}

		thread_should_exit = false;
		int vmount_task = px4_task_spawn_cmd("vmount",
						     SCHED_DEFAULT,
						     SCHED_PRIORITY_DEFAULT,
						     1900,
						     vmount_thread_main,
						     (char *const *)argv + 1);

		int counter = 0;

		while (!thread_running && vmount_task >= 0) {
			px4_usleep(5000);

			if (++counter >= 100) {
				break;
			}
		}

		if (vmount_task < 0) {
			PX4_ERR("failed to start");
			return -1;
		}

		return counter < 100 || thread_should_exit ? 0 : -1;
	}

	if (!strcmp(argv[1], "stop")) {

		/* this is not an error */
		if (!thread_running) {
			PX4_WARN("mount driver not running");
			return 0;
		}

		thread_should_exit = true;

		while (thread_running) {
			px4_usleep(100000);
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running && g_thread_data) {

			for (int i = 0; i < g_thread_data->input_objs_len; ++i) {
				g_thread_data->input_objs[i]->print_status();
			}

			if (g_thread_data->input_objs_len == 0) {
				PX4_INFO("Input: None");
			}

			if (g_thread_data->output_obj) {
				g_thread_data->output_obj->print_status();

			} else {
				PX4_INFO("Output: None");
			}

		} else {
			PX4_INFO("not running");
		}

		return 0;
	}

	PX4_ERR("unrecognized command");
	usage();
	return -1;
}

void update_params(ParameterHandles &param_handles, Parameters &params, bool &got_changes)
{
	Parameters prev_params = params;
	param_get(param_handles.mnt_mode_in, &params.mnt_mode_in);
	param_get(param_handles.mnt_mode_out, &params.mnt_mode_out);
	param_get(param_handles.mnt_mav_sysid, &params.mnt_mav_sysid);
	param_get(param_handles.mnt_mav_compid, &params.mnt_mav_compid);
	param_get(param_handles.mnt_ob_lock_mode, &params.mnt_ob_lock_mode);
	param_get(param_handles.mnt_ob_norm_mode, &params.mnt_ob_norm_mode);
	param_get(param_handles.mnt_man_pitch, &params.mnt_man_pitch);
	param_get(param_handles.mnt_man_roll, &params.mnt_man_roll);
	param_get(param_handles.mnt_man_yaw, &params.mnt_man_yaw);
	param_get(param_handles.mnt_do_stab, &params.mnt_do_stab);
	param_get(param_handles.mnt_range_pitch, &params.mnt_range_pitch);
	param_get(param_handles.mnt_range_roll, &params.mnt_range_roll);
	param_get(param_handles.mnt_range_yaw, &params.mnt_range_yaw);
	param_get(param_handles.mnt_off_pitch, &params.mnt_off_pitch);
	param_get(param_handles.mnt_off_roll, &params.mnt_off_roll);
	param_get(param_handles.mnt_off_yaw, &params.mnt_off_yaw);

	got_changes = prev_params != params;
}

bool get_params(ParameterHandles &param_handles, Parameters &params)
{
	param_handles.mnt_mode_in = param_find("MNT_MODE_IN");
	param_handles.mnt_mode_out = param_find("MNT_MODE_OUT");
	param_handles.mnt_mav_sysid = param_find("MNT_MAV_SYSID");
	param_handles.mnt_mav_compid = param_find("MNT_MAV_COMPID");
	param_handles.mnt_ob_lock_mode = param_find("MNT_OB_LOCK_MODE");
	param_handles.mnt_ob_norm_mode = param_find("MNT_OB_NORM_MODE");
	param_handles.mnt_man_pitch = param_find("MNT_MAN_PITCH");
	param_handles.mnt_man_roll = param_find("MNT_MAN_ROLL");
	param_handles.mnt_man_yaw = param_find("MNT_MAN_YAW");
	param_handles.mnt_do_stab = param_find("MNT_DO_STAB");
	param_handles.mnt_range_pitch = param_find("MNT_RANGE_PITCH");
	param_handles.mnt_range_roll = param_find("MNT_RANGE_ROLL");
	param_handles.mnt_range_yaw = param_find("MNT_RANGE_YAW");
	param_handles.mnt_off_pitch = param_find("MNT_OFF_PITCH");
	param_handles.mnt_off_roll = param_find("MNT_OFF_ROLL");
	param_handles.mnt_off_yaw = param_find("MNT_OFF_YAW");

	if (param_handles.mnt_mode_in == PARAM_INVALID ||
	    param_handles.mnt_mode_out == PARAM_INVALID ||
	    param_handles.mnt_mav_sysid == PARAM_INVALID ||
	    param_handles.mnt_mav_compid == PARAM_INVALID ||
	    param_handles.mnt_ob_lock_mode == PARAM_INVALID ||
	    param_handles.mnt_ob_norm_mode == PARAM_INVALID ||
	    param_handles.mnt_man_pitch == PARAM_INVALID ||
	    param_handles.mnt_man_roll == PARAM_INVALID ||
	    param_handles.mnt_man_yaw == PARAM_INVALID ||
	    param_handles.mnt_do_stab == PARAM_INVALID ||
	    param_handles.mnt_range_pitch == PARAM_INVALID ||
	    param_handles.mnt_range_roll == PARAM_INVALID ||
	    param_handles.mnt_range_yaw == PARAM_INVALID ||
	    param_handles.mnt_off_pitch == PARAM_INVALID ||
	    param_handles.mnt_off_roll == PARAM_INVALID ||
	    param_handles.mnt_off_yaw == PARAM_INVALID) {
		return false;
	}

	bool dummy;
	update_params(param_handles, params, dummy);
	return true;
}

static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Mount (Gimbal) control driver. It maps several different input methods (eg. RC or MAVLink) to a configured
output (eg. AUX channels or MAVLink).

Documentation how to use it is on the [gimbal_control](https://dev.px4.io/master/en/advanced/gimbal_control.html) page.

### Implementation
Each method is implemented in its own class, and there is a common base class for inputs and outputs.
They are connected via an API, defined by the `ControlData` data structure. This makes sure that each input method
can be used with each output method and new inputs/outputs can be added with minimal effort.

### Examples
Test the output by setting a fixed yaw angle (and the other axes to 0):
$ vmount stop
$ vmount test yaw 30
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vmount", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test the output: set a fixed angle for one axis (vmount must not be running)");
	PRINT_MODULE_USAGE_ARG("roll|pitch|yaw <angle>", "Specify an axis and an angle in degrees", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}
