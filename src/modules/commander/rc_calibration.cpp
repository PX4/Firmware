/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file rc_calibration.cpp
 * Remote Control calibration routine
 */

#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/defines.h>

#include "rc_calibration.h"
#include "commander_helper.h"

#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <systemlib/mavlink_log.h>
#include <parameters/param.h>
#include <systemlib/err.h>

int do_trim_calibration(orb_advert_t *mavlink_log_pub)
{
	uORB::Subscription manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	px4_usleep(400000);
	manual_control_setpoint_s manual_control_setpoint{};
	bool changed = manual_control_setpoint_sub.updated();

	if (!changed) {
		mavlink_log_critical(mavlink_log_pub, "no inputs, aborting");
		return PX4_ERROR;
	}

	manual_control_setpoint_sub.copy(&manual_control_setpoint);

	/* load trim values which are active */
	float roll_trim_active;
	param_get(param_find("TRIM_ROLL"), &roll_trim_active);
	float pitch_trim_active;
	param_get(param_find("TRIM_PITCH"), &pitch_trim_active);
	float yaw_trim_active;
	param_get(param_find("TRIM_YAW"), &yaw_trim_active);

	/* get manual control scale values */
	float roll_scale;
	param_get(param_find("FW_MAN_R_SC"), &roll_scale);
	float pitch_scale;
	param_get(param_find("FW_MAN_P_SC"), &pitch_scale);
	float yaw_scale;
	param_get(param_find("FW_MAN_Y_SC"), &yaw_scale);

	/* set parameters: the new trim values are the combination of active trim values
	   and the values coming from the remote control of the user
	*/
	float p = manual_control_setpoint.y * roll_scale + roll_trim_active;
	int p1r = param_set(param_find("TRIM_ROLL"), &p);
	/*
	 we explicitly swap sign here because the trim is added to the actuator controls
	 which are moving in an inverse sense to manual pitch inputs
	*/
	p = -manual_control_setpoint.x * pitch_scale + pitch_trim_active;
	int p2r = param_set(param_find("TRIM_PITCH"), &p);
	p = manual_control_setpoint.r * yaw_scale + yaw_trim_active;
	int p3r = param_set(param_find("TRIM_YAW"), &p);

	if (p1r != 0 || p2r != 0 || p3r != 0) {
		mavlink_log_critical(mavlink_log_pub, "TRIM: PARAM SET FAIL");
		return PX4_ERROR;
	}

	mavlink_log_info(mavlink_log_pub, "trim cal done");

	return PX4_OK;
}
