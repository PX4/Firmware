/****************************************************************************
 *
 *   Copyright (c) 2013-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_wheel_controller.cpp
 * Implementation of a simple PID wheel controller for heading tracking.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_wheel_controller.h"
#include <float.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

using matrix::wrap_pi;

float ECL_WheelController::control_bodyrate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.groundspeed) &&
	      PX4_ISFINITE(ctl_data.groundspeed_scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* input conditioning */
	float min_speed = 1.0f;

	/* Calculate body angular rate error */
	_rate_error = _rate_setpoint - ctl_data.body_z_rate; //body angular rate error

	if (!ctl_data.lock_integrator && _k_i > 0.0f && ctl_data.groundspeed > min_speed) {

		float id = _rate_error * dt * ctl_data.groundspeed_scaler;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		/* add and constrain */
		_integrator = math::constrain(_integrator + id * _k_i, -_integrator_max, _integrator_max);
	}

	/* Apply PI rate controller and store non-limited output */
	_last_output = _rate_setpoint * _k_ff * ctl_data.groundspeed_scaler +
		       ctl_data.groundspeed_scaler * ctl_data.groundspeed_scaler * (_rate_error * _k_p + _integrator);

	return math::constrain(_last_output, -1.0f, 1.0f);
}

float ECL_WheelController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.yaw_setpoint) &&
	      PX4_ISFINITE(ctl_data.yaw))) {

		return _rate_setpoint;
	}

	/* Calculate the error */
	float yaw_error = wrap_pi(ctl_data.yaw_setpoint - ctl_data.yaw);

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint =  yaw_error / _tc;

	/* limit the rate */
	if (_max_rate > 0.01f) {
		if (_rate_setpoint > 0.0f) {
			_rate_setpoint = (_rate_setpoint > _max_rate) ? _max_rate : _rate_setpoint;

		} else {
			_rate_setpoint = (_rate_setpoint < -_max_rate) ? -_max_rate : _rate_setpoint;
		}

	}

	return _rate_setpoint;
}
