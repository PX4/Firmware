/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * @file ecl_roll_controller.cpp
 * Implementation of a simple orthogonal roll PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_roll_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

float ECL_RollController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll))) {

		return _rate_setpoint;
	}

	/* Calculate the error */
	float roll_error = ctl_data.roll_setpoint - ctl_data.roll;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint = roll_error / _tc;

	return _rate_setpoint * _angle_error_gain_factor;
}

float ECL_RollController::control_bodyrate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.body_x_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - ctl_data.body_x_rate;

	if (!ctl_data.lock_integrator && _k_i > 0.0f) {

		/* Integral term scales with 1/IAS^2 */
		float id = _rate_error * dt * ctl_data.scaler * ctl_data.scaler;

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

	const float rate_gain = _k_p * ctl_data.scaler * ctl_data.scaler;
	const float rate_feed_forward = _bodyrate_setpoint * (rate_gain + _k_ff * ctl_data.scaler);
	const float rate_feed_back =  - ctl_data.body_x_rate * rate_gain;
	const float p_term = _bodyrate_setpoint * rate_gain + rate_feed_back;

	/* calculate gain compression factor required to prevent the rate feedback */
	/* term exceeding the actuator slew rate limit */
	if (_output_slew_rate_limit > 0.0f) {
		const float decay_tconst = _tc * 3.0f;
		_fb_limit_cycle_detector.set_parameters(_output_slew_rate_limit, decay_tconst);
		_ff_limit_cycle_detector.set_parameters(_output_slew_rate_limit, decay_tconst);
		_rate_gain_factor = _fb_limit_cycle_detector.calculate_gain_factor(rate_feed_back, dt);
		_angle_gain_factor = _ff_limit_cycle_detector.calculate_gain_factor(rate_feed_forward, dt);
		// If limit cycle is occurring in the demanded rate, then it is likely the root cause is
		// insufficient bandwidth in the rate controller so we should reduce rate demand and not
		// rate feedback.
		_rate_error_gain_factor = _angle_gain_factor / _rate_gain_factor;
		_angle_error_gain_factor = _angle_gain_factor;

	} else {
		_rate_error_gain_factor = 1.0f;
		_angle_error_gain_factor = 1.0f;
	}

	/* Apply PI rate controller and store non-limited output */
	/* FF terms scales with 1/TAS and P,I with 1/IAS^2 */
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       p_term * _rate_error_gain_factor +
		       _integrator;

	return math::constrain(_last_output, -1.0f, 1.0f);
}

float ECL_RollController::control_euler_rate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = ctl_data.roll_rate_setpoint - sinf(ctl_data.pitch) * ctl_data.yaw_rate_setpoint;

	set_bodyrate_setpoint(_bodyrate_setpoint);

	return control_bodyrate(dt, ctl_data);
}
