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
 * @file ecl_yaw_controller.cpp
 * Implementation of a simple orthogonal coordinated turn yaw PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_yaw_controller.h"
#include <float.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>

float ECL_YawController::control_attitude(const struct ECL_ControlData &ctl_data)
{
	switch (_coordinated_method) {
	case COORD_METHOD_OPEN:
		return control_attitude_impl_openloop(ctl_data);

	case COORD_METHOD_CLOSEACC:
		return control_attitude_impl_accclosedloop(ctl_data);

	default:
		static hrt_abstime last_print = 0;

		if (hrt_elapsed_time(&last_print) > 5e6) {
			PX4_WARN("invalid param setting FW_YCO_METHOD");
			last_print = hrt_absolute_time();
		}
	}

	return _rate_setpoint;
}

float ECL_YawController::control_attitude_impl_openloop(const struct ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.roll_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.pitch_rate_setpoint))) {

		return _rate_setpoint;
	}

	float constrained_roll;
	bool inverted = false;

	/* roll is used as feedforward term and inverted flight needs to be considered */
	if (fabsf(ctl_data.roll) < math::radians(90.0f)) {
		/* not inverted, but numerically still potentially close to infinity */
		constrained_roll = math::constrain(ctl_data.roll, math::radians(-80.0f), math::radians(80.0f));

	} else {
		inverted = true;

		// inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity
		//note: the ranges are extended by 10 deg here to avoid numeric resolution effects
		if (ctl_data.roll > 0.0f) {
			/* right hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(100.0f), math::radians(180.0f));

		} else {
			/* left hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(-180.0f), math::radians(-100.0f));
		}
	}

	constrained_roll = math::constrain(constrained_roll, -fabsf(ctl_data.roll_setpoint), fabsf(ctl_data.roll_setpoint));


	if (!inverted) {
		/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
		_rate_setpoint = tanf(constrained_roll) * cosf(ctl_data.pitch) * CONSTANTS_ONE_G / (ctl_data.airspeed <
				 ctl_data.airspeed_min ? ctl_data.airspeed_min : ctl_data.airspeed);
	}

	if (!PX4_ISFINITE(_rate_setpoint)) {
		PX4_WARN("yaw rate sepoint not finite");
		_rate_setpoint = 0.0f;
	}

	return _rate_setpoint;
}

float ECL_YawController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) && PX4_ISFINITE(ctl_data.pitch) && PX4_ISFINITE(ctl_data.body_y_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) && PX4_ISFINITE(ctl_data.pitch_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) && PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* get the usual dt estimate */
	uint64_t dt_micros = hrt_elapsed_time(&_last_run);
	_last_run = hrt_absolute_time();
	float dt = (float)dt_micros * 1e-6f;

	/* lock integral for long intervals */
	bool lock_integrator = ctl_data.lock_integrator;

	if (dt_micros > 500000) {
		lock_integrator = true;
	}

	/* input conditioning */
	float airspeed = ctl_data.airspeed;

	if (!PX4_ISFINITE(airspeed)) {
		/* airspeed is NaN, +- INF or not available, pick center of band */
		airspeed = 0.5f * (ctl_data.airspeed_min + ctl_data.airspeed_max);

	} else if (airspeed < ctl_data.airspeed_min) {
		airspeed = ctl_data.airspeed_min;
	}

	/* Close the acceleration loop if _coordinated_method wants this: change body_rate setpoint */
	if (_coordinated_method == COORD_METHOD_CLOSEACC) {
		// XXX lateral acceleration needs to go into integrator with a gain
		//_bodyrate_setpoint -= (ctl_data.acc_body_y / (airspeed * cosf(ctl_data.pitch)));
	}

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - ctl_data.body_z_rate; // body angular rate error

	if (!lock_integrator && _k_i > 0.0f && airspeed > 0.5f * ctl_data.airspeed_min) {

		float id = _rate_error * dt;

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
	_last_output = (_bodyrate_setpoint * _k_ff + _rate_error * _k_p + _integrator) * ctl_data.scaler *
		       ctl_data.scaler;  //scaler is proportional to 1/airspeed


	return math::constrain(_last_output, -1.0f, 1.0f);
}

float ECL_YawController::control_attitude_impl_accclosedloop(const struct ECL_ControlData &ctl_data)
{
	(void)ctl_data; // unused

	/* dont set a rate setpoint */
	return 0.0f;
}

float ECL_YawController::control_euler_rate(const struct ECL_ControlData &ctl_data)
{
	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = -sinf(ctl_data.roll) * ctl_data.pitch_rate_setpoint +
			     cosf(ctl_data.roll) * cosf(ctl_data.pitch) * _rate_setpoint;

	set_bodyrate_setpoint(_bodyrate_setpoint);

	return control_bodyrate(ctl_data);

}
