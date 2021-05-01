/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPositionHeadless.cpp
 */

#include "FlightTaskManualPositionHeadless.hpp"

using namespace matrix;

FlightTaskManualPositionHeadless::FlightTaskManualPositionHeadless() : _collision_prevention(this)
{

}

bool FlightTaskManualPositionHeadless::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	// all requirements from position-mode still have to hold
	bool ret = FlightTaskManualPosition::activate(last_setpoint);

	// set task specific constraint
	_reference_yaw = _yaw;

	// for headless-position-controlled mode, we need a reference yaw state
	return ret;
}

void FlightTaskManualPositionHeadless::_scaleSticks()
{
	/* Use same scaling as for FlightTaskManualAltitude */
	FlightTaskManualAltitude::_scaleSticks();

	/* Constrain length of stick inputs to 1 for xy*/
	Vector2f stick_xy = _sticks.getPositionExpo().slice<2, 1>(0, 0);

	const float mag = math::constrain(stick_xy.length(), 0.0f, 1.0f);

	if (mag > FLT_EPSILON) {
		stick_xy = stick_xy.normalized() * mag;
	}

	const float max_speed_from_estimator = _sub_vehicle_local_position.get().vxy_max;

	if (PX4_ISFINITE(max_speed_from_estimator)) {
		// use the minimum of the estimator and user specified limit
		_velocity_scale = fminf(_constraints.speed_xy, max_speed_from_estimator);
		// Allow for a minimum of 0.3 m/s for repositioning
		_velocity_scale = fmaxf(_velocity_scale, 0.3f);

	} else {
		_velocity_scale = _constraints.speed_xy;
	}

	_velocity_scale = fminf(_computeVelXYGroundDist(), _velocity_scale);

	// scale velocity to its maximum limits
	Vector2f vel_sp_xy = stick_xy * _velocity_scale;

	/* Rotate setpoint into local frame. */
	_rotateIntoHeadingFrame(vel_sp_xy);

	// collision prevention
	if (_collision_prevention.is_active()) {
		_collision_prevention.modifySetpoint(vel_sp_xy, _velocity_scale, _position.xy(), _velocity.xy());
	}

	_velocity_setpoint.xy() = vel_sp_xy;
}

void FlightTaskManualPositionHeadless::_rotateIntoHeadingFrame(Vector2f &v)
{
	float yaw_rotate = _reference_yaw;
	Vector3f v_r = Vector3f(Dcmf(Eulerf(0.0f, 0.0f, yaw_rotate)) * Vector3f(v(0), v(1), 0.0f));
	v(0) = v_r(0);
	v(1) = v_r(1);
}


float FlightTaskManualPositionHeadless::_computeVelXYGroundDist()
{
	float max_vel_xy = _constraints.speed_xy;

	// limit speed gradually within the altitudes MPC_LAND_ALT1 and MPC_LAND_ALT2
	if (PX4_ISFINITE(_dist_to_ground)) {
		max_vel_xy = math::gradual(_dist_to_ground,
					   _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
					   _param_mpc_land_vel_xy.get(), _constraints.speed_xy);
	}

	return max_vel_xy;
}
