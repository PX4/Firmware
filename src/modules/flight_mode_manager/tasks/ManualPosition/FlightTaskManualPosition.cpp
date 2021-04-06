/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPosition.cpp
 */

#include "FlightTaskManualPosition.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

FlightTaskManualPosition::FlightTaskManualPosition() : _collision_prevention(this)
{

}

bool FlightTaskManualPosition::updateInitialize()
{
	bool ret = FlightTaskManualAltitude::updateInitialize();
	// require valid position / velocity in xy
	return ret && PX4_ISFINITE(_position(0))
	       && PX4_ISFINITE(_position(1))
	       && PX4_ISFINITE(_velocity(0))
	       && PX4_ISFINITE(_velocity(1));
}

bool FlightTaskManualPosition::activate(const trajectory_setpoint_s &last_setpoint)
{
	// all requirements from altitude-mode still have to hold
	bool ret = FlightTaskManualAltitude::activate(last_setpoint);

	// set task specific constraint
	if (_constraints.speed_xy >= _param_mpc_vel_manual.get()) {
		_constraints.speed_xy = _param_mpc_vel_manual.get();
	}

	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);
	_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
	_velocity_scale = _constraints.speed_xy;

	// for position-controlled mode, we need a valid position and velocity state
	// in NE-direction
	return ret;
}

void FlightTaskManualPosition::_scaleSticks()
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

float FlightTaskManualPosition::_computeVelXYGroundDist()
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

void FlightTaskManualPosition::_updateXYlock()
{
	/* If position lock is not active, position setpoint is set to NAN.*/
	const float vel_xy_norm = Vector2f(_velocity).length();
	const bool apply_brake = Vector2f(_velocity_setpoint).length() < FLT_EPSILON;
	const bool stopped = (_param_mpc_hold_max_xy.get() < FLT_EPSILON || vel_xy_norm < _param_mpc_hold_max_xy.get());

	if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(0))) {
		_position_setpoint(0) = _position(0);
		_position_setpoint(1) = _position(1);

	} else if (PX4_ISFINITE(_position_setpoint(0)) && apply_brake) {
		// Position is locked but check if a reset event has happened.
		// We will shift the setpoints.
		if (_sub_vehicle_local_position.get().xy_reset_counter != _reset_counter) {
			_position_setpoint(0) = _position(0);
			_position_setpoint(1) = _position(1);
			_reset_counter = _sub_vehicle_local_position.get().xy_reset_counter;
		}

	} else {
		/* don't lock*/
		_position_setpoint(0) = NAN;
		_position_setpoint(1) = NAN;
	}
}

void FlightTaskManualPosition::_updateSetpoints()
{
	FlightTaskManualAltitude::_updateSetpoints(); // needed to get yaw and setpoints in z-direction
	_acceleration_setpoint.setNaN(); // don't use the horizontal setpoints from FlightTaskAltitude

	_updateXYlock(); // check for position lock

	// check if an external yaw handler is active and if yes, let it update the yaw setpoints
	if (_weathervane_yaw_handler != nullptr && _weathervane_yaw_handler->is_active()) {
		_yaw_setpoint = NAN;

		// only enable the weathervane to change the yawrate when position lock is active (and thus the pos. sp. are NAN)
		if (PX4_ISFINITE(_position_setpoint(0)) && PX4_ISFINITE(_position_setpoint(1))) {
			// vehicle is steady
			_yawspeed_setpoint += _weathervane_yaw_handler->get_weathervane_yawrate();
		}
	}
}
