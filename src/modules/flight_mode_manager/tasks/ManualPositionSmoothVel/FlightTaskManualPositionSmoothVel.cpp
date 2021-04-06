/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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

#include "FlightTaskManualPositionSmoothVel.hpp"

#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskManualPositionSmoothVel::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualPosition::activate(last_setpoint);

	// Check if the previous FlightTask provided setpoints
	Vector3f accel_prev{last_setpoint.acceleration};
	Vector3f vel_prev{last_setpoint.velocity};
	Vector3f pos_prev{last_setpoint.position};

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current postion
		if (!PX4_ISFINITE(pos_prev(i))) { pos_prev(i) = _position(i); }

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) { vel_prev(i) = _velocity(i); }

		// No acceleration estimate available, set to zero if the setpoint is NAN
		if (!PX4_ISFINITE(accel_prev(i))) { accel_prev(i) = 0.f; }
	}

	_smoothing_xy.reset(Vector2f{accel_prev}, Vector2f{vel_prev}, Vector2f{pos_prev});
	_smoothing_z.reset(accel_prev(2), vel_prev(2), pos_prev(2));

	return ret;
}

void FlightTaskManualPositionSmoothVel::reActivate()
{
	FlightTaskManualPosition::reActivate();
	// The task is reacivated while the vehicle is on the ground. To detect takeoff in mc_pos_control_main properly
	// using the generated jerk, reset the z derivatives to zero
	_smoothing_xy.reset(Vector2f(), _velocity.xy(), _position.xy());
	_smoothing_z.reset(0.f, 0.f, _position(2));
}

void FlightTaskManualPositionSmoothVel::_ekfResetHandlerPositionXY()
{
	_smoothing_xy.setCurrentPosition(_position.xy());
}

void FlightTaskManualPositionSmoothVel::_ekfResetHandlerVelocityXY()
{
	_smoothing_xy.setCurrentVelocity(_velocity.xy());
}

void FlightTaskManualPositionSmoothVel::_ekfResetHandlerPositionZ()
{
	_smoothing_z.setCurrentPosition(_position(2));
}

void FlightTaskManualPositionSmoothVel::_ekfResetHandlerVelocityZ()
{
	_smoothing_z.setCurrentVelocity(_velocity(2));
}

void FlightTaskManualPositionSmoothVel::_updateSetpoints()
{
	// Set max accel/vel/jerk
	// Has to be done before _updateTrajectories()
	_updateTrajConstraints();
	_updateTrajVelFeedback();
	_updateTrajCurrentPositionEstimate();

	// Get yaw setpoint, un-smoothed position setpoints
	FlightTaskManualPosition::_updateSetpoints();

	_updateTrajectories(_velocity_setpoint);

	// Fill the jerk, acceleration, velocity and position setpoint vectors
	_setOutputState();
}

void FlightTaskManualPositionSmoothVel::_updateTrajConstraints()
{
	_updateTrajConstraintsXY();
	_updateTrajConstraintsZ();
}

void FlightTaskManualPositionSmoothVel::_updateTrajConstraintsXY()
{
	_smoothing_xy.setMaxJerk(_param_mpc_jerk_max.get());
	_smoothing_xy.setMaxAccel(_param_mpc_acc_hor_max.get());
	_smoothing_xy.setMaxVel(_constraints.speed_xy);
}

void FlightTaskManualPositionSmoothVel::_updateTrajConstraintsZ()
{
	_smoothing_z.setMaxJerk(_param_mpc_jerk_max.get());

	_smoothing_z.setMaxAccelUp(_param_mpc_acc_up_max.get());
	_smoothing_z.setMaxVelUp(_constraints.speed_up);

	_smoothing_z.setMaxAccelDown(_param_mpc_acc_down_max.get());
	_smoothing_z.setMaxVelDown(_constraints.speed_down);
}

void FlightTaskManualPositionSmoothVel::_updateTrajVelFeedback()
{
	_smoothing_xy.setVelSpFeedback(_velocity_setpoint_feedback.xy());
	_smoothing_z.setVelSpFeedback(_velocity_setpoint_feedback(2));
}

void FlightTaskManualPositionSmoothVel::_updateTrajCurrentPositionEstimate()
{
	_smoothing_xy.setCurrentPositionEstimate(_position.xy());
	_smoothing_z.setCurrentPositionEstimate(_position(2));
}

void FlightTaskManualPositionSmoothVel::_updateTrajectories(Vector3f vel_target)
{
	_smoothing_xy.update(_deltatime, vel_target.xy());
	_smoothing_z.update(_deltatime, vel_target(2));
}

void FlightTaskManualPositionSmoothVel::_setOutputState()
{
	_setOutputStateXY();
	_setOutputStateZ();
}

void FlightTaskManualPositionSmoothVel::_setOutputStateXY()
{
	_jerk_setpoint.xy() = _smoothing_xy.getCurrentJerk();
	_acceleration_setpoint.xy() = _smoothing_xy.getCurrentAcceleration();
	_velocity_setpoint.xy() = _smoothing_xy.getCurrentVelocity();
	_position_setpoint.xy() = _smoothing_xy.getCurrentPosition();
}

void FlightTaskManualPositionSmoothVel::_setOutputStateZ()
{
	_jerk_setpoint(2) = _smoothing_z.getCurrentJerk();
	_acceleration_setpoint(2) = _smoothing_z.getCurrentAcceleration();
	_velocity_setpoint(2) = _smoothing_z.getCurrentVelocity();
	_position_setpoint(2) = _smoothing_z.getCurrentPosition();
}
