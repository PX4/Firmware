/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file FlightManualAltitude.cpp
 */

#include "FlightTaskManualAltitudeSmoothVel.hpp"

#include <float.h>

using namespace matrix;

bool FlightTaskManualAltitudeSmoothVel::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitude::activate(last_setpoint);

	// Check if the previous FlightTask provided setpoints

	// If the position setpoint is unknown, set to the current postion
	float z_sp_last = PX4_ISFINITE(last_setpoint.z) ? last_setpoint.z : _position(2);

	// If the velocity setpoint is unknown, set to the current velocity
	float vz_sp_last = PX4_ISFINITE(last_setpoint.vz) ? last_setpoint.vz : _velocity(2);

	// No acceleration estimate available, set to zero if the setpoint is NAN
	float az_sp_last = PX4_ISFINITE(last_setpoint.acceleration[2]) ? last_setpoint.acceleration[2] : 0.f;

	_smoothing.reset(az_sp_last, vz_sp_last, z_sp_last);

	return ret;
}

void FlightTaskManualAltitudeSmoothVel::reActivate()
{
	FlightTaskManualAltitude::reActivate();
	// The task is reacivated while the vehicle is on the ground. To detect takeoff in mc_pos_control_main properly
	// using the generated jerk, reset the z derivatives to zero
	_smoothing.reset(0.f, 0.f, _position(2));
}

void FlightTaskManualAltitudeSmoothVel::_ekfResetHandlerPositionZ()
{
	_smoothing.setCurrentPosition(_position(2));
}

void FlightTaskManualAltitudeSmoothVel::_ekfResetHandlerVelocityZ()
{
	_smoothing.setCurrentVelocity(_velocity(2));
}

void FlightTaskManualAltitudeSmoothVel::_updateSetpoints()
{
	// Set max accel/vel/jerk
	// Has to be done before _updateTrajectories()
	_updateTrajConstraints();

	_smoothing.setVelSpFeedback(_velocity_setpoint_feedback(2));
	_smoothing.setCurrentPositionEstimate(_position(2));

	// Get yaw setpoint, un-smoothed position setpoints
	FlightTaskManualAltitude::_updateSetpoints();

	_smoothing.update(_deltatime, _velocity_setpoint(2));

	// Fill the jerk, acceleration, velocity and position setpoint vectors
	_setOutputState();
}

void FlightTaskManualAltitudeSmoothVel::_updateTrajConstraints()
{
	_smoothing.setMaxJerk(_param_mpc_jerk_max.get());

	_smoothing.setMaxAccelUp(_param_mpc_acc_up_max.get());
	_smoothing.setMaxVelUp(_constraints.speed_up);

	_smoothing.setMaxAccelDown(_param_mpc_acc_down_max.get());
	_smoothing.setMaxVelDown(_constraints.speed_down);
}


void FlightTaskManualAltitudeSmoothVel::_setOutputState()
{
	_jerk_setpoint(2) = _smoothing.getCurrentJerk();
	_acceleration_setpoint(2) = _smoothing.getCurrentAcceleration();
	_velocity_setpoint(2) = _smoothing.getCurrentVelocity();
	_position_setpoint(2) = _smoothing.getCurrentPosition();
}
