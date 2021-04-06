/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualAcceleration.cpp
 */

#include "FlightTaskManualAcceleration.hpp"

using namespace matrix;

FlightTaskManualAcceleration::FlightTaskManualAcceleration() :
	_stick_acceleration_xy(this)
{};

bool FlightTaskManualAcceleration::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitudeSmoothVel::activate(last_setpoint);

	if (PX4_ISFINITE(last_setpoint.velocity[0])) {
		_stick_acceleration_xy.resetVelocity(Vector2f(last_setpoint.velocity[0], last_setpoint.velocity[1]));

	} else {
		_stick_acceleration_xy.resetVelocity(_velocity.xy());
	}

	_stick_acceleration_xy.resetPosition();

	if (PX4_ISFINITE(last_setpoint.acceleration[0])) {
		_stick_acceleration_xy.resetAcceleration(Vector2f(last_setpoint.acceleration[0], last_setpoint.acceleration[1]));
	}

	return ret;
}

bool FlightTaskManualAcceleration::update()
{
	bool ret = FlightTaskManualAltitudeSmoothVel::update();

	_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint,
				       _sticks.getPositionExpo()(3) * math::radians(_param_mpc_man_y_max.get()), _yaw, _deltatime);
	_stick_acceleration_xy.generateSetpoints(_sticks.getPositionExpo().slice<2, 1>(0, 0), _yaw, _yaw_setpoint, _position,
			_velocity_setpoint_feedback.xy(), _deltatime);
	_stick_acceleration_xy.getSetpoints(_position_setpoint, _velocity_setpoint, _acceleration_setpoint);

	_constraints.want_takeoff = _checkTakeoff();
	return ret;
}

void FlightTaskManualAcceleration::_ekfResetHandlerPositionXY()
{
	_stick_acceleration_xy.resetPosition();
}

void FlightTaskManualAcceleration::_ekfResetHandlerVelocityXY()
{
	_stick_acceleration_xy.resetVelocity(_velocity.xy());
}
