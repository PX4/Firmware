/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightManualPositionSmooth.cpp
 */

#include "FlightTaskManualPositionSmooth.hpp"

using namespace matrix;

FlightTaskManualPositionSmooth::FlightTaskManualPositionSmooth() :
	_smoothingXY(this, Vector2f(_velocity)),
	_smoothingZ(this, _velocity(2), _sticks(2))
{}

void FlightTaskManualPositionSmooth::_updateSetpoints()
{
	/* Get yaw setpont, un-smoothed position setpoints.*/
	FlightTaskManualPosition::_updateSetpoints();

	/* Smooth velocity setpoint in xy.*/
	Vector2f vel(_velocity);
	Vector2f vel_sp_xy(_velocity_setpoint);
	_smoothingXY.updateMaxVelocity(_constraints.speed_xy);
	_smoothingXY.smoothVelocity(vel_sp_xy, vel, _yaw, _yawspeed_setpoint, _deltatime);
	_velocity_setpoint(0) = vel_sp_xy(0);
	_velocity_setpoint(1) = vel_sp_xy(1);

	/* Check for xy position lock.*/
	_updateXYlock();

	/* Smooth velocity in z.*/
	_smoothingZ.smoothVelFromSticks(_velocity_setpoint(2), _deltatime);

	/* Check for altitude lock*/
	_updateAltitudeLock();
}
