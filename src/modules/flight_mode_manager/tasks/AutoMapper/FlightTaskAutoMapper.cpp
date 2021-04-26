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
 * @file FlightAutoMapper.cpp
 */

#include "FlightTaskAutoMapper.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

FlightTaskAutoMapper::FlightTaskAutoMapper() :
	_sticks(this)
{}

bool FlightTaskAutoMapper::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskAuto::activate(last_setpoint);
	_reset();
	return ret;
}

bool FlightTaskAutoMapper::update()
{
	bool ret = FlightTaskAuto::update();
	// always reset constraints because they might change depending on the type
	_setDefaultConstraints();

	// The only time a thrust set-point is sent out is during
	// idle. Hence, reset thrust set-point to NAN in case the
	// vehicle exits idle.

	if (_type_previous == WaypointType::idle) {
		_acceleration_setpoint.setNaN();
	}

	// during mission and reposition, raise the landing gears but only
	// if altitude is high enough
	if (_highEnoughForLandingGear()) {
		_gear.landing_gear = landing_gear_s::GEAR_UP;
	}

	switch (_type) {
	case WaypointType::idle:
		_prepareIdleSetpoints();
		break;

	case WaypointType::land:
		_prepareLandSetpoints();
		break;

	case WaypointType::loiter:

	/* fallthrought */
	case WaypointType::position:
		_preparePositionSetpoints();
		break;

	case WaypointType::takeoff:
		_prepareTakeoffSetpoints();
		break;

	case WaypointType::velocity:
		_prepareVelocitySetpoints();
		break;

	default:
		_preparePositionSetpoints();
		break;
	}

	if (_param_com_obs_avoid.get()) {
		_obstacle_avoidance.updateAvoidanceDesiredSetpoints(_position_setpoint, _velocity_setpoint, (int)_type);
		_obstacle_avoidance.injectAvoidanceSetpoints(_position_setpoint, _velocity_setpoint, _yaw_setpoint,
				_yawspeed_setpoint);
	}


	_generateSetpoints();

	// update previous type
	_type_previous = _type;

	return ret;
}

void FlightTaskAutoMapper::_reset()
{
	// Set setpoints equal current state.
	_velocity_setpoint = _velocity;
	_position_setpoint = _position;
}

void FlightTaskAutoMapper::_prepareIdleSetpoints()
{
	// Send zero thrust setpoint
	_position_setpoint.setNaN(); // Don't require any position/velocity setpoints
	_velocity_setpoint.setNaN();
	_acceleration_setpoint = Vector3f(0.f, 0.f, 100.f); // High downwards acceleration to make sure there's no thrust
}

void FlightTaskAutoMapper::_prepareLandSetpoints()
{
	// Keep xy-position and go down with landspeed
	float land_speed = _getLandSpeed();
	_position_setpoint = Vector3f(_target(0), _target(1), NAN);
	_velocity_setpoint = Vector3f(Vector3f(NAN, NAN, land_speed));
	_gear.landing_gear = landing_gear_s::GEAR_DOWN;
}

void FlightTaskAutoMapper::_prepareTakeoffSetpoints()
{
	// Takeoff is completely defined by target position
	_position_setpoint = _target;
	_velocity_setpoint = Vector3f(NAN, NAN, NAN);

	_gear.landing_gear = landing_gear_s::GEAR_DOWN;
}

void FlightTaskAutoMapper::_prepareVelocitySetpoints()
{
	// XY Velocity waypoint
	// TODO : Rewiew that. What is the expected behavior?
	_position_setpoint = Vector3f(NAN, NAN, _position(2));
	Vector2f vel_sp_xy = Vector2f(_velocity).unit_or_zero() * _mc_cruise_speed;
	_velocity_setpoint = Vector3f(vel_sp_xy(0), vel_sp_xy(1), NAN);
}

void FlightTaskAutoMapper::_preparePositionSetpoints()
{
	// Simple waypoint navigation: go to xyz target, with standard limitations
	_position_setpoint = _target;
	_velocity_setpoint.setNaN(); // No special velocity limitations
}

void FlightTaskAutoMapper::updateParams()
{
	FlightTaskAuto::updateParams();

	// make sure that alt1 is above alt2
	_param_mpc_land_alt1.set(math::max(_param_mpc_land_alt1.get(), _param_mpc_land_alt2.get()));
}

bool FlightTaskAutoMapper::_highEnoughForLandingGear()
{
	// return true if altitude is above two meters
	return _dist_to_ground > 2.0f;
}

float FlightTaskAutoMapper::_getLandSpeed()
{
	float land_speed = math::gradual(_dist_to_ground,
					 _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
					 _param_mpc_land_speed.get(), _constraints.speed_down);

	// user input assisted land speed
	if (_param_mpc_land_rc_help.get()
	    && (_dist_to_ground < _param_mpc_land_alt1.get())
	    && _sticks.checkAndSetStickInputs()) {
		// stick full up -1 -> stop, stick full down 1 -> double the speed
		land_speed *= (1 + _sticks.getPositionExpo()(2));
	}

	return land_speed;
}
