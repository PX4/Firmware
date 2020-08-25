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

/**
 * @file FlightTaskOrbit.cpp
 */

#include "FlightTaskOrbit.hpp"

#include <mathlib/mathlib.h>
#include <lib/ecl/geo/geo.h>

using namespace matrix;

FlightTaskOrbit::FlightTaskOrbit() : _circle_approach_line(_position)
{
	_sticks_data_required = false;
}

bool FlightTaskOrbit::applyCommandParameters(const vehicle_command_s &command)
{
	bool ret = true;
	// save previous velocity and roatation direction
	float v = fabsf(_v);
	bool clockwise = _v > 0;

	// commanded radius
	if (PX4_ISFINITE(command.param1)) {
		clockwise = command.param1 > 0;
		const float r = fabsf(command.param1);
		ret = ret && setRadius(r);
	}

	// commanded velocity, take sign of radius as rotation direction
	if (PX4_ISFINITE(command.param2)) {
		v = command.param2;
	}

	ret = ret && setVelocity(v * (clockwise ? 1.f : -1.f));

	// TODO: apply x,y / z independently in geo library
	// commanded center coordinates
	// if(PX4_ISFINITE(command.param5) && PX4_ISFINITE(command.param6)) {
	// 	map_projection_global_project(command.param5, command.param6, &_center(0), &_center(1));
	// }

	// commanded altitude
	// if(PX4_ISFINITE(command.param7)) {
	// 	_position_setpoint(2) = gl_ref.alt - command.param7;
	// }

	if (PX4_ISFINITE(command.param5) && PX4_ISFINITE(command.param6) && PX4_ISFINITE(command.param7)) {
		if (globallocalconverter_tolocal(command.param5, command.param6, command.param7, &_center(0), &_center(1),
						 &_position_setpoint(2))) {
			// global to local conversion failed
			ret = false;
		}
	}

	// perpendicularly approach the orbit circle again when new parameters get commanded
	_in_circle_approach = true;

	return ret;
}

bool FlightTaskOrbit::sendTelemetry()
{
	orbit_status_s orbit_status{};
	orbit_status.timestamp = hrt_absolute_time();
	orbit_status.radius = math::signNoZero(_v) * _r;
	orbit_status.frame = 0; // MAV_FRAME::MAV_FRAME_GLOBAL

	if (globallocalconverter_toglobal(_center(0), _center(1), _position_setpoint(2), &orbit_status.x, &orbit_status.y,
					  &orbit_status.z)) {
		return false; // don't send the message if the transformation failed
	}

	_orbit_status_pub.publish(orbit_status);

	return true;
}

bool FlightTaskOrbit::setRadius(float r)
{
	// clip the radius to be within range
	r = math::constrain(r, _radius_min, _radius_max);

	// small radius is more important than high velocity for safety
	if (!checkAcceleration(r, _v, _acceleration_max)) {
		_v = sign(_v) * sqrtf(_acceleration_max * r);
	}

	_r = r;
	return true;
}

bool FlightTaskOrbit::setVelocity(const float v)
{
	if (fabs(v) < _velocity_max &&
	    checkAcceleration(_r, v, _acceleration_max)) {
		_v = v;
		return true;
	}

	return false;
}

bool FlightTaskOrbit::checkAcceleration(float r, float v, float a)
{
	return v * v < a * r;
}

bool FlightTaskOrbit::activate(vehicle_local_position_setpoint_s last_setpoint)
{
	bool ret = FlightTaskManualAltitudeSmooth::activate(last_setpoint);
	_r = _radius_min;
	_v =  1.f;
	_center = Vector2f(_position);
	_center(0) -= _r;

	// need a valid position and velocity
	ret = ret && PX4_ISFINITE(_position(0))
	      && PX4_ISFINITE(_position(1))
	      && PX4_ISFINITE(_position(2))
	      && PX4_ISFINITE(_velocity(0))
	      && PX4_ISFINITE(_velocity(1))
	      && PX4_ISFINITE(_velocity(2));

	return ret;
}

bool FlightTaskOrbit::update()
{
	// update altitude
	FlightTaskManualAltitudeSmooth::update();

	// stick input adjusts parameters within a fixed time frame
	const float r = _r - _sticks_expo(0) * _deltatime * (_radius_max / 8.f);
	const float v = _v - _sticks_expo(1) * _deltatime * (_velocity_max / 4.f);

	setRadius(r);
	setVelocity(v);

	Vector2f center_to_position = Vector2f(_position) - _center;

	// make vehicle front always point towards the center
	_yaw_setpoint = atan2f(center_to_position(1), center_to_position(0)) + M_PI_F;

	if (_in_circle_approach) {
		generate_circle_approach_setpoints();

	} else {
		generate_circle_setpoints(center_to_position);
	}

	// publish information to UI
	sendTelemetry();

	return true;
}

void FlightTaskOrbit::generate_circle_approach_setpoints()
{
	if (_circle_approach_line.isEndReached()) {
		// calculate target point on circle and plan a line trajectory
		Vector2f start_to_center = _center - Vector2f(_position);
		Vector2f start_to_circle = (start_to_center.norm() - _r) * start_to_center.unit_or_zero();
		Vector2f closest_circle_point = Vector2f(_position) + start_to_circle;
		Vector3f target = Vector3f(closest_circle_point(0), closest_circle_point(1), _position(2));
		_circle_approach_line.setLineFromTo(_position, target);
		_circle_approach_line.setSpeed(_param_mpc_xy_cruise.get());
	}

	// follow the planned line and switch to orbiting once the circle is reached
	_circle_approach_line.generateSetpoints(_position_setpoint, _velocity_setpoint);
	_in_circle_approach = !_circle_approach_line.isEndReached();

	// yaw stays constant
	_yawspeed_setpoint = NAN;
}

void FlightTaskOrbit::generate_circle_setpoints(Vector2f center_to_position)
{
	// xy velocity to go around in a circle
	Vector2f velocity_xy(-center_to_position(1), center_to_position(0));
	velocity_xy = velocity_xy.unit_or_zero();
	velocity_xy *= _v;

	// xy velocity adjustment to stay on the radius distance
	velocity_xy += (_r - center_to_position.norm()) * center_to_position.unit_or_zero();

	_velocity_setpoint(0) = velocity_xy(0);
	_velocity_setpoint(1) = velocity_xy(1);
	_position_setpoint(0) = _position_setpoint(1) = NAN;

	// yawspeed feed-forward because we know the necessary angular rate
	_yawspeed_setpoint = _v / _r;
}
