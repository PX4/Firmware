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

	// commanded heading behaviour
	if (PX4_ISFINITE(command.param3)) {
		_yaw_behaviour = command.param3;
	}

	// save current yaw estimate for ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING
	_initial_heading = _yaw;

	// commanded center coordinates
	if (PX4_ISFINITE(command.param5) && PX4_ISFINITE(command.param6)) {
		if (map_projection_initialized(&_global_local_proj_ref)) {
			map_projection_project(&_global_local_proj_ref,
					       command.param5, command.param6,
					       &_center(0), &_center(1));

		} else {
			ret = false;
		}
	}

	// commanded altitude
	if (PX4_ISFINITE(command.param7)) {
		if (map_projection_initialized(&_global_local_proj_ref)) {
			_position_setpoint(2) = _global_local_alt0 - command.param7;

		} else {
			ret = false;
		}
	}

	// perpendicularly approach the orbit circle again when new parameters get commanded
	_in_circle_approach = true;
	_circle_approach_line.reset();

	return ret;
}

bool FlightTaskOrbit::sendTelemetry()
{
	orbit_status_s orbit_status{};
	orbit_status.radius = math::signNoZero(_v) * _r;
	orbit_status.frame = 0; // MAV_FRAME::MAV_FRAME_GLOBAL
	orbit_status.yaw_behaviour = _yaw_behaviour;

	if (map_projection_initialized(&_global_local_proj_ref)) {
		// local -> global
		map_projection_reproject(&_global_local_proj_ref, _center(0), _center(1), &orbit_status.x, &orbit_status.y);
		orbit_status.z = _global_local_alt0 - _position_setpoint(2);

	} else {
		return false; // don't send the message if the transformation failed
	}

	orbit_status.timestamp = hrt_absolute_time();
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

	if (fabs(_r - r) > FLT_EPSILON) {
		_circle_approach_line.reset();
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

bool FlightTaskOrbit::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitudeSmoothVel::activate(last_setpoint);
	_r = _radius_min;
	_v =  1.f;
	_center = _position.xy();
	_initial_heading = _yaw;
	_slew_rate_yaw.setForcedValue(_yaw);
	_slew_rate_yaw.setSlewRate(math::radians(_param_mpc_yawrauto_max.get()));

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
	bool ret = FlightTaskManualAltitudeSmoothVel::update();

	// stick input adjusts parameters within a fixed time frame
	const float r = _r - _sticks.getPositionExpo()(0) * _deltatime * (_radius_max / 8.f);
	const float v = _v - _sticks.getPositionExpo()(1) * _deltatime * (_velocity_max / 4.f);

	setRadius(r);
	setVelocity(v);

	const Vector2f center_to_position = Vector2f(_position) - _center;

	if (_in_circle_approach) {
		generate_circle_approach_setpoints(center_to_position);

	} else {
		generate_circle_setpoints(center_to_position);
		generate_circle_yaw_setpoints(center_to_position);
	}

	// Apply yaw smoothing
	_yaw_setpoint = _slew_rate_yaw.update(_yaw_setpoint, _deltatime);

	// publish information to UI
	sendTelemetry();

	return ret;
}

void FlightTaskOrbit::generate_circle_approach_setpoints(const Vector2f &center_to_position)
{
	const Vector2f start_to_circle = (_r - center_to_position.norm()) * center_to_position.unit_or_zero();

	if (_circle_approach_line.isEndReached()) {
		// calculate target point on circle and plan a line trajectory
		const Vector2f closest_circle_point = Vector2f(_position) + start_to_circle;
		const Vector3f target = Vector3f(closest_circle_point(0), closest_circle_point(1), _position(2));
		_circle_approach_line.setLineFromTo(_position, target);
		_circle_approach_line.setSpeed(_param_mpc_xy_cruise.get());
	}

	_yaw_setpoint = atan2f(start_to_circle(1), start_to_circle(0));

	// follow the planned line and switch to orbiting once the circle is reached
	_circle_approach_line.generateSetpoints(_position_setpoint, _velocity_setpoint);
	_in_circle_approach = !_circle_approach_line.isEndReached();
}

void FlightTaskOrbit::generate_circle_setpoints(const Vector2f &center_to_position)
{
	// xy velocity to go around in a circle
	Vector2f velocity_xy(-center_to_position(1), center_to_position(0));
	velocity_xy = velocity_xy.unit_or_zero();
	velocity_xy *= _v;

	// xy velocity adjustment to stay on the radius distance
	velocity_xy += (_r - center_to_position.norm()) * center_to_position.unit_or_zero();

	_position_setpoint(0) = _position_setpoint(1) = NAN;
	_velocity_setpoint.xy() = velocity_xy;
	_acceleration_setpoint.xy() = -center_to_position.unit_or_zero() * _v * _v / _r;
}

void FlightTaskOrbit::generate_circle_yaw_setpoints(const Vector2f &center_to_position)
{
	switch (_yaw_behaviour) {
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING:
		// make vehicle keep the same heading as when the orbit was commanded
		_yaw_setpoint = _initial_heading;
		_yawspeed_setpoint = NAN;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_UNCONTROLLED:
		// no yaw setpoint
		_yaw_setpoint = NAN;
		_yawspeed_setpoint = NAN;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE:
		_yaw_setpoint = atan2f(sign(_v) * center_to_position(0), -sign(_v) * center_to_position(1));
		_yawspeed_setpoint = _v / _r;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED:
		// inherit setpoint from altitude flight task
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER:
	default:
		_yaw_setpoint = atan2f(-center_to_position(1), -center_to_position(0));
		// yawspeed feed-forward because we know the necessary angular rate
		_yawspeed_setpoint = _v / _r;
		break;
	}
}
