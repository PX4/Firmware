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
 * @file FlightTaskAuto.cpp
 */

#include "FlightTaskAuto.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

static constexpr float SIGMA_NORM	= 0.001f;

FlightTaskAuto::FlightTaskAuto() :
	_obstacle_avoidance(this)
{

}

bool FlightTaskAuto::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	_position_setpoint = _position;
	_velocity_setpoint = _velocity;
	_yaw_setpoint = _yaw_sp_prev = _yaw;
	_yawspeed_setpoint = 0.0f;
	_setDefaultConstraints();
	return ret;
}

bool FlightTaskAuto::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_sub_home_position.update();
	_sub_vehicle_status.update();
	_sub_triplet_setpoint.update();

	// require valid reference and valid target
	ret = ret && _evaluateGlobalReference() && _evaluateTriplets();
	// require valid position
	ret = ret && PX4_ISFINITE(_position(0))
	      && PX4_ISFINITE(_position(1))
	      && PX4_ISFINITE(_position(2))
	      && PX4_ISFINITE(_velocity(0))
	      && PX4_ISFINITE(_velocity(1))
	      && PX4_ISFINITE(_velocity(2));

	return ret;
}

bool FlightTaskAuto::updateFinalize()
{
	// All the auto FlightTasks have to comply with defined maximum yaw rate
	// If the FlightTask generates a yaw or a yawrate setpoint that exceeds this value
	// it will see its setpoint constrained here
	_limitYawRate();
	_constraints.want_takeoff = _checkTakeoff();
	return true;
}

void FlightTaskAuto::_limitYawRate()
{
	const float yawrate_max = math::radians(_param_mpc_yawrauto_max.get());

	_yaw_sp_aligned = true;

	if (PX4_ISFINITE(_yaw_setpoint) && PX4_ISFINITE(_yaw_sp_prev)) {
		// Limit the rate of change of the yaw setpoint
		const float dyaw_desired = matrix::wrap_pi(_yaw_setpoint - _yaw_sp_prev);
		const float dyaw_max = yawrate_max * _deltatime;
		const float dyaw = math::constrain(dyaw_desired, -dyaw_max, dyaw_max);
		const float yaw_setpoint_sat = matrix::wrap_pi(_yaw_sp_prev + dyaw);

		// The yaw setpoint is aligned when it is within tolerance
		_yaw_sp_aligned = fabsf(matrix::wrap_pi(_yaw_setpoint - yaw_setpoint_sat)) < math::radians(_param_mis_yaw_err.get());

		_yaw_setpoint = yaw_setpoint_sat;
		_yaw_sp_prev = _yaw_setpoint;

		if (!PX4_ISFINITE(_yawspeed_setpoint) && (_deltatime > FLT_EPSILON)) {
			// Create a feedforward
			_yawspeed_setpoint = dyaw / _deltatime;
		}
	}

	if (PX4_ISFINITE(_yawspeed_setpoint)) {
		// The yaw setpoint is aligned when its rate is not saturated
		_yaw_sp_aligned = _yaw_sp_aligned && (fabsf(_yawspeed_setpoint) < yawrate_max);

		_yawspeed_setpoint = math::constrain(_yawspeed_setpoint, -yawrate_max, yawrate_max);
	}
}

bool FlightTaskAuto::_evaluateTriplets()
{
	// TODO: fix the issues mentioned below
	// We add here some conditions that are only required because:
	// 1. navigator continuously sends triplet during mission due to yaw setpoint. This
	// should be removed in the navigator and only updates if the current setpoint actually has changed.
	//
	// 2. navigator should be responsible to send always three valid setpoints. If there is only one setpoint,
	// then previous will be set to current vehicle position and next will be set equal to setpoint.
	//
	// 3. navigator originally only supports gps guided maneuvers. However, it now also supports some flow-specific features
	// such as land and takeoff. The navigator should use for auto takeoff/land with flow the position in xy at the moment the
	// takeoff/land was initiated. Until then we do this kind of logic here.

	// Check if triplet is valid. There must be at least a valid altitude.

	if (!_sub_triplet_setpoint.get().current.valid || !PX4_ISFINITE(_sub_triplet_setpoint.get().current.alt)) {
		// Best we can do is to just set all waypoints to current state
		_prev_prev_wp = _triplet_prev_wp = _triplet_target = _triplet_next_wp = _position;
		_type = WaypointType::loiter;
		_yaw_setpoint = _yaw;
		_yawspeed_setpoint = NAN;
		_target_acceptance_radius = _sub_triplet_setpoint.get().current.acceptance_radius;
		_updateInternalWaypoints();
		return true;
	}

	_type = (WaypointType)_sub_triplet_setpoint.get().current.type;

	// Always update cruise speed since that can change without waypoint changes.
	_mc_cruise_speed = _sub_triplet_setpoint.get().current.cruising_speed;

	if (!PX4_ISFINITE(_mc_cruise_speed) || (_mc_cruise_speed < 0.0f)) {
		// If no speed is planned use the default cruise speed as limit
		_mc_cruise_speed = _constraints.speed_xy;
	}

	// Ensure planned cruise speed is below the maximum such that the smooth trajectory doesn't get capped
	_mc_cruise_speed = math::min(_mc_cruise_speed, _param_mpc_xy_vel_max.get());

	// Temporary target variable where we save the local reprojection of the latest navigator current triplet.
	Vector3f tmp_target;

	if (!PX4_ISFINITE(_sub_triplet_setpoint.get().current.lat)
	    || !PX4_ISFINITE(_sub_triplet_setpoint.get().current.lon)) {
		// No position provided in xy. Lock position
		if (!PX4_ISFINITE(_lock_position_xy(0))) {
			tmp_target(0) = _lock_position_xy(0) = _position(0);
			tmp_target(1) = _lock_position_xy(1) = _position(1);

		} else {
			tmp_target(0) = _lock_position_xy(0);
			tmp_target(1) = _lock_position_xy(1);
		}

	} else {
		// reset locked position if current lon and lat are valid
		_lock_position_xy.setAll(NAN);

		// Convert from global to local frame.
		map_projection_project(&_reference_position,
				       _sub_triplet_setpoint.get().current.lat, _sub_triplet_setpoint.get().current.lon, &tmp_target(0), &tmp_target(1));
	}

	tmp_target(2) = -(_sub_triplet_setpoint.get().current.alt - _reference_altitude);

	// Check if anything has changed. We do that by comparing the temporary target
	// to the internal _triplet_target.
	// TODO This is a hack and it would be much better if the navigator only sends out a waypoints once they have changed.

	bool triplet_update = true;
	const bool prev_next_validity_changed = (_prev_was_valid != _sub_triplet_setpoint.get().previous.valid)
						|| (_next_was_valid != _sub_triplet_setpoint.get().next.valid);

	if (PX4_ISFINITE(_triplet_target(0))
	    && PX4_ISFINITE(_triplet_target(1))
	    && PX4_ISFINITE(_triplet_target(2))
	    && fabsf(_triplet_target(0) - tmp_target(0)) < 0.001f
	    && fabsf(_triplet_target(1) - tmp_target(1)) < 0.001f
	    && fabsf(_triplet_target(2) - tmp_target(2)) < 0.001f
	    && !prev_next_validity_changed) {
		// Nothing has changed: just keep old waypoints.
		triplet_update = false;

	} else {
		_triplet_target = tmp_target;
		_target_acceptance_radius = _sub_triplet_setpoint.get().current.acceptance_radius;

		if (!PX4_ISFINITE(_triplet_target(0)) || !PX4_ISFINITE(_triplet_target(1))) {
			// Horizontal target is not finite.
			_triplet_target(0) = _position(0);
			_triplet_target(1) = _position(1);
		}

		if (!PX4_ISFINITE(_triplet_target(2))) {
			_triplet_target(2) = _position(2);
		}

		// If _triplet_target has updated, update also _triplet_prev_wp and _triplet_next_wp.
		_prev_prev_wp = _triplet_prev_wp;

		if (_isFinite(_sub_triplet_setpoint.get().previous) && _sub_triplet_setpoint.get().previous.valid) {
			map_projection_project(&_reference_position, _sub_triplet_setpoint.get().previous.lat,
					       _sub_triplet_setpoint.get().previous.lon, &_triplet_prev_wp(0), &_triplet_prev_wp(1));
			_triplet_prev_wp(2) = -(_sub_triplet_setpoint.get().previous.alt - _reference_altitude);

		} else {
			_triplet_prev_wp = _position;
		}

		_prev_was_valid = _sub_triplet_setpoint.get().previous.valid;

		if (_type == WaypointType::loiter) {
			_triplet_next_wp = _triplet_target;

		} else if (_isFinite(_sub_triplet_setpoint.get().next) && _sub_triplet_setpoint.get().next.valid) {
			map_projection_project(&_reference_position, _sub_triplet_setpoint.get().next.lat,
					       _sub_triplet_setpoint.get().next.lon, &_triplet_next_wp(0), &_triplet_next_wp(1));
			_triplet_next_wp(2) = -(_sub_triplet_setpoint.get().next.alt - _reference_altitude);

		} else {
			_triplet_next_wp = _triplet_target;
		}

		_next_was_valid = _sub_triplet_setpoint.get().next.valid;
	}

	if (_ext_yaw_handler != nullptr) {
		// activation/deactivation of weather vane is based on parameter WV_EN and setting of navigator (allow_weather_vane)
		(_param_wv_en.get() && !_sub_triplet_setpoint.get().current.disable_weather_vane) ?	_ext_yaw_handler->activate() :
		_ext_yaw_handler->deactivate();
	}

	// Calculate the current vehicle state and check if it has updated.
	State previous_state = _current_state;
	_current_state = _getCurrentState();

	if (triplet_update || (_current_state != previous_state) || _current_state == State::offtrack) {
		_updateInternalWaypoints();
		_yaw_lock = false;
	}

	if (_param_com_obs_avoid.get()
	    && _sub_vehicle_status.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

		_obstacle_avoidance.updateAvoidanceDesiredWaypoints(_triplet_target, _yaw_setpoint, _yawspeed_setpoint,
				_triplet_next_wp, _sub_triplet_setpoint.get().next.yaw, NAN,
				_ext_yaw_handler != nullptr && _ext_yaw_handler->is_active(), _sub_triplet_setpoint.get().current.type);

		_obstacle_avoidance.checkAvoidanceProgress(_position, _triplet_prev_wp, _target_acceptance_radius, _closest_pt);
	}

	// set heading
	if (_ext_yaw_handler != nullptr && _ext_yaw_handler->is_active()) {
		_yaw_setpoint = _yaw;
		// use the yawrate setpoint from WV only if not moving lateral (velocity setpoint below half of _param_mpc_xy_cruise)
		// otherwise, keep heading constant (as output from WV is not according to wind in this case)
		bool vehicle_is_moving_lateral = _velocity_setpoint.xy().longerThan(_param_mpc_xy_cruise.get() / 2.0f);

		if (vehicle_is_moving_lateral) {
			_yawspeed_setpoint = 0.0f;

		} else {
			_yawspeed_setpoint = _ext_yaw_handler->get_weathervane_yawrate();
		}

	} else {
		if ((_type != WaypointType::takeoff || _sub_triplet_setpoint.get().current.disable_weather_vane)
		    && _sub_triplet_setpoint.get().current.yaw_valid) {
			// Use the yaw computed in Navigator except during takeoff because
			// Navigator is not handling the yaw reset properly.
			// But: use if from Navigator during takeoff if disable_weather_vane is true,
			// because we're then aligning to the transition waypoint.
			// TODO: fix in navigator
			_yaw_setpoint = _sub_triplet_setpoint.get().current.yaw;

		} else {
			_set_heading_from_mode();
		}

		_yawspeed_setpoint = NAN;
	}

	return true;
}

void FlightTaskAuto::_set_heading_from_mode()
{

	Vector2f v; // Vector that points towards desired location

	switch (_param_mpc_yaw_mode.get()) {

	case 0: // Heading points towards the current waypoint.
	case 4: // Same as 0 but yaw first and then go
		v = Vector2f(_target) - Vector2f(_position);
		break;

	case 1: // Heading points towards home.
		if (_sub_home_position.get().valid_lpos) {
			v = Vector2f(&_sub_home_position.get().x) - Vector2f(_position);
		}

		break;

	case 2: // Heading point away from home.
		if (_sub_home_position.get().valid_lpos) {
			v = Vector2f(_position) - Vector2f(&_sub_home_position.get().x);
		}

		break;

	case 3: // Along trajectory.
		// The heading depends on the kind of setpoint generation. This needs to be implemented
		// in the subclasses where the velocity setpoints are generated.
		v.setAll(NAN);
		break;
	}

	if (PX4_ISFINITE(v.length())) {
		// We only adjust yaw if vehicle is outside of acceptance radius. Once we enter acceptance
		// radius, lock yaw to current yaw.
		// This prevents excessive yawing.
		if (!_yaw_lock) {
			if (v.length() < _target_acceptance_radius) {
				_yaw_setpoint = _yaw;
				_yaw_lock = true;

			} else {
				_compute_heading_from_2D_vector(_yaw_setpoint, v);
			}
		}

	} else {
		_yaw_lock = false;
		_yaw_setpoint = NAN;
	}
}

bool FlightTaskAuto::_isFinite(const position_setpoint_s &sp)
{
	return (PX4_ISFINITE(sp.lat) && PX4_ISFINITE(sp.lon) && PX4_ISFINITE(sp.alt));
}

bool FlightTaskAuto::_evaluateGlobalReference()
{
	// check if reference has changed and update.
	// Only update if reference timestamp has changed AND no valid reference altitude
	// is available.
	// TODO: this needs to be revisited and needs a more clear implementation
	if (_sub_vehicle_local_position.get().ref_timestamp == _time_stamp_reference && PX4_ISFINITE(_reference_altitude)) {
		// don't need to update anything
		return true;
	}

	double ref_lat = _sub_vehicle_local_position.get().ref_lat;
	double ref_lon = _sub_vehicle_local_position.get().ref_lon;
	_reference_altitude = _sub_vehicle_local_position.get().ref_alt;

	if (!_sub_vehicle_local_position.get().z_global) {
		// we have no valid global altitude
		// set global reference to local reference
		_reference_altitude = 0.0f;
	}

	if (!_sub_vehicle_local_position.get().xy_global) {
		// we have no valid global alt/lat
		// set global reference to local reference
		ref_lat = 0.0;
		ref_lon = 0.0;
	}

	// init projection
	map_projection_init(&_reference_position, ref_lat, ref_lon);

	// check if everything is still finite
	return PX4_ISFINITE(_reference_altitude) && PX4_ISFINITE(ref_lat) && PX4_ISFINITE(ref_lon);
}

void FlightTaskAuto::_setDefaultConstraints()
{
	FlightTask::_setDefaultConstraints();

	// only adjust limits if the new limit is lower
	if (_constraints.speed_xy >= _param_mpc_xy_cruise.get()) {
		_constraints.speed_xy = _param_mpc_xy_cruise.get();
	}
}

State FlightTaskAuto::_getCurrentState()
{
	// Calculate the vehicle current state based on the Navigator triplets and the current position.
	Vector2f u_prev_to_target = Vector2f(_triplet_target - _triplet_prev_wp).unit_or_zero();
	Vector2f pos_to_target(_triplet_target - _position);
	Vector2f prev_to_pos(_position - _triplet_prev_wp);
	// Calculate the closest point to the vehicle position on the line prev_wp - target
	_closest_pt = Vector2f(_triplet_prev_wp) + u_prev_to_target * (prev_to_pos * u_prev_to_target);

	State return_state = State::none;

	if (u_prev_to_target * pos_to_target < 0.0f) {
		// Target is behind.
		return_state = State::target_behind;

	} else if (u_prev_to_target * prev_to_pos < 0.0f && prev_to_pos.length() > _target_acceptance_radius) {
		// Current position is more than cruise speed in front of previous setpoint.
		return_state = State::previous_infront;

	} else if (Vector2f(Vector2f(_position) - _closest_pt).length() > _target_acceptance_radius) {
		// Vehicle is more than cruise speed off track.
		return_state = State::offtrack;

	}

	return return_state;
}

void FlightTaskAuto::_updateInternalWaypoints()
{
	// The internal Waypoints might differ from _triplet_prev_wp, _triplet_target and _triplet_next_wp.
	// The cases where it differs:
	// 1. The vehicle already passed the target -> go straight to target
	// 2. The vehicle is more than cruise speed in front of previous waypoint -> go straight to previous waypoint
	// 3. The vehicle is more than cruise speed from track -> go straight to closest point on track
	switch (_current_state) {
	case State::target_behind:
		_target = _triplet_target;
		_prev_wp = _position;
		_next_wp = _triplet_next_wp;
		break;

	case State::previous_infront:
		_next_wp = _triplet_target;
		_target = _triplet_prev_wp;
		_prev_wp = _position;
		break;

	case State::offtrack:
		_next_wp = _triplet_target;
		_target = matrix::Vector3f(_closest_pt(0), _closest_pt(1), _triplet_target(2));
		_prev_wp = _position;
		break;

	case State::none:
		_target = _triplet_target;
		_prev_wp = _triplet_prev_wp;
		_next_wp = _triplet_next_wp;
		break;

	default:
		break;

	}
}

bool FlightTaskAuto::_compute_heading_from_2D_vector(float &heading, Vector2f v)
{
	if (PX4_ISFINITE(v.length()) && v.length() > SIGMA_NORM) {
		v.normalize();
		// To find yaw: take dot product of x = (1,0) and v
		// and multiply by the sign given of cross product of x and v.
		// Dot product: (x(0)*v(0)+(x(1)*v(1)) = v(0)
		// Cross product: x(0)*v(1) - v(0)*x(1) = v(1)
		heading =  sign(v(1)) * wrap_pi(acosf(v(0)));
		return true;
	}

	// heading unknown and therefore do not change heading
	return false;
}
