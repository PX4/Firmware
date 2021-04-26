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
 * @file FlightManualAltitude.cpp
 */

#include "FlightTaskManualAltitude.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <ecl/geo/geo.h>

using namespace matrix;

FlightTaskManualAltitude::FlightTaskManualAltitude() :
	_sticks(this)
{}

bool FlightTaskManualAltitude::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_sticks.checkAndSetStickInputs();

	if (_sticks_data_required) {
		ret = ret && _sticks.isAvailable();
	}

	// in addition to manual require valid position and velocity in D-direction and valid yaw
	return ret && PX4_ISFINITE(_position(2)) && PX4_ISFINITE(_velocity(2)) && PX4_ISFINITE(_yaw);
}

bool FlightTaskManualAltitude::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	_yaw_setpoint = NAN;
	_yawspeed_setpoint = 0.f;
	_acceleration_setpoint = Vector3f(0.f, 0.f, NAN); // altitude is controlled from position/velocity
	_position_setpoint(2) = _position(2);
	_velocity_setpoint(2) = 0.f;
	_setDefaultConstraints();

	_updateConstraintsFromEstimator();

	_max_speed_up = _constraints.speed_up;
	_max_speed_down = _constraints.speed_down;

	return ret;
}

void FlightTaskManualAltitude::_updateConstraintsFromEstimator()
{
	if (PX4_ISFINITE(_sub_vehicle_local_position.get().hagl_min)) {
		_min_distance_to_ground = _sub_vehicle_local_position.get().hagl_min;

	} else {
		_min_distance_to_ground = -INFINITY;
	}

	if (PX4_ISFINITE(_sub_vehicle_local_position.get().hagl_max)) {
		_max_distance_to_ground = _sub_vehicle_local_position.get().hagl_max;

	} else {
		_max_distance_to_ground = INFINITY;
	}
}

void FlightTaskManualAltitude::_scaleSticks()
{
	// Use stick input with deadzone, exponential curve and first order lpf for yawspeed
	const float yawspeed_target = _sticks.getPositionExpo()(3) * math::radians(_param_mpc_man_y_max.get());
	_yawspeed_setpoint = _applyYawspeedFilter(yawspeed_target);

	// Use sticks input with deadzone and exponential curve for vertical velocity
	const float vel_max_z = (_sticks.getPosition()(2) > 0.0f) ? _constraints.speed_down : _constraints.speed_up;
	_velocity_setpoint(2) = vel_max_z * _sticks.getPositionExpo()(2);
}

float FlightTaskManualAltitude::_applyYawspeedFilter(float yawspeed_target)
{
	const float den = math::max(_param_mpc_man_y_tau.get() + _deltatime, 0.001f);
	const float alpha = _deltatime / den;
	_yawspeed_filter_state = (1.f - alpha) * _yawspeed_filter_state + alpha * yawspeed_target;
	return _yawspeed_filter_state;
}

void FlightTaskManualAltitude::_updateAltitudeLock()
{
	// Depending on stick inputs and velocity, position is locked.
	// If not locked, altitude setpoint is set to NAN.

	// Check if user wants to break
	const bool apply_brake = fabsf(_sticks.getPositionExpo()(2)) <= FLT_EPSILON;

	// Check if vehicle has stopped
	const bool stopped = (_param_mpc_hold_max_z.get() < FLT_EPSILON || fabsf(_velocity(2)) < _param_mpc_hold_max_z.get());

	// Manage transition between use of distance to ground and distance to local origin
	// when terrain hold behaviour has been selected.
	if (_param_mpc_alt_mode.get() == 2) {
		// Use horizontal speed as a transition criteria
		float spd_xy = Vector2f(_velocity).length();

		// Use presence of horizontal stick inputs as a transition criteria
		float stick_xy = Vector2f(_sticks.getPositionExpo().slice<2, 1>(0, 0)).length();
		bool stick_input = stick_xy > 0.001f;

		if (_terrain_hold) {
			bool too_fast = spd_xy > _param_mpc_hold_max_xy.get();

			if (stick_input || too_fast || !PX4_ISFINITE(_dist_to_bottom)) {
				// Stop using distance to ground
				_terrain_hold = false;
				_terrain_follow = false;

				// Adjust the setpoint to maintain the same height error to reduce control transients
				if (PX4_ISFINITE(_dist_to_ground_lock) && PX4_ISFINITE(_dist_to_bottom)) {
					_position_setpoint(2) = _position(2) - (_dist_to_ground_lock - _dist_to_bottom);

				} else {
					_position_setpoint(2) = _position(2);
				}
			}

		} else {
			bool not_moving = spd_xy < 0.5f * _param_mpc_hold_max_xy.get();

			if (!stick_input && not_moving && PX4_ISFINITE(_dist_to_bottom)) {
				// Start using distance to ground
				_terrain_hold = true;
				_terrain_follow = true;

				// Adjust the setpoint to maintain the same height error to reduce control transients
				if (PX4_ISFINITE(_position_setpoint(2))) {
					_dist_to_ground_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));
				}
			}
		}

	}

	if ((_param_mpc_alt_mode.get() == 1 || _terrain_follow) && PX4_ISFINITE(_dist_to_bottom)) {
		// terrain following
		_terrainFollowing(apply_brake, stopped);
		// respect maximum altitude
		_respectMaxAltitude();

	} else {
		// normal mode where height is dependent on local frame

		if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(2))) {
			// lock position
			_position_setpoint(2) = _position(2);

			// Ensure that minimum altitude is respected if
			// there is a distance sensor and distance to bottom is below minimum.
			if (PX4_ISFINITE(_dist_to_bottom) && _dist_to_bottom < _min_distance_to_ground) {
				_terrainFollowing(apply_brake, stopped);

			} else {
				_dist_to_ground_lock = NAN;
			}

		} else if (PX4_ISFINITE(_position_setpoint(2)) && apply_brake) {
			// Position is locked but check if a reset event has happened.
			// We will shift the setpoints.
			if (_sub_vehicle_local_position.get().z_reset_counter != _reset_counter) {
				_position_setpoint(2) = _position(2);
				_reset_counter = _sub_vehicle_local_position.get().z_reset_counter;
			}

		} else  {
			// user demands velocity change
			_position_setpoint(2) = NAN;
			// ensure that maximum altitude is respected
			_respectMaxAltitude();
		}
	}
}

void FlightTaskManualAltitude::_respectMinAltitude()
{
	// Height above ground needs to be limited (flow / range-finder)
	if (PX4_ISFINITE(_dist_to_bottom) && (_dist_to_bottom < _min_distance_to_ground)) {
		// increase altitude to minimum flow distance
		_position_setpoint(2) = _position(2) - (_min_distance_to_ground - _dist_to_bottom);
	}
}

void FlightTaskManualAltitude::_terrainFollowing(bool apply_brake, bool stopped)
{
	if (apply_brake && stopped && !PX4_ISFINITE(_dist_to_ground_lock)) {
		// User wants to break and vehicle reached zero velocity. Lock height to ground.

		// lock position
		_position_setpoint(2) = _position(2);
		// ensure that minimum altitude is respected
		_respectMinAltitude();
		// lock distance to ground but adjust first for minimum altitude
		_dist_to_ground_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));

	} else if (apply_brake && PX4_ISFINITE(_dist_to_ground_lock)) {
		// vehicle needs to follow terrain

		// difference between the current distance to ground and the desired distance to ground
		const float delta_distance_to_ground = _dist_to_ground_lock - _dist_to_bottom;
		// adjust position setpoint for the delta (note: NED frame)
		_position_setpoint(2) = _position(2) - delta_distance_to_ground;

	} else {
		// user demands velocity change in D-direction
		_dist_to_ground_lock = _position_setpoint(2) = NAN;
	}
}

void FlightTaskManualAltitude::_respectMaxAltitude()
{
	if (PX4_ISFINITE(_dist_to_bottom)) {

		// if there is a valid maximum distance to ground, linearly increase speed limit with distance
		// below the maximum, preserving control loop vertical position error gain.
		if (PX4_ISFINITE(_max_distance_to_ground)) {
			_constraints.speed_up = math::constrain(_param_mpc_z_p.get() * (_max_distance_to_ground - _dist_to_bottom),
								-_max_speed_down, _max_speed_up);

		} else {
			_constraints.speed_up = _max_speed_up;
		}

		// if distance to bottom exceeded maximum distance, slowly approach maximum distance
		if (_dist_to_bottom > _max_distance_to_ground) {
			// difference between current distance to ground and maximum distance to ground
			const float delta_distance_to_max = _dist_to_bottom - _max_distance_to_ground;
			// set position setpoint to maximum distance to ground
			_position_setpoint(2) = _position(2) + delta_distance_to_max;
			// limit speed downwards to 0.7m/s
			_constraints.speed_down = math::min(_max_speed_down, 0.7f);

		} else {
			_constraints.speed_down = _max_speed_down;
		}
	}
}

void FlightTaskManualAltitude::_respectGroundSlowdown()
{
	// limit speed gradually within the altitudes MPC_LAND_ALT1 and MPC_LAND_ALT2
	if (PX4_ISFINITE(_dist_to_ground)) {
		const float limit_down = math::gradual(_dist_to_ground,
						       _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
						       _param_mpc_land_speed.get(), _constraints.speed_down);
		const float limit_up = math::gradual(_dist_to_ground,
						     _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
						     _param_mpc_tko_speed.get(), _constraints.speed_up);
		_velocity_setpoint(2) = math::constrain(_velocity_setpoint(2), -limit_up, limit_down);
	}
}

void FlightTaskManualAltitude::_rotateIntoHeadingFrame(Vector2f &v)
{
	float yaw_rotate = PX4_ISFINITE(_yaw_setpoint) ? _yaw_setpoint : _yaw;
	Vector3f v_r = Vector3f(Dcmf(Eulerf(0.0f, 0.0f, yaw_rotate)) * Vector3f(v(0), v(1), 0.0f));
	v(0) = v_r(0);
	v(1) = v_r(1);
}

void FlightTaskManualAltitude::_updateHeadingSetpoints()
{
	if (_isYawInput()) {
		_unlockYaw();

	} else {
		_lockYaw();
	}
}

bool FlightTaskManualAltitude::_isYawInput()
{
	/*
	 * A threshold larger than FLT_EPSILON is required because the
	 * _yawspeed_setpoint comes from an IIR filter and takes too much
	 * time to reach zero.
	 */
	return fabsf(_yawspeed_setpoint) > 0.001f;
}

void FlightTaskManualAltitude::_unlockYaw()
{
	// no fixed heading when rotating around yaw by stick
	_yaw_setpoint = NAN;
}

void FlightTaskManualAltitude::_lockYaw()
{
	// hold the current heading when no more rotation commanded
	if (!PX4_ISFINITE(_yaw_setpoint)) {
		_yaw_setpoint = _yaw;
	}
}

void FlightTaskManualAltitude::_ekfResetHandlerHeading(float delta_psi)
{
	// Only reset the yaw setpoint when the heading is locked
	if (PX4_ISFINITE(_yaw_setpoint)) {
		_yaw_setpoint += delta_psi;
	}
}

void FlightTaskManualAltitude::_updateSetpoints()
{
	_updateHeadingSetpoints(); // get yaw setpoint

	// Thrust in xy are extracted directly from stick inputs. A magnitude of
	// 1 means that maximum thrust along xy is demanded. A magnitude of 0 means no
	// thrust along xy is demanded. The maximum thrust along xy depends on the thrust
	// setpoint along z-direction, which is computed in PositionControl.cpp.

	Vector2f sp(_sticks.getPosition().slice<2, 1>(0, 0));

	_man_input_filter.setParameters(_deltatime, _param_mc_man_tilt_tau.get());
	_man_input_filter.update(sp);
	sp = _man_input_filter.getState();
	_rotateIntoHeadingFrame(sp);

	if (sp.length() > 1.0f) {
		sp.normalize();
	}

	_acceleration_setpoint.xy() = sp * tanf(math::radians(_param_mpc_man_tilt_max.get())) * CONSTANTS_ONE_G;

	_updateAltitudeLock();
	_respectGroundSlowdown();
}

bool FlightTaskManualAltitude::_checkTakeoff()
{
	// stick is deflected above 65% throttle (throttle stick is in the range [-1,1])
	return _sticks.getPosition()(2) < -0.3f;
}

bool FlightTaskManualAltitude::update()
{
	bool ret = FlightTask::update();
	_updateConstraintsFromEstimator();
	_scaleSticks();
	_updateSetpoints();
	_constraints.want_takeoff = _checkTakeoff();

	return ret;
}
