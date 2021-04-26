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
 * @file FlightAutoLine.cpp
 */

#include "FlightTaskAutoLineSmoothVel.hpp"

#include "TrajectoryConstraints.hpp"

using namespace matrix;

bool FlightTaskAutoLineSmoothVel::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskAutoMapper::activate(last_setpoint);

	Vector3f vel_prev{last_setpoint.vx, last_setpoint.vy, last_setpoint.vz};
	Vector3f pos_prev{last_setpoint.x, last_setpoint.y, last_setpoint.z};
	Vector3f accel_prev{last_setpoint.acceleration};

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current postion
		if (!PX4_ISFINITE(pos_prev(i))) { pos_prev(i) = _position(i); }

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) { vel_prev(i) = _velocity(i); }

		// No acceleration estimate available, set to zero if the setpoint is NAN
		if (!PX4_ISFINITE(accel_prev(i))) { accel_prev(i) = 0.f; }
	}

	for (int i = 0; i < 3; ++i) {
		_trajectory[i].reset(accel_prev(i), vel_prev(i), pos_prev(i));
	}

	_yaw_sp_prev = PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw;
	_updateTrajConstraints();

	return ret;
}

void FlightTaskAutoLineSmoothVel::reActivate()
{
	FlightTaskAutoMapper::reActivate();

	// On ground, reset acceleration and velocity to zero
	for (int i = 0; i < 2; ++i) {
		_trajectory[i].reset(0.f, 0.f, _position(i));
	}

	_trajectory[2].reset(0.f, 0.7f, _position(2));
}

/**
 * EKF reset handling functions
 * Those functions are called by the base FlightTask in
 * case of an EKF reset event
 */
void FlightTaskAutoLineSmoothVel::_ekfResetHandlerPositionXY()
{
	_trajectory[0].setCurrentPosition(_position(0));
	_trajectory[1].setCurrentPosition(_position(1));
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerVelocityXY()
{
	_trajectory[0].setCurrentVelocity(_velocity(0));
	_trajectory[1].setCurrentVelocity(_velocity(1));
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerPositionZ()
{
	_trajectory[2].setCurrentPosition(_position(2));
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerVelocityZ()
{
	_trajectory[2].setCurrentVelocity(_velocity(2));
}

void FlightTaskAutoLineSmoothVel::_ekfResetHandlerHeading(float delta_psi)
{
	_yaw_sp_prev += delta_psi;
}

void FlightTaskAutoLineSmoothVel::_generateSetpoints()
{
	_updateTurningCheck();
	_prepareSetpoints();
	_generateTrajectory();

	if (!PX4_ISFINITE(_yaw_setpoint) && !PX4_ISFINITE(_yawspeed_setpoint)) {
		// no valid heading -> generate heading in this flight task
		_generateHeading();
	}
}

void FlightTaskAutoLineSmoothVel::_updateTurningCheck()
{
	const Vector2f vel_traj(_trajectory[0].getCurrentVelocity(),
				_trajectory[1].getCurrentVelocity());
	const Vector2f pos_traj(_trajectory[0].getCurrentPosition(),
				_trajectory[1].getCurrentPosition());
	const Vector2f target_xy(_target);
	const Vector2f u_vel_traj = vel_traj.unit_or_zero();
	const Vector2f pos_to_target = Vector2f(target_xy - pos_traj);
	const float cos_align = u_vel_traj * pos_to_target.unit_or_zero();

	// The vehicle is turning if the angle between the velocity vector
	// and the direction to the target is greater than 10 degrees, the
	// velocity is large enough and the drone isn't in the acceptance
	// radius of the last WP.
	_is_turning = vel_traj.longerThan(0.2f)
		      && cos_align < 0.98f
		      && pos_to_target.longerThan(_target_acceptance_radius);
}

void FlightTaskAutoLineSmoothVel::_generateHeading()
{
	// Generate heading along trajectory if possible, otherwise hold the previous yaw setpoint
	if (!_generateHeadingAlongTraj()) {
		_yaw_setpoint = _yaw_sp_prev;
	}
}

bool FlightTaskAutoLineSmoothVel::_generateHeadingAlongTraj()
{
	bool res = false;
	Vector2f vel_sp_xy(_velocity_setpoint);
	Vector2f traj_to_target = Vector2f(_target) - Vector2f(_position);

	if ((vel_sp_xy.length() > .1f) &&
	    (traj_to_target.length() > 2.f)) {
		// Generate heading from velocity vector, only if it is long enough
		// and if the drone is far enough from the target
		_compute_heading_from_2D_vector(_yaw_setpoint, vel_sp_xy);
		res = true;
	}

	return res;
}

/* Constrain some value vith a constrain depending on the sign of the constraint
 * Example: 	- if the constrain is -5, the value will be constrained between -5 and 0
 * 		- if the constrain is 5, the value will be constrained between 0 and 5
 */
float FlightTaskAutoLineSmoothVel::_constrainOneSide(float val, float constraint)
{
	const float min = (constraint < FLT_EPSILON) ? constraint : 0.f;
	const float max = (constraint > FLT_EPSILON) ? constraint : 0.f;

	return math::constrain(val, min, max);
}

float FlightTaskAutoLineSmoothVel::_constrainAbs(float val, float max)
{
	return sign(val) * math::min(fabsf(val), fabsf(max));
}

float FlightTaskAutoLineSmoothVel::_getMaxXYSpeed() const
{
	Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
			  _trajectory[1].getCurrentPosition(),
			  _trajectory[2].getCurrentPosition());

	math::trajectory::VehicleDynamicLimits config;
	config.z_accept_rad = _param_nav_mc_alt_rad.get();
	config.xy_accept_rad = _target_acceptance_radius;
	config.max_acc_xy = _trajectory[0].getMaxAccel();
	config.max_jerk = _trajectory[0].getMaxJerk();
	config.max_speed_xy = _mc_cruise_speed;
	config.max_acc_xy_radius_scale = _param_mpc_xy_traj_p.get();

	// constrain velocity to go to the position setpoint first if the position setpoint has been modified by an external source
	// (eg. Obstacle Avoidance)

	Vector3f waypoints[3] = {pos_traj, _target, _next_wp};

	if (isTargetModified()) {
		waypoints[2] = waypoints[1] = _position_setpoint;
	}

	float max_xy_speed = math::trajectory::computeXYSpeedFromWaypoints<3>(waypoints, config);

	return max_xy_speed;
}

float FlightTaskAutoLineSmoothVel::_getMaxZSpeed() const
{
	Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
			  _trajectory[1].getCurrentPosition(),
			  _trajectory[2].getCurrentPosition());
	float z_setpoint = _target(2);

	// constrain velocity to go to the position setpoint first if the position setpoint has been modified by an external source
	// (eg. Obstacle Avoidance)
	bool z_valid = PX4_ISFINITE(_position_setpoint(2));
	bool z_modified =  z_valid && fabsf((_target - _position_setpoint)(2)) > FLT_EPSILON;

	if (z_modified) {
		z_setpoint = _position_setpoint(2);
	}

	const float distance_start_target = fabs(z_setpoint - pos_traj(2));
	const float arrival_z_speed = 0.f;

	float max_speed = math::min(_trajectory[2].getMaxVel(), math::trajectory::computeMaxSpeedFromDistance(
					    _trajectory[2].getMaxJerk(), _trajectory[2].getMaxAccel(),
					    distance_start_target, arrival_z_speed));

	return max_speed;
}

Vector3f FlightTaskAutoLineSmoothVel::getCrossingPoint() const
{
	Vector3f pos_crossing_point{};

	if (isTargetModified()) {
		// Strictly follow the modified setpoint
		pos_crossing_point = _position_setpoint;

	} else {
		if (_is_turning) {
			// Get the crossing point using L1-style guidance
			pos_crossing_point.xy() = getL1Point();
			pos_crossing_point(2) = _target(2);

		} else {
			pos_crossing_point = _target;
		}
	}

	return pos_crossing_point;
}

bool FlightTaskAutoLineSmoothVel::isTargetModified() const
{
	const bool xy_modified = (_target - _position_setpoint).xy().longerThan(FLT_EPSILON);
	const bool z_valid = PX4_ISFINITE(_position_setpoint(2));
	const bool z_modified =  z_valid && fabs((_target - _position_setpoint)(2)) > FLT_EPSILON;

	return xy_modified || z_modified;
}

Vector2f FlightTaskAutoLineSmoothVel::getL1Point() const
{
	const Vector2f pos_traj(_trajectory[0].getCurrentPosition(),
				_trajectory[1].getCurrentPosition());
	const Vector2f target_xy(_target);
	const Vector2f u_prev_to_target = Vector2f(target_xy - Vector2f(_prev_wp)).unit_or_zero();
	const Vector2f prev_to_pos(pos_traj - Vector2f(_prev_wp));
	const Vector2f prev_to_closest(u_prev_to_target * (prev_to_pos * u_prev_to_target));
	const Vector2f closest_pt = Vector2f(_prev_wp) + prev_to_closest;

	// Compute along-track error using L1 distance and cross-track error
	const float crosstrack_error = Vector2f(closest_pt - pos_traj).length();

	const float l1 = math::max(_target_acceptance_radius, 5.f);
	float alongtrack_error = 0.f;

	// Protect against sqrt of a negative number
	if (l1 > crosstrack_error) {
		alongtrack_error = sqrtf(l1 * l1 - crosstrack_error * crosstrack_error);
	}

	// Position of the point on the line where L1 intersect the line between the two waypoints
	const Vector2f l1_point = closest_pt + alongtrack_error * u_prev_to_target;

	return l1_point;
}

void FlightTaskAutoLineSmoothVel::_prepareSetpoints()
{
	// Interface: A valid position setpoint generates a velocity target using conservative motion constraints.
	// If a velocity is specified, that is used as a feedforward to track the position setpoint
	// (ie. it assumes the position setpoint is moving at the specified velocity)
	// If the position setpoints are set to NAN, the values in the velocity setpoints are used as velocity targets: nothing to do here.

	_want_takeoff = false;

	if (_param_mpc_yaw_mode.get() == 4 && !_yaw_sp_aligned) {
		// Wait for the yaw setpoint to be aligned
		_velocity_setpoint.setAll(0.f);
		return;
	}

	const bool xy_pos_setpoint_valid = PX4_ISFINITE(_position_setpoint(0)) && PX4_ISFINITE(_position_setpoint(1));
	const bool z_pos_setpoint_valid = PX4_ISFINITE(_position_setpoint(2));

	if (xy_pos_setpoint_valid && z_pos_setpoint_valid) {
		// Use 3D position setpoint to generate a 3D velocity setpoint
		Vector3f pos_traj(_trajectory[0].getCurrentPosition(),
				  _trajectory[1].getCurrentPosition(),
				  _trajectory[2].getCurrentPosition());

		const Vector3f u_pos_traj_to_dest((getCrossingPoint() - pos_traj).unit_or_zero());

		float xy_speed = _getMaxXYSpeed();
		const float z_speed = _getMaxZSpeed();

		if (_is_turning) {
			// Lock speed during turn
			xy_speed = math::min(_max_speed_prev, xy_speed);

		} else {
			_max_speed_prev = xy_speed;
		}

		Vector3f vel_sp_constrained = u_pos_traj_to_dest * sqrtf(xy_speed * xy_speed + z_speed * z_speed);
		math::trajectory::clampToXYNorm(vel_sp_constrained, xy_speed, 0.5f);
		math::trajectory::clampToZNorm(vel_sp_constrained, z_speed, 0.5f);

		for (int i = 0; i < 3; i++) {
			// If available, use the existing velocity as a feedforward, otherwise replace it
			if (PX4_ISFINITE(_velocity_setpoint(i))) {
				_velocity_setpoint(i) += vel_sp_constrained(i);

			} else {
				_velocity_setpoint(i) = vel_sp_constrained(i);
			}
		}
	}

	else if (xy_pos_setpoint_valid) {
		// Use 2D position setpoint to generate a 2D velocity setpoint

		// Get various path specific vectors
		Vector2f pos_traj(_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition());
		Vector2f pos_traj_to_dest_xy = Vector2f(getCrossingPoint()) - pos_traj;
		Vector2f u_pos_traj_to_dest_xy(pos_traj_to_dest_xy.unit_or_zero());

		float xy_speed = _getMaxXYSpeed();

		if (_is_turning) {
			// Lock speed during turn
			xy_speed = math::min(_max_speed_prev, xy_speed);

		} else {
			_max_speed_prev = xy_speed;
		}

		Vector2f vel_sp_constrained_xy = u_pos_traj_to_dest_xy * xy_speed;

		for (int i = 0; i < 2; i++) {
			// If available, use the existing velocity as a feedforward, otherwise replace it
			if (PX4_ISFINITE(_velocity_setpoint(i))) {
				_velocity_setpoint(i) += vel_sp_constrained_xy(i);

			} else {
				_velocity_setpoint(i) = vel_sp_constrained_xy(i);
			}
		}
	}

	else if (z_pos_setpoint_valid) {
		// Use Z position setpoint to generate a Z velocity setpoint

		const float z_dir = sign(_position_setpoint(2) - _trajectory[2].getCurrentPosition());
		const float vel_sp_z = z_dir * _getMaxZSpeed();

		// If available, use the existing velocity as a feedforward, otherwise replace it
		if (PX4_ISFINITE(_velocity_setpoint(2))) {
			_velocity_setpoint(2) += vel_sp_z;

		} else {
			_velocity_setpoint(2) = vel_sp_z;
		}
	}

	_want_takeoff = _velocity_setpoint(2) < -0.3f;
}

void FlightTaskAutoLineSmoothVel::_updateTrajConstraints()
{
	// Update the constraints of the trajectories
	_trajectory[0].setMaxAccel(_param_mpc_acc_hor.get()); // TODO : Should be computed using heading
	_trajectory[1].setMaxAccel(_param_mpc_acc_hor.get());
	_trajectory[0].setMaxVel(_param_mpc_xy_vel_max.get());
	_trajectory[1].setMaxVel(_param_mpc_xy_vel_max.get());
	_trajectory[0].setMaxJerk(_param_mpc_jerk_auto.get()); // TODO : Should be computed using heading
	_trajectory[1].setMaxJerk(_param_mpc_jerk_auto.get());
	_trajectory[2].setMaxJerk(_param_mpc_jerk_auto.get());

	if (_velocity_setpoint(2) < 0.f) { // up
		float z_accel_constraint = _param_mpc_acc_up_max.get();
		float z_vel_constraint = _param_mpc_z_vel_max_up.get();

		// The constraints are broken because they are used as hard limits by the position controller, so put this here
		// until the constraints don't do things like cause controller integrators to saturate. Once the controller
		// doesn't use z speed constraints, this can go in AutoMapper::_prepareTakeoffSetpoints(). Accel limit is to
		// emulate the motor ramp (also done in the controller) so that the controller can actually track the setpoint.
		if (_type == WaypointType::takeoff &&  _dist_to_ground < _param_mpc_land_alt1.get()) {
			z_vel_constraint = _param_mpc_tko_speed.get();
			z_accel_constraint = math::min(z_accel_constraint, _param_mpc_tko_speed.get() / _param_mpc_tko_ramp_t.get());

			// Keep the altitude setpoint at the current altitude
			// to avoid having it going down into the ground during
			// the initial ramp as the velocity does not start at 0
			_trajectory[2].setCurrentPosition(_position(2));
		}

		_trajectory[2].setMaxVel(z_vel_constraint);
		_trajectory[2].setMaxAccel(z_accel_constraint);

	} else { // down
		_trajectory[2].setMaxAccel(_param_mpc_acc_down_max.get());
		_trajectory[2].setMaxVel(_param_mpc_z_vel_max_dn.get());
	}
}

void FlightTaskAutoLineSmoothVel::_generateTrajectory()
{
	if (!PX4_ISFINITE(_velocity_setpoint(0)) || !PX4_ISFINITE(_velocity_setpoint(1))
	    || !PX4_ISFINITE(_velocity_setpoint(2))) {
		return;
	}

	/* Slow down the trajectory by decreasing the integration time based on the position error.
	 * This is only performed when the drone is behind the trajectory
	 */
	Vector2f position_trajectory_xy(_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition());
	Vector2f position_xy(_position);
	Vector2f vel_traj_xy(_trajectory[0].getCurrentVelocity(), _trajectory[1].getCurrentVelocity());
	Vector2f drone_to_trajectory_xy(position_trajectory_xy - position_xy);
	float position_error = drone_to_trajectory_xy.length();

	float time_stretch = 1.f - math::constrain(position_error / _param_mpc_xy_err_max.get(), 0.f, 1.f);

	// Don't stretch time if the drone is ahead of the position setpoint
	if (drone_to_trajectory_xy.dot(vel_traj_xy) < 0.f) {
		time_stretch = 1.f;
	}

	Vector3f jerk_sp_smooth;
	Vector3f accel_sp_smooth;
	Vector3f vel_sp_smooth;
	Vector3f pos_sp_smooth;

	for (int i = 0; i < 3; ++i) {
		_trajectory[i].updateTraj(_deltatime, time_stretch);
		jerk_sp_smooth(i) = _trajectory[i].getCurrentJerk();
		accel_sp_smooth(i) = _trajectory[i].getCurrentAcceleration();
		vel_sp_smooth(i) = _trajectory[i].getCurrentVelocity();
		pos_sp_smooth(i) = _trajectory[i].getCurrentPosition();
	}

	_updateTrajConstraints();

	for (int i = 0; i < 3; ++i) {
		_trajectory[i].updateDurations(_velocity_setpoint(i));
	}

	VelocitySmoothing::timeSynchronization(_trajectory, 3);

	_jerk_setpoint = jerk_sp_smooth;
	_acceleration_setpoint = accel_sp_smooth;
	_velocity_setpoint = vel_sp_smooth;
	_position_setpoint = pos_sp_smooth;
}
