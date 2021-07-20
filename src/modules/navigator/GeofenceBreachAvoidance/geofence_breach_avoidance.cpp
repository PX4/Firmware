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

#include "geofence_breach_avoidance.h"
#include <lib/geo/geo.h>
#include <motion_planning/VelocitySmoothing.hpp>

using Vector2d = matrix::Vector2<double>;


GeofenceBreachAvoidance::GeofenceBreachAvoidance(ModuleParams *parent) :
	ModuleParams(parent)
{
	_paramHandle.param_mpc_jerk_max = param_find("MPC_JERK_MAX");
	_paramHandle.param_mpc_acc_hor = param_find("MPC_ACC_HOR");
	_paramHandle.param_mpc_acc_hor_max = param_find("MPC_ACC_HOR_MAX");
	_paramHandle.param_mpc_jerk_auto = param_find("MPC_JERK_AUTO");
	_paramHandle.param_mpc_acc_up_max = param_find("MPC_ACC_UP_MAX");
	_paramHandle.param_mpc_acc_down_max = param_find("MPC_ACC_DOWN_MAX");

	updateParameters();
}

void GeofenceBreachAvoidance::updateParameters()
{
	ModuleParams::updateParams();
	param_get(_paramHandle.param_mpc_jerk_max, &_params.param_mpc_jerk_max);
	param_get(_paramHandle.param_mpc_acc_hor, &_params.param_mpc_acc_hor);
	param_get(_paramHandle.param_mpc_acc_hor_max, &_params.param_mpc_acc_hor_max);
	param_get(_paramHandle.param_mpc_jerk_auto, &_params.param_mpc_jerk_auto);
	param_get(_paramHandle.param_mpc_acc_up_max, &_params.param_mpc_acc_up_max);
	param_get(_paramHandle.param_mpc_acc_down_max, &_params.param_mpc_acc_down_max);

	updateMinHorDistToFenceMultirotor();
	updateMinVertDistToFenceMultirotor();
}

void GeofenceBreachAvoidance::setCurrentPosition(double lat, double lon, float alt)
{
	_current_pos_lat_lon = Vector2d(lat, lon);
	_current_alt_amsl = alt;
}

void GeofenceBreachAvoidance::setHomePosition(double lat, double lon, float alt)
{
	_home_lat_lon(0) = lat;
	_home_lat_lon(1) = lon;
	_home_alt_amsl = alt;
}

matrix::Vector2<double> GeofenceBreachAvoidance::waypointFromBearingAndDistance(matrix::Vector2<double>
		current_pos_lat_lon, float test_point_bearing, float test_point_distance)
{
	// TODO: Remove this once the underlying geo function has been fixed
	if (test_point_distance < 0.0f) {
		test_point_distance *= -1.0f;
		test_point_bearing = matrix::wrap_2pi(test_point_bearing + M_PI_F);
	}

	double fence_violation_test_point_lat, fence_violation_test_point_lon;
	waypoint_from_heading_and_distance(current_pos_lat_lon(0), current_pos_lat_lon(1), test_point_bearing,
					   test_point_distance, &fence_violation_test_point_lat, &fence_violation_test_point_lon);

	return Vector2d(fence_violation_test_point_lat, fence_violation_test_point_lon);
}

Vector2d
GeofenceBreachAvoidance::getFenceViolationTestPoint()
{
	return waypointFromBearingAndDistance(_current_pos_lat_lon, _test_point_bearing, _test_point_distance);
}

Vector2d
GeofenceBreachAvoidance::generateLoiterPointForFixedWing(geofence_violation_type_u violation_type, Geofence *geofence)
{
	if (violation_type.flags.fence_violation) {
		const float bearing_90_left = matrix::wrap_2pi(_test_point_bearing - M_PI_F * 0.5f);
		const float bearing_90_right = matrix::wrap_2pi(_test_point_bearing + M_PI_F * 0.5f);

		double loiter_center_lat, loiter_center_lon;
		double fence_violation_test_point_lat, fence_violation_test_point_lon;

		waypoint_from_heading_and_distance(_current_pos_lat_lon(0), _current_pos_lat_lon(1), bearing_90_left,
						   _test_point_distance, &fence_violation_test_point_lat, &fence_violation_test_point_lon);

		const bool left_side_is_inside_fence = geofence->isInsidePolygonOrCircle(fence_violation_test_point_lat,
						       fence_violation_test_point_lon, _current_alt_amsl);

		waypoint_from_heading_and_distance(_current_pos_lat_lon(0), _current_pos_lat_lon(1), bearing_90_right,
						   _test_point_distance, &fence_violation_test_point_lat, &fence_violation_test_point_lon);

		const bool right_side_is_inside_fence = geofence->isInsidePolygonOrCircle(fence_violation_test_point_lat,
							fence_violation_test_point_lon, _current_alt_amsl);

		float bearing_to_loiter_point;

		if (right_side_is_inside_fence && !left_side_is_inside_fence) {
			bearing_to_loiter_point = bearing_90_right;

		} else if (left_side_is_inside_fence && !right_side_is_inside_fence) {
			bearing_to_loiter_point = bearing_90_left;

		} else {
			bearing_to_loiter_point = matrix::wrap_2pi(_test_point_bearing + M_PI_F);
		}

		waypoint_from_heading_and_distance(_current_pos_lat_lon(0), _current_pos_lat_lon(1), bearing_to_loiter_point,
						   _test_point_distance, &loiter_center_lat, &loiter_center_lon);

		return Vector2d(loiter_center_lat, loiter_center_lon);

	} else if (violation_type.flags.dist_to_home_exceeded) {

		return waypointFromHomeToTestPointAtDist(math::max(_max_hor_dist_home - 2 * _test_point_distance, 0.0f));

	} else {
		return _current_pos_lat_lon;
	}
}

Vector2d
GeofenceBreachAvoidance::generateLoiterPointForMultirotor(geofence_violation_type_u violation_type, Geofence *geofence)
{

	if (violation_type.flags.fence_violation) {
		float current_distance = _test_point_distance * 0.5f;
		float current_min = 0.0f;
		float current_max = _test_point_distance;
		Vector2d test_point;

		// binary search for the distance from the drone to the geofence in the given direction
		while (abs(current_max - current_min) > 0.5f) {
			test_point = waypointFromBearingAndDistance(_current_pos_lat_lon, _test_point_bearing, current_distance);

			if (!geofence->isInsidePolygonOrCircle(test_point(0), test_point(1), _current_alt_amsl)) {
				current_max = current_distance;

			} else {
				current_min = current_distance;
			}

			current_distance = (current_max + current_min) * 0.5f;
		}

		test_point = waypointFromBearingAndDistance(_current_pos_lat_lon, _test_point_bearing, current_distance);

		if (_multirotor_braking_distance > current_distance - _min_hor_dist_to_fence_mc) {
			return waypointFromBearingAndDistance(test_point, _test_point_bearing + M_PI_F, _min_hor_dist_to_fence_mc);

		} else {
			return waypointFromBearingAndDistance(_current_pos_lat_lon, _test_point_bearing, _multirotor_braking_distance);
		}

	} else if (violation_type.flags.dist_to_home_exceeded) {

		return waypointFromHomeToTestPointAtDist(math::max(_max_hor_dist_home - _min_hor_dist_to_fence_mc, 0.0f));

	} else {
		if (_velocity_hor_abs > 0.5f) {
			return waypointFromBearingAndDistance(_current_pos_lat_lon, _test_point_bearing, _multirotor_braking_distance);

		} else {
			return _current_pos_lat_lon;
		}
	}
}

float GeofenceBreachAvoidance::generateLoiterAltitudeForFixedWing(geofence_violation_type_u violation_type)
{
	if (violation_type.flags.max_altitude_exceeded) {
		return _current_alt_amsl - 2.0f * _vertical_test_point_distance;

	} else {
		return _current_alt_amsl;
	}
}

float GeofenceBreachAvoidance::generateLoiterAltitudeForMulticopter(geofence_violation_type_u violation_type)
{
	if (violation_type.flags.max_altitude_exceeded) {
		return _current_alt_amsl + _multirotor_vertical_braking_distance - _min_vert_dist_to_fence_mc;

	} else {
		return _current_alt_amsl;
	}
}

float GeofenceBreachAvoidance::computeBrakingDistanceMultirotor()
{
	const float accel_delay_max = math::max(_params.param_mpc_acc_hor, _params.param_mpc_acc_hor_max);
	VelocitySmoothing predictor(accel_delay_max, _velocity_hor_abs, 0.f);
	predictor.setMaxVel(_velocity_hor_abs);
	predictor.setMaxAccel(_params.param_mpc_acc_hor);
	predictor.setMaxJerk(_params.param_mpc_jerk_auto);
	predictor.updateDurations(0.f);
	predictor.updateTraj(predictor.getTotalTime());

	_multirotor_braking_distance =  predictor.getCurrentPosition() + (GEOFENCE_CHECK_INTERVAL_US / 1e6f) *
					_velocity_hor_abs;

	return _multirotor_braking_distance;
}

float GeofenceBreachAvoidance::computeVerticalBrakingDistanceMultirotor()
{
	const float accel_delay_max = math::max(_params.param_mpc_acc_up_max, _params.param_mpc_acc_down_max);
	const float vertical_vel_abs = fabsf(_climbrate);


	VelocitySmoothing predictor(accel_delay_max, vertical_vel_abs, 0.f);
	predictor.setMaxVel(vertical_vel_abs);
	predictor.setMaxAccel(_climbrate > 0 ? _params.param_mpc_acc_down_max : _params.param_mpc_acc_up_max);
	predictor.setMaxJerk(_params.param_mpc_jerk_auto);
	predictor.updateDurations(0.f);
	predictor.updateTraj(predictor.getTotalTime());

	_multirotor_vertical_braking_distance =  predictor.getCurrentPosition() + (GEOFENCE_CHECK_INTERVAL_US / 1e6f) *
			vertical_vel_abs;

	_multirotor_vertical_braking_distance = matrix::sign(_climbrate) * _multirotor_vertical_braking_distance;

	return _multirotor_vertical_braking_distance;
}

void GeofenceBreachAvoidance::updateMinHorDistToFenceMultirotor()
{
	const float accel_delay_max = math::max(_params.param_mpc_acc_hor, _params.param_mpc_acc_hor_max);
	VelocitySmoothing predictor(accel_delay_max, 0.0f, 0.f);
	predictor.setMaxVel(0.0f);
	predictor.setMaxAccel(_params.param_mpc_acc_hor);
	predictor.setMaxJerk(_params.param_mpc_jerk_auto);
	predictor.updateDurations(0.f);
	predictor.updateTraj(predictor.getTotalTime());

	_min_hor_dist_to_fence_mc = 2.0f * predictor.getCurrentPosition();

}

void GeofenceBreachAvoidance::updateMinVertDistToFenceMultirotor()
{
	const float accel_delay_max = math::max(_params.param_mpc_acc_up_max, _params.param_mpc_acc_down_max);

	VelocitySmoothing predictor(accel_delay_max, 0, 0.f);
	predictor.setMaxVel(0);
	predictor.setMaxAccel(_params.param_mpc_acc_down_max);
	predictor.setMaxJerk(_params.param_mpc_jerk_auto);
	predictor.updateDurations(0.f);
	predictor.updateTraj(predictor.getTotalTime());

	_min_vert_dist_to_fence_mc =  2.0f * predictor.getCurrentPosition();
}

Vector2d GeofenceBreachAvoidance::waypointFromHomeToTestPointAtDist(float distance)
{
	Vector2d test_point = getFenceViolationTestPoint();
	float bearing_home_current_pos = get_bearing_to_next_waypoint(_home_lat_lon(0), _home_lat_lon(1), test_point(0),
					 test_point(1));
	double loiter_center_lat, loiter_center_lon;

	waypoint_from_heading_and_distance(_home_lat_lon(0), _home_lat_lon(1), bearing_home_current_pos, distance,
					   &loiter_center_lat, &loiter_center_lon);

	return Vector2d(loiter_center_lat, loiter_center_lon);
}
