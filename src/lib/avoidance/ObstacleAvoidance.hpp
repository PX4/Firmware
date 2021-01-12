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
 * @file ObstacleAvoidance.hpp
 * This class is used to inject the setpoints of an obstacle avoidance system
 * into the FlightTasks
 *
 * @author Martina Rivizzigno
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_bezier.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/position_setpoint.h>

#include <lib/hysteresis/hysteresis.h>

#include <matrix/matrix/math.hpp>

const vehicle_trajectory_waypoint_s empty_trajectory_waypoint = {0, 0, {0, 0, 0, 0, 0, 0, 0},
	{	{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, UINT8_MAX, {0, 0}},
		{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, UINT8_MAX, {0, 0}},
		{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, UINT8_MAX, {0, 0}},
		{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, UINT8_MAX, {0, 0}},
		{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, UINT8_MAX, {0, 0}}
	}
};

class ObstacleAvoidance : public ModuleParams
{
public:
	ObstacleAvoidance(ModuleParams *parent);
	~ObstacleAvoidance() = default;

	/**
	 * Inject setpoints from obstacle avoidance system into FlightTasks.
	 * @param pos_sp, position setpoint
	 * @param vel_sp, velocity setpoint
	 * @param yaw_sp, yaw setpoint
	 * @param yaw_speed_sp, yaw speed setpoint
	 */
	void injectAvoidanceSetpoints(matrix::Vector3f &pos_sp, matrix::Vector3f &vel_sp, float &yaw_sp, float &yaw_speed_sp);

	/**
	 * Updates the desired waypoints to send to the obstacle avoidance system. These messages don't have any direct impact on the flight.
	 * @param curr_wp, current position triplet
	 * @param curr_yaw, current yaw triplet
	 * @param curr_yawspeed, current yaw speed triplet
	 * @param next_wp, next position triplet
	 * @param next_yaw, next yaw triplet
	 * @param next_yawspeed, next yaw speed triplet
	 * @param wp_type, current triplet type
	 */
	void updateAvoidanceDesiredWaypoints(const matrix::Vector3f &curr_wp, const float curr_yaw, const float curr_yawspeed,
					     const matrix::Vector3f &next_wp, const float next_yaw, const float next_yawspeed, const bool ext_yaw_active,
					     const int wp_type);
	/**
	 * Updates the desired setpoints to send to the obstacle avoidance system.
	 * @param pos_sp, desired position setpoint computed by the active FlightTask
	 * @param vel_sp, desired velocity setpoint computed by the active FlightTask
	 * @param type, current triplet type
	 */
	void updateAvoidanceDesiredSetpoints(const matrix::Vector3f &pos_sp, const matrix::Vector3f &vel_sp, const int type);

	/**
	 * Checks the vehicle progress between previous and current position waypoint of the triplet.
	 * @param pos, vehicle position
	 * @param prev_wp, previous position triplet
	 * @param target_acceptance_radius, current position triplet xy acceptance radius
	 * @param closest_pt, closest point to the vehicle on the line previous-current position triplet
	 */
	void checkAvoidanceProgress(const matrix::Vector3f &pos, const matrix::Vector3f &prev_wp,
				    float target_acceptance_radius, const matrix::Vector2f &closest_pt);

protected:

	uORB::SubscriptionData<vehicle_trajectory_bezier_s> _sub_vehicle_trajectory_bezier{ORB_ID(vehicle_trajectory_bezier)}; /**< vehicle trajectory waypoint subscription */
	uORB::SubscriptionData<vehicle_trajectory_waypoint_s> _sub_vehicle_trajectory_waypoint{ORB_ID(vehicle_trajectory_waypoint)}; /**< vehicle trajectory waypoint subscription */
	uORB::SubscriptionData<vehicle_status_s> _sub_vehicle_status{ORB_ID(vehicle_status)}; /**< vehicle status subscription */

	vehicle_trajectory_waypoint_s _desired_waypoint{};  /**< desired vehicle trajectory waypoint to be sent to OA */

	uORB::Publication<vehicle_trajectory_waypoint_s> _pub_traj_wp_avoidance_desired{ORB_ID(vehicle_trajectory_waypoint_desired)};	/**< trajectory waypoint desired publication */
	uORB::Publication<position_controller_status_s> _pub_pos_control_status{ORB_ID(position_controller_status)};	/**< position controller status publication */
	uORB::Publication<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};	/**< vehicle command do publication */

	matrix::Vector3f _curr_wp = {}; /**< current position triplet */
	matrix::Vector3f _position = {}; /**< current vehicle position */
	matrix::Vector3f _failsafe_position = {}; /**< vehicle position when entered in failsafe */

	systemlib::Hysteresis _avoidance_point_not_valid_hysteresis{false}; /**< becomes true if the companion doesn't start sending valid setpoints */
	systemlib::Hysteresis _no_progress_z_hysteresis{false}; /**< becomes true if the vehicle is not making progress towards the z component of the goal */

	float _prev_pos_to_target_z = -1.f; /**< z distance to the goal */

	bool _ext_yaw_active = false; /**< true, if external yaw handling is active */

	/**
	 * Publishes vehicle command.
	 */
	void _publishVehicleCmdDoLoiter();
	void _generateBezierSetpoints(matrix::Vector3f &position, matrix::Vector3f &velocity, float &yaw, float &yaw_velocity);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_MC_ALT_RAD>) _param_nav_mc_alt_rad    /**< Acceptance radius for multicopter altitude */
	);

};
