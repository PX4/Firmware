/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <avoidance/ObstacleAvoidance.hpp>
#include <uORB/Subscription.hpp>


using namespace matrix;
// to run: make tests TESTFILTER=ObstacleAvoidance

class ObstacleAvoidanceTest : public ::testing::Test
{
public:
	Vector3f pos_sp;
	Vector3f vel_sp;
	float yaw_sp = 0.123f;
	float yaw_speed_sp = NAN;
	void SetUp() override

	{
		param_control_autosave(false);
		param_reset_all();
		pos_sp = Vector3f(1.f, 1.2f, 0.1f);
		vel_sp = Vector3f(0.3f, 0.4f, 0.1f);
	}
};

class TestObstacleAvoidance : public ::ObstacleAvoidance
{
public:
	TestObstacleAvoidance() : ObstacleAvoidance(nullptr) {}
	void paramsChanged() {ObstacleAvoidance::updateParamsImpl();}
	void test_setPosition(Vector3f &pos) {_position = pos;}
};

TEST_F(ObstacleAvoidanceTest, instantiation) { ObstacleAvoidance oa(nullptr); }

TEST_F(ObstacleAvoidanceTest, oa_enabled_healthy)
{
	// GIVEN: the flight controller setpoints from FlightTaskAutoMapper and a vehicle_trajectory_waypoint message coming
	// from offboard
	TestObstacleAvoidance oa;

	vehicle_trajectory_waypoint_s message = empty_trajectory_waypoint;
	message.timestamp = hrt_absolute_time();
	message.type = vehicle_trajectory_waypoint_s::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;
	message.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[0] = 2.6f;
	message.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[1] = 2.4f;
	message.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[2] = 2.7f;
	message.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw = 0.23f;
	message.waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid = true;

	// GIVEN: and we publish the vehicle_trajectory_waypoint message and vehicle status message
	uORB::Publication<vehicle_trajectory_waypoint_s> vehicle_trajectory_waypoint_pub{ORB_ID(vehicle_trajectory_waypoint)};
	vehicle_trajectory_waypoint_pub.publish(message);

	vehicle_status_s vehicle_status{};
	vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	uORB::Publication<vehicle_status_s> vehicle_status_pub{ORB_ID(vehicle_status)};
	vehicle_status_pub.publish(vehicle_status);

	// WHEN: we inject the vehicle_trajectory_waypoint in the interface
	oa.injectAvoidanceSetpoints(pos_sp, vel_sp, yaw_sp, yaw_speed_sp);

	// THEN: the setpoints should be injected
	EXPECT_FLOAT_EQ(message.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[0], pos_sp(0));
	EXPECT_FLOAT_EQ(message.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[1], pos_sp(1));
	EXPECT_FLOAT_EQ(message.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[2], pos_sp(2));
	EXPECT_TRUE(vel_sp.isAllNan());
	EXPECT_FLOAT_EQ(message.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw, yaw_sp);
	EXPECT_FALSE(PX4_ISFINITE(yaw_speed_sp));
}

TEST_F(ObstacleAvoidanceTest, oa_enabled_healthy_bezier)
{
	// GIVEN: the flight controller setpoints from FlightTaskAutoMapper and a vehicle_trajectory_waypoint message coming
	// from offboard
	TestObstacleAvoidance oa;

	vehicle_trajectory_bezier_s message {};
	message.timestamp = hrt_absolute_time();
	message.bezier_order = 2;
	message.control_points[vehicle_trajectory_bezier_s::POINT_0].position[0] = 2.6f;
	message.control_points[vehicle_trajectory_bezier_s::POINT_0].position[1] = 2.4f;
	message.control_points[vehicle_trajectory_bezier_s::POINT_0].position[2] = 2.7f;
	message.control_points[vehicle_trajectory_bezier_s::POINT_0].yaw = 0.23f;
	message.control_points[vehicle_trajectory_bezier_s::POINT_0].delta = NAN;
	message.control_points[vehicle_trajectory_bezier_s::POINT_1].position[0] = 2.6f;
	message.control_points[vehicle_trajectory_bezier_s::POINT_1].position[1] = 2.4f;
	message.control_points[vehicle_trajectory_bezier_s::POINT_1].position[2] = 3.7f;
	message.control_points[vehicle_trajectory_bezier_s::POINT_1].yaw = 0.23f;
	message.control_points[vehicle_trajectory_bezier_s::POINT_1].delta = 0.5f;

	// GIVEN: and we publish the vehicle_trajectory_waypoint message and vehicle status message
	uORB::Publication<vehicle_trajectory_bezier_s> vehicle_trajectory_bezier_pub{ORB_ID(vehicle_trajectory_bezier)};
	vehicle_trajectory_bezier_pub.publish(message);

	vehicle_status_s vehicle_status{};
	vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	uORB::Publication<vehicle_status_s> vehicle_status_pub{ORB_ID(vehicle_status)};
	vehicle_status_pub.publish(vehicle_status);

	// WHEN: we inject the vehicle_trajectory_waypoint in the interface
	oa.injectAvoidanceSetpoints(pos_sp, vel_sp, yaw_sp, yaw_speed_sp);

	// THEN: the setpoints should be injected
	EXPECT_FLOAT_EQ(2.6f, pos_sp(0));
	EXPECT_FLOAT_EQ(2.4f, pos_sp(1));
	EXPECT_LT(2.7f, pos_sp(2));
	EXPECT_GT(2.8f, pos_sp(2)); // probably only a tiny bit above 2.7, but let's not have flakey tests
	EXPECT_FLOAT_EQ(vel_sp.xy().norm(), 0);
	EXPECT_FLOAT_EQ(vel_sp(2), (3.7f - 2.7f) / 0.5f);
	EXPECT_FLOAT_EQ(0.23, yaw_sp);
	EXPECT_FLOAT_EQ(yaw_speed_sp, 0);
}

TEST_F(ObstacleAvoidanceTest, oa_enabled_not_healthy)
{
	// GIVEN: the flight controller setpoints from FlightTaskAutoMapper and a vehicle_trajectory_waypoint message
	TestObstacleAvoidance oa;

	vehicle_trajectory_waypoint_s message = empty_trajectory_waypoint;
	Vector3f pos(3.1f, 4.7f, 5.2f);
	oa.test_setPosition(pos);

	// GIVEN: and we publish the vehicle_trajectory_waypoint message and vehicle_status
	uORB::Publication<vehicle_trajectory_waypoint_s> vehicle_trajectory_waypoint_pub{ORB_ID(vehicle_trajectory_waypoint)};
	vehicle_trajectory_waypoint_pub.publish(message);

	vehicle_status_s vehicle_status{};
	vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	uORB::Publication<vehicle_status_s> vehicle_status_pub{ORB_ID(vehicle_status)};
	vehicle_status_pub.publish(vehicle_status);

	// WHEN: we inject the vehicle_trajectory_waypoint in the interface
	oa.injectAvoidanceSetpoints(pos_sp, vel_sp, yaw_sp, yaw_speed_sp);

	// THEN: the setpoints shouldn't be injected
	EXPECT_FLOAT_EQ(pos(0), pos_sp(0));
	EXPECT_FLOAT_EQ(pos(1), pos_sp(1));
	EXPECT_FLOAT_EQ(pos(2), pos_sp(2));
	EXPECT_TRUE(vel_sp.isAllNan());
	EXPECT_FALSE(PX4_ISFINITE(yaw_sp));
	EXPECT_FALSE(PX4_ISFINITE(yaw_speed_sp));
}

TEST_F(ObstacleAvoidanceTest, oa_desired)
{
	// GIVEN: the flight controller setpoints from FlightTaskAutoMapper and the waypoints from FLightTaskAuto
	TestObstacleAvoidance oa;

	pos_sp = Vector3f(1.f, 1.2f, NAN);
	vel_sp = Vector3f(NAN, NAN, 0.7f);
	int type = 4;
	Vector3f curr_wp(1.f, 1.2f, 5.0f);
	float curr_yaw = 1.02f;
	float curr_yawspeed = NAN;
	Vector3f next_wp(11.f, 1.2f, 5.0f);
	float next_yaw = 0.82f;
	float next_yawspeed = NAN;
	bool ext_yaw_active = false;

	// WHEN: we inject the setpoints and waypoints in the interface
	oa.updateAvoidanceDesiredWaypoints(curr_wp, curr_yaw, curr_yawspeed, next_wp, next_yaw, next_yawspeed, ext_yaw_active,
					   type);
	oa.updateAvoidanceDesiredSetpoints(pos_sp, vel_sp, type);

	// WHEN: we subscribe to the uORB message out of the interface
	uORB::SubscriptionData<vehicle_trajectory_waypoint_s> _sub_traj_wp_avoidance_desired{ORB_ID(vehicle_trajectory_waypoint_desired)};
	_sub_traj_wp_avoidance_desired.update();

	// THEN: we expect the setpoints in POINT_0 and waypoints in POINT_1 and POINT_2
	EXPECT_FLOAT_EQ(pos_sp(0),
			_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[0]);
	EXPECT_FLOAT_EQ(pos_sp(1),
			_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[1]);
	EXPECT_FALSE(PX4_ISFINITE(
			     _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[2]));
	EXPECT_FALSE(PX4_ISFINITE(
			     _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[0]));
	EXPECT_FALSE(PX4_ISFINITE(
			     _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[1]));
	EXPECT_FLOAT_EQ(vel_sp(2),
			_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[2]);
	EXPECT_EQ(type, _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].type);
	EXPECT_TRUE(_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid);

	EXPECT_FLOAT_EQ(curr_wp(0),
			_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_1].position[0]);
	EXPECT_FLOAT_EQ(curr_wp(1),
			_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_1].position[1]);
	EXPECT_FLOAT_EQ(curr_wp(2),
			_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_1].position[2]);
	EXPECT_FLOAT_EQ(curr_yaw, _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_1].yaw);
	EXPECT_FALSE(PX4_ISFINITE(
			     _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_1].yaw_speed));
	EXPECT_EQ(type, _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_1].type);
	EXPECT_TRUE(_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_1].point_valid);

	EXPECT_FLOAT_EQ(next_wp(0),
			_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_2].position[0]);
	EXPECT_FLOAT_EQ(next_wp(1),
			_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_2].position[1]);
	EXPECT_FLOAT_EQ(next_wp(2),
			_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_2].position[2]);
	EXPECT_FLOAT_EQ(next_yaw, _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_2].yaw);
	EXPECT_FALSE(PX4_ISFINITE(
			     _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_2].yaw_speed));
	EXPECT_EQ(UINT8_MAX, _sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_2].type);
	EXPECT_TRUE(_sub_traj_wp_avoidance_desired.get().waypoints[vehicle_trajectory_waypoint_s::POINT_2].point_valid);

}
