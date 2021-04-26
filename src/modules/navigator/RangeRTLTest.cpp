#define MODULE_NAME "Navigator"

#include "navigator.h"
#include "rtl.h"

#include <future>


#include <gtest/gtest.h>

TEST(Navigator_and_RTL, interact_correctly)
{
	Navigator n;
	RTL rtl(&n);


	home_position_s home_pos{};
	home_pos.valid_hpos = true;
	home_pos.valid_lpos = true;
	home_pos.valid_alt = true;
	home_pos.timestamp = 1000;

	vehicle_global_position_s glob_pos{};

	vehicle_status_s v_status{};
	v_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	// TODO: can't do this, it hangs forever in Navigator's while loop
	// uORB::Publication<home_position_s> home_pos_pub{ORB_ID(home_position)};
	// uORB::Publication<vehicle_global_position_s> global_pos_pub{ORB_ID(vehicle_global_position)};
	// uORB::Publication<vehicle_local_position_s> local_pos_pub{ORB_ID(vehicle_local_position)};
	// uORB::Publication<vehicle_status_s> vehicle_status_pub{ORB_ID(vehicle_status)};
	// home_pos_pub.publish(home_pos);
	// global_pos_pub.publish(glob_pos);
	// local_pos_pub.publish(local_pos);
	// vehicle_status_pub.publish(v_status);
	// n.run();

	// Hacky-hack, don't use pub-sub, just set them directly in navigator. NB! This isn't the "real" API, they should
	// be set via pub-sub otherwise this will be a constant drag on development
	*n.get_home_position() = home_pos;
	*n.get_global_position() = glob_pos;
	*n.get_vstatus() = v_status;

	uORB::SubscriptionData<rtl_flight_time_s> _rtl_flight_time_sub{ORB_ID(rtl_flight_time)};
	ASSERT_FALSE(_rtl_flight_time_sub.update());
	rtl.find_RTL_destination();
	ASSERT_TRUE(_rtl_flight_time_sub.update());
	auto msg = _rtl_flight_time_sub.get();
	EXPECT_EQ(msg.rtl_time_s, 0);

	// WHEN: we set the vehicle type to multirotor
	v_status.vehicle_type = n.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	float xy, z;
	rtl.get_rtl_xy_z_speed(xy, z);

	// THEN: the RTL speed should correspond to multirotor parameters
	float xy_desired, z_desired;
	param_get(param_handle(px4::params::MPC_XY_CRUISE), &xy_desired);
	param_get(param_handle(px4::params::MPC_Z_VEL_MAX_DN), &z_desired);
	EXPECT_FLOAT_EQ(xy, xy_desired);
	EXPECT_FLOAT_EQ(z, z_desired);

	// WHEN: it is a fixed wing
	n.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	rtl.get_rtl_xy_z_speed(xy, z);

	// THEN: it should be fixed wing parameters
	param_get(param_handle(px4::params::FW_AIRSPD_TRIM), &xy_desired);
	param_get(param_handle(px4::params::FW_T_SINK_MIN), &z_desired);
	EXPECT_FLOAT_EQ(xy, xy_desired);
	EXPECT_FLOAT_EQ(z, z_desired);

	// WHEN: it is rover
	n.get_vstatus()->vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;
	rtl.get_rtl_xy_z_speed(xy, z);

	// THEN: it should be rover parameters, and z should just be large (no RTL time in Z -> high speed)
	param_get(param_handle(px4::params::GND_SPEED_THR_SC), &xy_desired);
	EXPECT_FLOAT_EQ(xy, xy_desired);
	EXPECT_GT(z, 1000);
}

class RangeRTL_tth : public ::testing::Test
{
public:
	matrix::Vector3f rtl_vector;
	matrix::Vector3f rtl_point_local_pos;
	matrix::Vector2f wind_vel;
	float vehicle_speed;
	float vehicle_descent_speed;

	void SetUp() override
	{
		rtl_vector  = matrix::Vector3f(0, 0, 0);
		rtl_point_local_pos  = matrix::Vector3f(0, 0, 0);
		wind_vel  = matrix::Vector2f(0, 0);
		vehicle_speed = 5;
		vehicle_descent_speed = 1;
	}
};

TEST_F(RangeRTL_tth, zero_distance_zero_time)
{
	// GIVEN: zero distances (defaults)

	// WHEN: we get the tth
	float tth = time_to_home(rtl_vector, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be zero
	EXPECT_FLOAT_EQ(tth, 0.f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy)
{
	// GIVEN: 10 seconds of distance
	vehicle_speed = 6.2f;
	rtl_vector(0) = rtl_vector(1) = -(vehicle_speed * 10) / sqrtf(2);

	// WHEN: we get the tth
	float tth = time_to_home(rtl_vector, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be ten seconds
	EXPECT_FLOAT_EQ(tth, 10.f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy_5_seconds_z)
{
	// GIVEN: 10 seconds of xy distance and 5 seconds of Z
	vehicle_speed = 4.2f;
	vehicle_descent_speed = 1.2f;
	rtl_vector(0) = rtl_vector(1) = -(vehicle_speed * 10) / sqrtf(2);
	rtl_vector(2) = vehicle_descent_speed * 5;

	// WHEN: we get the tth
	float tth = time_to_home(rtl_vector, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be 15 seconds
	EXPECT_FLOAT_EQ(tth, 15.f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy_downwind_to_home)
{
	// GIVEN: 10 seconds of xy distance and 5 seconds of Z, and the wind is towards home
	vehicle_speed = 4.2f;
	rtl_vector(0) = rtl_vector(1) = -(vehicle_speed * 10) / sqrtf(2);

	wind_vel = matrix::Vector2f(-1, -1);

	// WHEN: we get the tth
	float tth = time_to_home(rtl_vector, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be 10, because we don't rely on wind towards home for RTL
	EXPECT_FLOAT_EQ(tth, 10.f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy_upwind_to_home)
{
	// GIVEN: 10 seconds of distance
	vehicle_speed = 4.2f;
	vehicle_descent_speed = 1.2f;
	rtl_vector(0) = rtl_vector(1) = -(vehicle_speed * 10) / sqrtf(2);

	wind_vel = matrix::Vector2f(1, 1) / sqrt(2) * vehicle_speed / 10;

	// WHEN: we get the tth
	float tth = time_to_home(rtl_vector, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be 11.111111... because it slows us down by 10% and time = dist/speed
	EXPECT_FLOAT_EQ(tth, 10 / 0.9f);
}

TEST_F(RangeRTL_tth, ten_seconds_xy_z_wind_across_home)
{
	// GIVEN: a 3 4 5 triangle, with vehicle airspeed being 5, wind 3, ground speed 4
	// and the vehicle is 10 seconds away

	vehicle_speed = 5.f;
	wind_vel = matrix::Vector2f(-1, 1) / sqrt(2) * 3.;
	rtl_vector(0) = rtl_vector(1) = -(4 * 10) / sqrtf(2);

	// WHEN: we get the tth
	float tth = time_to_home(rtl_vector, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should be 10
	EXPECT_FLOAT_EQ(tth, 10);
}

TEST_F(RangeRTL_tth, too_strong_upwind_to_home)
{
	// GIVEN: 10 seconds of distance
	vehicle_speed = 4.2f;
	vehicle_descent_speed = 1.2f;
	rtl_vector(0) = rtl_vector(1) = -(vehicle_speed * 10) / sqrtf(2);

	wind_vel = matrix::Vector2f(1, 1) / sqrt(2) * vehicle_speed * 1.001f;

	// WHEN: we get the tth
	float tth = time_to_home(rtl_vector, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should never get home
	EXPECT_TRUE(std::isinf(tth)) << tth;
}

TEST_F(RangeRTL_tth, too_strong_crosswind_to_home)
{
	// GIVEN: 10 seconds of distance
	vehicle_speed = 4.2f;
	vehicle_descent_speed = 1.2f;
	rtl_vector(0) = rtl_vector(1) = -(vehicle_speed * 10) / sqrtf(2);

	wind_vel = matrix::Vector2f(1, -1) / sqrt(2) * vehicle_speed * 1.001f;

	// WHEN: we get the tth
	float tth = time_to_home(rtl_vector, wind_vel, vehicle_speed, vehicle_descent_speed);

	// THEN: it should never get home
	EXPECT_TRUE(std::isinf(tth)) << tth;
}
