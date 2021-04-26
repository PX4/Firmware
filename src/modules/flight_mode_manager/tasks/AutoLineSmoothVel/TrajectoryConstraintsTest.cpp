#include <gtest/gtest.h>

#include "TrajectoryConstraints.hpp"

using namespace matrix;
using namespace math::trajectory;

class TrajectoryConstraintsTest : public ::testing::Test
{
public:
	VehicleDynamicLimits config;

	Vector3f vehicle_location;
	Vector3f target;
	Vector3f next_target;

	float final_speed = 0;

	void SetUp() override
	{
		config.z_accept_rad = 1.f;
		config.xy_accept_rad = 0.99f;

		config.max_acc_xy = 3.f;
		config.max_jerk = 10.f;

		config.max_speed_xy = 10.f;

		config.max_acc_xy_radius_scale = 0.8f;

		/*
		 *             (20,20)
		 *              Next target
		 *
		 *              ^
		 *              |
		 *
		 * (10,10)      (20,10)
		 * Vehicle  ->  Target
		 *
		 */
		vehicle_location = Vector3f(10, 10, 5);
		target = Vector3f(20, 10, 5);
		next_target = Vector3f(20, 20, 5);
	}
};

TEST_F(TrajectoryConstraintsTest, testStraight)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.5f * (next_target - vehicle_location);

	// WHEN: we get the speed for straight line travel
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed directly to the end point
	Vector3f direct_points[2] = {vehicle_location, next_target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightNaN)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.5f * (next_target - vehicle_location);
	next_target(0) = NAN;
	next_target(1) = NAN;

	// WHEN: we get the speed for points which are NaN afterwards
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed to the closer point
	Vector3f direct_points[2] = {vehicle_location, target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightLowJerkClose)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.05f * (next_target - vehicle_location);
	config.max_jerk = 8.f;

	// WHEN: we get the speed for straight line travel
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed directly to the end point
	Vector3f direct_points[2] = {vehicle_location, next_target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightMidClose)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.05f * (next_target - vehicle_location);

	// WHEN: we get the speed for straight line travel
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed directly to the end point
	Vector3f direct_points[2] = {vehicle_location, next_target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}

TEST_F(TrajectoryConstraintsTest, testStraightMidFar)
{
	// GIVEN: 3 waypoints in straight line
	next_target = target + 2.f * (target - vehicle_location);
	target = vehicle_location + 0.95f * (next_target - vehicle_location);

	// WHEN: we get the speed for straight line travel
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be the same as speed directly to the end point
	Vector3f direct_points[2] = {vehicle_location, next_target};
	float direct_speed = computeXYSpeedFromWaypoints<2>(direct_points, config);

	EXPECT_FLOAT_EQ(through_speed, direct_speed);
}


TEST_F(TrajectoryConstraintsTest, test90Angle)
{
	// GIVEN: 3 waypoints in 90 degree angle
	EXPECT_FLOAT_EQ(0.f, (vehicle_location - target).dot(target - next_target));

	// WHEN: we get the speed for travel around the path
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be slightly faster than stopping at the intermediate point
	Vector3f stop_points[2] = {vehicle_location, target};
	float stop_speed = computeXYSpeedFromWaypoints<2>(stop_points, config);

	EXPECT_GT(through_speed, stop_speed); //faster
	EXPECT_LT(through_speed, stop_speed * 1.03f); // but less than 3% faster
}

TEST_F(TrajectoryConstraintsTest, test45Angle)
{
	// GIVEN: 3 waypoints in 45 degree angle
	next_target = Vector3f(25, 15, 5);

	// WHEN: we get the speed for travel around the path
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be slightly faster than stopping at the intermediate point
	Vector3f stop_points[2] = {vehicle_location, target};
	float stop_speed = computeXYSpeedFromWaypoints<2>(stop_points, config);

	EXPECT_GT(through_speed, stop_speed * 1.03f); // more than 3% faster
	EXPECT_LT(through_speed, stop_speed * 1.06f); // but less than 6% faster
}

TEST_F(TrajectoryConstraintsTest, test10Angle)
{
	// GIVEN: 3 waypoints in 10 degree angle
	next_target = Vector3f(30, 11.7, 5);

	// WHEN: we get the speed for travel around the path
	Vector3f waypoints[3] = {vehicle_location, target, next_target};
	float through_speed = computeXYSpeedFromWaypoints<3>(waypoints, config);

	// THEN: it should be slightly faster than stopping at the intermediate point
	Vector3f stop_points[2] = {vehicle_location, target};
	float stop_speed = computeXYSpeedFromWaypoints<2>(stop_points, config);

	EXPECT_GT(through_speed, stop_speed * 1.25f); // more than 25% faster
	EXPECT_LT(through_speed, stop_speed * 1.3f); // but less than 30% faster
}

TEST_F(TrajectoryConstraintsTest, test10AngleFarNext)
{
	// GIVEN: 3 waypoints in 10 degree angle, but next waypoint is far
	next_target = 2.f * (Vector3f(30, 11.7, 5) - target) + target;

	// WHEN: we get the speed for travel around the path
	Vector3f far_waypoints[3] = {vehicle_location, target, next_target};
	float far_speed = computeXYSpeedFromWaypoints<3>(far_waypoints, config);

	// THEN: it should be the same speed as a closer next waypoint at the same angle, since the bottleneck is the turn
	next_target = Vector3f(30, 11.7, 5);
	Vector3f close_waypoints[3] = {vehicle_location, target, next_target};
	float close_speed = computeXYSpeedFromWaypoints<3>(close_waypoints, config);

	EXPECT_FLOAT_EQ(far_speed, close_speed);
}

TEST_F(TrajectoryConstraintsTest, test10AngleCloseNext)
{
	// GIVEN: 3 waypoints in right angle, but next waypoint is far
	next_target = .2f * (Vector3f(30, 11.7, 5) - target) + target;

	// WHEN: we get the speed for travel around the path
	Vector3f close_waypoints[3] = {vehicle_location, target, next_target};
	float close_speed = computeXYSpeedFromWaypoints<3>(close_waypoints, config);

	// THEN: it should be slower than a further next waypoint at the same angle, since the bottleneck is the distance
	next_target = Vector3f(30, 11.7, 5);
	Vector3f normal_waypoints[3] = {vehicle_location, target, next_target};
	float normal_speed = computeXYSpeedFromWaypoints<3>(normal_waypoints, config);

	EXPECT_LT(close_speed, normal_speed);
}

TEST(TrajectoryConstraintsClamp, clampToXYNormNoEffectLarge)
{
	// GIVEN: a short vector
	Vector3f vec(1, 2, 3);

	// WHEN: we clamp it on XY with a long cutoff
	clampToXYNorm(vec, 1000.f);

	// THEN: it shouldn't change
	EXPECT_EQ(vec, Vector3f(1, 2, 3));
}

TEST(TrajectoryConstraintsClamp, clampToZNormNoEffect)
{
	// GIVEN: a short vector
	Vector3f vec(1, 2, 3);

	// WHEN: we clamp it on XY with a long cutoff
	clampToZNorm(vec, 1000.f);

	// THEN: it shouldn't change
	EXPECT_EQ(vec, Vector3f(1, 2, 3));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormNoEffectExact)
{
	// GIVEN: a vector
	Vector3f vec(3, 4, 1);

	// WHEN: we clamp it on XY with exact cutoff
	clampToXYNorm(vec, 5.f);

	// THEN: it shouldn't change
	EXPECT_EQ(vec, Vector3f(3, 4, 1));
}

TEST(TrajectoryConstraintsClamp, clampToZNormNoEffectExact)
{
	// GIVEN: a vector
	Vector3f vec(3, 4, -1);

	// WHEN: we clamp it on Z with exact cutoff
	clampToZNorm(vec, 1.f);

	// THEN: it shouldn't change
	EXPECT_EQ(vec, Vector3f(3, 4, -1));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormHalf)
{
	// GIVEN: a vector
	Vector3f vec(3, 4, 1);

	// WHEN: we clamp it on XY with half hypot length
	clampToXYNorm(vec, 2.5f);

	// THEN: it should be half length
	EXPECT_TRUE(vec == Vector3f(1.5f, 2.f, 0.5f));
}

TEST(TrajectoryConstraintsClamp, clampToZNormHalf)
{
	// GIVEN: a vector
	Vector3f vec(3, 4, 10);

	// WHEN: we clamp it on Z with half length
	clampToZNorm(vec, 5.f);

	// THEN: it should be half length
	EXPECT_TRUE(vec == Vector3f(1.5f, 2.f, 5.f));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormZero)
{
	// GIVEN: a vector
	Vector3f vec(3, 4, 1);

	// WHEN: we clamp it on XY with half hypot length
	clampToXYNorm(vec, 0.f);

	// THEN: it should be 0
	EXPECT_TRUE(vec == Vector3f(0.f, 0.f, 0.f));
}

TEST(TrajectoryConstraintsClamp, clampToZNormZero)
{
	// GIVEN: a vector
	Vector3f vec(3, 4, 1);

	// WHEN: we clamp it on Z with half hypot length
	clampToZNorm(vec, 0.f);

	// THEN: it should be 0
	EXPECT_TRUE(vec == Vector3f(0.f, 0.f, 0.f));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormVecZero)
{
	// GIVEN: a vector
	Vector3f vec(0, 0, 0);

	// WHEN: we clamp it on XY
	clampToXYNorm(vec, 1.f);

	// THEN: it should be 0 still
	EXPECT_TRUE(vec == Vector3f(0.f, 0.f, 0.f));
}

TEST(TrajectoryConstraintsClamp, clampToZNormVecZero)
{
	// GIVEN: a vector
	Vector3f vec(0, 0, 0);

	// WHEN: we clamp it on Z
	clampToZNorm(vec, 1.f);

	// THEN: it should be 0 still
	EXPECT_TRUE(vec == Vector3f(0.f, 0.f, 0.f));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormVecZeroToZero)
{
	// GIVEN: a vector
	Vector3f vec(0, 0, 0);

	// WHEN: we clamp it on XY
	clampToXYNorm(vec, 0.f);

	// THEN: it should be 0 still
	EXPECT_TRUE(vec == Vector3f(0.f, 0.f, 0.f));
}

TEST(TrajectoryConstraintsClamp, clampToZNormVecZeroToZero)
{
	// GIVEN: a vector
	Vector3f vec(0, 0, 0);

	// WHEN: we clamp it on XY
	clampToZNorm(vec, 0.f);

	// THEN: it should be 0 still
	EXPECT_TRUE(vec == Vector3f(0.f, 0.f, 0.f));
}
