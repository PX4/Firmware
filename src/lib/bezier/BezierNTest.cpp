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

/**
 * Test code for the Velocity Smoothing library
 * Run this test only using make tests TESTFILTER=BezierN
 *
 * @author Julian Kent <julian@auterion.com>
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "BezierN.hpp"

TEST(BezierN_calculateBezier, checks_validity)
{
	matrix::Vector3f points[10];
	matrix::Vector3f a, b;
	EXPECT_FALSE(bezier::calculateBezierPosVel(nullptr, 10, 0.5f, a, b));
	EXPECT_FALSE(bezier::calculateBezierPosVel(points, 0, 0.5f, a, b));
	EXPECT_FALSE(bezier::calculateBezierPosVel(points, 10, -0.5f, a, b));
	EXPECT_FALSE(bezier::calculateBezierPosVel(points, 10, 1.5f, a, b));
}

TEST(BezierN_calculateBezier, checks_validity_accel)
{
	matrix::Vector3f points[10];
	matrix::Vector3f a, b, c;
	EXPECT_FALSE(bezier::calculateBezierPosVelAcc(nullptr, 10, 0.5f, a, b, c));
	EXPECT_FALSE(bezier::calculateBezierPosVelAcc(points, 0, 0.5f, a, b, c));
	EXPECT_FALSE(bezier::calculateBezierPosVelAcc(points, 10, -0.5f, a, b, c));
	EXPECT_FALSE(bezier::calculateBezierPosVelAcc(points, 10, 1.5f, a, b, c));
}

TEST(BezierN_calculateBezier, work_1_point)
{
	// GIVEN: a single point bezier curve
	matrix::Vector3f points[2] = {matrix::Vector3f(1, 2, 3), matrix::Vector3f(NAN, NAN, NAN)};
	matrix::Vector3f pos, vel;
	pos *= NAN;
	vel *= NAN;

	// WHEN: we get the half-way point
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 1, 0.5f, pos, vel));

	// THEN: it should be the same as the point, and the velocity should be 0
	EXPECT_EQ((pos - points[0]).norm(), 0.f);
	EXPECT_EQ(vel.norm(), 0.f);
}

TEST(BezierN_calculateBezier, works_2_points)
{
	// GIVEN: a 2-point bezier curve
	matrix::Vector3f points[3] = {matrix::Vector3f(1, 2, 3), matrix::Vector3f(5, 0, 1), matrix::Vector3f(NAN, NAN, NAN)};
	matrix::Vector3f pos, vel;
	pos *= NAN;
	vel *= NAN;

	// WHEN: we get the half-way point
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 2, 0.5f, pos, vel));

	// THEN: the position should be the mid-point between the start and end, and velocity should be the length
	EXPECT_EQ((pos - matrix::Vector3f(3, 1, 2)).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel - (points[1] - points[0])).norm(), 0.f);

	// WHEN: we get the beginning point
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 2, 0.f, pos, vel));

	// THEN: the position should be the first point, and the velocity should still be the length
	EXPECT_EQ((pos - points[0]).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel - (points[1] - points[0])).norm(), 0.f);

	// WHEN: we get the end point
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 2, 1.f, pos, vel));

	// THEN: the position should be the first point, and the velocity should still be the length
	EXPECT_EQ((pos - points[1]).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel - (points[1] - points[0])).norm(), 0.f);
}

TEST(BezierN_calculateBezier, works_3_points_zero_accel)
{
	// GIVEN: 3 points bezier, evenly spaced in a straight line
	matrix::Vector3f points[4] = {matrix::Vector3f(1, 2, 3), matrix::Vector3f(5, 0, 1), matrix::Vector3f(9, -2, -1), matrix::Vector3f(NAN, NAN, NAN)};
	matrix::Vector3f pos, vel;
	pos *= NAN;
	vel *= NAN;

	// WHEN: we get the half-way point
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 3, 0.5f, pos, vel));

	// THEN: it should be the middle point, with velocity of 1st to last
	EXPECT_FLOAT_EQ((pos - points[1]).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel - (points[2] - points[0])).norm(), 0.f);

	matrix::Vector3f pos2, vel2, accel2;

	// WHEN: we use the accel interface
	EXPECT_TRUE(bezier::calculateBezierPosVelAcc(points, 3, 0.5f, pos2, vel2, accel2));

	// THEN: it should give same position, velocity as the non-accel interface, and zero accel (since this curve is 0 accel)
	EXPECT_FLOAT_EQ((pos2 - pos).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel2 - vel).norm(), 0.f);
	EXPECT_FLOAT_EQ(accel2.norm(), 0.f);

	// WHEN: we check at the beginning
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 3, 0.f, pos, vel));
	EXPECT_TRUE(bezier::calculateBezierPosVelAcc(points, 3, 0.f, pos2, vel2, accel2));

	// THEN: it should be the starting point and same velocity
	EXPECT_FLOAT_EQ((pos - points[0]).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel - (points[2] - points[0])).norm(), 0.f);
	EXPECT_FLOAT_EQ((pos2 - pos).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel2 - vel).norm(), 0.f);
	EXPECT_FLOAT_EQ(accel2.norm(), 0.f);

	// WHEN: we check at the end
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 3, 1.f, pos, vel));
	EXPECT_TRUE(bezier::calculateBezierPosVelAcc(points, 3, 1.f, pos2, vel2, accel2));

	// THEN: it should be the ending point and same velocity
	EXPECT_FLOAT_EQ((pos - points[2]).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel - (points[2] - points[0])).norm(), 0.f);
	EXPECT_FLOAT_EQ((pos2 - pos).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel2 - vel).norm(), 0.f);
	EXPECT_FLOAT_EQ(accel2.norm(), 0.f);
}

TEST(BezierN_calculateBezier, works_3_points_accel)
{
	// GIVEN: 3 points bezier, in a curve
	matrix::Vector3f points[4] = {matrix::Vector3f(1, 2, 3), matrix::Vector3f(5, 0, 1), matrix::Vector3f(19, -8, 1), matrix::Vector3f(NAN, NAN, NAN)};
	matrix::Vector3f pos, vel;
	pos *= NAN;
	vel *= NAN;

	matrix::Vector3f pos2;
	pos2 *= NAN;

	matrix::Vector3f accel_start, accel_mid, accel_end;
	matrix::Vector3f vel_start, vel_mid, vel_end;


	// WHEN: we check at the beginning
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 3, 0.f, pos, vel));
	EXPECT_TRUE(bezier::calculateBezierPosVelAcc(points, 3, 0.f, pos2, vel_start, accel_start));

	// THEN: it should give same position, velocity as the non-accel interface, and non-zero accel
	EXPECT_FLOAT_EQ((pos2 - pos).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel_start - vel).norm(), 0.f);
	EXPECT_GT(accel_start.norm(), 0.f);

	// WHEN: we use the accel interface to get the half-way point
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 3, 0.5f, pos, vel));
	EXPECT_TRUE(bezier::calculateBezierPosVelAcc(points, 3, 0.5f, pos2, vel_mid, accel_mid));

	// THEN: the values should matche between accel and non-accel version
	EXPECT_FLOAT_EQ((pos2 - pos).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel_mid - vel).norm(), 0.f);

	// AND: the accel should be the same as the start
	EXPECT_FLOAT_EQ((accel_mid - accel_start).norm(), 0.f);


	// WHEN: we check at the end
	EXPECT_TRUE(bezier::calculateBezierPosVel(points, 3, 1.f, pos, vel));
	EXPECT_TRUE(bezier::calculateBezierPosVelAcc(points, 3, 1.f, pos2, vel_end, accel_end));

	// THEN: it should be the ending point, and accel should match
	EXPECT_FLOAT_EQ((pos - points[2]).norm(), 0.f);
	EXPECT_FLOAT_EQ((pos2 - pos).norm(), 0.f);
	EXPECT_FLOAT_EQ((vel_end - vel).norm(), 0.f);
	EXPECT_FLOAT_EQ((accel_end - accel_start).norm(), 0.f);

	// FINALLY: mid point velocity should be average of start and end velocity
	EXPECT_FLOAT_EQ((vel_mid - 0.5f * (vel_start + vel_end)).norm(), 0.f);
}

TEST(BezierN_calculateBezierYaw, checks_validity)
{
	float points[10];
	float a, b;
	EXPECT_FALSE(bezier::calculateBezierYaw(nullptr, 10, 0.5f, a, b));
	EXPECT_FALSE(bezier::calculateBezierYaw(points, 0, 0.5f, a, b));
	EXPECT_FALSE(bezier::calculateBezierYaw(points, 10, -0.5f, a, b));
	EXPECT_FALSE(bezier::calculateBezierYaw(points, 10, 1.5f, a, b));
}

TEST(BezierN_calculateBezierYaw, work_1_point)
{
	// GIVEN: a single yaw point
	float points[2] = {M_PI / 2, NAN};
	float yaw, yaw_speed;

	// WHEN: we use it as a 1-point bezier curve
	EXPECT_TRUE(bezier::calculateBezierYaw(points, 1, 0.5f, yaw, yaw_speed));

	// THEN: it should have that same value, and the velocity should be 0
	EXPECT_FLOAT_EQ(yaw, M_PI / 2);
	EXPECT_FLOAT_EQ(yaw_speed, 0);
}

TEST(BezierN_calculateBezierYaw, work_2_points)
{
	// GIVEN: a single yaw point
	float points[3] = {0, M_PI / 2, NAN};
	float yaw, yaw_speed;

	// WHEN: we get the beginning
	EXPECT_TRUE(bezier::calculateBezierYaw(points, 2, 0.f, yaw, yaw_speed));

	// THEN: it should have the beginning value, and the velocity should be the difference between first and last
	EXPECT_FLOAT_EQ(yaw, 0);
	EXPECT_FLOAT_EQ(yaw_speed, M_PI / 2);

	// WHEN: we get the middle
	EXPECT_TRUE(bezier::calculateBezierYaw(points, 2, 0.5f, yaw, yaw_speed));

	// THEN: it should have the beginning value, and the velocity should be the difference between first and last
	EXPECT_FLOAT_EQ(yaw, M_PI / 4);
	EXPECT_FLOAT_EQ(yaw_speed, M_PI / 2);

	// WHEN: we get the end
	EXPECT_TRUE(bezier::calculateBezierYaw(points, 2, 1.f, yaw, yaw_speed));

	// THEN: it should have the beginning value, and the velocity should be the difference between first and last
	EXPECT_FLOAT_EQ(yaw, M_PI / 2);
	EXPECT_FLOAT_EQ(yaw_speed, M_PI / 2);
}

TEST(BezierN_calculateBezierYaw, work_2_points_wrap)
{
	// GIVEN: 2 yaw points on either side of the +- PI wrap line
	float points[3] = {-M_PI + 0.1, M_PI - 0.1, NAN};
	float yaw, yaw_speed;

	// WHEN: we get the beginning
	EXPECT_TRUE(bezier::calculateBezierYaw(points, 2, 0.f, yaw, yaw_speed));

	// THEN: it should have the beginning value, and the velocity should be the wrapped distance between first and last
	EXPECT_FLOAT_EQ(yaw, -M_PI + 0.1);
	EXPECT_NEAR(yaw_speed, -0.2, 1e-6f);

	// WHEN: we get the middle
	EXPECT_TRUE(bezier::calculateBezierYaw(points, 2, 0.5f, yaw, yaw_speed));

	// THEN: it should have the wrapped middle value, and the velocity should be the wrapped distance between first and last
	EXPECT_FLOAT_EQ(matrix::wrap_pi(yaw - float(M_PI)), 0);
	EXPECT_NEAR(yaw_speed, -0.2, 1e-6f);

	// WHEN: we get the end
	EXPECT_TRUE(bezier::calculateBezierYaw(points, 2, 1.f, yaw, yaw_speed));

	// THEN: it should have the end value, and the velocity should be the wrapped distance between first and last
	EXPECT_FLOAT_EQ(yaw, M_PI - 0.1);
	EXPECT_NEAR(yaw_speed, -0.2, 1e-6f);
}


TEST(BezierN_calculateT, rejects_bad_timestamps)
{
	float f = NAN;
	EXPECT_FALSE(bezier::calculateT(100, 1000, 99, f));
	EXPECT_FALSE(bezier::calculateT(100, 1000, 1001, f));
	EXPECT_FALSE(bezier::calculateT(1001, 1000, 1001, f));
}


TEST(BezierN_calculateT, begin_middle_end)
{
	float f = NAN;
	EXPECT_TRUE(bezier::calculateT(100, 1000, 100, f));
	EXPECT_FLOAT_EQ(f, 0.f);

	EXPECT_TRUE(bezier::calculateT(100, 1000, 550, f));
	EXPECT_FLOAT_EQ(f, 0.5f);

	EXPECT_TRUE(bezier::calculateT(100, 1000, 1000, f));
	EXPECT_FLOAT_EQ(f, 1.f);
}

TEST(BezierN_calculateT, giant_offset)
{
	int64_t offset = 0xFFFFFFFFFFFF; // 48 bit max
	float f = NAN;
	EXPECT_TRUE(bezier::calculateT(offset + 100, offset + 1000, offset + 100, f));
	EXPECT_FLOAT_EQ(f, 0.f);

	EXPECT_TRUE(bezier::calculateT(offset + 100, offset + 1000, offset + 550, f));
	EXPECT_FLOAT_EQ(f, 0.5f);

	EXPECT_TRUE(bezier::calculateT(offset + 100, offset + 1000, offset + 1000, f));
	EXPECT_FLOAT_EQ(f, 1.f);
}
