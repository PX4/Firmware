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
 * Run this test only using make tests TESTFILTER=VelocitySmoothing
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "VelocitySmoothing.hpp"

using namespace matrix;

class VelocitySmoothingTest : public ::testing::Test
{
public:
	void setConstraints(float j_max, float a_max, float v_max);
	void setInitialConditions(Vector3f acc, Vector3f vel, Vector3f pos);
	void updateTrajectories(float dt, Vector3f velocity_setpoints);

	VelocitySmoothing _trajectories[3];
};

void VelocitySmoothingTest::setConstraints(float j_max, float a_max, float v_max)
{
	for (int i = 0; i < 3; i++) {
		_trajectories[i].setMaxJerk(j_max);
		_trajectories[i].setMaxAccel(a_max);
		_trajectories[i].setMaxVel(v_max);
	}
}

void VelocitySmoothingTest::setInitialConditions(Vector3f a0, Vector3f v0, Vector3f x0)
{
	for (int i = 0; i < 3; i++) {
		_trajectories[i].setCurrentAcceleration(a0(i));
		_trajectories[i].setCurrentVelocity(v0(i));
		_trajectories[i].setCurrentPosition(x0(i));
	}
}

void VelocitySmoothingTest::updateTrajectories(float dt, Vector3f velocity_setpoints)
{
	for (int i = 0; i < 3; i++) {
		_trajectories[i].updateTraj(dt);
		EXPECT_LE(fabsf(_trajectories[i].getCurrentJerk()), _trajectories[i].getMaxJerk());
		EXPECT_LE(fabsf(_trajectories[i].getCurrentAcceleration()), _trajectories[i].getMaxAccel());
		EXPECT_LE(fabsf(_trajectories[i].getCurrentVelocity()), _trajectories[i].getMaxVel());
	}

	for (int i = 0; i < 3; i++) {
		_trajectories[i].updateDurations(velocity_setpoints(i));
	}

	VelocitySmoothing::timeSynchronization(_trajectories, 2);
}

TEST_F(VelocitySmoothingTest, testTimeSynchronization)
{
	// GIVEN: A set of constraints
	const float j_max = 55.2f;
	const float a_max = 6.f;
	const float v_max = 6.f;

	setConstraints(j_max, a_max, v_max);

	// AND: A set of initial conditions
	Vector3f a0(0.22f, 0.f, 0.22f);
	Vector3f v0(2.47f, -5.59e-6f, 2.47f);
	Vector3f x0(0.f, 0.f, 0.f);

	setInitialConditions(a0, v0, x0);

	// WHEN: We generate trajectories (time synchronized in XY) with constant setpoints and dt
	Vector3f velocity_setpoints(-3.f, 1.f, 0.f);
	updateTrajectories(0.f, velocity_setpoints);


	// THEN: The X and Y trajectories should have the same total time (= time sunchronized)
	EXPECT_LE(fabsf(_trajectories[0].getTotalTime() - _trajectories[1].getTotalTime()), 0.0001);
}

TEST_F(VelocitySmoothingTest, testConstantSetpoint)
{
	// GIVEN: A set of constraints
	const float j_max = 55.2f;
	const float a_max = 6.f;
	const float v_max = 6.f;

	setConstraints(j_max, a_max, v_max);

	// AND: A set of initial conditions
	Vector3f a0(0.f, 0.f, 0.f);
	Vector3f v0(0.f, 0.f, 0.f);
	Vector3f x0(0.f, 0.f, 0.f);

	setInitialConditions(a0, v0, x0);

	// WHEN: We generate trajectories with constant setpoints and dt
	Vector3f velocity_setpoints(-3.f, 0.f, -1.f);

	// Compute the number of steps required to reach desired value
	// The updateTrajectories is first called once to compute the total time
	const float dt = 0.01;
	updateTrajectories(0.f, velocity_setpoints);
	float t123 = _trajectories[0].getTotalTime();
	int nb_steps = ceil(t123 / dt);

	for (int i = 0; i < nb_steps; i++) {
		updateTrajectories(dt, velocity_setpoints);
	}

	// THEN: All the trajectories should have reach their
	// final state: desired velocity target and zero acceleration
	for (int i = 0; i < 3; i++) {
		EXPECT_LE(fabsf(_trajectories[i].getCurrentVelocity() - velocity_setpoints(i)), 0.01f);
		EXPECT_LE(fabsf(_trajectories[i].getCurrentAcceleration()), 0.0001f);
	}
}

TEST_F(VelocitySmoothingTest, testZeroSetpoint)
{
	// GIVEN: A set of null initial conditions
	Vector3f a0(0.f, 0.f, 0.f);
	Vector3f v0(0.f, 0.f, 0.f);
	Vector3f x0(0.f, 0.f, 0.f);

	setInitialConditions(a0, v0, x0);

	// AND: Zero setpoints
	Vector3f velocity_setpoints(0.f, 0.f, 0.f);
	float t = 0.f;
	const float dt = 0.01f;

	// WHEN: We run a few times the algorithm
	for (int i = 0; i < 60; i++) {
		updateTrajectories(dt, velocity_setpoints);
		t += dt;
	}

	// THEN: All the trajectories should still be zero
	for (int i = 0; i < 3; i++) {
		EXPECT_FLOAT_EQ(_trajectories[i].getCurrentJerk(), 0.f);
		EXPECT_FLOAT_EQ(_trajectories[i].getCurrentAcceleration(), 0.f);
		EXPECT_FLOAT_EQ(_trajectories[i].getCurrentVelocity(), 0.f);
		EXPECT_FLOAT_EQ(_trajectories[i].getCurrentPosition(), 0.f);
	}
}
