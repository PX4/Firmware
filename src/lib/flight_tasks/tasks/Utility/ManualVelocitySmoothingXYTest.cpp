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
 * Test code for the Manual Velocity Smoothing library
 * Run this test only using make tests TESTFILTER=ManualVelocitySmoothing
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "ManualVelocitySmoothingXY.hpp"

using namespace matrix;

class ManualVelocitySmoothingXYTest : public ::testing::Test
{
public:
	ManualVelocitySmoothingXY _smoothing;
};


TEST_F(ManualVelocitySmoothingXYTest, setGet)
{
	// GIVEN: Some max values
	_smoothing.setMaxJerk(11.f);
	_smoothing.setMaxAccel(7.f);
	_smoothing.setMaxVel(5.f);

	// THEN: We should be able to get them back
	EXPECT_FLOAT_EQ(_smoothing.getMaxJerk(), 11.f);
	EXPECT_FLOAT_EQ(_smoothing.getMaxAccel(), 7.f);
	EXPECT_FLOAT_EQ(_smoothing.getMaxVel(), 5.f);
}

TEST_F(ManualVelocitySmoothingXYTest, getCurrentState)
{
	// GIVEN: the initial conditions
	Vector2f v0(11.f, 13.f);
	_smoothing.setCurrentVelocity(v0);

	// WHEN: we get the current state
	Vector3f j_end;
	Vector3f a_end;
	Vector3f v_end;
	Vector3f x_end;
	j_end.xy() = _smoothing.getCurrentJerk();
	a_end.xy() = _smoothing.getCurrentAcceleration();
	v_end.xy() = _smoothing.getCurrentVelocity();
	x_end.xy() = _smoothing.getCurrentPosition();

	// THEN: the returned values should match the input
	EXPECT_EQ(j_end, Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(a_end, Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(v_end, Vector3f(11.f, 13.f, 0.f));
	EXPECT_EQ(x_end, Vector3f(0.f, 0.f, 0.f));
}
