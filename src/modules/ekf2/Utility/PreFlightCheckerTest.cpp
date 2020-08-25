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
 * Test code for PreFlightChecker class
 * Run this test only using make tests TESTFILTER=PreFlightChecker
 */

#include <gtest/gtest.h>

#include "PreFlightChecker.hpp"

class PreFlightCheckerTest : public ::testing::Test
{
};

TEST_F(PreFlightCheckerTest, testInnovFailed)
{
	const float test_limit = 1.0; ///< is the limit for innovation_lpf
	const float spike_limit = 2.f * test_limit; ///< is the limit for innovation_lpf
	const float innovations[9] = 	 {0.0, 1.5, 2.5, -1.5, -2.5, 1.5, -1.5, -2.5, -2.5};
	const float innovations_lpf[9] = {0.0, 0.9, 0.9, -0.9, -0.9, 1.1, -1.1, -1.1, 1.1};
	const bool expected_result[9] = {false, false, true, false, true, true, true, true, true};

	for (int i = 0; i < 9; i++) {
		EXPECT_EQ(PreFlightChecker::checkInnovFailed(innovations_lpf[i], innovations[i], test_limit, spike_limit),
			  expected_result[i]);
	}

	// Smaller test limit, all the checks should fail except the first
	EXPECT_FALSE(PreFlightChecker::checkInnovFailed(innovations_lpf[0], innovations[0], 0.0, 0.0));

	for (int i = 1; i < 9; i++) {
		EXPECT_TRUE(PreFlightChecker::checkInnovFailed(innovations_lpf[i], innovations[i], 0.0, 0.0));
	}

	// Larger test limit, none of the checks should fail
	for (int i = 0; i < 9; i++) {
		EXPECT_FALSE(PreFlightChecker::checkInnovFailed(innovations_lpf[i], innovations[i], 2.0, 4.0));
	}
}

TEST_F(PreFlightCheckerTest, testInnov2dFailed)
{
	const float test_limit = 1.0;
	const float spike_limit = 2.0;
	Vector2f innovations[4] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, -2.5}, {1.5, -1.5}};
	Vector2f innovations_lpf[4] = {{0.0, 0.0}, {1.1, 0.0}, {0.5, 0.5}, {1.0, -1.0}};
	const bool expected_result[4] = {false, true, true, true};

	for (int i = 0; i < 4; i++) {
		EXPECT_EQ(PreFlightChecker::checkInnov2DFailed(innovations_lpf[i], innovations[i], test_limit, spike_limit),
			  expected_result[i]);
	}

	// Smaller test limit, all the checks should fail except the first
	EXPECT_FALSE(PreFlightChecker::checkInnov2DFailed(innovations[0], innovations_lpf[0], 0.0, 0.0));

	for (int i = 1; i < 4; i++) {
		EXPECT_TRUE(PreFlightChecker::checkInnov2DFailed(innovations_lpf[i], innovations[i], 0.0, 0.0));
	}

	// Larger test limit, none of the checks should fail
	for (int i = 0; i < 4; i++) {
		EXPECT_FALSE(PreFlightChecker::checkInnov2DFailed(innovations_lpf[i], innovations[i], 1.42, 2.84));
	}
}
