/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
#include "Functions.hpp"

using namespace math;

TEST(FunctionsTest, signNoZero)
{
	EXPECT_FLOAT_EQ(signNoZero(-123.456f), -1.f);
	EXPECT_FLOAT_EQ(signNoZero(-1.f), -1.f);
	EXPECT_FLOAT_EQ(signNoZero(-.0001f), -1.f);
	EXPECT_FLOAT_EQ(signNoZero(0.f), 1.f);
	EXPECT_FLOAT_EQ(signNoZero(.0001f), 1.f);
	EXPECT_FLOAT_EQ(signNoZero(1.f), 1.f);
	EXPECT_FLOAT_EQ(signNoZero(123.456f), 1.f);
}

TEST(FunctionsTest, expo)
{
	// input value limits
	EXPECT_FLOAT_EQ(expo(-12.f, .5f), -1.f);
	EXPECT_FLOAT_EQ(expo(-1.f, .5f), -1.f);
	EXPECT_FLOAT_EQ(expo(0.f, .5f), 0.f);
	EXPECT_FLOAT_EQ(expo(1.f, .5f), 1.f);
	EXPECT_FLOAT_EQ(expo(45.f, .5f), 1.f);

	// expo parameter limits
	EXPECT_FLOAT_EQ(expo(.1f, 0.f), .1f);
	EXPECT_FLOAT_EQ(expo(.1f, -12.f), .1f);
	EXPECT_FLOAT_EQ(expo(.1f, 1.f), .001f);
	EXPECT_FLOAT_EQ(expo(.1f, 12.f), .001f);

	// different in range values
	EXPECT_FLOAT_EQ(expo(.1f, .5f), .0505f);
	EXPECT_FLOAT_EQ(expo(-.2f, .5f), -.104f);
	EXPECT_FLOAT_EQ(expo(.3f, .7f), .1089f);
	EXPECT_FLOAT_EQ(expo(-.4f, .9f), -.0976f);
}

TEST(FunctionsTest, superexpo)
{
	// input value limits
	EXPECT_FLOAT_EQ(superexpo(-12.f, .5f, .5f), -1.f);
	EXPECT_FLOAT_EQ(superexpo(-1.f, .5f, .5f), -1.f);
	EXPECT_FLOAT_EQ(superexpo(0.f, .5f, .5f), 0.f);
	EXPECT_FLOAT_EQ(superexpo(1.f, .5f, .5f), 1.f);
	EXPECT_FLOAT_EQ(superexpo(45.f, .5f, .5f), 1.f);

	// superexpo parameter limits
	EXPECT_FLOAT_EQ(superexpo(.1f, 0.f, 0.f), .1f);
	EXPECT_FLOAT_EQ(superexpo(.1f, 0.f, -12.f), .1f);
	EXPECT_NEAR(superexpo(.1f, 0.f, 0.99f), .0011098779134295227f, 1e-8f);
	EXPECT_NEAR(superexpo(.1f, 0.f, 12.f), .0011098779134295227f, 1e-8f);

	// different in range values
	EXPECT_FLOAT_EQ(superexpo(.5f, 0.f, .5f), .33333333333f);
	EXPECT_FLOAT_EQ(superexpo(-.2f, .5f, .5f), -.057777781f);
	EXPECT_FLOAT_EQ(superexpo(.3f, .7f, .5f), .064058825f);
	EXPECT_FLOAT_EQ(superexpo(-.4f, .9f, .5f), -.061000008f);
}

TEST(FunctionsTest, deadzone)
{
	// input value limits
	EXPECT_FLOAT_EQ(deadzone(-12.f, .5f), -1.f);
	EXPECT_FLOAT_EQ(deadzone(-1.f, .5f), -1.f);
	EXPECT_FLOAT_EQ(deadzone(0.f, .5f), 0.f);
	EXPECT_FLOAT_EQ(deadzone(1.f, .5f), 1.f);
	EXPECT_FLOAT_EQ(deadzone(45.f, .5f), 1.f);

	// deadzone parameter limits
	EXPECT_FLOAT_EQ(deadzone(.1f, 0.f), .1f);
	EXPECT_FLOAT_EQ(deadzone(.1f, -12.f), .1f);
	EXPECT_FLOAT_EQ(deadzone(.1f, 1.f), 0.f);
	EXPECT_FLOAT_EQ(deadzone(.1f, 12.f), 0.f);

	// different in range values
	EXPECT_FLOAT_EQ(deadzone(.1f, 0.f), .1f);
	EXPECT_FLOAT_EQ(deadzone(.1f, .1f), 0.f);
	EXPECT_FLOAT_EQ(deadzone(.2f, .1f), 0.1111111111f);
	EXPECT_FLOAT_EQ(deadzone(-.2f, .5f), 0.f);
	EXPECT_FLOAT_EQ(deadzone(.7f, .3f), .57142854f);
	EXPECT_FLOAT_EQ(deadzone(-.9f, .4f), -.83333325f);
}

TEST(FunctionsTest, expo_deadzone)
{
	// input value limits
	EXPECT_FLOAT_EQ(expo_deadzone(-12.f, .5f, .5f), -1.f);
	EXPECT_FLOAT_EQ(expo_deadzone(-1.f, .5f, .5f), -1.f);
	EXPECT_FLOAT_EQ(expo_deadzone(0.f, .5f, .5f), 0.f);
	EXPECT_FLOAT_EQ(expo_deadzone(1.f, .5f, .5f), 1.f);
	EXPECT_FLOAT_EQ(expo_deadzone(45.f, .5f, .5f), 1.f);

	// different in range values
	EXPECT_FLOAT_EQ(expo_deadzone(.5f, 0.f, .5f), 0.f);
	EXPECT_FLOAT_EQ(expo_deadzone(-.5f, .5f, .2f), -.21386719f);
	EXPECT_FLOAT_EQ(expo_deadzone(.5f, .7f, .3f), .10204081f);
	EXPECT_FLOAT_EQ(expo_deadzone(-.5f, .9f, .4f), -.020833336f);
}

TEST(FunctionsTest, gradual)
{
	// factor of *2, offset +1
	EXPECT_FLOAT_EQ(gradual(-12.f, 0.f, 1.f, 1.f, 3.f), 1.f);
	EXPECT_FLOAT_EQ(gradual(0.f, 0.f, 1.f, 1.f, 3.f), 1.f);
	EXPECT_FLOAT_EQ(gradual(.25f, 0.f, 1.f, 1.f, 3.f), 1.5f);
	EXPECT_FLOAT_EQ(gradual(.5f, 0.f, 1.f, 1.f, 3.f), 2.f);
	EXPECT_FLOAT_EQ(gradual(.75f, 0.f, 1.f, 1.f, 3.f), 2.5f);
	EXPECT_FLOAT_EQ(gradual(1.f, 0.f, 1.f, 1.f, 3.f), 3.f);
	EXPECT_FLOAT_EQ(gradual(12.f, 0.f, 1.f, 1.f, 3.f), 3.f);

	// factor of *2, offset +3
	EXPECT_FLOAT_EQ(gradual(-12.f, 1.f, 2.f, 4.f, 6.f), 4.f);
	EXPECT_FLOAT_EQ(gradual(1.f, 1.f, 2.f, 4.f, 6.f), 4.f);
	EXPECT_FLOAT_EQ(gradual(1.25f, 1.f, 2.f, 4.f, 6.f), 4.5f);
	EXPECT_FLOAT_EQ(gradual(1.5f, 1.f, 2.f, 4.f, 6.f), 5.f);
	EXPECT_FLOAT_EQ(gradual(1.75f, 1.f, 2.f, 4.f, 6.f), 5.5f);
	EXPECT_FLOAT_EQ(gradual(2.f, 1.f, 2.f, 4.f, 6.f), 6.f);
	EXPECT_FLOAT_EQ(gradual(12.f, 1.f, 2.f, 4.f, 6.f), 6.f);
}

TEST(FunctionsTest, gradual3)
{
	// factor of *2, offset +1
	EXPECT_FLOAT_EQ(gradual3(-12.f,
				 0.f, .5f, 1.5f,
				 1.f, 2.f, 3.f), 1.f);
	EXPECT_FLOAT_EQ(gradual3(0.f,
				 0.f, .5f, 1.5f,
				 1.f, 2.f, 3.f), 1.f);
	EXPECT_FLOAT_EQ(gradual3(.25f,
				 0.f, .5f, 1.5f,
				 1.f, 2.f, 3.f), 1.5f);
	EXPECT_FLOAT_EQ(gradual3(.5f,
				 0.f, .5f, 1.5f,
				 1.f, 2.f, 3.f), 2.f);
	EXPECT_FLOAT_EQ(gradual3(.75f,
				 0.f, .5f, 1.5f,
				 1.f, 2.f, 3.f), 2.25f);
	EXPECT_FLOAT_EQ(gradual3(1.f,
				 0.f, .5f, 1.5f,
				 1.f, 2.f, 3.f), 2.5f);
	EXPECT_FLOAT_EQ(gradual3(1.25f,
				 0.f, .5f, 1.5f,
				 1.f, 2.f, 3.f), 2.75f);
	EXPECT_FLOAT_EQ(gradual3(1.5f,
				 0.f, .5f, 1.5f,
				 1.f, 2.f, 3.f), 3.f);
	EXPECT_FLOAT_EQ(gradual3(12.f,
				 0.f, .5f, 1.5f,
				 1.f, 2.f, 3.f), 3.f);
}

TEST(FunctionsTest, sqrt_linear)
{
	EXPECT_FLOAT_EQ(sqrt_linear(-12.f), 0.f);
	EXPECT_FLOAT_EQ(sqrt_linear(-2.f), 0.f);
	EXPECT_FLOAT_EQ(sqrt_linear(0.f), 0.f);
	EXPECT_FLOAT_EQ(sqrt_linear(.5f), 0.70710678f);
	EXPECT_FLOAT_EQ(sqrt_linear(1.f), 1.f);
	EXPECT_FLOAT_EQ(sqrt_linear(2.f), 2.f);
	EXPECT_FLOAT_EQ(sqrt_linear(120.f), 120.f);
}

TEST(FunctionsTest, lerp)
{
	EXPECT_FLOAT_EQ(lerp(0.f, 1.f, -.123f), -.123f);
	EXPECT_FLOAT_EQ(lerp(0.f, 1.f, 0.f), 0.f);
	EXPECT_FLOAT_EQ(lerp(0.f, 1.f, .123f), .123f);
	EXPECT_FLOAT_EQ(lerp(0.f, 1.f, 1.f), 1.f);
	EXPECT_FLOAT_EQ(lerp(0.f, 1.f, 1.123f), 1.123f);

	EXPECT_FLOAT_EQ(lerp(.2f, .3f, -.1f), .19f);
	EXPECT_FLOAT_EQ(lerp(-.4f, .3f, 1.1f), .37f);
}
