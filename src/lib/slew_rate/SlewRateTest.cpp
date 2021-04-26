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
#include "SlewRate.hpp"

TEST(SlewRateTest, SlewUpLimited)
{
	SlewRate<float> _slew_rate;
	_slew_rate.setSlewRate(.1f);
	_slew_rate.setForcedValue(-5.5f);

	for (int i = 1; i <= 10; i++) {
		EXPECT_FLOAT_EQ(_slew_rate.update(20.f, .2f), -5.5f + i * .02f);
	}
}

TEST(SlewRateTest, SlewDownLimited)
{
	SlewRate<float> _slew_rate;
	_slew_rate.setSlewRate(1.1f);
	_slew_rate.setForcedValue(17.3f);

	for (int i = 1; i <= 10; i++) {
		EXPECT_FLOAT_EQ(_slew_rate.update(-50.f, .3f), 17.3f - i * .33f);
	}
}

TEST(SlewRateTest, ReachValueSlewed)
{
	SlewRate<float> _slew_rate;
	_slew_rate.setSlewRate(.2f);
	_slew_rate.setForcedValue(8.f);

	for (int i = 1; i <= 10; i++) {
		EXPECT_FLOAT_EQ(_slew_rate.update(10.f, 1.f), 8.f + i * .2f);
	}

	for (int i = 1; i <= 10; i++) {
		EXPECT_FLOAT_EQ(_slew_rate.update(10.f, 1.f), 10.f);
	}
}
