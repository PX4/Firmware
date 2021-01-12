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

/**
 * Test code for the Median filter
 * Run this test only using make tests TESTFILTER=MedianFilter
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

#include "MedianFilter.hpp"

using namespace math;
using matrix::Vector3f;

class MedianFilterTest : public ::testing::Test
{
public:


};

TEST_F(MedianFilterTest, test3f_simple)
{
	MedianFilter<float, 3> median_filter3;

	for (int i = 0; i < 3; i++) {
		median_filter3.insert(i);
	}

	EXPECT_EQ(median_filter3.median(), 1); // 0, 1, 2
}

TEST_F(MedianFilterTest, test3f_100)
{
	MedianFilter<float, 3> median_filter3;

	for (int i = 0; i < 100; i++) {
		EXPECT_EQ(median_filter3.apply(i), max(0, i - 1));
	}
}

TEST_F(MedianFilterTest, test5u_simple)
{
	MedianFilter<uint16_t, 5> median_filter5;

	for (int i = 0; i < 5; i++) {
		median_filter5.insert(i);
	}

	EXPECT_EQ(median_filter5.median(), 2); // 0, 1, 2, 4, 5
}

TEST_F(MedianFilterTest, test5i_100)
{
	MedianFilter<uint16_t, 5> median_filter5;

	for (int i = 0; i < 100; i++) {
		EXPECT_EQ(median_filter5.apply(i), max(0, i - 2));
	}
}
