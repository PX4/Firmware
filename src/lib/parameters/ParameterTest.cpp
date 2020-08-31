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

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/uORBManager.hpp>

#include <gtest/gtest.h>

class ParameterTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}
};


TEST_F(ParameterTest, testParamReadWrite)
{
	// GIVEN a parameter handle
	param_t param = param_handle(px4::params::CP_DIST);

	// WHEN: we get the parameter
	float value = -999.f;
	int status = param_get(param, &value);

	// THEN it should be successful and have the default value
	EXPECT_EQ(0, status);
	EXPECT_FLOAT_EQ(-1.f, value);

	// WHEN: we set the parameter
	value = 42.f;
	status = param_set(param, &value);

	// THEN: it should be successful
	EXPECT_EQ(0, status);

	// WHEN: we get the parameter again
	float value2 = -1999.f;
	status = param_get(param, &value2);

	// THEN: it should be exactly the value we set
	EXPECT_EQ(0, status);
	EXPECT_FLOAT_EQ(42.f, value2);
}


TEST_F(ParameterTest, testUorbSendReceive)
{
	// GIVEN: a uOrb message
	obstacle_distance_s message;
	memset(&message, 0xDEAD, sizeof(message));
	message.min_distance = 1.f;
	message.max_distance = 10.f;

	// AND: a subscriber
	uORB::SubscriptionData<obstacle_distance_s> sub_obstacle_distance{ORB_ID(obstacle_distance)};

	// WHEN we send the message
	orb_advert_t obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &message);
	ASSERT_TRUE(obstacle_distance_pub != nullptr);

	// THEN: the subscriber should receive the message
	sub_obstacle_distance.update();
	const obstacle_distance_s &obstacle_distance = sub_obstacle_distance.get();

	// AND: the values we got should be the same
	EXPECT_EQ(message.timestamp, obstacle_distance.timestamp);
	EXPECT_FLOAT_EQ(message.min_distance, obstacle_distance.min_distance);
	EXPECT_FLOAT_EQ(message.max_distance, obstacle_distance.max_distance);

	// AND: all the bytes should be equal
	EXPECT_EQ(0, memcmp(&message, &obstacle_distance, sizeof(message)));
}
