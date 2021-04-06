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

#include <gtest/gtest.h>
#include <PositionControl.hpp>
#include <px4_defines.h>

using namespace matrix;

TEST(PositionControlTest, EmptySetpoint)
{
	PositionControl position_control;

	vehicle_local_position_setpoint_s output_setpoint{};
	position_control.getLocalPositionSetpoint(output_setpoint);
	EXPECT_FLOAT_EQ(output_setpoint.x, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.y, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.z, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.yaw, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.yawspeed, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.vx, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.vy, 0.f);
	EXPECT_FLOAT_EQ(output_setpoint.vz, 0.f);
	EXPECT_EQ(Vector3f(output_setpoint.acceleration), Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(Vector3f(output_setpoint.thrust), Vector3f(0, 0, 0));

	vehicle_attitude_setpoint_s attitude{};
	position_control.getAttitudeSetpoint(attitude);
	EXPECT_FLOAT_EQ(attitude.roll_body, 0.f);
	EXPECT_FLOAT_EQ(attitude.pitch_body, 0.f);
	EXPECT_FLOAT_EQ(attitude.yaw_body, 0.f);
	EXPECT_FLOAT_EQ(attitude.yaw_sp_move_rate, 0.f);
	EXPECT_EQ(Quatf(attitude.q_d), Quatf(1.f, 0.f, 0.f, 0.f));
	EXPECT_EQ(Vector3f(attitude.thrust_body), Vector3f(0.f, 0.f, 0.f));
	EXPECT_EQ(attitude.roll_reset_integral, false);
	EXPECT_EQ(attitude.pitch_reset_integral, false);
	EXPECT_EQ(attitude.yaw_reset_integral, false);
	EXPECT_EQ(attitude.fw_control_yaw, false);
	EXPECT_FLOAT_EQ(attitude.apply_flaps, 0.f);//vehicle_attitude_setpoint_s::FLAPS_OFF); // TODO why no reference?
}

class PositionControlBasicTest : public ::testing::Test
{
public:
	PositionControlBasicTest()
	{
		_position_control.setPositionGains(Vector3f(1.f, 1.f, 1.f));
		_position_control.setVelocityGains(Vector3f(20.f, 20.f, 20.f), Vector3f(20.f, 20.f, 20.f), Vector3f(20.f, 20.f, 20.f));
		_position_control.setVelocityLimits(1.f, 1.f, 1.f);
		_position_control.setThrustLimits(0.1f, 0.9f);
		_position_control.setTiltLimit(1.f);
		_position_control.setHoverThrust(.5f);

		resetInputSetpoint();
	}

	void resetInputSetpoint()
	{
		_input_setpoint.position[0] = NAN;
		_input_setpoint.position[1] = NAN;
		_input_setpoint.position[2] = NAN;
		_input_setpoint.velocity[0] = NAN;
		_input_setpoint.velocity[1] = NAN;
		_input_setpoint.velocity[2] = NAN;
		Vector3f(NAN, NAN, NAN).copyTo(_input_setpoint.acceleration);
		_input_setpoint.yaw = NAN;
		_input_setpoint.yawspeed = NAN;
	}

	bool runController()
	{
		_position_control.setInputSetpoint(_input_setpoint);
		const bool ret = _position_control.update(.1f);
		_position_control.getLocalPositionSetpoint(_output_setpoint);
		_position_control.getAttitudeSetpoint(_attitude);
		return ret;
	}

	PositionControl _position_control;
	trajectory_setpoint_s _input_setpoint{};
	vehicle_local_position_setpoint_s _output_setpoint{};
	vehicle_attitude_setpoint_s _attitude{};
};

class PositionControlBasicDirectionTest : public PositionControlBasicTest
{
public:
	void checkDirection()
	{
		Vector3f thrust(_output_setpoint.thrust);
		EXPECT_GT(thrust(0), 0.f);
		EXPECT_GT(thrust(1), 0.f);
		EXPECT_LT(thrust(2), 0.f);

		Vector3f body_z = Quatf(_attitude.q_d).dcm_z();
		EXPECT_LT(body_z(0), 0.f);
		EXPECT_LT(body_z(1), 0.f);
		EXPECT_GT(body_z(2), 0.f);
	}
};

TEST_F(PositionControlBasicDirectionTest, PositionDirection)
{
	_input_setpoint.position[0] = .1f;
	_input_setpoint.position[1] = .1f;
	_input_setpoint.position[2] = -.1f;
	EXPECT_TRUE(runController());
	checkDirection();
}

TEST_F(PositionControlBasicDirectionTest, VelocityDirection)
{
	_input_setpoint.velocity[0] = .1f;
	_input_setpoint.velocity[1] = .1f;
	_input_setpoint.velocity[2] = -.1f;
	EXPECT_TRUE(runController());
	checkDirection();
}

TEST_F(PositionControlBasicTest, TiltLimit)
{
	_input_setpoint.position[0] = 10.f;
	_input_setpoint.position[1] = 10.f;
	_input_setpoint.position[2] = -0.f;

	EXPECT_TRUE(runController());
	Vector3f body_z = Quatf(_attitude.q_d).dcm_z();
	float angle = acosf(body_z.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_GT(angle, 0.f);
	EXPECT_LE(angle, 1.f);

	_position_control.setTiltLimit(0.5f);
	EXPECT_TRUE(runController());
	body_z = Quatf(_attitude.q_d).dcm_z();
	angle = acosf(body_z.dot(Vector3f(0.f, 0.f, 1.f)));
	EXPECT_GT(angle, 0.f);
	EXPECT_LE(angle, .50001f);

	_position_control.setTiltLimit(1.f);  // restore original
}

TEST_F(PositionControlBasicTest, VelocityLimit)
{
	_input_setpoint.position[0] = 10.f;
	_input_setpoint.position[1] = 10.f;
	_input_setpoint.position[2] = -10.f;

	EXPECT_TRUE(runController());
	Vector2f velocity_xy(_output_setpoint.vx, _output_setpoint.vy);
	EXPECT_LE(velocity_xy.norm(), 1.f);
	EXPECT_LE(abs(_output_setpoint.vz), 1.f);
}

TEST_F(PositionControlBasicTest, PositionControlMaxThrustLimit)
{
	_input_setpoint.position[0] = 10.f;
	_input_setpoint.position[1] = 10.f;
	_input_setpoint.position[2] = -10.f;

	runController();
	Vector3f thrust(_output_setpoint.thrust);
	EXPECT_FLOAT_EQ(thrust(0), 0.f);
	EXPECT_FLOAT_EQ(thrust(1), 0.f);
	EXPECT_FLOAT_EQ(thrust(2), -0.9f);

	EXPECT_EQ(_attitude.thrust_body[0], 0.f);
	EXPECT_EQ(_attitude.thrust_body[1], 0.f);
	EXPECT_FLOAT_EQ(_attitude.thrust_body[2], -0.9f);

	EXPECT_FLOAT_EQ(_attitude.roll_body, 0.f);
	EXPECT_FLOAT_EQ(_attitude.pitch_body, 0.f);
}

TEST_F(PositionControlBasicTest, PositionControlMinThrustLimit)
{
	_input_setpoint.position[0] = 10.f;
	_input_setpoint.position[1] = 0.f;
	_input_setpoint.position[2] = 10.f;

	runController();
	Vector3f thrust(_output_setpoint.thrust);
	EXPECT_FLOAT_EQ(thrust.length(), 0.1f);

	EXPECT_FLOAT_EQ(_attitude.thrust_body[2], -0.1f);

	EXPECT_FLOAT_EQ(_attitude.roll_body, 0.f);
	EXPECT_FLOAT_EQ(_attitude.pitch_body, -1.f);
}

TEST_F(PositionControlBasicTest, FailsafeInput)
{
	_input_setpoint.velocity[2] = .1f;
	_input_setpoint.acceleration[0] = _input_setpoint.acceleration[1] = 0.f;

	EXPECT_TRUE(runController());
	EXPECT_FLOAT_EQ(_attitude.thrust_body[0], 0.f);
	EXPECT_FLOAT_EQ(_attitude.thrust_body[1], 0.f);
	EXPECT_LT(_output_setpoint.thrust[2], -.1f);
	EXPECT_GT(_output_setpoint.thrust[2], -.5f);
	EXPECT_GT(_attitude.thrust_body[2], -.5f);
	EXPECT_LE(_attitude.thrust_body[2], -.1f);
}

TEST_F(PositionControlBasicTest, IdleThrustInput)
{
	// High downwards acceleration to make sure there's no thrust
	Vector3f(0.f, 0.f, 100.f).copyTo(_input_setpoint.acceleration);

	EXPECT_TRUE(runController());
	EXPECT_FLOAT_EQ(_output_setpoint.thrust[0], 0.f);
	EXPECT_FLOAT_EQ(_output_setpoint.thrust[1], 0.f);
	EXPECT_FLOAT_EQ(_output_setpoint.thrust[2], -.1f);
}

TEST_F(PositionControlBasicTest, InputCombinationsPosition)
{
	_input_setpoint.position[0] = .1f;
	_input_setpoint.position[1] = .2f;
	_input_setpoint.position[2] = .3f;

	EXPECT_TRUE(runController());
	EXPECT_FLOAT_EQ(_output_setpoint.x, .1f);
	EXPECT_FLOAT_EQ(_output_setpoint.y, .2f);
	EXPECT_FLOAT_EQ(_output_setpoint.z, .3f);
	EXPECT_FALSE(isnan(_output_setpoint.vx));
	EXPECT_FALSE(isnan(_output_setpoint.vy));
	EXPECT_FALSE(isnan(_output_setpoint.vz));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[0]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[1]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[2]));
}

TEST_F(PositionControlBasicTest, InputCombinationsPositionVelocity)
{
	_input_setpoint.velocity[0] = .1f;
	_input_setpoint.velocity[1] = .2f;
	_input_setpoint.position[2] = .3f; // altitude

	EXPECT_TRUE(runController());
	// EXPECT_TRUE(isnan(_output_setpoint.x));
	// EXPECT_TRUE(isnan(_output_setpoint.y));
	EXPECT_FLOAT_EQ(_output_setpoint.z, .3f);
	EXPECT_FLOAT_EQ(_output_setpoint.vx, .1f);
	EXPECT_FLOAT_EQ(_output_setpoint.vy, .2f);
	EXPECT_FALSE(isnan(_output_setpoint.vz));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[0]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[1]));
	EXPECT_FALSE(isnan(_output_setpoint.thrust[2]));
}

TEST_F(PositionControlBasicTest, SetpointValiditySimple)
{
	EXPECT_FALSE(runController());
	_input_setpoint.position[0] = .1f;
	EXPECT_FALSE(runController());
	_input_setpoint.position[1] = .2f;
	EXPECT_FALSE(runController());
	_input_setpoint.acceleration[2] = .3f;
	EXPECT_TRUE(runController());
}

TEST_F(PositionControlBasicTest, SetpointValidityAllCombinations)
{
	// This test runs any combination of set and unset (NAN) setpoints and checks if it gets accepted or rejected correctly
	float *const setpoint_loop_access_map[] = {&_input_setpoint.position[0], &_input_setpoint.velocity[0], &_input_setpoint.acceleration[0],
						   &_input_setpoint.position[1], &_input_setpoint.velocity[1], &_input_setpoint.acceleration[1],
						   &_input_setpoint.position[2], &_input_setpoint.velocity[2], &_input_setpoint.acceleration[2]
						  };

	for (int combination = 0; combination < 512; combination++) {
		resetInputSetpoint();

		for (int j = 0; j < 9; j++) {
			if (combination & (1 << j)) {
				// Set arbitrary finite value, some values clearly hit the limits to check these corner case combinations
				*(setpoint_loop_access_map[j]) = static_cast<float>(combination) / static_cast<float>(j + 1);
			}
		}

		// Expect at least one setpoint per axis
		const bool has_x_setpoint = ((combination & 7) != 0);
		const bool has_y_setpoint = (((combination >> 3) & 7) != 0);
		const bool has_z_setpoint = (((combination >> 6) & 7) != 0);
		// Expect xy setpoints to come in pairs
		const bool has_xy_pairs = (combination & 7) == ((combination >> 3) & 7);
		const bool expected_result = has_x_setpoint && has_y_setpoint && has_z_setpoint && has_xy_pairs;

		EXPECT_EQ(runController(), expected_result) << "combination " << combination << std::endl
				<< "input" << std::endl
				<< "position     " << _input_setpoint.position[0] << ", " << _input_setpoint.position[1] << ", " <<
				_input_setpoint.position[2] << std::endl
				<< "velocity     " << _input_setpoint.velocity[0] << ", " << _input_setpoint.velocity[1] << ", " <<
				_input_setpoint.velocity[2] << std::endl
				<< "acceleration " << _input_setpoint.acceleration[0] << ", "
				<< _input_setpoint.acceleration[1] << ", " << _input_setpoint.acceleration[2] << std::endl
				<< "output" << std::endl
				<< "position     " << _output_setpoint.x << ", " << _output_setpoint.y << ", " << _output_setpoint.z << std::endl
				<< "velocity     " << _output_setpoint.vx << ", " << _output_setpoint.vy << ", " << _output_setpoint.vz << std::endl
				<< "acceleration " << _output_setpoint.acceleration[0] << ", "
				<< _output_setpoint.acceleration[1] << ", " << _output_setpoint.acceleration[2] << std::endl;
	}
}

TEST_F(PositionControlBasicTest, InvalidState)
{
	_input_setpoint.position[0] = .1f;
	_input_setpoint.position[1] = .2f;
	_input_setpoint.position[2] = .3f;

	PositionControlStates states{};
	states.position(0) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	states.velocity(0) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	states.position(0) = 0.f;
	_position_control.setState(states);
	EXPECT_FALSE(runController());

	states.velocity(0) = 0.f;
	states.acceleration(1) = NAN;
	_position_control.setState(states);
	EXPECT_FALSE(runController());
}


TEST_F(PositionControlBasicTest, UpdateHoverThrust)
{
	// GIVEN: some hover thrust and 0 velocity change
	const float hover_thrust = 0.6f;
	_position_control.setHoverThrust(hover_thrust);

	_input_setpoint.velocity[0] = 0.f;
	_input_setpoint.velocity[1] = 0.f;
	_input_setpoint.velocity[2] = -0.f;

	// WHEN: we run the controller
	EXPECT_TRUE(runController());

	// THEN: the output thrust equals the hover thrust
	EXPECT_EQ(_output_setpoint.thrust[2], -hover_thrust);

	// HOWEVER WHEN: we set a new hover thrust through the update function
	const float hover_thrust_new = 0.7f;
	_position_control.updateHoverThrust(hover_thrust_new);
	EXPECT_TRUE(runController());

	// THEN: the integral is updated to avoid discontinuities and
	// the output is still the same
	EXPECT_EQ(_output_setpoint.thrust[2], -hover_thrust);
}
