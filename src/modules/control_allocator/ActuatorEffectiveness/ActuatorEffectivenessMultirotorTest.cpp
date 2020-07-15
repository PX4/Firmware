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
 * @file ActuatorEffectivenessMultirotorTest.cpp
 *
 * Tests for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include <gtest/gtest.h>
#include <ActuatorEffectivenessMultirotor.hpp>

using namespace matrix;

TEST(ActuatorEffectivenessMultirotorTest, AllZeroCase)
{
	// Quad wide geometry
	ActuatorEffectivenessMultirotor::MultirotorGeometry geometry = {};
	geometry.rotors[0].position_x = 1.0f;
	geometry.rotors[0].position_y = 1.0f;
	geometry.rotors[0].position_z = 0.0f;
	geometry.rotors[0].axis_x = 0.0f;
	geometry.rotors[0].axis_y = 0.0f;
	geometry.rotors[0].axis_z = -1.0f;
	geometry.rotors[0].thrust_coef = 1.0f;
	geometry.rotors[0].moment_ratio = 0.05f;

	geometry.rotors[1].position_x = -1.0f;
	geometry.rotors[1].position_y = -1.0f;
	geometry.rotors[1].position_z = 0.0f;
	geometry.rotors[1].axis_x = 0.0f;
	geometry.rotors[1].axis_y = 0.0f;
	geometry.rotors[1].axis_z = -1.0f;
	geometry.rotors[1].thrust_coef = 1.0f;
	geometry.rotors[1].moment_ratio = 0.05f;

	geometry.rotors[2].position_x = 1.0f;
	geometry.rotors[2].position_y = -1.0f;
	geometry.rotors[2].position_z = 0.0f;
	geometry.rotors[2].axis_x = 0.0f;
	geometry.rotors[2].axis_y = 0.0f;
	geometry.rotors[2].axis_z = -1.0f;
	geometry.rotors[2].thrust_coef = 1.0f;
	geometry.rotors[2].moment_ratio = -0.05f;

	geometry.rotors[3].position_x = -1.0f;
	geometry.rotors[3].position_y = 1.0f;
	geometry.rotors[3].position_z = 0.0f;
	geometry.rotors[3].axis_x = 0.0f;
	geometry.rotors[3].axis_y = 0.0f;
	geometry.rotors[3].axis_z = -1.0f;
	geometry.rotors[3].thrust_coef = 1.0f;
	geometry.rotors[3].moment_ratio = -0.05f;

	matrix::Matrix<float, ActuatorEffectiveness::NUM_AXES, ActuatorEffectiveness::NUM_ACTUATORS> effectiveness =
		ActuatorEffectivenessMultirotor::computeEffectivenessMatrix(geometry);

	const float expected[ActuatorEffectiveness::NUM_AXES][ActuatorEffectiveness::NUM_ACTUATORS] = {
		{-1.0f,   1.0f,   1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 1.0f,  -1.0f,   1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.05f,  0.05f, -0.05f, -0.05f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-1.0f,  -1.0f,  -1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	matrix::Matrix<float, ActuatorEffectiveness::NUM_AXES, ActuatorEffectiveness::NUM_ACTUATORS> effectiveness_expected(
		expected);

	EXPECT_EQ(effectiveness, effectiveness_expected);
}
