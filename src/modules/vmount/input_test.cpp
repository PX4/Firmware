/****************************************************************************
*
*   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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
 * @file input_test.cpp
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "input_test.h"

#include <math.h>

#include <px4_platform_common/posix.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>


namespace vmount
{

InputTest::InputTest(float roll_deg, float pitch_deg, float yaw_deg)
{
	_angles[0] = roll_deg;
	_angles[1] = pitch_deg;
	_angles[2] = yaw_deg;
}

bool InputTest::finished()
{
	return true; /* only a single-shot test (for now) */
}

int InputTest::update(unsigned int timeout_ms, ControlData **control_data, bool already_active)
{
	//we directly override the update() here, since we don't need the initialization from the base class

	_control_data.type = ControlData::Type::Angle;

	_control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
	_control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
	_control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

	matrix::Eulerf euler(
		math::radians(_angles[0]),
		math::radians(_angles[1]),
		math::radians(_angles[2]));
	matrix::Quatf q(euler);

	q.copyTo(_control_data.type_data.angle.q);

	_control_data.gimbal_shutter_retract = false;
	*control_data = &_control_data;
	return 0;
}

int InputTest::initialize()
{
	return 0;
}

void InputTest::print_status()
{
	PX4_INFO("Input: Test");
}

} /* namespace vmount */
