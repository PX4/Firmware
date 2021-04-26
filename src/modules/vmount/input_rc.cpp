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
 * @file input_rc.cpp
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "input_rc.h"

#include <math.h>
#include <errno.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>


namespace vmount
{


InputRC::InputRC(int aux_channel_roll, int aux_channel_pitch, int aux_channel_yaw)
{
	_aux_channels[0] = aux_channel_roll;
	_aux_channels[1] = aux_channel_pitch;
	_aux_channels[2] = aux_channel_yaw;
}

InputRC::~InputRC()
{
	if (_manual_control_setpoint_sub >= 0) {
		orb_unsubscribe(_manual_control_setpoint_sub);
	}
}

int InputRC::initialize()
{
	_manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	if (_manual_control_setpoint_sub < 0) {
		return -errno;
	}

	return 0;
}

int InputRC::update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active)
{
	// Default to no change signalled by NULL.
	*control_data = nullptr;

	px4_pollfd_struct_t polls[1];
	polls[0].fd = 		_manual_control_setpoint_sub;
	polls[0].events = 	POLLIN;

	int ret = px4_poll(polls, 1, timeout_ms);

	if (ret < 0) {
		return -errno;
	}

	if (ret == 0) {
		// Timeout, leave NULL
	} else {
		if (polls[0].revents & POLLIN) {
			// Only if there was a change, we update the control data, otherwise leave it NULL.
			if (_read_control_data_from_subscription(_control_data, already_active)) {
				*control_data = &_control_data;
			}
		}
	}

	return 0;
}

bool InputRC::_read_control_data_from_subscription(ControlData &control_data, bool already_active)
{
	manual_control_setpoint_s manual_control_setpoint;
	orb_copy(ORB_ID(manual_control_setpoint), _manual_control_setpoint_sub, &manual_control_setpoint);
	control_data.type = ControlData::Type::Angle;

	float new_aux_values[3];

	for (int i = 0; i < 3; ++i) {
		new_aux_values[i] = _get_aux_value(manual_control_setpoint, i);
	}

	// If we were already active previously, we just update normally. Otherwise, there needs to be
	// a major stick movement to re-activate manual (or it's running for the very first time).
	bool major_movement = false;

	// Detect a big stick movement
	for (int i = 0; i < 3; ++i) {
		if (fabsf(_last_set_aux_values[i] - new_aux_values[i]) > 0.25f) {
			major_movement = true;
		}
	}

	if (already_active || major_movement || _first_time) {

		_first_time = false;

		matrix::Eulerf euler(new_aux_values[0] * M_PI_F, new_aux_values[1] * M_PI_F,
				     new_aux_values[2] * M_PI_F);
		matrix::Quatf q(euler);
		q.copyTo(control_data.type_data.angle.q);

		for (int i = 0; i < 3; ++i) {
			// We always use follow mode with RC input for now.
			control_data.type_data.angle.frames[i] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
			_last_set_aux_values[i] = new_aux_values[i];
		}

		control_data.gimbal_shutter_retract = false;
		return true;

	} else {
		return false;
	}
}

float InputRC::_get_aux_value(const manual_control_setpoint_s &manual_control_setpoint, int channel_idx)
{
	switch (_aux_channels[channel_idx]) {

	case 1:
		return manual_control_setpoint.aux1;

	case 2:
		return manual_control_setpoint.aux2;

	case 3:
		return manual_control_setpoint.aux3;

	case 4:
		return manual_control_setpoint.aux4;

	case 5:
		return manual_control_setpoint.aux5;

	case 6:
		return manual_control_setpoint.aux6;

	default:
		return 0.0f;
	}
}

void InputRC::print_status()
{
	PX4_INFO("Input: RC (channels: roll=%i, pitch=%i, yaw=%i)", _aux_channels[0], _aux_channels[1], _aux_channels[2]);
}

} /* namespace vmount */
