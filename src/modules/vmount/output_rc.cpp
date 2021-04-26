/****************************************************************************
*
*   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file output_rc.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "output_rc.h"

#include <uORB/topics/actuator_controls.h>
#include <px4_platform_common/defines.h>
#include <matrix/matrix/math.hpp>

namespace vmount
{

OutputRC::OutputRC(const OutputConfig &output_config)
	: OutputBase(output_config)
{
}

int OutputRC::update(const ControlData *control_data)
{
	if (control_data) {
		//got new command
		_retract_gimbal = control_data->gimbal_shutter_retract;
		_set_angle_setpoints(control_data);
	}

	_handle_position_update();

	hrt_abstime t = hrt_absolute_time();
	_calculate_angle_output(t);

	actuator_controls_s actuator_controls{};
	actuator_controls.timestamp = hrt_absolute_time();
	// _angle_outputs are in radians, actuator_controls are in [-1, 1]
	actuator_controls.control[0] = (_angle_outputs[0] + _config.roll_offset) * _config.roll_scale;
	actuator_controls.control[1] = (_angle_outputs[1] + _config.pitch_offset) * _config.pitch_scale;
	actuator_controls.control[2] = (_angle_outputs[2] + _config.yaw_offset) * _config.yaw_scale;
	actuator_controls.control[3] = _retract_gimbal ? _config.gimbal_retracted_mode_value : _config.gimbal_normal_mode_value;

	_stream_device_attitude_status();

	_actuator_controls_pub.publish(actuator_controls);

	_last_update = t;

	return 0;
}

void OutputRC::print_status()
{
	PX4_INFO("Output: AUX");
}

void OutputRC::_stream_device_attitude_status()
{
	gimbal_device_attitude_status_s attitude_status{};
	attitude_status.timestamp = hrt_absolute_time();
	attitude_status.target_system = 0;
	attitude_status.target_component = 0;
	attitude_status.device_flags = gimbal_device_attitude_status_s::DEVICE_FLAGS_NEUTRAL |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_ROLL_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_PITCH_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK;

	matrix::Eulerf euler(_angle_outputs[0], _angle_outputs[1], _angle_outputs[2]);
	matrix::Quatf q(euler);
	q.copyTo(attitude_status.q);

	attitude_status.failure_flags = 0;
	_attitude_status_pub.publish(attitude_status);
}

} /* namespace vmount */
