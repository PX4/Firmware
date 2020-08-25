/****************************************************************************
*
*   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file output_mavlink.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "output_mavlink.h"

#include <math.h>

#include <uORB/topics/vehicle_command.h>
#include <px4_platform_common/defines.h>


namespace vmount
{

OutputMavlink::OutputMavlink(const OutputConfig &output_config)
	: OutputBase(output_config)
{
}

int OutputMavlink::update(const ControlData *control_data)
{
	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.target_system = (uint8_t)_config.mavlink_sys_id;
	vehicle_command.target_component = (uint8_t)_config.mavlink_comp_id;

	if (control_data) {
		//got new command
		_set_angle_setpoints(control_data);

		vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE;
		vehicle_command.timestamp = hrt_absolute_time();

		if (control_data->type == ControlData::Type::Neutral) {
			vehicle_command.param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL;

			vehicle_command.param5 = 0.0;
			vehicle_command.param6 = 0.0;
			vehicle_command.param7 = 0.0f;

		} else {
			vehicle_command.param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;

			vehicle_command.param5 = static_cast<double>(control_data->type_data.angle.frames[0]);
			vehicle_command.param6 = static_cast<double>(control_data->type_data.angle.frames[1]);
			vehicle_command.param7 = static_cast<float>(control_data->type_data.angle.frames[2]);
		}

		vehicle_command.param2 = control_data->stabilize_axis[0];
		vehicle_command.param3 = control_data->stabilize_axis[1];
		vehicle_command.param4 = control_data->stabilize_axis[2];

		_vehicle_command_pub.publish(vehicle_command);
	}

	_handle_position_update();

	hrt_abstime t = hrt_absolute_time();
	_calculate_output_angles(t);

	vehicle_command.timestamp = t;
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;

	// vmount spec has roll, pitch on channels 0, 1, respectively; MAVLink spec has roll, pitch on channels 1, 0, respectively
	// vmount uses radians, MAVLink uses degrees
	vehicle_command.param1 = (_angle_outputs[1] + _config.pitch_offset) * M_RAD_TO_DEG_F;
	vehicle_command.param2 = (_angle_outputs[0] + _config.roll_offset) * M_RAD_TO_DEG_F;
	vehicle_command.param3 = (_angle_outputs[2] + _config.yaw_offset) * M_RAD_TO_DEG_F;
	vehicle_command.param7 = 2.0f; // MAV_MOUNT_MODE_MAVLINK_TARGETING;

	_vehicle_command_pub.publish(vehicle_command);

	_last_update = t;

	return 0;
}

void OutputMavlink::print_status()
{
	PX4_INFO("Output: Mavlink");
}

} /* namespace vmount */

