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
 * @file input_mavlink.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "input.h"
#include "input_rc.h"
#include <cstdint>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <uORB/topics/gimbal_manager_information.h>
#include <uORB/topics/gimbal_manager_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/vehicle_global_position.h>
#include <lib/ecl/geo/geo.h>


namespace vmount
{
/**
 ** class InputMavlinkROI
 ** Input based on the vehicle_roi topic
 */
class InputMavlinkROI : public InputBase
{
public:
	InputMavlinkROI() = default;
	virtual ~InputMavlinkROI();

	virtual void print_status();

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active);
	virtual int initialize();

private:
	void _read_control_data_from_position_setpoint_sub();

	int _vehicle_roi_sub = -1;
	int _position_setpoint_triplet_sub = -1;
	uint8_t _cur_roi_mode {vehicle_roi_s::ROI_NONE};
};


/**
 ** class InputMavlinkCmdMount
 ** Input based on the VEHICLE_CMD_DO_MOUNT_CONTROL mavlink command
 */
class InputMavlinkCmdMount : public InputBase
{
public:
	InputMavlinkCmdMount();
	virtual ~InputMavlinkCmdMount();

	virtual void print_status();

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active);
	virtual int initialize();

private:
	void _ack_vehicle_command(vehicle_command_s *cmd);

	int _vehicle_command_sub = -1;

	int32_t _mav_sys_id{1}; ///< our mavlink system id
	int32_t _mav_comp_id{1}; ///< our mavlink component id
};

class InputMavlinkGimbalV2 : public InputBase
{
public:
	InputMavlinkGimbalV2(uint8_t mav_sys_id, uint8_t mav_comp_id, float &mnt_rate_pitch, float &mnt_rate_yaw);
	virtual ~InputMavlinkGimbalV2();

	virtual void print_status();

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active);
	virtual int initialize();

private:
	void _set_control_data_from_set_attitude(const uint32_t flags, const matrix::Quatf &q,
			const matrix::Vector3f &angular_velocity);
	void _ack_vehicle_command(vehicle_command_s *cmd, uint8_t result);
	void _stream_gimbal_manager_information();
	void _stream_gimbal_manager_status();
	void _read_control_data_from_position_setpoint_sub();

	int _vehicle_roi_sub = -1;
	int _gimbal_manager_set_attitude_sub = -1;
	int _gimbal_manager_set_manual_control_sub = -1;
	int _position_setpoint_triplet_sub = -1;
	int _vehicle_command_sub = -1;

	uint8_t _mav_sys_id{1}; ///< our mavlink system id
	uint8_t _mav_comp_id{1}; ///< our mavlink component id

	uint8_t _sys_id_primary_control{0};
	uint8_t _comp_id_primary_control{0};

	float &_mnt_rate_pitch;
	float &_mnt_rate_yaw;

	uORB::Subscription _gimbal_device_attitude_status_sub{ORB_ID(gimbal_device_attitude_status)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Publication<gimbal_manager_information_s> _gimbal_manager_info_pub{ORB_ID(gimbal_manager_information)};
	uORB::Publication<gimbal_manager_status_s> _gimbal_manager_status_pub{ORB_ID(gimbal_manager_status)};
	uint8_t _cur_roi_mode = vehicle_roi_s::ROI_NONE;
};

} /* namespace vmount */
