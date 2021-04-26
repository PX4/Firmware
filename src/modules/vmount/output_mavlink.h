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
 * @file output_mavlink.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "output.h"

#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/gimbal_device_set_attitude.h>
#include <uORB/topics/gimbal_device_information.h>


namespace vmount
{
/**
 ** class OutputMavlink
 *  Output via vehicle_command topic
 */
class OutputMavlinkV1 : public OutputBase
{
public:
	OutputMavlinkV1(const OutputConfig &output_config);
	virtual ~OutputMavlinkV1() = default;

	virtual int update(const ControlData *control_data);

	virtual void print_status();

private:
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
};

class OutputMavlinkV2 : public OutputBase
{
public:
	OutputMavlinkV2(int32_t mav_sys_id, int32_t mav_comp_id, const OutputConfig &output_config);
	virtual ~OutputMavlinkV2() = default;

	virtual int update(const ControlData *control_data);

	virtual void print_status();

private:
	void _publish_gimbal_device_set_attitude();
	void _request_gimbal_device_information();
	void _check_for_gimbal_device_information();

	uORB::Publication<gimbal_device_set_attitude_s> _gimbal_device_set_attitude_pub{ORB_ID(gimbal_device_set_attitude)};
	uORB::Subscription _gimbal_device_information_sub{ORB_ID(gimbal_device_information)};

	int32_t _mav_sys_id{1}; ///< our mavlink system id
	int32_t _mav_comp_id{1}; ///< our mavlink component id
	uint8_t _gimbal_device_compid{0};
	hrt_abstime _last_gimbal_device_checked{0};
	bool _gimbal_device_found {false};
};

} /* namespace vmount */
