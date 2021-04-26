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
 * @file input_rc.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "input.h"
#include <uORB/topics/manual_control_setpoint.h>

namespace vmount
{

class InputMavlinkROI;
class InputMavlinkCmdMount;

/**
 ** class InputRC
 * RC input class using manual_control_setpoint topic
 */
class InputRC : public InputBase
{
public:

	/**
	 * @param aux_channel_roll   which aux channel to use for roll (set to 0 to use a fixed angle of 0)
	 * @param aux_channel_pitch
	 * @param aux_channel_yaw
	 */
	InputRC(int aux_channel_roll, int aux_channel_pitch, int aux_channel_yaw);
	virtual ~InputRC();

	virtual void print_status();

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active);
	virtual int initialize();

	/**
	 * @return true if there was a change in control data
	 */
	virtual bool _read_control_data_from_subscription(ControlData &control_data, bool already_active);

	int _get_subscription_fd() const { return _manual_control_setpoint_sub; }

	float _get_aux_value(const manual_control_setpoint_s &manual_control_setpoint, int channel_idx);

private:
	int _aux_channels[3];
	int _manual_control_setpoint_sub = -1;

	bool _first_time = true;
	float _last_set_aux_values[3] = {};
};


} /* namespace vmount */
