/****************************************************************************
*
*   Copyright (c) 2016-2017 PX4 Development Team. All rights reserved.
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
 * @file input.cpp
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "input.h"


namespace vmount
{

int InputBase::update(unsigned int timeout_ms, ControlData **control_data, bool already_active)
{
	if (!_initialized) {
		int ret = initialize();

		if (ret) {
			return ret;
		}

		//on startup, set the mount to a neutral position
		_control_data.type = ControlData::Type::Neutral;
		_control_data.gimbal_shutter_retract = true;
		*control_data = &_control_data;
		_initialized = true;
		return 0;
	}

	return update_impl(timeout_ms, control_data, already_active);
}

void InputBase::control_data_set_lon_lat(double lon, double lat, float altitude, float roll_angle,
		float pitch_fixed_angle)
{
	_control_data.type = ControlData::Type::LonLat;
	_control_data.type_data.lonlat.lon = lon;
	_control_data.type_data.lonlat.lat = lat;
	_control_data.type_data.lonlat.altitude = altitude;
	_control_data.type_data.lonlat.roll_angle = roll_angle;
	_control_data.type_data.lonlat.pitch_fixed_angle = pitch_fixed_angle;
	_control_data.type_data.lonlat.pitch_angle_offset = 0.f;
	_control_data.type_data.lonlat.yaw_angle_offset = 0.f;
}

void InputBase::set_stabilize(bool roll_stabilize, bool pitch_stabilize, bool yaw_stabilize)
{
	_control_data.stabilize_axis[0] = roll_stabilize;
	_control_data.stabilize_axis[1] = pitch_stabilize;
	_control_data.stabilize_axis[2] = yaw_stabilize;
}

} /* namespace vmount */
