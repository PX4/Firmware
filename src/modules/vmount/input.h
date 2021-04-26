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
 * @file input.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "common.h"

namespace vmount
{


/**
 ** class InputBase
 * Base class for all driver input classes
 */
class InputBase
{
public:
	InputBase() = default;
	virtual ~InputBase() = default;

	/**
	 * Wait for an input update, with a timeout.
	 * @param timeout_ms timeout in ms
	 * @param control_data unchanged on error. On success it is nullptr if no new
	 *                     data is available, otherwise set to an object.
	 *                     If it is set, the returned object will not be changed for
	 *                     subsequent calls to update() that return no new data
	 *                     (in other words: if (some) control_data values change,
	 *                     non-null will be returned).
	 * @param already_active true if the mode was already active last time, false if it was not and "major"
	 *                       change is necessary such as big stick movement for RC.
	 * @return 0 on success, <0 otherwise
	 */
	virtual int update(unsigned int timeout_ms, ControlData **control_data, bool already_active);

	/** report status to stdout */
	virtual void print_status() = 0;

	void set_stabilize(bool roll_stabilize, bool pitch_stabilize, bool yaw_stabilize);

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active) = 0;

	virtual int initialize() { return 0; }

	void control_data_set_lon_lat(double lon, double lat, float altitude, float roll_angle = 0.f,
				      float pitch_fixed_angle = -10.f);

	ControlData _control_data;

private:
	bool _initialized = false;
};


} /* namespace vmount */
