/****************************************************************************
 *
 *   Copyright (c) 2015-2021 PX4 Development Team. All rights reserved.
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
 * @file camera_interface.h
 */

#pragma once

#include <parameters/param.h>
#include <px4_platform_common/log.h>

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

class CameraInterface
{
public:
	CameraInterface() = default;
	virtual ~CameraInterface() = default;

	/**
	 * trigger the camera
	 * @param enable
	 */
	virtual void trigger(bool trigger_on_true) {}

	/**
	 * send command to turn the camera on/off
	 * @param enable
	 */
	virtual void send_toggle_power(bool enable) {}

	/**
	 * send command to prevent the camera from sleeping
	 * @param enable
	 */
	virtual void send_keep_alive(bool enable) {}

	/**
	 * Display info.
	 */
	virtual void info() {}

	/**
	 * Checks if the interface has support for
	 * camera power control
	 * @return true if power control is supported
	 */
	virtual bool has_power_control() { return false; }

	/**
	 * Checks if the camera connected to the interface
	 * is turned on.
	 * @return true if camera is on
	 */
	virtual bool is_powered_on() { return true; }

protected:

	/**
	 * setup the interface
	 */
	virtual void setup() {}

	/**
	 * get the hardware configuration
	 */
	void get_pins();

	param_t _p_pin{PARAM_INVALID};

	int _pins[6] {};

};
