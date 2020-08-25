/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file status_display.cpp
 * Status Display: this decouples the LED and tune logic from the control logic in commander
 *
 * @author Simone Guscetti <simone@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @editor Christoph Tobler <christoph@px4.io>
 */

#include "status_display.h"
#include <drivers/drv_led.h>

namespace events
{
namespace status
{

StatusDisplay::StatusDisplay()
{
	// set the base color
	_led_control.priority = 0;
	_led_control.led_mask = 0xff;
	_led_control.color = led_control_s::COLOR_CYAN;
	_led_control.mode = led_control_s::MODE_ON;
	publish();

	_led_control.priority = 1;
	_led_control.num_blinks = 0;	// infinite blinking
}

bool StatusDisplay::check_for_updates()
{
	bool got_updates = false;

	if (_battery_status_sub.update()) {
		got_updates = true;
	}

	if (_cpu_load_sub.update()) {
		got_updates = true;
	}

	if (_vehicle_status_flags_sub.update()) {
		got_updates = true;
	}

	if (_vehicle_status_sub.update()) {
		got_updates = true;
	}

	return got_updates;
}

void StatusDisplay::process()
{
	if (!check_for_updates()) {
		return;
	}

	set_leds();
}

void StatusDisplay::publish()
{
	_led_control.timestamp = hrt_absolute_time();

	_led_control_pub.publish(_led_control);
}

} /* namespace status */
} /* namespace events */
