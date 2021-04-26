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

#ifdef __PX4_NUTTX

#include <sys/ioctl.h>
#include <lib/mathlib/mathlib.h>

#include "drivers/drv_pwm_trigger.h"
#include "seagull_map2.h"

// PWM levels of the interface to Seagull MAP 2 converter to
// Multiport (http://www.seagulluav.com/manuals/Seagull_MAP2-Manual.pdf)
#define PWM_CAMERA_DISARMED			900
#define PWM_CAMERA_NEUTRAL			1500
#define PWM_1_CAMERA_ON				1100
#define PWM_1_CAMERA_AUTOFOCUS_SHOOT	1300
#define PWM_1_CAMERA_INSTANT_SHOOT	1700
#define PWM_1_CAMERA_OFF				1900
#define PWM_2_CAMERA_KEEP_ALIVE		1700
#define PWM_2_CAMERA_ON_OFF			1900

CameraInterfaceSeagull::CameraInterfaceSeagull()
{
	get_pins();
	setup();
}

CameraInterfaceSeagull::~CameraInterfaceSeagull()
{
	// Deinitialise trigger channels
	up_pwm_trigger_deinit();
}

void CameraInterfaceSeagull::setup()
{
	for (unsigned i = 0; i < arraySize(_pins); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {

			// Initialize the interface
			uint32_t pin_bitmask = (1 << _pins[i + 1]) | (1 << _pins[i]);
			up_pwm_trigger_init(pin_bitmask);

			// Set both interface pins to disarmed
			int ret1 = up_pwm_trigger_set(_pins[i + 1], PWM_CAMERA_DISARMED);
			PX4_DEBUG("pwm trigger set %d %d=%d, ret=%d", i + 1, _pins[i + 1], PWM_CAMERA_DISARMED, ret1);

			int ret2 = up_pwm_trigger_set(_pins[i], PWM_CAMERA_DISARMED);
			PX4_DEBUG("pwm trigger set %d %d=%d, ret=%d", i, _pins[i], PWM_CAMERA_DISARMED, ret2);

			// We only support 2 consecutive pins while using the Seagull MAP2
			return;
		}
	}

	PX4_ERR("Bad pin configuration - Seagull MAP2 requires 2 consecutive pins for control.");
}

void CameraInterfaceSeagull::trigger(bool trigger_on_true)
{
	if (!_camera_is_on) {
		return;
	}

	for (unsigned i = 0; i < arraySize(_pins); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			// Set channel 1 to shoot or neutral levels
			up_pwm_trigger_set(_pins[i + 1], trigger_on_true ? PWM_1_CAMERA_INSTANT_SHOOT : PWM_CAMERA_NEUTRAL);
		}
	}
}

void CameraInterfaceSeagull::send_keep_alive(bool enable)
{
	// This should alternate between enable and !enable to keep the camera alive
	if (!_camera_is_on) {
		return;
	}

	for (unsigned i = 0; i < arraySize(_pins); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			// Set channel 2 pin to keep_alive or netural signal
			up_pwm_trigger_set(_pins[i], enable ? PWM_2_CAMERA_KEEP_ALIVE : PWM_CAMERA_NEUTRAL);
		}
	}
}

void CameraInterfaceSeagull::send_toggle_power(bool enable)
{
	// This should alternate between enable and !enable to toggle camera power
	for (unsigned i = 0; i < arraySize(_pins); i = i + 2) {
		if (_pins[i] >= 0 && _pins[i + 1] >= 0) {
			// Set channel 1 to neutral
			up_pwm_trigger_set(_pins[i + 1], PWM_CAMERA_NEUTRAL);
			// Set channel 2 to on_off or neutral signal
			up_pwm_trigger_set(_pins[i], enable ? PWM_2_CAMERA_ON_OFF : PWM_CAMERA_NEUTRAL);
		}
	}

	if (!enable) {
		_camera_is_on = !_camera_is_on;
	}
}

void CameraInterfaceSeagull::info()
{
	PX4_INFO("PWM trigger mode (Seagull MAP2) , pins enabled : [%d][%d][%d][%d][%d][%d]",
		 _pins[5], _pins[4], _pins[3], _pins[2], _pins[1], _pins[0]);
}

#endif /* ifdef __PX4_NUTTX */
