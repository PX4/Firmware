/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ManualControl.hpp"
#include <drivers/drv_hrt.h>

using namespace time_literals;

enum OverrideBits {
	OVERRIDE_AUTO_MODE_BIT = (1 << 0),
	OVERRIDE_OFFBOARD_MODE_BIT = (1 << 1),
	OVERRIDE_IGNORE_THROTTLE_BIT = (1 << 2)
};

bool ManualControl::update()
{
	bool updated = false;

	if (_manual_control_setpoint_sub.updated()) {
		_last_manual_control_setpoint = _manual_control_setpoint;
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

		updated = true;
	}

	_rc_available = _rc_allowed
			&& _manual_control_setpoint.timestamp != 0
			&& (hrt_elapsed_time(&_manual_control_setpoint.timestamp) < (_param_com_rc_loss_t.get() * 1_s));

	return updated && _rc_available;
}

bool ManualControl::wantsOverride(const vehicle_control_mode_s &vehicle_control_mode)
{
	const bool override_auto_mode = (_param_rc_override.get() & OverrideBits::OVERRIDE_AUTO_MODE_BIT)
					&& vehicle_control_mode.flag_control_auto_enabled;

	const bool override_offboard_mode = (_param_rc_override.get() & OverrideBits::OVERRIDE_OFFBOARD_MODE_BIT)
					    && vehicle_control_mode.flag_control_offboard_enabled;

	if (_rc_available && (override_auto_mode || override_offboard_mode)) {
		const float minimum_stick_change = .01f * _param_com_rc_stick_ov.get();

		const bool rpy_moved = (fabsf(_manual_control_setpoint.x - _last_manual_control_setpoint.x) > minimum_stick_change)
				       || (fabsf(_manual_control_setpoint.y - _last_manual_control_setpoint.y) > minimum_stick_change)
				       || (fabsf(_manual_control_setpoint.r - _last_manual_control_setpoint.r) > minimum_stick_change);
		// Throttle change value doubled to achieve the same scaling even though the range is [0,1] instead of [-1,1]
		const bool throttle_moved =
			(fabsf(_manual_control_setpoint.z - _last_manual_control_setpoint.z) * 2.f > minimum_stick_change);
		const bool use_throttle = !(_param_rc_override.get() & OverrideBits::OVERRIDE_IGNORE_THROTTLE_BIT);

		if (rpy_moved || (use_throttle && throttle_moved)) {
			return true;
		}
	}

	return false;
}

bool ManualControl::wantsDisarm(const vehicle_control_mode_s &vehicle_control_mode,
				const vehicle_status_s &vehicle_status,
				manual_control_switches_s &manual_control_switches, const bool landed)
{
	bool ret = false;

	const bool use_stick = manual_control_switches.arm_switch == manual_control_switches_s::SWITCH_POS_NONE;
	const bool use_button = !use_stick && _param_com_arm_swisbtn.get();
	const bool use_switch = !use_stick && !use_button;

	const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool stick_in_lower_left = use_stick
					 && isThrottleLow()
					 && _manual_control_setpoint.r < -.9f;
	const bool arm_button_pressed = (manual_control_switches.arm_switch == manual_control_switches_s::SWITCH_POS_ON)
					&& use_button;
	const bool arm_switch_to_disarm_transition = use_switch
			&& (_last_manual_control_switches_arm_switch == manual_control_switches_s::SWITCH_POS_ON)
			&& (manual_control_switches.arm_switch == manual_control_switches_s::SWITCH_POS_OFF);
	const bool mc_manual_thrust_mode = vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					   && vehicle_control_mode.flag_control_manual_enabled
					   && !vehicle_control_mode.flag_control_climb_rate_enabled;

	if (armed
	    && (landed || mc_manual_thrust_mode)
	    && (stick_in_lower_left || arm_button_pressed || arm_switch_to_disarm_transition)) {

		const bool last_disarm_hysteresis = _stick_disarm_hysteresis.get_state();
		_stick_disarm_hysteresis.set_state_and_update(true, hrt_absolute_time());
		const bool disarm_trigger = !last_disarm_hysteresis && _stick_disarm_hysteresis.get_state()
					    && !_stick_arm_hysteresis.get_state();

		if (disarm_trigger || arm_switch_to_disarm_transition) {
			ret = true;
		}

	} else if (!arm_button_pressed) {

		_stick_disarm_hysteresis.set_state_and_update(false, hrt_absolute_time());
	}

	return ret;
}

bool ManualControl::wantsArm(const vehicle_control_mode_s &vehicle_control_mode, const vehicle_status_s &vehicle_status,
			     const manual_control_switches_s &manual_control_switches, const bool landed)
{
	bool ret = false;

	const bool use_stick = manual_control_switches.arm_switch == manual_control_switches_s::SWITCH_POS_NONE;
	const bool use_button = !use_stick && _param_com_arm_swisbtn.get();
	const bool use_switch = !use_stick && !use_button;

	const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool stick_in_lower_right = use_stick
					  && isThrottleLow()
					  && _manual_control_setpoint.r > .9f;
	const bool arm_button_pressed = use_button
					&& (manual_control_switches.arm_switch == manual_control_switches_s::SWITCH_POS_ON);
	const bool arm_switch_to_arm_transition = use_switch
			&& (_last_manual_control_switches_arm_switch == manual_control_switches_s::SWITCH_POS_OFF)
			&& (manual_control_switches.arm_switch == manual_control_switches_s::SWITCH_POS_ON);

	if (!armed
	    && (stick_in_lower_right || arm_button_pressed || arm_switch_to_arm_transition)) {

		const bool last_arm_hysteresis = _stick_arm_hysteresis.get_state();
		_stick_arm_hysteresis.set_state_and_update(true, hrt_absolute_time());
		const bool arm_trigger = !last_arm_hysteresis && _stick_arm_hysteresis.get_state()
					 && !_stick_disarm_hysteresis.get_state();

		if (arm_trigger || arm_switch_to_arm_transition) {
			ret = true;
		}

	} else if (!arm_button_pressed) {

		_stick_arm_hysteresis.set_state_and_update(false, hrt_absolute_time());
	}

	_last_manual_control_switches_arm_switch = manual_control_switches.arm_switch; // After disarm and arm check

	return ret;
}

void ManualControl::updateParams()
{
	ModuleParams::updateParams();
	_stick_disarm_hysteresis.set_hysteresis_time_from(false, _param_rc_arm_hyst.get() * 1_ms);
	_stick_arm_hysteresis.set_hysteresis_time_from(false, _param_rc_arm_hyst.get() * 1_ms);
}
