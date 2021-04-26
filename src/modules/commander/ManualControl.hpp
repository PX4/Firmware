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

/**
 * @file ManualControl.hpp
 *
 * @brief Logic for handling RC or Joystick input
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

class ManualControl : ModuleParams
{
public:
	ManualControl(ModuleParams *parent) : ModuleParams(parent) {};
	~ManualControl() override = default;

	void setRCAllowed(const bool rc_allowed) { _rc_allowed = rc_allowed; }

	/**
	 * Check for manual control input changes and process them
	 * @return true if there was new data
	 */
	bool update();
	bool isRCAvailable() const { return _rc_available; }
	bool isMavlink() const { return _manual_control_setpoint.data_source > manual_control_setpoint_s::SOURCE_RC; }
	bool wantsOverride(const vehicle_control_mode_s &vehicle_control_mode);
	bool wantsDisarm(const vehicle_control_mode_s &vehicle_control_mode, const vehicle_status_s &vehicle_status,
			 manual_control_switches_s &manual_control_switches, const bool landed);
	bool wantsArm(const vehicle_control_mode_s &vehicle_control_mode, const vehicle_status_s &vehicle_status,
		      const manual_control_switches_s &manual_control_switches, const bool landed);
	bool isThrottleLow() const { return _last_manual_control_setpoint.z < 0.1f; }
	bool isThrottleAboveCenter() const { return _last_manual_control_setpoint.z > 0.6f; }
	hrt_abstime getLastRcTimestamp() const { return _last_manual_control_setpoint.timestamp; }

private:
	void updateParams() override;

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s _manual_control_setpoint{};
	manual_control_setpoint_s _last_manual_control_setpoint{};

	// Availability
	bool _rc_allowed{false};
	bool _rc_available{false};

	// Arming/disarming
	systemlib::Hysteresis _stick_disarm_hysteresis{false};
	systemlib::Hysteresis _stick_arm_hysteresis{false};
	uint8_t _last_manual_control_switches_arm_switch{manual_control_switches_s::SWITCH_POS_NONE};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::COM_RC_LOSS_T>) _param_com_rc_loss_t,
		(ParamInt<px4::params::COM_RC_ARM_HYST>) _param_rc_arm_hyst,
		(ParamBool<px4::params::COM_ARM_SWISBTN>) _param_com_arm_swisbtn,
		(ParamBool<px4::params::COM_REARM_GRACE>) _param_com_rearm_grace,
		(ParamInt<px4::params::COM_RC_OVERRIDE>) _param_rc_override,
		(ParamFloat<px4::params::COM_RC_STICK_OV>) _param_com_rc_stick_ov
	)
};
