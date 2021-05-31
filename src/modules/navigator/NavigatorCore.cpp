/***************************************************************************
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
 * @file NavigatorCore.cpp
 *
 */

#include "NavigatorCore.h"
#include <px4_platform_common/defines.h>


NavigatorCore::NavigatorCore() :
	ModuleParams(nullptr)
{

}

NavigatorCore::~NavigatorCore()
{

}

bool NavigatorCore::forceVTOL()
{

	return _status.is_vtol && (isRotaryWing() || _status.in_transition_to_fw) && _param_nav_force_vt.get();
}

float NavigatorCore::getHorAcceptanceRadiusMeter()
{
	if (isRotaryWing()) {
		return getDefaultHorAcceptanceRadiusMeter();

	} else {
		return math::max(_pos_ctrl_status.acceptance_radius, getDefaultHorAcceptanceRadiusMeter());
	}
}

float NavigatorCore::getAltAcceptanceRadMeter()
{
	if (isFixedWing()) {
		const position_setpoint_s &next_sp =  getCurrentTriplet().next;

		if (next_sp.type == position_setpoint_s::SETPOINT_TYPE_LAND && next_sp.valid) {
			// Use separate (tighter) altitude acceptance for clean altitude starting point before landing
			return getFixedWingLandingAltAcceptanceRadius();
		}
	}

	return getDefaultAltAcceptanceRadiusMeter();
}

float NavigatorCore::getDefaultAltAcceptanceRadiusMeter()
{

	if (isFixedWing()) {
		return getFixedWingAltAcceptanceRadiusMeter();

	} else if (isRover()) {
		return INFINITY;

	} else {
		float alt_acceptance_radius = getMulticopterAltAcceptanceRadiusMeter();

		if ((_pos_ctrl_status.timestamp > getCurrentTriplet().timestamp)
		    && _pos_ctrl_status.altitude_acceptance > alt_acceptance_radius) {
			alt_acceptance_radius = _pos_ctrl_status.altitude_acceptance;
		}

		return alt_acceptance_radius;
	}
}

float NavigatorCore::getAcceptanceRadiusMeter()
{

	float acceptance_radius = _param_nav_acc_rad.get();

	// for fixed-wing and rover, return the max of NAV_ACC_RAD and the controller acceptance radius (e.g. L1 distance)
	if (!isRotaryWing() && PX4_ISFINITE(_pos_ctrl_status.acceptance_radius) && _pos_ctrl_status.timestamp != 0) {

		acceptance_radius = math::max(acceptance_radius, _pos_ctrl_status.acceptance_radius);
	}

	return acceptance_radius;
}

float NavigatorCore::getAltAcceptanceRadiusMeter()
{

	if (isFixedWing()) {
		const position_setpoint_s &next_sp = _triplet.next;

		if (next_sp.type == position_setpoint_s::SETPOINT_TYPE_LAND && next_sp.valid) {
			// Use separate (tighter) altitude acceptance for clean altitude starting point before landing
			return _param_nav_fw_altl_rad.get();
		}
	}

	return getDefaultAltAcceptanceRadiusMeter();
}
