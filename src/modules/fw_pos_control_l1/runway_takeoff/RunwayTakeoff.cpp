/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file RunwayTakeoff.cpp
 * Runway takeoff handling for fixed-wing UAVs with steerable wheels.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "RunwayTakeoff.h"
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>

using namespace time_literals;

namespace runwaytakeoff
{

RunwayTakeoff::RunwayTakeoff(ModuleParams *parent) :
	ModuleParams(parent),
	_state(),
	_initialized(false),
	_initialized_time(0),
	_init_yaw(0),
	_climbout(false)
{
}

void RunwayTakeoff::init(const hrt_abstime &now, float yaw, double current_lat, double current_lon)
{
	_init_yaw = yaw;
	_initialized = true;
	_state = RunwayTakeoffState::THROTTLE_RAMP;
	_initialized_time = now;
	_climbout = true; // this is true until climbout is finished
	_start_wp(0) = current_lat;
	_start_wp(1) = current_lon;
}

void RunwayTakeoff::update(const hrt_abstime &now, float airspeed, float alt_agl,
			   double current_lat, double current_lon, orb_advert_t *mavlink_log_pub)
{
	switch (_state) {
	case RunwayTakeoffState::THROTTLE_RAMP:
		if (((now - _initialized_time) > (_param_rwto_ramp_time.get() * 1_s))
		    || (airspeed > (_param_fw_airspd_min.get() * _param_rwto_airspd_scl.get() * 0.9f))) {

			_state = RunwayTakeoffState::CLAMPED_TO_RUNWAY;
		}

		break;

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY:
		if (airspeed > _param_fw_airspd_min.get() * _param_rwto_airspd_scl.get()) {
			_state = RunwayTakeoffState::TAKEOFF;
			mavlink_log_info(mavlink_log_pub, "#Takeoff airspeed reached");
		}

		break;

	case RunwayTakeoffState::TAKEOFF:
		if (alt_agl > _param_rwto_nav_alt.get()) {
			_state = RunwayTakeoffState::CLIMBOUT;

			/*
			 * If we started in heading hold mode, move the navigation start WP to the current location now.
			 * The navigator will take this as starting point to navigate towards the takeoff WP.
			 */
			if (_param_rwto_hdg.get() == 0) {
				_start_wp(0) = current_lat;
				_start_wp(1) = current_lon;
			}

			mavlink_log_info(mavlink_log_pub, "#Climbout");
		}

		break;

	case RunwayTakeoffState::CLIMBOUT:
		if (alt_agl > _param_fw_clmbout_diff.get()) {
			_climbout = false;
			_state = RunwayTakeoffState::FLY;
			mavlink_log_info(mavlink_log_pub, "#Navigating to waypoint");
		}

		break;

	default:
		break;
	}
}

/*
 * Returns true as long as we're below navigation altitude
 */
bool RunwayTakeoff::controlYaw()
{
	// keep controlling yaw directly until we start navigation
	return _state < RunwayTakeoffState::CLIMBOUT;
}

/*
 * Returns pitch setpoint to use.
 *
 * Limited (parameter) as long as the plane is on runway. Otherwise
 * use the one from TECS
 */
float RunwayTakeoff::getPitch(float tecsPitch)
{
	if (_state <= RunwayTakeoffState::CLAMPED_TO_RUNWAY) {
		return math::radians(_param_rwto_psp.get());
	}

	return tecsPitch;
}

/*
 * Returns the roll setpoint to use.
 */
float RunwayTakeoff::getRoll(float navigatorRoll)
{
	// until we have enough ground clearance, set roll to 0
	if (_state < RunwayTakeoffState::CLIMBOUT) {
		return 0.0f;
	}

	// allow some roll during climbout
	else if (_state < RunwayTakeoffState::FLY) {
		return math::constrain(navigatorRoll,
				       math::radians(-_param_rwto_max_roll.get()),
				       math::radians(_param_rwto_max_roll.get()));
	}

	return navigatorRoll;
}

/*
 * Returns the yaw setpoint to use.
 *
 * In heading hold mode (_heading_mode == 0), it returns initial yaw as long as it's on the
 * runway. When it has enough ground clearance we start navigation towards WP.
 */
float RunwayTakeoff::getYaw(float navigatorYaw)
{
	if (_param_rwto_hdg.get() == 0 && _state < RunwayTakeoffState::CLIMBOUT) {
		return _init_yaw;

	} else {
		return navigatorYaw;
	}
}

/*
 * Returns the throttle setpoint to use.
 *
 * Ramps up in the beginning, until it lifts off the runway it is set to
 * parameter value, then it returns the TECS throttle.
 */
float RunwayTakeoff::getThrottle(const hrt_abstime &now, float tecsThrottle)
{
	switch (_state) {
	case RunwayTakeoffState::THROTTLE_RAMP: {
			float throttle = ((now - _initialized_time) / (_param_rwto_ramp_time.get() * 1_s)) * _param_rwto_max_thr.get();
			return math::min(throttle, _param_rwto_max_thr.get());
		}

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY:
		return _param_rwto_max_thr.get();

	default:
		return tecsThrottle;
	}
}

bool RunwayTakeoff::resetIntegrators()
{
	// reset integrators if we're still on runway
	return _state < RunwayTakeoffState::TAKEOFF;
}

/*
 * Returns the minimum pitch for TECS to use.
 *
 * In climbout we either want what was set on the waypoint (sp_min) but at least
 * the climbtout minimum pitch (parameter).
 * Otherwise use the minimum that is enforced generally (parameter).
 */
float RunwayTakeoff::getMinPitch(float climbout_min, float min)
{
	if (_state < RunwayTakeoffState::FLY) {
		return climbout_min;

	} else {
		return min;
	}
}

/*
 * Returns the maximum pitch for TECS to use.
 *
 * Limited by parameter (if set) until climbout is done.
 */
float RunwayTakeoff::getMaxPitch(float max)
{
	// use max pitch from parameter if set (> 0.1)
	if (_state < RunwayTakeoffState::FLY && _param_rwto_max_pitch.get() > 0.1f) {
		return _param_rwto_max_pitch.get();
	}

	else {
		return max;
	}
}

void RunwayTakeoff::reset()
{
	_initialized = false;
	_state = RunwayTakeoffState::THROTTLE_RAMP;
}

}
