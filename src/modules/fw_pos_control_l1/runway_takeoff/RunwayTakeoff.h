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
 * @file RunwayTakeoff.h
 * Runway takeoff handling for fixed-wing UAVs with steerable wheels.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#ifndef RUNWAYTAKEOFF_H
#define RUNWAYTAKEOFF_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace runwaytakeoff
{

enum RunwayTakeoffState {
	THROTTLE_RAMP = 0, /**< ramping up throttle */
	CLAMPED_TO_RUNWAY = 1, /**< clamped to runway, controlling yaw directly (wheel or rudder) */
	TAKEOFF = 2, /**< taking off, get ground clearance, roll 0 */
	CLIMBOUT = 3, /**< climbout to safe height before navigation, roll limited */
	FLY = 4 /**< fly towards takeoff waypoint */
};

class __EXPORT RunwayTakeoff : public ModuleParams
{
public:
	RunwayTakeoff(ModuleParams *parent);
	~RunwayTakeoff() = default;

	void init(const hrt_abstime &now, float yaw, double current_lat, double current_lon);
	void update(const hrt_abstime &now, float airspeed, float alt_agl, double current_lat, double current_lon,
		    orb_advert_t *mavlink_log_pub);

	RunwayTakeoffState getState() { return _state; }
	bool isInitialized() { return _initialized; }

	bool runwayTakeoffEnabled() { return _param_rwto_tkoff.get(); }
	float getMinAirspeedScaling() { return _param_rwto_airspd_scl.get(); }
	float getInitYaw() { return _init_yaw; }

	bool controlYaw();
	bool climbout() { return _climbout; }
	float getPitch(float tecsPitch);
	float getRoll(float navigatorRoll);
	float getYaw(float navigatorYaw);
	float getThrottle(const hrt_abstime &now, float tecsThrottle);
	bool resetIntegrators();
	float getMinPitch(float climbout_min, float min);
	float getMaxPitch(float max);
	const matrix::Vector2d &getStartWP() const { return _start_wp; };

	void reset();

private:
	/** state variables **/
	RunwayTakeoffState _state{THROTTLE_RAMP};
	bool _initialized{false};
	hrt_abstime _initialized_time{0};
	float _init_yaw{0.f};
	bool _climbout{false};
	matrix::Vector2d _start_wp;

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::RWTO_TKOFF>) _param_rwto_tkoff,
		(ParamInt<px4::params::RWTO_HDG>) _param_rwto_hdg,
		(ParamFloat<px4::params::RWTO_NAV_ALT>) _param_rwto_nav_alt,
		(ParamFloat<px4::params::RWTO_MAX_THR>) _param_rwto_max_thr,
		(ParamFloat<px4::params::RWTO_PSP>) _param_rwto_psp,
		(ParamFloat<px4::params::RWTO_MAX_PITCH>) _param_rwto_max_pitch,
		(ParamFloat<px4::params::RWTO_MAX_ROLL>) _param_rwto_max_roll,
		(ParamFloat<px4::params::RWTO_AIRSPD_SCL>) _param_rwto_airspd_scl,
		(ParamFloat<px4::params::RWTO_RAMP_TIME>) _param_rwto_ramp_time,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_CLMBOUT_DIFF>) _param_fw_clmbout_diff
	)

};

}

#endif // RUNWAYTAKEOFF_H
