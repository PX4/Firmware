/***************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file gpsfailure.h
 * Helper class for Data Link Loss Mode according to the OBC rules
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>

#include "mission_block.h"

#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>

class Navigator;

class GpsFailure : public MissionBlock, public ModuleParams
{
public:
	GpsFailure(Navigator *navigator);
	~GpsFailure() = default;

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_GPSF_LT>) _param_nav_gpsf_lt,
		(ParamFloat<px4::params::NAV_GPSF_R>) _param_nav_gpsf_r,
		(ParamFloat<px4::params::NAV_GPSF_P>) _param_nav_gpsf_p,
		(ParamFloat<px4::params::NAV_GPSF_TR>) _param_nav_gpsf_tr
	)

	enum GPSFState {
		GPSF_STATE_NONE = 0,
		GPSF_STATE_LOITER = 1,
		GPSF_STATE_TERMINATE = 2,
		GPSF_STATE_END = 3,
	} _gpsf_state{GPSF_STATE_NONE};

	hrt_abstime _timestamp_activation{0}; //*< timestamp when this mode was activated */

	uORB::Publication<vehicle_attitude_setpoint_s>	_att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s>	_fw_virtual_att_sp_pub{ORB_ID(fw_virtual_attitude_setpoint)};
	/**
	 * Set the GPSF item
	 */
	void		set_gpsf_item();

	/**
	 * Move to next GPSF item
	 */
	void		advance_gpsf();

	/**
	 *  boolean to determine if the vehicle was in a landing state prior to activating gps failure
	 */
	bool was_landing{false};

};
