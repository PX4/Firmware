/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file FixedwingLandDetector.h
 * Land detector implementation for fixedwing.
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <uORB/topics/airspeed.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/vehicle_attitude.h>

#include "LandDetector.h"

using namespace time_literals;
using math::radians;
using matrix::Quatf;
using matrix::Eulerf;
using matrix::Dcmf;

namespace land_detector
{

class FixedwingLandDetector final : public LandDetector
{
public:
	FixedwingLandDetector();
	~FixedwingLandDetector() override = default;

protected:

	bool _get_landed_state() override;
	void _update_topics() override;

private:

	/** Time in us that landing conditions have to hold before triggering a land. */
	static constexpr hrt_abstime LANDED_TRIGGER_TIME_US = 2_s;
	static constexpr hrt_abstime FLYING_TRIGGER_TIME_US = 0_us;

	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _controller_sub{ORB_ID(position_controller_status)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};

	airspeed_s _airspeed{};
	vehicle_attitude_s _attitude{};
	position_controller_status_s _status{};

	//variables for calculating the pitch of the vehicle
	void vehicle_calcPitch();
	Dcmf _PQ;
	float _pitch{0.0};

	float _airspeed_filtered{0.0f};
	float _velocity_xy_filtered{0.0f};
	float _velocity_z_filtered{0.0f};
	float _xy_accel_filtered{0.0f};

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		LandDetector,
		(ParamFloat<px4::params::LNDFW_XYACC_MAX>)  _param_lndfw_xyaccel_max,
		(ParamFloat<px4::params::LNDFW_AIRSPD_MAX>) _param_lndfw_airspd,
		(ParamFloat<px4::params::LNDFW_VEL_XY_MAX>) _param_lndfw_vel_xy_max,
		(ParamFloat<px4::params::LNDFW_VEL_Z_MAX>)  _param_lndfw_vel_z_max,
		(ParamFloat<px4::params::LAUN_CAT_PMAX>) _param_laun_cat_pmax,
		(ParamFloat<px4::params::LAUN_CAT_A>) _param_laun_cat_accel
	);
};

} // namespace land_detector
