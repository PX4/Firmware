/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskOffboard.hpp
 */

#pragma once

#include "FlightTask.hpp"
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>

class FlightTaskOffboard : public FlightTask
{
public:
	FlightTaskOffboard() = default;

	virtual ~FlightTaskOffboard() = default;
	bool update() override;
	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;

protected:
	uORB::SubscriptionData<position_setpoint_triplet_s> _sub_triplet_setpoint{ORB_ID(position_setpoint_triplet)};

private:
	matrix::Vector3f _position_lock{};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
					(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
					(ParamFloat<px4::params::MIS_TAKEOFF_ALT>) _param_mis_takeoff_alt
				       )
};
