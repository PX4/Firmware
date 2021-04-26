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
 * @file FlightTaskAutoMapper.hpp
 *
 * Abstract Flight task which generates local setpoints
 * based on the triplet type.
 */

#pragma once

#include "FlightTaskAuto.hpp"
#include "Sticks.hpp"

class FlightTaskAutoMapper : public FlightTaskAuto
{
public:
	FlightTaskAutoMapper();
	virtual ~FlightTaskAutoMapper() = default;
	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
	bool update() override;

protected:

	virtual void _generateSetpoints() = 0; /**< Generate velocity and position setpoint for following line. */

	void _prepareIdleSetpoints();
	void _prepareLandSetpoints();
	void _prepareVelocitySetpoints();
	void _prepareTakeoffSetpoints();
	void _preparePositionSetpoints();

	void updateParams() override; /**< See ModuleParam class */

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskAuto,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
					(ParamInt<px4::params::MPC_LAND_RC_HELP>) _param_mpc_land_rc_help,
					(ParamFloat<px4::params::MPC_LAND_ALT1>)
					_param_mpc_land_alt1, // altitude at which speed limit downwards reaches maximum speed
					(ParamFloat<px4::params::MPC_LAND_ALT2>)
					_param_mpc_land_alt2, // altitude at which speed limit downwards reached minimum speed
					(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
					(ParamFloat<px4::params::MPC_TKO_RAMP_T>)
					_param_mpc_tko_ramp_t // time constant for smooth takeoff ramp
				       );

private:
	Sticks _sticks;
	void _reset(); /**< Resets member variables to current vehicle state */
	WaypointType _type_previous{WaypointType::idle}; /**< Previous type of current target triplet. */
	bool _highEnoughForLandingGear(); /**< Checks if gears can be lowered. */
	float _getLandSpeed(); /**< Returns landing descent speed. */
};
