/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPosition.hpp
 *
 * Flight task for manual position controlled mode.
 *
 */

#pragma once

#include <lib/collision_prevention/CollisionPrevention.hpp>
#include "FlightTaskManualAltitude.hpp"

class FlightTaskManualPosition : public FlightTaskManualAltitude
{
public:
	FlightTaskManualPosition();

	virtual ~FlightTaskManualPosition() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;

	/**
	 * Sets an external yaw handler which can be used to implement a different yaw control strategy.
	 */
	void setYawHandler(WeatherVane *yaw_handler) override { _weathervane_yaw_handler = yaw_handler; }


protected:
	void _updateXYlock(); /**< applies position lock based on stick and velocity */
	void _updateSetpoints() override;
	void _scaleSticks() override;

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualAltitude,
					(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual,
					(ParamFloat<px4::params::MPC_LAND_VEL_XY>) _param_mpc_land_vel_xy,
					(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _param_mpc_acc_hor_max,
					(ParamFloat<px4::params::MPC_HOLD_MAX_XY>) _param_mpc_hold_max_xy
				       )
private:
	float _computeVelXYGroundDist();
	float _velocity_scale{0.0f}; //scales the stick input to velocity
	uint8_t _reset_counter{0}; /**< counter for estimator resets in xy-direction */

	WeatherVane *_weathervane_yaw_handler =
		nullptr;	/**< external weathervane library, used to implement a yaw control law that turns the vehicle nose into the wind */

	CollisionPrevention _collision_prevention;	/**< collision avoidance setpoint amendment */
};
