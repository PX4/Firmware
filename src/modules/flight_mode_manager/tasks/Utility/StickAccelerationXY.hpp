/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file StickAccelerationXY.hpp
 * @brief Generate horizontal position, velocity and acceleration from stick input
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <lib/ecl/AlphaFilter/AlphaFilter.hpp>
#include <matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/takeoff_status.h>

#include "SlewRate.hpp"

class StickAccelerationXY : public ModuleParams
{
public:
	StickAccelerationXY(ModuleParams *parent);
	~StickAccelerationXY() = default;

	void resetPosition();
	void resetVelocity(const matrix::Vector2f &velocity);
	void resetAcceleration(const matrix::Vector2f &acceleration);
	void generateSetpoints(matrix::Vector2f stick_xy, const float yaw, const float yaw_sp, const matrix::Vector3f &pos,
			       const matrix::Vector2f &vel_sp_feedback, const float dt);
	void getSetpoints(matrix::Vector3f &pos_sp, matrix::Vector3f &vel_sp, matrix::Vector3f &acc_sp);

private:
	void applyJerkLimit(const float dt);
	matrix::Vector2f calculateDrag(matrix::Vector2f drag_coefficient, const float dt, const matrix::Vector2f &stick_xy,
				       const matrix::Vector2f &vel_sp);
	void applyTiltLimit(matrix::Vector2f &acceleration);
	void lockPosition(const matrix::Vector3f &pos, const matrix::Vector2f &vel_sp_feedback, const float dt);

	uORB::Subscription _takeoff_status_sub{ORB_ID(takeoff_status)};

	SlewRate<float> _acceleration_slew_rate_x;
	SlewRate<float> _acceleration_slew_rate_y;
	AlphaFilter<float> _brake_boost_filter;

	matrix::Vector2f _position_setpoint;
	matrix::Vector2f _velocity_setpoint;
	matrix::Vector2f _acceleration_setpoint;
	matrix::Vector2f _acceleration_setpoint_prev;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor,
		(ParamFloat<px4::params::MPC_JERK_MAX>) _param_mpc_jerk_max,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air
	)
};
