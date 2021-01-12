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
 * @file Sticks.hpp
 *
 * Library abstracting stick input from manual_control_setpoint
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>

class Sticks : public ModuleParams
{
public:
	Sticks(ModuleParams *parent);
	~Sticks() = default;

	bool checkAndSetStickInputs();
	bool isAvailable() { return _input_available; };
	const matrix::Vector<float, 4> &getPosition() { return _positions; };
	const matrix::Vector<float, 4> &getPositionExpo() { return _positions_expo; };


	/**
	 * Limit the the horizontal input from a square shaped joystick gimbal to a unit circle
	 * @param v Vector containing x, y, z axis of the joystick gimbal. x, y get adjusted
	 */
	static void limitStickUnitLengthXY(matrix::Vector2f &v);

	/**
	 * Rotate horizontal component of joystick input into the vehicle body orientation
	 * @param v Vector containing x, y, z axis of the joystick input.
	 * @param yaw Current vehicle yaw heading
	 * @param yaw_setpoint Current yaw setpoint if it's locked else NAN
	 */
	static void rotateIntoHeadingFrameXY(matrix::Vector2f &v, const float yaw, const float yaw_setpoint);

private:
	bool _input_available{false};
	matrix::Vector<float, 4> _positions; ///< unmodified manual stick inputs
	matrix::Vector<float, 4> _positions_expo; ///< modified manual sticks using expo function

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_HOLD_DZ>) _param_mpc_hold_dz,
		(ParamFloat<px4::params::MPC_XY_MAN_EXPO>) _param_mpc_xy_man_expo,
		(ParamFloat<px4::params::MPC_Z_MAN_EXPO>) _param_mpc_z_man_expo,
		(ParamFloat<px4::params::MPC_YAW_EXPO>) _param_mpc_yaw_expo
	)
};
