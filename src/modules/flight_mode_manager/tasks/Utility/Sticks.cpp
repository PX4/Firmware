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
 * @file Sticks.cpp
 */

#include "Sticks.hpp"

using namespace time_literals;
using namespace matrix;

Sticks::Sticks(ModuleParams *parent) :
	ModuleParams(parent)
{};

bool Sticks::checkAndSetStickInputs()
{
	// Sticks are rescaled linearly and exponentially to [-1,1]
	manual_control_setpoint_s manual_control_setpoint;

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		// Linear scale
		_positions(0) = manual_control_setpoint.x; // NED x, pitch [-1,1]
		_positions(1) = manual_control_setpoint.y; // NED y, roll [-1,1]
		_positions(2) = -(math::constrain(manual_control_setpoint.z, 0.0f,
						  1.0f) - 0.5f) * 2.f; // NED z, thrust resacaled from [0,1] to [-1,1]
		_positions(3) = manual_control_setpoint.r; // yaw [-1,1]

		// Exponential scale
		_positions_expo(0) = math::expo_deadzone(_positions(0), _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
		_positions_expo(1) = math::expo_deadzone(_positions(1), _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
		_positions_expo(2) = math::expo_deadzone(_positions(2), _param_mpc_z_man_expo.get(),  _param_mpc_hold_dz.get());
		_positions_expo(3) = math::expo_deadzone(_positions(3), _param_mpc_yaw_expo.get(),    _param_mpc_hold_dz.get());

		// valid stick inputs are required
		const bool valid_sticks = PX4_ISFINITE(_positions(0))
					  && PX4_ISFINITE(_positions(1))
					  && PX4_ISFINITE(_positions(2))
					  && PX4_ISFINITE(_positions(3));

		_input_available = valid_sticks;

	} else {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {
			if (vehicle_status.rc_signal_lost) {
				_input_available = false;
			}
		}
	}

	if (!_input_available) {
		// Timeout: set all sticks to zero
		_positions.zero();
		_positions_expo.zero();
	}

	return _input_available;
}

void Sticks::limitStickUnitLengthXY(Vector2f &v)
{
	const float vl = v.length();

	if (vl > 1.0f) {
		v /= vl;
	}
}

void Sticks::rotateIntoHeadingFrameXY(Vector2f &v, const float yaw, const float yaw_setpoint)
{
	Vector3f v3(v(0), v(1), 0.f);
	const float yaw_rotate = PX4_ISFINITE(yaw_setpoint) ? yaw_setpoint : yaw;
	v = Vector2f(Dcmf(Eulerf(0.0f, 0.0f, yaw_rotate)) * v3);
}
