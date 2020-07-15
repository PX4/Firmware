/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AngularVelocityControl.cpp
 */

#include <AngularVelocityControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void AngularVelocityControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void AngularVelocityControl::setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
		const matrix::Vector<bool, 3> &saturation_negative)
{
	_saturation_positive = saturation_positive;
	_saturation_negative = saturation_negative;
}

void AngularVelocityControl::update(const Vector3f &angular_velocity, const Vector3f &angular_velocity_sp,
				    const Vector3f &angular_acceleration, const float dt, const bool landed)
{
	// angular rates error
	Vector3f angular_velocity_error = angular_velocity_sp - angular_velocity;

	// P + D control
	_angular_accel_sp = _gain_p.emult(angular_velocity_error) - _gain_d.emult(angular_acceleration);

	// I + FF control
	Vector3f torque_feedforward = _angular_velocity_int + _gain_ff.emult(angular_velocity_sp);

	// compute torque setpoint
	_torque_sp = _inertia * _angular_accel_sp + torque_feedforward + angular_velocity.cross(_inertia * angular_velocity);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(angular_velocity_error, dt);
	}
}

void AngularVelocityControl::updateIntegral(Vector3f &angular_velocity_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_saturation_positive(i)) {
			angular_velocity_error(i) = math::min(angular_velocity_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_saturation_negative(i)) {
			angular_velocity_error(i) = math::max(angular_velocity_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = angular_velocity_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float angular_velocity_i = _angular_velocity_int(i) + i_factor * _gain_i(i) * angular_velocity_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(angular_velocity_i)) {
			_angular_velocity_int(i) = math::constrain(angular_velocity_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void AngularVelocityControl::reset()
{
	_angular_velocity_int.zero();
	_torque_sp.zero();
	_angular_accel_sp.zero();
}
