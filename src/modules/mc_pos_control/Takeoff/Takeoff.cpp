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
 * @file Takeoff.cpp
 */

#include "Takeoff.hpp"
#include <mathlib/mathlib.h>
#include <lib/ecl/geo/geo.h>

void Takeoff::generateInitialRampValue(float velocity_p_gain)
{
	velocity_p_gain = math::max(velocity_p_gain, 0.01f);
	_takeoff_ramp_vz_init = -CONSTANTS_ONE_G / velocity_p_gain;
}

void Takeoff::updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				 const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us)
{
	_spoolup_time_hysteresis.set_state_and_update(armed, now_us);

	switch (_takeoff_state) {
	case TakeoffState::disarmed:
		if (armed) {
			_takeoff_state = TakeoffState::spoolup;

		} else {
			break;
		}

	// FALLTHROUGH
	case TakeoffState::spoolup:
		if (_spoolup_time_hysteresis.get_state()) {
			_takeoff_state = TakeoffState::ready_for_takeoff;

		} else {
			break;
		}

	// FALLTHROUGH
	case TakeoffState::ready_for_takeoff:
		if (want_takeoff) {
			_takeoff_state = TakeoffState::rampup;
			_takeoff_ramp_progress = 0.f;

		} else {
			break;
		}

	// FALLTHROUGH
	case TakeoffState::rampup:
		if (_takeoff_ramp_progress >= 1.f) {
			_takeoff_state = TakeoffState::flight;

		} else {
			break;
		}

	// FALLTHROUGH
	case TakeoffState::flight:
		if (landed) {
			_takeoff_state = TakeoffState::ready_for_takeoff;
		}

		break;

	default:
		break;
	}

	if (armed && skip_takeoff) {
		_takeoff_state = TakeoffState::flight;
	}

	// TODO: need to consider free fall here
	if (!armed) {
		_takeoff_state = TakeoffState::disarmed;
	}
}

float Takeoff::updateRamp(const float dt, const float takeoff_desired_vz)
{
	float upwards_velocity_limit = takeoff_desired_vz;

	if (_takeoff_state < TakeoffState::rampup) {
		upwards_velocity_limit = _takeoff_ramp_vz_init;
	}

	if (_takeoff_state == TakeoffState::rampup) {
		if (_takeoff_ramp_time > dt) {
			_takeoff_ramp_progress += dt / _takeoff_ramp_time;

		} else {
			_takeoff_ramp_progress = 1.f;
		}

		if (_takeoff_ramp_progress < 1.f) {
			upwards_velocity_limit = _takeoff_ramp_vz_init + _takeoff_ramp_progress * (takeoff_desired_vz - _takeoff_ramp_vz_init);
		}
	}

	return upwards_velocity_limit;
}
