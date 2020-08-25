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
 * @file FixedwingLandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#include "FixedwingLandDetector.h"

namespace land_detector
{

FixedwingLandDetector::FixedwingLandDetector()
{
	// Use Trigger time when transitioning from in-air (false) to landed (true) / ground contact (true).
	_landed_hysteresis.set_hysteresis_time_from(false, LANDED_TRIGGER_TIME_US);
	_landed_hysteresis.set_hysteresis_time_from(true, FLYING_TRIGGER_TIME_US);
}

void FixedwingLandDetector::_update_topics()
{
	LandDetector::_update_topics();
	_airspeed_validated_sub.update(&_airspeed_validated);
}

bool FixedwingLandDetector::_get_landed_state()
{
	// Only trigger flight conditions if we are armed.
	if (!_actuator_armed.armed) {
		return true;
	}

	bool landDetected = false;

	if (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 1_s) {

		// Horizontal velocity complimentary filter.
		float val = 0.97f * _velocity_xy_filtered + 0.03f * sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx +
				_vehicle_local_position.vy * _vehicle_local_position.vy);

		if (PX4_ISFINITE(val)) {
			_velocity_xy_filtered = val;
		}

		// Vertical velocity complimentary filter.
		val = 0.99f * _velocity_z_filtered + 0.01f * fabsf(_vehicle_local_position.vz);

		if (PX4_ISFINITE(val)) {
			_velocity_z_filtered = val;
		}

		// set _airspeed_filtered to 0 if airspeed data is invalid
		if (!PX4_ISFINITE(_airspeed_validated.true_airspeed_m_s) || hrt_elapsed_time(&_airspeed_validated.timestamp) > 1_s) {
			_airspeed_filtered = 0.0f;

		} else {
			_airspeed_filtered = 0.95f * _airspeed_filtered + 0.05f * _airspeed_validated.true_airspeed_m_s;
		}

		// A leaking lowpass prevents biases from building up, but
		// gives a mostly correct response for short impulses.
		const matrix::Vector3f accel{_vehicle_acceleration.xyz};
		const float acc_hor = sqrtf(accel(0) * accel(0) + accel(1) * accel(1));

		_xy_accel_filtered = _xy_accel_filtered * 0.8f + acc_hor * 0.18f;

		// Crude land detector for fixedwing.
		landDetected = _airspeed_filtered       < _param_lndfw_airspd.get()
			       && _velocity_xy_filtered < _param_lndfw_vel_xy_max.get()
			       && _velocity_z_filtered  < _param_lndfw_vel_z_max.get()
			       && _xy_accel_filtered    < _param_lndfw_xyaccel_max.get();

	} else {
		// Control state topic has timed out and we need to assume we're landed.
		landDetected = true;
	}

	return landDetected;
}

} // namespace land_detector
