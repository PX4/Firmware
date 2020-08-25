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
* @file FailureDetector.cpp
*
* @author Mathieu Bresciani	<brescianimathieu@gmail.com>
*
*/

#include "FailureDetector.hpp"

using namespace time_literals;

FailureDetector::FailureDetector(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool FailureDetector::resetStatus()
{
	bool status_changed = _status != FAILURE_NONE;
	_status = FAILURE_NONE;

	return status_changed;
}

bool
FailureDetector::update(const vehicle_status_s &vehicle_status)
{
	bool updated(false);

	if (isAttitudeStabilized(vehicle_status)) {
		updated = updateAttitudeStatus();

		if (_param_fd_ext_ats_en.get()) {
			updated |= updateExternalAtsStatus();
		}

	} else {
		updated = resetStatus();
	}

	return updated;
}

bool
FailureDetector::isAttitudeStabilized(const vehicle_status_s &vehicle_status)
{
	bool attitude_is_stabilized{false};
	const uint8_t vehicle_type = vehicle_status.vehicle_type;
	const uint8_t nav_state = vehicle_status.nav_state;

	if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		attitude_is_stabilized =  nav_state != vehicle_status_s::NAVIGATION_STATE_ACRO &&
					  nav_state != vehicle_status_s::NAVIGATION_STATE_RATTITUDE;

	} else if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		attitude_is_stabilized =  nav_state != vehicle_status_s::NAVIGATION_STATE_MANUAL &&
					  nav_state != vehicle_status_s::NAVIGATION_STATE_ACRO &&
					  nav_state != vehicle_status_s::NAVIGATION_STATE_RATTITUDE;
	}

	return attitude_is_stabilized;
}

bool
FailureDetector::updateAttitudeStatus()
{
	bool updated(false);
	vehicle_attitude_s attitude;

	if (_sub_vehicule_attitude.update(&attitude)) {

		const matrix::Eulerf euler(matrix::Quatf(attitude.q));
		const float roll(euler.phi());
		const float pitch(euler.theta());

		const float max_roll_deg = _param_fd_fail_r.get();
		const float max_pitch_deg = _param_fd_fail_p.get();
		const float max_roll(fabsf(math::radians(max_roll_deg)));
		const float max_pitch(fabsf(math::radians(max_pitch_deg)));

		const bool roll_status = (max_roll > 0.0f) && (fabsf(roll) > max_roll);
		const bool pitch_status = (max_pitch > 0.0f) && (fabsf(pitch) > max_pitch);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_roll_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_r_ttri.get()));
		_pitch_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_p_ttri.get()));
		_roll_failure_hysteresis.set_state_and_update(roll_status, time_now);
		_pitch_failure_hysteresis.set_state_and_update(pitch_status, time_now);

		// Update bitmask
		_status &= ~(FAILURE_ROLL | FAILURE_PITCH);

		if (_roll_failure_hysteresis.get_state()) {
			_status |= FAILURE_ROLL;
		}

		if (_pitch_failure_hysteresis.get_state()) {
			_status |= FAILURE_PITCH;
		}

		updated = true;
	}

	return updated;
}

bool
FailureDetector::updateExternalAtsStatus()
{
	pwm_input_s pwm_input;
	bool updated = _sub_pwm_input.update(&pwm_input);

	if (updated) {

		uint32_t pulse_width = pwm_input.pulse_width;
		bool ats_trigger_status = (pulse_width >= (uint32_t)_param_fd_ext_ats_trig.get()) && (pulse_width < 3_ms);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_ext_ats_failure_hysteresis.set_hysteresis_time_from(false, 100_ms); // 5 consecutive pulses at 50hz
		_ext_ats_failure_hysteresis.set_state_and_update(ats_trigger_status, time_now);

		_status &= ~FAILURE_EXT;

		if (_ext_ats_failure_hysteresis.get_state()) {
			_status |= FAILURE_EXT;
		}
	}

	return updated;
}