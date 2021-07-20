/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_limit_cycle_detector.cpp
 *
 * Definition of an algorithm to detect limit cycle buildup from the supplied
 * control action and calculate a gain factor to apply to the control action to
 * stabilise the limit cycle.
 *
 */

#include "ecl_limit_cycle_detector.h"
#include <float.h>
#include <mathlib/mathlib.h>

float ECL_LimitCycleDetector::calculate_gain_factor(const float input, const float dt)
{
	if (_slew_rate_max <= 0.0f) {
		return 1.0f;
	}

	// Calculate a low pass filtered slew rate
	slew_filter.setParameters(dt, DERIVATIVE_CUTOFF_TCONST);
	const float slew_rate = slew_filter.update((input - _last_input) / dt);
	_last_input = input;

	const float decay_alpha = fminf(dt, _slew_rate_tau) / _slew_rate_tau;

	// Store a series of positive slew rate exceedance events
	uint64_t now_us = hrt_absolute_time();

	if (!_pos_event_stored && slew_rate > _slew_rate_max) {
		if (_pos_event_index >= N_EVENTS) {
			_pos_event_index = 0;
		}

		_pos_event_us[_pos_event_index] = now_us;
		_pos_event_index++;
		_pos_event_stored = true;
		_neg_event_stored = false;
	}

	// Store a series of negative slew rate exceedance events
	if (!_neg_event_stored && slew_rate < - _slew_rate_max) {
		if (_neg_event_index >= N_EVENTS) {
			_neg_event_index = 0;
		}

		_neg_event_us[_neg_event_index] = now_us;
		_neg_event_index++;
		_neg_event_stored = true;
		_pos_event_stored = false;
	}

	// Find the oldest event time
	uint64_t oldest_us = now_us;

	for (uint8_t index = 0; index < N_EVENTS; index++) {
		if (_pos_event_us[index] < oldest_us) {
			oldest_us = _pos_event_us[index];
		}

		if (_neg_event_us[index] < oldest_us) {
			oldest_us = _neg_event_us[index];
		}
	}

	// Decay the peak positive and negative slew rate if they are outside the window
	// Never drop PID gains below 25% of configured value
	if (slew_rate > _max_pos_slew_rate) {
		_max_pos_slew_rate = fminf(slew_rate, (4.0f / COMPRESSOR_GAIN) * _slew_rate_max);
		_max_pos_slew_event_us = now_us;

	} else if (now_us - _max_pos_slew_event_us > WINDOW_US) {
		_max_pos_slew_rate *= (1.0f - decay_alpha);
	}

	if (slew_rate < -_max_neg_slew_rate) {
		_max_neg_slew_rate = fminf(-slew_rate, (4.0f / COMPRESSOR_GAIN) * _slew_rate_max);
		_max_neg_slew_event_us = now_us;

	} else if (now_us - _max_neg_slew_event_us > WINDOW_US) {
		_max_neg_slew_rate *= (1.0f - decay_alpha);
	}

	const float raw_slew_rate = 0.5f * (_max_pos_slew_rate + _max_neg_slew_rate);

	// Apply a further reduction when the oldest exceedance event falls outside the specified window
	// This prevents spikes due to control mode changed, etc causing unwanted gain reduction and is
	// only done to the slew rate used for the gain reduction calculation
	float modifier_input = raw_slew_rate;

	if (now_us - oldest_us > (N_EVENTS + 1) * WINDOW_US) {
		const float oldest_time_from_window = 0.001f * (float)(now_us - oldest_us - (N_EVENTS + 1) * WINDOW_US);
		modifier_input *= expf(-oldest_time_from_window / _slew_rate_tau);
	}

	// Apply a filter to increases in slew rate only to reduce the effect of gusts and large controller
	// setpoint changeschanges
	const float attack_alpha = fminf(2.0f * decay_alpha, 1.0f);

	_modifier_slew_rate = (1.0f - attack_alpha) * _modifier_slew_rate + attack_alpha * modifier_input;
	_modifier_slew_rate = fminf(_modifier_slew_rate, modifier_input);

	_output_slew_rate = (1.0f - attack_alpha) * _output_slew_rate + attack_alpha * raw_slew_rate;
	_output_slew_rate = fminf(_output_slew_rate, raw_slew_rate);

	// Calculate the gain adjustment
	float gain_factor;

	if (_modifier_slew_rate > _slew_rate_max) {
		gain_factor = _slew_rate_max / (_slew_rate_max + COMPRESSOR_GAIN * (_modifier_slew_rate - _slew_rate_max));

	} else {
		gain_factor = 1.0f;
	}

	return gain_factor;
}
