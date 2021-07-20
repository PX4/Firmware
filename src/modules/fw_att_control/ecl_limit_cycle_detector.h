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
 * @file ecl_limit_cycle_detector.h
 *
 * Definition of an algorithm to detect limit cycle buildup from the supplied
 * control action and calculate a gain factor to apply to the control action to
 * stabilise the limit cycle.
 *
 * @author Paul Riseborough <gncsolns@gmail.com>
 *
*/

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/ecl/AlphaFilter/AlphaFilter.hpp>

#define N_EVENTS 2
#define WINDOW_US 300000
#define COMPRESSOR_GAIN 1.5f
#define DERIVATIVE_CUTOFF_TCONST 0.02f

class ECL_LimitCycleDetector
{
public:
	ECL_LimitCycleDetector() = default;
	void set_parameters(float slew_rate_max, float slew_rate_tau) {_slew_rate_max = slew_rate_max; _slew_rate_tau = slew_rate_tau; }
	float calculate_gain_factor(const float input, const float dt);
	float get_limit_cycle_slew_rate() { return _output_slew_rate; }

private:
	float _slew_rate_max{5.0f};
	float _slew_rate_tau{0.5f};
	AlphaFilter<float> slew_filter;
	float _output_slew_rate{0.0f};
	float _modifier_slew_rate{0.0f};
	float _last_input{0.0f};
	float _max_pos_slew_rate{0.0f};
	float _max_neg_slew_rate{0.0f};
	uint64_t _max_pos_slew_event_us{0};
	uint64_t _max_neg_slew_event_us{0};
	uint8_t _pos_event_index{0};
	uint8_t _neg_event_index{0};
	uint64_t _pos_event_us[N_EVENTS] = {};
	uint64_t _neg_event_us[N_EVENTS] = {};
	bool _pos_event_stored{false};
	bool _neg_event_stored{false};
};
