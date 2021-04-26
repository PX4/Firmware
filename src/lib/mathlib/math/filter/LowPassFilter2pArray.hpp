/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

/// @file	LowPassFilter2pArray.hpp
/// @brief	A class to implement a second order low pass filter

#pragma once

#include "LowPassFilter2p.hpp"

#include <px4_platform_common/defines.h>

namespace math
{

class LowPassFilter2pArray : public LowPassFilter2p
{
public:

	LowPassFilter2pArray(float sample_freq, float cutoff_freq) : LowPassFilter2p(sample_freq, cutoff_freq)
	{
	}

	// Filter array of samples in place using the Direct form II.
	inline void apply(float samples[], uint8_t num_samples)
	{
		for (int n = 0; n < num_samples; n++) {
			// Direct Form II implementation
			float delay_element_0{samples[n] - _delay_element_1 *_a1 - _delay_element_2 * _a2};

			// don't allow bad values to propagate via the filter
			if (!PX4_ISFINITE(delay_element_0)) {
				delay_element_0 = samples[n];
			}

			samples[n] = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

			_delay_element_2 = _delay_element_1;
			_delay_element_1 = delay_element_0;
		}
	}

};

} // namespace math
