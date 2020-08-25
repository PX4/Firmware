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

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	inline float apply(const int16_t samples[], uint8_t num_samples)
	{
		float output = 0.0f;

		for (int n = 0; n < num_samples; n++) {
			// do the filtering
			float delay_element_0 = samples[n] - _delay_element_1 * _a1 - _delay_element_2 * _a2;

			if (n == num_samples - 1) {
				output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;
			}

			_delay_element_2 = _delay_element_1;
			_delay_element_1 = delay_element_0;
		}

		// don't allow bad values to propagate via the filter
		if (!PX4_ISFINITE(output)) {
			reset(samples[num_samples - 1]);
			output = samples[num_samples - 1];
		}

		// return the value. Should be no need to check limits
		return output;
	}

};

} // namespace math
