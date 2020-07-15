/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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

/// @file	LowPassFilter2pVector3f.hpp
/// @brief	A class to implement a second order low pass filter on a Vector3f
/// Based on LowPassFilter2p.hpp by Leonard Hall <LeonardTHall@gmail.com>

#pragma once

#include <matrix/math.hpp>

namespace math
{
class LowPassFilter2pVector3f
{
public:

	LowPassFilter2pVector3f(float sample_freq, float cutoff_freq)
	{
		// set initial parameters
		set_cutoff_frequency(sample_freq, cutoff_freq);
	}

	// Change filter parameters
	void set_cutoff_frequency(float sample_freq, float cutoff_freq);

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	inline matrix::Vector3f apply(const matrix::Vector3f &sample)
	{
		// do the filtering
		const matrix::Vector3f delay_element_0{sample - _delay_element_1 *_a1 - _delay_element_2 * _a2};
		const matrix::Vector3f output{delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2};

		_delay_element_2 = _delay_element_1;
		_delay_element_1 = delay_element_0;

		return output;
	}

	// Return the cutoff frequency
	float get_cutoff_freq() const { return _cutoff_freq; }

	// Return the sample frequency
	float get_sample_freq() const { return _sample_freq; }

	// Reset the filter state to this value
	matrix::Vector3f reset(const matrix::Vector3f &sample);

private:

	float _cutoff_freq{0.0f};
	float _sample_freq{0.0f};

	float _a1{0.0f};
	float _a2{0.0f};

	float _b0{0.0f};
	float _b1{0.0f};
	float _b2{0.0f};

	matrix::Vector3f _delay_element_1{0.0f, 0.0f, 0.0f};	// buffered sample -1
	matrix::Vector3f _delay_element_2{0.0f, 0.0f, 0.0f};	// buffered sample -2
};

} // namespace math
