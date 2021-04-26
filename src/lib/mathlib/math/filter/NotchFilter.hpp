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

/*
 * @file NotchFilter.hpp
 *
 * @brief Implementation of a Notch filter.
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 * @author Samuel Garcin <samuel.garcin@wecorpindustries.com>
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <cmath>
#include <float.h>
#include <matrix/math.hpp>

namespace math
{

inline bool isFinite(const float &value)
{
	return PX4_ISFINITE(value);
}

inline bool isFinite(const matrix::Vector3f &value)
{
	return PX4_ISFINITE(value(0)) && PX4_ISFINITE(value(1)) && PX4_ISFINITE(value(2));
}

template<typename T>
class NotchFilter
{
public:
	NotchFilter() = default;
	~NotchFilter() = default;


	void setParameters(float sample_freq, float notch_freq, float bandwidth);

	/**
	 * Add a new raw value to the filter using the Direct form II
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T &sample)
	{
		// Direct Form II implementation
		const T delay_element_0{sample - _delay_element_1 *_a1 - _delay_element_2 * _a2};
		const T output{delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2};

		_delay_element_2 = _delay_element_1;
		_delay_element_1 = delay_element_0;

		return output;
	}

	/**
	 * Add a new raw value to the filter using the Direct Form I
	 *
	 * @return retrieve the filtered result
	 */
	inline T applyDF1(const T &sample)
	{
		// Direct Form I implementation
		const T output = _b0 * sample + _b1 * _delay_element_1 + _b2 * _delay_element_2 - _a1 * _delay_element_output_1 - _a2 *
				 _delay_element_output_2;

		// shift inputs
		_delay_element_2 = _delay_element_1;
		_delay_element_1 = sample;

		// shift outputs
		_delay_element_output_2 = _delay_element_output_1;
		_delay_element_output_1 = output;

		return output;
	}

	float getNotchFreq() const { return _notch_freq; }
	float getBandwidth() const { return _bandwidth; }

	// Used in unit test only
	void getCoefficients(float a[3], float b[3]) const
	{
		a[0] = 1.f;
		a[1] = _a1;
		a[2] = _a2;
		b[0] = _b0;
		b[1] = _b1;
		b[2] = _b2;
	}

	float getMagnitudeResponse(float frequency) const
	{
		float w = 2.f * M_PI_F * frequency / _sample_freq;

		float numerator = _b0 * _b0 + _b1 * _b1 + _b2 * _b2
				  + 2.f * (_b0 * _b1 + _b1 * _b2) * cosf(w) + 2.f * _b0 * _b2 * cosf(2.f * w);

		float denominator = 1.f + _a1 * _a1 + _a2 * _a2 + 2.f * (_a1 + _a1 * _a2) * cosf(w) + 2.f * _a2 * cosf(2.f * w);

		return sqrtf(numerator / denominator);
	}

	/**
	 * Bypasses the filter update to directly set different filter coefficients.
	 * Note: the filtered frequency and quality factor saved on the filter lose their
	 * physical meaning if you use this method to change the filter's coefficients.
	 * Used for creating clones of a specific filter.
	 */
	void setCoefficients(float a[2], float b[3])
	{
		_a1 = a[0];
		_a2 = a[1];
		_b0 = b[0];
		_b1 = b[1];
		_b2 = b[2];
	}


	T reset(const T &sample);

protected:
	float _notch_freq{};
	float _bandwidth{};
	float _sample_freq{};

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	float _a1{};
	float _a2{};

	float _b0{};
	float _b1{};
	float _b2{};

	T _delay_element_1;
	T _delay_element_2;
	T _delay_element_output_1;
	T _delay_element_output_2;
};

/**
 * Initialises the filter by setting its parameters and coefficients.
 * If using the direct form I (applyDF1) method, allows to dynamically
 * update the filtered frequency, refresh rate and quality factor while
 * conserving the filter's history
 */
template<typename T>
void NotchFilter<T>::setParameters(float sample_freq, float notch_freq, float bandwidth)
{
	_notch_freq = notch_freq;
	_bandwidth = bandwidth;
	_sample_freq = sample_freq;

	if (notch_freq <= 0.f) {
		// no filtering
		_b0 = 1.0f;
		_b1 = 0.0f;
		_b2 = 0.0f;

		_a1 = 0.0f;
		_a2 = 0.0f;

		return;
	}

	const float alpha = tanf(M_PI_F * bandwidth / sample_freq);
	const float beta = -cosf(2.f * M_PI_F * notch_freq / sample_freq);
	const float a0_inv = 1.f / (alpha + 1.f);

	_b0 = a0_inv;
	_b1 = 2.f * beta * a0_inv;
	_b2 = a0_inv;

	_a1 = _b1;
	_a2 = (1.f - alpha) * a0_inv;
}

template<typename T>
T NotchFilter<T>::reset(const T &sample)
{
	T dval = sample;

	if (fabsf(_b0 + _b1 + _b2) > FLT_EPSILON) {
		dval = dval / (_b0 + _b1 + _b2);
	}

	_delay_element_1 = dval;
	_delay_element_2 = dval;
	_delay_element_output_1 = {};
	_delay_element_output_2 = {};

	return apply(sample);
}

} // namespace math
