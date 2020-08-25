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
	 * Add a new raw value to the filter
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

	T reset(const T &sample);

protected:
	float _notch_freq{};
	float _bandwidth{};

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	float _a1{};
	float _a2{};

	float _b0{};
	float _b1{};
	float _b2{};

	T _delay_element_1;
	T _delay_element_2;
};

template<typename T>
void NotchFilter<T>::setParameters(float sample_freq, float notch_freq, float bandwidth)
{
	_notch_freq = notch_freq;
	_bandwidth = bandwidth;

	_delay_element_1 = {};
	_delay_element_2 = {};

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

	return apply(sample);
}

} // namespace math
