/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file WelfordMean.hpp
 *
 * Welford's online algorithm for computing mean and variance.
 */

#pragma once

namespace math
{

template<typename T>
class WelfordMean
{
public:
	// For a new value, compute the new count, new mean, the new M2.
	void update(const T &new_value)
	{
		if (_count == 0) {
			_mean = new_value;
		}

		_count++;

		// mean accumulates the mean of the entire dataset
		const T delta{new_value - _mean};
		_mean += delta / _count;

		// M2 aggregates the squared distance from the mean
		// count aggregates the number of samples seen so far
		_M2 += delta.emult(new_value - _mean);
	}

	bool valid() const { return _count > 2; }
	unsigned count() const { return _count; }

	void reset()
	{
		_count = 0;
		_mean = {};
		_M2 = {};
	}

	// Retrieve the mean, variance and sample variance
	T mean() const { return _mean; }
	T variance() const { return _M2 / _count; }
	T sample_variance() const { return _M2 / (_count - 1); }
private:
	T _mean{};
	T _M2{};
	unsigned _count{0};
};

} // namespace math
