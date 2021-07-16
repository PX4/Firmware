/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdint.h>

namespace px4
{

template <size_t N>
class Bitset
{
public:

	size_t count() const
	{
		size_t total = 0;

		for (auto x : _data) {
			while (x) {
				total += x & 1;
				x >>= 1;
			}
		}

		return total;
	}

	size_t size() const { return N; }

	bool operator[](size_t position) const
	{
		return _data[array_index(position)] & element_mask(position);
	}

	void set(size_t pos, bool val = true)
	{
		const uint8_t bitmask = element_mask(pos);

		if (val) {
			_data[array_index(pos)] |= bitmask;

		} else {
			_data[array_index(pos)] &= ~bitmask;
		}
	}

	void reset()
	{
		// set bits to false
		for (auto &d : _data) {
			d = 0;
		}
	}

private:
	static constexpr uint8_t BITS_PER_ELEMENT = 8;
	static constexpr size_t ARRAY_SIZE = (N % BITS_PER_ELEMENT == 0) ? N / BITS_PER_ELEMENT : N / BITS_PER_ELEMENT + 1;
	static constexpr size_t ALLOCATED_BITS = ARRAY_SIZE * BITS_PER_ELEMENT;

	size_t array_index(size_t position) const { return position / BITS_PER_ELEMENT; }
	uint8_t element_mask(size_t position) const { return (1 << position % BITS_PER_ELEMENT); }

	uint8_t _data[ARRAY_SIZE] {};
};

} // namespace px4
