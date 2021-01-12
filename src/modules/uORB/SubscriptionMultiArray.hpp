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

/**
 * @file SubscriptionMultiArray.hpp
 *
 */

#pragma once

#include <uORB/uORB.h>

#include <px4_platform_common/defines.h>
#include <lib/mathlib/mathlib.h>

#include "SubscriptionInterval.hpp"

namespace uORB
{

/**
 * An array of uORB::Subscriptions of the same topic
 */
template<typename T, uint8_t SIZE = ORB_MULTI_MAX_INSTANCES>
class SubscriptionMultiArray
{
public:
	static_assert(SIZE <= ORB_MULTI_MAX_INSTANCES, "size must be <= uORB max instances");

	static constexpr uint8_t size() { return SIZE; }

	/**
	 * Constructor
	 *
	 * @param id The uORB ORB_ID enum for the topic.
	 */
	explicit SubscriptionMultiArray(ORB_ID id)
	{
		for (uint8_t i = 0; i < SIZE; i++) {
			_subscriptions[i] = SubscriptionInterval{id, 0, i};
			_subscriptions[i].subscribe();
		}
	}

	~SubscriptionMultiArray() = default;

	SubscriptionInterval &operator [](int i) { return _subscriptions[i]; }
	const SubscriptionInterval &operator [](int i) const { return _subscriptions[i]; }

	SubscriptionInterval *begin() { return _subscriptions; }
	SubscriptionInterval *end() { return _subscriptions + SIZE; }

	const SubscriptionInterval *begin() const { return _subscriptions; }
	const SubscriptionInterval *end() const { return _subscriptions + SIZE; }

	// true if any instance is advertised
	bool advertised()
	{
		for (auto &s : _subscriptions) {
			if (s.advertised()) {
				return true;
			}
		}

		return false;
	}

	// return the number of instances currently advertised
	uint8_t advertised_count()
	{
		uint8_t count = 0;

		for (auto &s : _subscriptions) {
			if (s.advertised()) {
				count++;
			}
		}

		return count;
	}

	// true if any instance is updated
	bool updated()
	{
		for (auto &s : _subscriptions) {
			if (s.updated()) {
				return true;
			}
		}

		return false;
	}

private:
	SubscriptionInterval _subscriptions[SIZE];
};

} // namespace uORB
