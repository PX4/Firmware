/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file SubscriptionInterval.hpp
 *
 */

#pragma once

#include <uORB/uORB.h>
#include <px4_platform_common/defines.h>

#include "uORBDeviceNode.hpp"
#include "uORBManager.hpp"
#include "uORBUtils.hpp"

#include "Subscription.hpp"

#include <mathlib/mathlib.h>

namespace uORB
{

// Base subscription wrapper class
class SubscriptionInterval
{
public:

	/**
	 * Constructor
	 *
	 * @param id The uORB ORB_ID enum for the topic.
	 * @param interval The requested maximum update interval in microseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionInterval(ORB_ID id, uint32_t interval_us = 0, uint8_t instance = 0) :
		_subscription{id, instance},
		_interval_us(interval_us)
	{}

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval The requested maximum update interval in microseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionInterval(const orb_metadata *meta, uint32_t interval_us = 0, uint8_t instance = 0) :
		_subscription{meta, instance},
		_interval_us(interval_us)
	{}

	SubscriptionInterval() : _subscription{nullptr} {}

	~SubscriptionInterval() = default;

	bool subscribe() { return _subscription.subscribe(); }
	void unsubscribe() { _subscription.unsubscribe(); }

	bool advertised() { return _subscription.advertised(); }

	/**
	 * Check if there is a new update.
	 * */
	bool updated()
	{
		if (advertised() && (hrt_elapsed_time(&_last_update) >= _interval_us)) {
			return _subscription.updated();
		}

		return false;
	}

	/**
	 * Copy the struct if updated.
	 * @param dst The destination pointer where the struct will be copied.
	 * @return true only if topic was updated and copied successfully.
	 */
	bool update(void *dst)
	{
		if (updated()) {
			return copy(dst);
		}

		return false;
	}

	/**
	 * Copy the struct
	 * @param dst The destination pointer where the struct will be copied.
	 * @return true only if topic was copied successfully.
	 */
	bool copy(void *dst)
	{
		if (_subscription.copy(dst)) {
			const hrt_abstime now = hrt_absolute_time();
			// shift last update time forward, but don't let it get further behind than the interval
			_last_update = math::constrain(_last_update + _interval_us, now - _interval_us, now);
			return true;
		}

		return false;
	}

	bool		valid() const { return _subscription.valid(); }

	uint8_t		get_instance() const { return _subscription.get_instance(); }
	uint32_t        get_interval_us() const { return _interval_us; }
	unsigned	get_last_generation() const { return _subscription.get_last_generation(); }
	orb_id_t	get_topic() const { return _subscription.get_topic(); }

	/**
	 * Set the interval in microseconds
	 * @param interval The interval in microseconds.
	 */
	void		set_interval_us(uint32_t interval) { _interval_us = interval; }

	/**
	 * Set the interval in milliseconds
	 * @param interval The interval in milliseconds.
	 */
	void		set_interval_ms(uint32_t interval) { _interval_us = interval * 1000; }

protected:

	Subscription	_subscription;
	uint64_t	_last_update{0};	// last update in microseconds
	uint32_t	_interval_us{0};	// maximum update interval in microseconds

};

} // namespace uORB
