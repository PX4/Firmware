/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file Subscription.hpp
 *
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/uORBTopics.hpp>

#include <px4_platform_common/defines.h>
#include <lib/mathlib/mathlib.h>

#include "uORBManager.hpp"
#include "uORBUtils.hpp"

namespace uORB
{

class SubscriptionCallback;

// Base subscription wrapper class
class Subscription
{
public:

	/**
	 * Constructor
	 *
	 * @param id The uORB ORB_ID enum for the topic.
	 * @param instance The instance for multi sub.
	 */
	Subscription(ORB_ID id, uint8_t instance = 0) :
		_orb_id(id),
		_instance(instance)
	{
		subscribe();
	}

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	Subscription(const orb_metadata *meta, uint8_t instance = 0) :
		_orb_id((meta == nullptr) ? ORB_ID::INVALID : static_cast<ORB_ID>(meta->o_id)),
		_instance(instance)
	{
		subscribe();
	}

	// Copy constructor
	Subscription(const Subscription &other) : _orb_id(other._orb_id), _instance(other._instance) {}

	// Move constructor
	Subscription(const Subscription &&other) noexcept : _orb_id(other._orb_id), _instance(other._instance) {}

	// copy assignment
	Subscription &operator=(const Subscription &other)
	{
		unsubscribe();
		_orb_id = other._orb_id;
		_instance = other._instance;
		return *this;
	}

	// move assignment
	Subscription &operator=(Subscription &&other) noexcept
	{
		unsubscribe();
		_orb_id = other._orb_id;
		_instance = other._instance;
		return *this;
	}

	~Subscription()
	{
		unsubscribe();
	}

	bool subscribe();
	void unsubscribe();

	bool valid() const { return _node != nullptr; }
	bool advertised()
	{
		if (valid()) {
			return Manager::is_advertised(_node);
		}

		// try to initialize
		if (subscribe()) {
			// check again if valid
			if (valid()) {
				return Manager::is_advertised(_node);
			}
		}

		return false;
	}

	/**
	 * Check if there is a new update.
	 */
	bool updated() { return advertised() && Manager::updates_available(_node, _last_generation); }

	/**
	 * Update the struct
	 * @param dst The uORB message struct we are updating.
	 */
	bool update(void *dst) { return updated() && Manager::orb_data_copy(_node, dst, _last_generation); }

	/**
	 * Copy the struct
	 * @param dst The uORB message struct we are updating.
	 */
	bool copy(void *dst) { return advertised() && Manager::orb_data_copy(_node, dst, _last_generation); }

	/**
	 * Change subscription instance
	 * @param instance The new multi-Subscription instance
	 */
	bool ChangeInstance(uint8_t instance);

	uint8_t  get_instance() const { return _instance; }
	unsigned get_last_generation() const { return _last_generation; }
	orb_id_t get_topic() const { return get_orb_meta(_orb_id); }

protected:

	friend class SubscriptionCallback;
	friend class SubscriptionCallbackWorkItem;

	void *get_node() { return _node; }

	void *_node{nullptr};

	unsigned _last_generation{0}; /**< last generation the subscriber has seen */

	ORB_ID _orb_id{ORB_ID::INVALID};
	uint8_t _instance{0};
};

// Subscription wrapper class with data
template<class T>
class SubscriptionData : public Subscription
{
public:
	/**
	 * Constructor
	 *
	 * @param id The uORB metadata ORB_ID enum for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionData(ORB_ID id, uint8_t instance = 0) :
		Subscription(id, instance)
	{
		copy(&_data);
	}

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionData(const orb_metadata *meta, uint8_t instance = 0) :
		Subscription(meta, instance)
	{
		copy(&_data);
	}

	~SubscriptionData() = default;

	// no copy, assignment, move, move assignment
	SubscriptionData(const SubscriptionData &) = delete;
	SubscriptionData &operator=(const SubscriptionData &) = delete;
	SubscriptionData(SubscriptionData &&) = delete;
	SubscriptionData &operator=(SubscriptionData &&) = delete;

	// update the embedded struct.
	bool update() { return Subscription::update((void *)(&_data)); }

	const T &get() const { return _data; }

private:

	T _data{};
};

} // namespace uORB
