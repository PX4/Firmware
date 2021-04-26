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
 * @file SubscriptionCallback.hpp
 *
 */

#pragma once

#include <uORB/SubscriptionInterval.hpp>
#include <containers/List.hpp>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

namespace uORB
{

// Subscription wrapper class with callbacks on new publications
class SubscriptionCallback : public SubscriptionInterval, public ListNode<SubscriptionCallback *>
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval_us The requested maximum update interval in microseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionCallback(const orb_metadata *meta, uint32_t interval_us = 0, uint8_t instance = 0) :
		SubscriptionInterval(meta, interval_us, instance)
	{
	}

	virtual ~SubscriptionCallback()
	{
		unregisterCallback();
	};

	bool registerCallback()
	{
		if (!_registered) {
			if (_subscription.get_node() && _subscription.get_node()->register_callback(this)) {
				// registered
				_registered = true;

			} else {
				// force topic creation by subscribing with old API
				int fd = orb_subscribe_multi(_subscription.get_topic(), _subscription.get_instance());

				// try to register callback again
				if (_subscription.subscribe()) {
					if (_subscription.get_node() && _subscription.get_node()->register_callback(this)) {
						_registered = true;
					}
				}

				orb_unsubscribe(fd);
			}
		}

		return _registered;
	}

	void unregisterCallback()
	{
		if (_subscription.get_node()) {
			_subscription.get_node()->unregister_callback(this);
		}

		_registered = false;
	}

	/**
	 * Change subscription instance
	 * @param instance The new multi-Subscription instance
	 */
	bool ChangeInstance(uint8_t instance)
	{
		bool ret = false;

		if (instance != get_instance()) {
			const bool registered = _registered;

			if (registered) {
				unregisterCallback();
			}

			if (_subscription.ChangeInstance(instance)) {
				ret = true;
			}

			if (registered) {
				registerCallback();
			}

		} else {
			// already on desired index
			return true;
		}

		return ret;
	}

	virtual void call() = 0;

	bool registered() const { return _registered; }

protected:

	bool _registered{false};

};

// Subscription with callback that schedules a WorkItem
class SubscriptionCallbackWorkItem : public SubscriptionCallback
{
public:
	/**
	 * Constructor
	 *
	 * @param work_item The WorkItem that will be scheduled immediately on new publications.
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionCallbackWorkItem(px4::WorkItem *work_item, const orb_metadata *meta, uint8_t instance = 0) :
		SubscriptionCallback(meta, 0, instance),	// interval 0
		_work_item(work_item)
	{
	}

	virtual ~SubscriptionCallbackWorkItem() = default;

	void call() override
	{
		// schedule immediately if updated (queue depth or subscription interval)
		if ((_required_updates == 0)
		    || (_subscription.get_node()->updates_available(_subscription.get_last_generation()) >= _required_updates)) {
			if (updated()) {
				_work_item->ScheduleNow();
			}
		}
	}

	/**
	 * Optionally limit callback until more samples are available.
	 *
	 * @param required_updates Number of queued updates required before a callback can be called.
	 */
	void set_required_updates(uint8_t required_updates)
	{
		// TODO: constrain to queue depth?
		_required_updates = required_updates;
	}

private:
	px4::WorkItem *_work_item;

	uint8_t _required_updates{0};
};

} // namespace uORB
