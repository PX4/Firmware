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
 * @file PublicationQueuedMulti.hpp
 *
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>

namespace uORB
{

/**
 * Queued publication with queue length set as a message constant (ORB_QUEUE_LENGTH)
 */
template<typename T>
class PublicationQueuedMulti
{
public:

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 */
	PublicationQueuedMulti(const orb_metadata *meta, uint8_t priority = ORB_PRIO_DEFAULT) :
		_meta(meta),
		_priority(priority)
	{}

	~PublicationQueuedMulti()
	{
		//orb_unadvertise(_handle);
	}

	/**
	 * Publish the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool publish(const T &data)
	{
		if (_handle != nullptr) {
			return (orb_publish(_meta, _handle, &data) == PX4_OK);

		} else {
			int instance = 0;
			orb_advert_t handle = orb_advertise_multi_queue(_meta, &data, &instance, _priority, T::ORB_QUEUE_LENGTH);

			if (handle != nullptr) {
				_handle = handle;
				return true;
			}
		}

		return false;
	}

protected:
	const orb_metadata *_meta;

	orb_advert_t _handle{nullptr};

	const uint8_t _priority;
};

} // namespace uORB
