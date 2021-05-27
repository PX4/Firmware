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
 * @file Publication.hpp
 *
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include "uORBManager.hpp"
#include <uORB/topics/uORBTopics.hpp>

namespace uORB
{

template <typename U> class DefaultQueueSize
{
private:
	template <typename T>
	static constexpr uint8_t get_queue_size(decltype(T::ORB_QUEUE_LENGTH) *)
	{
		return T::ORB_QUEUE_LENGTH;
	}

	template <typename T> static constexpr uint8_t get_queue_size(...)
	{
		return 1;
	}

public:
	static constexpr unsigned value = get_queue_size<U>(nullptr);
};

class PublicationBase
{
public:

	bool advertised() const { return _handle != nullptr; }

	bool unadvertise() { return (Manager::orb_unadvertise(_handle) == PX4_OK); }

	orb_id_t get_topic() const { return get_orb_meta(_orb_id); }

protected:

	PublicationBase(ORB_ID id) : _orb_id(id) {}

	~PublicationBase()
	{
		if (_handle != nullptr) {
			// don't automatically unadvertise queued publications (eg vehicle_command)
			if (Manager::orb_get_queue_size(_handle) == 1) {
				unadvertise();
			}
		}
	}

	orb_advert_t _handle{nullptr};
	const ORB_ID _orb_id;
};

/**
 * uORB publication wrapper class
 */
template<typename T, uint8_t ORB_QSIZE = DefaultQueueSize<T>::value>
class Publication : public PublicationBase
{
public:

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 */
	Publication(ORB_ID id) : PublicationBase(id) {}
	Publication(const orb_metadata *meta) : PublicationBase(static_cast<ORB_ID>(meta->o_id)) {}

	bool advertise()
	{
		if (!advertised()) {
			_handle = orb_advertise_queue(get_topic(), nullptr, ORB_QSIZE);
		}

		return advertised();
	}

	/**
	 * Publish the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool publish(const T &data)
	{
		if (!advertised()) {
			advertise();
		}

		return (Manager::orb_publish(get_topic(), _handle, &data) == PX4_OK);
	}
};

/**
 * The publication class with data embedded.
 */
template<typename T>
class PublicationData : public Publication<T>
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 */
	PublicationData(ORB_ID id) : Publication<T>(id) {}
	PublicationData(const orb_metadata *meta) : Publication<T>(meta) {}

	T	&get() { return _data; }
	void	set(const T &data) { _data = data; }

	// Publishes the embedded struct.
	bool	update() { return Publication<T>::publish(_data); }
	bool	update(const T &data)
	{
		_data = data;
		return Publication<T>::publish(_data);
	}

private:
	T _data{};
};

} // namespace uORB
