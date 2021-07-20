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
 * @file Publication2.hpp
 *
 */

//#warning uORB::Publication2 is under development and not intended for regular usage

#pragma once

#include <px4_platform_common/defines.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include "uORBDeviceNode.hpp"
#include <uORB/topics/uORBTopics.hpp>

namespace uORB
{

class Publication2Base
{
public:
	bool advertised() const { return _handle != nullptr; }
	bool unadvertise() { return (DeviceNode::unadvertise(_handle) == PX4_OK); }

protected:
	Publication2Base() = default;
	~Publication2Base()
	{
		if (_handle != nullptr) {
			// don't automatically unadvertise queued publications (eg vehicle_command)
			if (static_cast<DeviceNode *>(_handle)->get_queue_size() == 1) {
				unadvertise();
			}
		}
	}

	orb_advert_t _handle{nullptr};
};

/**
 * uORB publication wrapper class
 */
template<ORB_ID T, uint8_t ORB_QSIZE = 1>
class Publication2 : public Publication2Base
{
public:
	Publication2() = default;

	bool advertise()
	{
		if (!advertised()) {
			_handle = orb_advertise_queue(get_orb_meta(T), nullptr, ORB_QSIZE);
		}

		return advertised();
	}

	using S = typename ORBTypeMap<T>::type;

	/**
	 * Publish the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool publish(const S &data)
	{
		if (!advertised()) {
			advertise();
		}

		return (DeviceNode::publish(get_orb_meta(T), _handle, &data) == PX4_OK);
	}
};

} // namespace uORB
