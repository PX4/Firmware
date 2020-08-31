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

#include "../mavlink_messages.h"

#include <uORB/topics/actuator_armed.h>

class MavlinkStreamFlightInformation : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamFlightInformation::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "FLIGHT_INFORMATION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_FLIGHT_INFORMATION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamFlightInformation(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
private:
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	param_t _param_com_flight_uuid;

	/* do not allow top copying this class */
	MavlinkStreamFlightInformation(MavlinkStreamFlightInformation &) = delete;
	MavlinkStreamFlightInformation &operator = (const MavlinkStreamFlightInformation &) = delete;
protected:
	explicit MavlinkStreamFlightInformation(Mavlink *mavlink) : MavlinkStream(mavlink)
	{

		_param_com_flight_uuid = param_find("COM_FLIGHT_UUID");
	}

	bool send(const hrt_abstime t) override
	{
		actuator_armed_s actuator_armed{};
		bool ret = _armed_sub.copy(&actuator_armed);

		if (ret && actuator_armed.timestamp != 0) {
			int32_t flight_uuid;
			param_get(_param_com_flight_uuid, &flight_uuid);

			mavlink_flight_information_t flight_info{};
			flight_info.flight_uuid = static_cast<uint64_t>(flight_uuid);
			flight_info.arming_time_utc = flight_info.takeoff_time_utc = actuator_armed.armed_time_ms;
			flight_info.time_boot_ms = hrt_absolute_time() / 1000;
			mavlink_msg_flight_information_send_struct(_mavlink->get_channel(), &flight_info);
		}

		return ret;
	}
};
