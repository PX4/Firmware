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

#ifndef FLIGHT_INFORMATION_HPP
#define FLIGHT_INFORMATION_HPP

#include <uORB/topics/vehicle_status.h>

class MavlinkStreamFlightInformation : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamFlightInformation(mavlink); }

	static constexpr const char *get_name_static() { return "FLIGHT_INFORMATION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_FLIGHT_INFORMATION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamFlightInformation(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
		_param_com_flight_uuid = param_find("COM_FLIGHT_UUID");
	}

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	param_t _param_com_flight_uuid{PARAM_INVALID};

	bool send() override
	{
		vehicle_status_s vehicle_status{};

		if (_vehicle_status_sub.copy(&vehicle_status) && vehicle_status.timestamp != 0) {
			mavlink_flight_information_t flight_info{};
			flight_info.time_boot_ms = hrt_absolute_time() / 1000;
			flight_info.arming_time_utc = vehicle_status.armed_time;
			flight_info.takeoff_time_utc = vehicle_status.takeoff_time;

			int32_t flight_uuid;

			if (param_get(_param_com_flight_uuid, &flight_uuid) == PX4_OK) {
				flight_info.flight_uuid = static_cast<uint64_t>(flight_uuid);
			}

			mavlink_msg_flight_information_send_struct(_mavlink->get_channel(), &flight_info);

			return true;
		}

		return false;
	}
};

#endif // FLIGHT_INFORMATION_HPP
