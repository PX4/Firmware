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

#ifndef GPS_GLOBAL_ORIGIN_HPP
#define GPS_GLOBAL_ORIGIN_HPP

#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamGpsGlobalOrigin : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGpsGlobalOrigin::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GPS_GLOBAL_ORIGIN";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGpsGlobalOrigin(mavlink);
	}

	unsigned get_size() override
	{
		return _vehicle_local_position_sub.advertised() ?
		       (MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	/* do not allow top copying this class */
	MavlinkStreamGpsGlobalOrigin(MavlinkStreamGpsGlobalOrigin &) = delete;
	MavlinkStreamGpsGlobalOrigin &operator = (const MavlinkStreamGpsGlobalOrigin &) = delete;

protected:
	explicit MavlinkStreamGpsGlobalOrigin(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		vehicle_local_position_s vehicle_local_position;

		if (_vehicle_local_position_sub.copy(&vehicle_local_position)
		    && vehicle_local_position.xy_global && vehicle_local_position.z_global) {
			mavlink_gps_global_origin_t msg{};
			msg.latitude = static_cast<int32_t>(vehicle_local_position.ref_lat * 1e7); // double degree -> int32 degreeE7
			msg.longitude = static_cast<int32_t>(vehicle_local_position.ref_lon * 1e7); // double degree -> int32 degreeE7
			msg.altitude = static_cast<int32_t>(vehicle_local_position.ref_alt * 1e3f); // float m -> int32 mm
			msg.time_usec = vehicle_local_position.timestamp; // int64 time since system boot
			mavlink_msg_gps_global_origin_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // GPS_GLOBAL_ORIGIN_HPP
