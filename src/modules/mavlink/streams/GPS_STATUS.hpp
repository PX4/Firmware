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

#ifndef GPS_STATUS_HPP
#define GPS_STATUS_HPP

#include <uORB/topics/satellite_info.h>

class MavlinkStreamGPSStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGPSStatus(mavlink); }

	static constexpr const char *get_name_static() { return "GPS_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GPS_STATUS; }

	const char *get_name() const override { return MavlinkStreamGPSStatus::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _satellite_info_sub.advertised() ? (MAVLINK_MSG_ID_GPS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamGPSStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _satellite_info_sub{ORB_ID(satellite_info)};

	bool send() override
	{
		satellite_info_s sat;

		if (_satellite_info_sub.update(&sat)) {
			mavlink_gps_status_t msg{};

			msg.satellites_visible = sat.count;

			size_t sat_count = math::min(static_cast<size_t>(sat.count),
						     sizeof(msg.satellite_used) / sizeof(msg.satellite_used[0]));

			for (size_t i = 0; i < sat_count; i++) {
				msg.satellite_used[i]      = sat.used[i];
				msg.satellite_elevation[i] = sat.elevation[i];
				msg.satellite_azimuth[i]   = sat.azimuth[i];
				msg.satellite_snr[i]       = sat.snr[i];
				msg.satellite_prn[i]       = sat.prn[i];
			}

			mavlink_msg_gps_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GPS_STATUS_HPP
