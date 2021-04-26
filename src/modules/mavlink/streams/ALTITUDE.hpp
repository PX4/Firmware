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

#ifndef ALTITUDE_HPP
#define ALTITUDE_HPP

#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/wind.h>

class MavlinkStreamAltitude : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAltitude(mavlink); }

	static constexpr const char *get_name_static() { return "ALTITUDE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ALTITUDE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _local_pos_sub.advertised() ? MAVLINK_MSG_ID_ALTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamAltitude(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};

	bool send() override
	{
		mavlink_altitude_t msg{};

		msg.altitude_monotonic = NAN;
		msg.altitude_amsl = NAN;
		msg.altitude_local = NAN;
		msg.altitude_relative = NAN;
		msg.altitude_terrain = NAN;
		msg.bottom_clearance = NAN;

		// always update monotonic altitude
		bool air_data_updated = false;
		vehicle_air_data_s air_data{};
		_air_data_sub.copy(&air_data);

		if (air_data.timestamp > 0) {
			msg.altitude_monotonic = air_data.baro_alt_meter;

			air_data_updated = true;
		}

		bool lpos_updated = false;

		vehicle_local_position_s local_pos{};

		if (_local_pos_sub.copy(&local_pos)) {

			if (local_pos.z_valid) {
				if (local_pos.z_global) {
					msg.altitude_amsl = -local_pos.z + local_pos.ref_alt;

				} else {
					msg.altitude_amsl = msg.altitude_monotonic;
				}

				msg.altitude_local = -local_pos.z;

				home_position_s home{};
				_home_sub.copy(&home);

				if (home.valid_alt) {
					msg.altitude_relative = -(local_pos.z - home.z);

				} else {
					msg.altitude_relative = -local_pos.z;
				}

				if (local_pos.dist_bottom_valid) {
					msg.altitude_terrain = -local_pos.z - local_pos.dist_bottom;
					msg.bottom_clearance = local_pos.dist_bottom;
				}
			}

			lpos_updated = true;
		}

		// local position timeout after 10 ms
		// avoid publishing only baro altitude_monotonic if possible
		bool lpos_timeout = (hrt_elapsed_time(&local_pos.timestamp) > 10_ms);

		if (lpos_updated || (air_data_updated && lpos_timeout)) {
			msg.time_usec = hrt_absolute_time();
			mavlink_msg_altitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // ALTITUDE_HPP
