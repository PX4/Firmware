/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef HOME_POSITION_HPP
#define HOME_POSITION_HPP

#include <uORB/topics/home_position.h>

class MavlinkStreamHomePosition : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHomePosition(mavlink); }

	static constexpr const char *get_name_static() { return "HOME_POSITION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HOME_POSITION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _home_sub.advertised() ? (MAVLINK_MSG_ID_HOME_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamHomePosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _home_sub{ORB_ID(home_position)};

	bool send() override
	{
		// we're sending the GPS home periodically to ensure the
		// the GCS does pick it up at one point
		home_position_s home;

		if (_home_sub.advertised() && _home_sub.copy(&home)) {
			if (home.valid_hpos) {
				mavlink_home_position_t msg{};

				msg.latitude  = home.lat * 1e7;
				msg.longitude = home.lon * 1e7;
				msg.altitude  = home.alt * 1e3f;

				msg.x = home.x;
				msg.y = home.y;
				msg.z = home.z;

				matrix::Quatf q(matrix::Eulerf(0.f, 0.f, home.yaw));
				q.copyTo(msg.q);

				msg.approach_x = 0.f;
				msg.approach_y = 0.f;
				msg.approach_z = 0.f;

				msg.time_usec = home.timestamp;

				mavlink_msg_home_position_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

#endif // HOME_POSITION_HPP
