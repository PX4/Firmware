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

#ifndef LOCAL_POSITION_NED_HPP
#define LOCAL_POSITION_NED_HPP

#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamLocalPositionNED : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamLocalPositionNED(mavlink); }

	static constexpr const char *get_name_static() { return "LOCAL_POSITION_NED"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_LOCAL_POSITION_NED; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _lpos_sub.advertised() ? MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamLocalPositionNED(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	bool send() override
	{
		vehicle_local_position_s lpos;

		if (_lpos_sub.update(&lpos)) {
			if (lpos.xy_valid && lpos.v_xy_valid) {
				mavlink_local_position_ned_t msg{};

				msg.time_boot_ms = lpos.timestamp / 1000;
				msg.x = lpos.x;
				msg.y = lpos.y;
				msg.z = lpos.z;
				msg.vx = lpos.vx;
				msg.vy = lpos.vy;
				msg.vz = lpos.vz;

				mavlink_msg_local_position_ned_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

#endif // LOCAL_POSITION_NED_HPP
