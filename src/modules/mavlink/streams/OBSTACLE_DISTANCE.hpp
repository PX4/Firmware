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

#ifndef OBSTACLE_DISTANCE_HPP
#define OBSTACLE_DISTANCE_HPP

#include <uORB/topics/obstacle_distance.h>

class MavlinkStreamObstacleDistance : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamObstacleDistance(mavlink); }

	static constexpr const char *get_name_static() { return "OBSTACLE_DISTANCE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_OBSTACLE_DISTANCE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _obstacle_distance_fused_sub.advertised() ? (MAVLINK_MSG_ID_OBSTACLE_DISTANCE_LEN +
				MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamObstacleDistance(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _obstacle_distance_fused_sub{ORB_ID(obstacle_distance_fused)};

	bool send() override
	{
		obstacle_distance_s obstacle_distance;

		if (_obstacle_distance_fused_sub.update(&obstacle_distance)) {
			mavlink_obstacle_distance_t msg{};

			msg.time_usec = obstacle_distance.timestamp;
			msg.sensor_type = obstacle_distance.sensor_type;
			memcpy(msg.distances, obstacle_distance.distances, sizeof(msg.distances));
			msg.increment = 0;
			msg.min_distance = obstacle_distance.min_distance;
			msg.max_distance = obstacle_distance.max_distance;
			msg.angle_offset = obstacle_distance.angle_offset;
			msg.increment_f = obstacle_distance.increment;
			msg.frame = obstacle_distance.frame;

			mavlink_msg_obstacle_distance_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // OBSTACLE_DISTANCE_HPP
