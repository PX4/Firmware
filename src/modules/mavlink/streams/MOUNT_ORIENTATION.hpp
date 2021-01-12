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

#ifndef MOUNT_ORIENTATION_HPP
#define MOUNT_ORIENTATION_HPP

#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamMountOrientation : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMountOrientation(mavlink); }

	static constexpr const char *get_name_static() { return "MOUNT_ORIENTATION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MOUNT_ORIENTATION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _mount_orientation_sub.advertised() ? MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMountOrientation(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _mount_orientation_sub{ORB_ID(mount_orientation)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	bool send() override
	{
		mount_orientation_s mount_orientation;

		if (_mount_orientation_sub.update(&mount_orientation)) {
			mavlink_mount_orientation_t msg{};

			msg.roll = math::degrees(mount_orientation.attitude_euler_angle[0]);
			msg.pitch = math::degrees(mount_orientation.attitude_euler_angle[1]);
			msg.yaw = math::degrees(mount_orientation.attitude_euler_angle[2]);

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);
			msg.yaw_absolute = math::degrees(matrix::wrap_2pi(lpos.heading + mount_orientation.attitude_euler_angle[2]));

			mavlink_msg_mount_orientation_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // MOUNT_ORIENTATION
