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

#ifndef GIMBAL_MANAGER_INFORMATION_HPP
#define GIMBAL_MANAGER_INFORMATION_HPP

#include <uORB/topics/gimbal_manager_information.h>

class MavlinkStreamGimbalManagerInformation : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGimbalManagerInformation(mavlink); }

	static constexpr const char *get_name_static() { return "GIMBAL_MANAGER_INFORMATION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_gimbal_manager_information_sub.advertised()) {
			return MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamGimbalManagerInformation(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _gimbal_manager_information_sub{ORB_ID(gimbal_manager_information)};

	bool send() override
	{
		gimbal_manager_information_s gimbal_manager_information;

		if (_gimbal_manager_information_sub.advertised() && _gimbal_manager_information_sub.copy(&gimbal_manager_information)) {
			// send out gimbal_manager_info with info from gimbal_manager_information
			mavlink_gimbal_manager_information_t msg{};

			msg.time_boot_ms = gimbal_manager_information.timestamp / 1000;
			msg.cap_flags = gimbal_manager_information.cap_flags;
			msg.gimbal_device_id = 0;
			msg.roll_min = gimbal_manager_information.roll_min;
			msg.roll_max = gimbal_manager_information.roll_max;
			msg.pitch_min = gimbal_manager_information.pitch_min;
			msg.pitch_max = gimbal_manager_information.pitch_max;
			msg.yaw_min = gimbal_manager_information.yaw_min;
			msg.yaw_max = gimbal_manager_information.yaw_max;

			mavlink_msg_gimbal_manager_information_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GIMBAL_MANAGER_INFORMATION_HPP
