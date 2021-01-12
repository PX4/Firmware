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

#ifndef POSITION_TARGET_LOCAL_NED_HPP
#define POSITION_TARGET_LOCAL_NED_HPP

#include <uORB/topics/vehicle_local_position_setpoint.h>

class MavlinkStreamPositionTargetLocalNed : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamPositionTargetLocalNed(mavlink); }

	static constexpr const char *get_name_static() { return "POSITION_TARGET_LOCAL_NED"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _pos_sp_sub.advertised() ? MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamPositionTargetLocalNed(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _pos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};

	bool send() override
	{
		vehicle_local_position_setpoint_s pos_sp;

		if (_pos_sp_sub.update(&pos_sp)) {
			mavlink_position_target_local_ned_t msg{};

			msg.time_boot_ms = pos_sp.timestamp / 1000;
			msg.coordinate_frame = MAV_FRAME_LOCAL_NED;

			// position
			if (!PX4_ISFINITE(pos_sp.x)) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_X_IGNORE;
			}

			if (!PX4_ISFINITE(pos_sp.y)) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_Y_IGNORE;
			}

			if (!PX4_ISFINITE(pos_sp.z)) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_Z_IGNORE;
			}

			// velocity
			if (!PX4_ISFINITE(pos_sp.vx)) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_VX_IGNORE;
			}

			if (!PX4_ISFINITE(pos_sp.vy)) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_VY_IGNORE;
			}

			if (!PX4_ISFINITE(pos_sp.vz)) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_VZ_IGNORE;
			}

			// acceleration
			if (!PX4_ISFINITE(pos_sp.acceleration[0])) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_AX_IGNORE;
			}

			if (!PX4_ISFINITE(pos_sp.acceleration[1])) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_AY_IGNORE;
			}

			if (!PX4_ISFINITE(pos_sp.acceleration[2])) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_AZ_IGNORE;
			}

			// yaw
			if (!PX4_ISFINITE(pos_sp.yaw)) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;
			}

			// yaw rate
			if (!PX4_ISFINITE(pos_sp.yawspeed)) {
				msg.type_mask |= POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
			}

			msg.x = pos_sp.x;
			msg.y = pos_sp.y;
			msg.z = pos_sp.z;
			msg.vx = pos_sp.vx;
			msg.vy = pos_sp.vy;
			msg.vz = pos_sp.vz;
			msg.afx = pos_sp.acceleration[0];
			msg.afy = pos_sp.acceleration[1];
			msg.afz = pos_sp.acceleration[2];
			msg.yaw = pos_sp.yaw;
			msg.yaw_rate = pos_sp.yawspeed;

			mavlink_msg_position_target_local_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // POSITION_TARGET_LOCAL_NED
