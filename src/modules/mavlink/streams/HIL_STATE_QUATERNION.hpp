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

#ifndef HIL_STATE_QUATERNION_HPP
#define HIL_STATE_QUATERNION_HPP

class MavlinkStreamHILStateQuaternion : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHILStateQuaternion(mavlink); }

	static constexpr const char *get_name_static() { return "HIL_STATE_QUATERNION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HIL_STATE_QUATERNION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_att_sub.advertised() || _gpos_sub.advertised()) {
			return MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamHILStateQuaternion(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity_groundtruth)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position_groundtruth)};

	bool send() override
	{
		if (_angular_velocity_sub.updated() || _att_sub.updated() || _gpos_sub.updated() || _lpos_sub.updated()) {
			vehicle_attitude_s att{};
			_att_sub.copy(&att);

			vehicle_global_position_s gpos{};
			_gpos_sub.copy(&gpos);

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);

			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			mavlink_hil_state_quaternion_t msg{};

			// vehicle_attitude -> hil_state_quaternion
			msg.attitude_quaternion[0] = att.q[0];
			msg.attitude_quaternion[1] = att.q[1];
			msg.attitude_quaternion[2] = att.q[2];
			msg.attitude_quaternion[3] = att.q[3];
			msg.rollspeed = angular_velocity.xyz[0];
			msg.pitchspeed = angular_velocity.xyz[1];
			msg.yawspeed = angular_velocity.xyz[2];

			// vehicle_global_position -> hil_state_quaternion
			// same units as defined in mavlink/common.xml
			msg.lat = gpos.lat * 1e7;
			msg.lon = gpos.lon * 1e7;
			msg.alt = gpos.alt * 1e3f;
			msg.vx = lpos.vx * 1e2f;
			msg.vy = lpos.vy * 1e2f;
			msg.vz = lpos.vz * 1e2f;
			msg.ind_airspeed = 0;
			msg.true_airspeed = 0;
			msg.xacc = lpos.ax;
			msg.yacc = lpos.ay;
			msg.zacc = lpos.az;

			mavlink_msg_hil_state_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // HIL_STATE_QUATERNION_HPP
