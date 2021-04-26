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

#ifndef AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_HPP
#define AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_HPP

#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamAutopilotStateForGimbalDevice : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAutopilotStateForGimbalDevice(mavlink); }

	static constexpr const char *get_name_static() { return "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_att_sub.advertised()) {
			return MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamAutopilotStateForGimbalDevice(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _landed_sub{ORB_ID(vehicle_land_detected)};

	bool send() override
	{
		vehicle_attitude_s att;

		if (_att_sub.update(&att)) {
			mavlink_autopilot_state_for_gimbal_device_t msg{};

			//msg.target_system = 0; // TODO
			//msg.target_component = 0; // TODO

			msg.time_boot_us = att.timestamp;
			msg.q[0] = att.q[0];
			msg.q[1] = att.q[1];
			msg.q[2] = att.q[2];
			msg.q[3] = att.q[3];
			msg.q_estimated_delay_us = 0; // I don't know.

			{
				vehicle_local_position_s lpos;

				if (_lpos_sub.copy(&lpos)) {
					msg.vx = lpos.vx;
					msg.vy = lpos.vy;
					msg.vz = lpos.vz;
					msg.v_estimated_delay_us = 0; // I don't know.
				}
			}

			{
				vehicle_attitude_setpoint_s att_sp;

				if (_att_sp_sub.copy(&att_sp)) {
					msg.feed_forward_angular_velocity_z = att_sp.yaw_sp_move_rate;
				}
			}

			{
				estimator_selector_status_s estimator_selector_status;

				if (_estimator_selector_status_sub.update(&estimator_selector_status)) {
					_estimator_status_sub.ChangeInstance(estimator_selector_status.primary_instance);

				}

				estimator_status_s est;

				if (_estimator_status_sub.copy(&est)) {
					msg.estimator_status = est.solution_status_flags;
				}
			}

			{
				vehicle_land_detected_s land_detected;

				if (_landed_sub.copy(&land_detected)) {
					// Ignore take-off and landing states for now.
					msg.landed_state = land_detected.landed ? MAV_LANDED_STATE_ON_GROUND : MAV_LANDED_STATE_IN_AIR;
				}
			}

			mavlink_msg_autopilot_state_for_gimbal_device_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_HPP
