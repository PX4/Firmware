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

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <uORB/topics/vehicle_odometry.h>

class MavlinkStreamOdometry : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOdometry(mavlink); }

	static constexpr const char *get_name_static() { return "ODOMETRY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ODOMETRY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_mavlink->odometry_loopback_enabled()) {
			return _vodom_sub.advertised() ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;

		} else {
			return _odom_sub.advertised() ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
		}
	}

private:
	explicit MavlinkStreamOdometry(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _odom_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _vodom_sub{ORB_ID(vehicle_visual_odometry)};

	bool send() override
	{
		vehicle_odometry_s odom;
		// check if it is to send visual odometry loopback or not
		bool odom_updated = false;

		mavlink_odometry_t msg{};

		if (_mavlink->odometry_loopback_enabled()) {
			odom_updated = _vodom_sub.update(&odom);

			// set the frame_id according to the local frame of the data
			if (odom.local_frame == vehicle_odometry_s::LOCAL_FRAME_NED) {
				msg.frame_id = MAV_FRAME_LOCAL_NED;

			} else {
				msg.frame_id = MAV_FRAME_LOCAL_FRD;
			}

			// source: external vision system
			msg.estimator_type = MAV_ESTIMATOR_TYPE_VISION;

		} else {
			odom_updated = _odom_sub.update(&odom);

			msg.frame_id = MAV_FRAME_LOCAL_NED;

			// source: PX4 estimator
			msg.estimator_type = MAV_ESTIMATOR_TYPE_AUTOPILOT;
		}

		if (odom_updated) {
			msg.time_usec = odom.timestamp_sample;
			msg.child_frame_id = MAV_FRAME_BODY_FRD;

			// Current position
			msg.x = odom.x;
			msg.y = odom.y;
			msg.z = odom.z;

			// Current orientation
			msg.q[0] = odom.q[0];
			msg.q[1] = odom.q[1];
			msg.q[2] = odom.q[2];
			msg.q[3] = odom.q[3];

			switch (odom.velocity_frame) {
			case vehicle_odometry_s::BODY_FRAME_FRD:
				msg.vx = odom.vx;
				msg.vy = odom.vy;
				msg.vz = odom.vz;
				break;

			case vehicle_odometry_s::LOCAL_FRAME_FRD:
			case vehicle_odometry_s::LOCAL_FRAME_NED:
				// Body frame to local frame
				const matrix::Dcmf R_body_to_local(matrix::Quatf(odom.q));

				// Rotate linear velocity from local to body frame
				const matrix::Vector3f linvel_body(R_body_to_local.transpose() *
								   matrix::Vector3f(odom.vx, odom.vy, odom.vz));

				msg.vx = linvel_body(0);
				msg.vy = linvel_body(1);
				msg.vz = linvel_body(2);
				break;
			}

			// Current body rates
			msg.rollspeed = odom.rollspeed;
			msg.pitchspeed = odom.pitchspeed;
			msg.yawspeed = odom.yawspeed;

			// get the covariance matrix size

			// pose_covariance
			static constexpr size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);
			static_assert(POS_URT_SIZE == (sizeof(msg.pose_covariance) / sizeof(msg.pose_covariance[0])),
				      "Odometry Pose Covariance matrix URT array size mismatch");

			// velocity_covariance
			static constexpr size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);
			static_assert(VEL_URT_SIZE == (sizeof(msg.velocity_covariance) / sizeof(msg.velocity_covariance[0])),
				      "Odometry Velocity Covariance matrix URT array size mismatch");

			// copy pose covariances
			for (size_t i = 0; i < POS_URT_SIZE; i++) {
				msg.pose_covariance[i] = odom.pose_covariance[i];
			}

			// copy velocity covariances
			//TODO: Apply rotation matrix to transform from body-fixed NED to earth-fixed NED frame
			for (size_t i = 0; i < VEL_URT_SIZE; i++) {
				msg.velocity_covariance[i] = odom.velocity_covariance[i];
			}

			mavlink_msg_odometry_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // ODOMETRY_HPP
