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

#ifndef VIBRATION_HPP
#define VIBRATION_HPP

#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_imu_status.h>

class MavlinkStreamVibration : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamVibration(mavlink); }

	static constexpr const char *get_name_static() { return "VIBRATION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_VIBRATION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_sensor_selection_sub.advertised() && _vehicle_imu_status_subs.advertised()) {
			return MAVLINK_MSG_ID_VIBRATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamVibration(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::SubscriptionMultiArray<vehicle_imu_status_s, 3> _vehicle_imu_status_subs{ORB_ID::vehicle_imu_status};

	bool send() override
	{
		if (_vehicle_imu_status_subs.updated()) {
			mavlink_vibration_t msg{};
			msg.time_usec = hrt_absolute_time();

			// VIBRATION usage not to mavlink spec, this is our current usage.
			//  vibration_x : Primary gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
			//  vibration_y : Primary gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
			//  vibration_z : Primary accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)

			sensor_selection_s sensor_selection{};
			_sensor_selection_sub.copy(&sensor_selection);

			// primary accel high frequency vibration metric
			if (sensor_selection.accel_device_id != 0) {
				for (auto &x : _vehicle_imu_status_subs) {
					vehicle_imu_status_s status;

					if (x.copy(&status)) {
						if (status.accel_device_id == sensor_selection.accel_device_id) {
							msg.vibration_x = status.gyro_coning_vibration;
							msg.vibration_y = status.gyro_vibration_metric;
							msg.vibration_z = status.accel_vibration_metric;
							break;
						}
					}
				}
			}

			// accel 0, 1, 2 cumulative clipping
			for (int i = 0; i < math::min(static_cast<uint8_t>(3), _vehicle_imu_status_subs.size()); i++) {
				vehicle_imu_status_s status;

				if (_vehicle_imu_status_subs[i].copy(&status)) {

					const uint32_t clipping = status.accel_clipping[0] + status.accel_clipping[1] + status.accel_clipping[2];

					switch (i) {
					case 0:
						msg.clipping_0 = clipping;
						break;

					case 1:
						msg.clipping_1 = clipping;
						break;

					case 2:
						msg.clipping_2 = clipping;
						break;
					}
				}
			}

			mavlink_msg_vibration_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // VIBRATION_HPP
