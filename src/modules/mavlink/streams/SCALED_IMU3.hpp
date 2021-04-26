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

#ifndef SCALED_IMU3_HPP
#define SCALED_IMU3_HPP

#include <lib/ecl/geo/geo.h>
#include <lib/matrix/matrix/math.hpp>

#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_imu.h>

class MavlinkStreamScaledIMU3 : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamScaledIMU3(mavlink); }

	static constexpr const char *get_name_static() { return "SCALED_IMU3"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SCALED_IMU3; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_imu_sub.advertised() || _sensor_mag_sub.advertised()) {
			return MAVLINK_MSG_ID_SCALED_IMU3_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamScaledIMU3(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_imu_sub{ORB_ID(vehicle_imu), 2};
	uORB::Subscription _sensor_mag_sub{ORB_ID(sensor_mag), 2};

	bool send() override
	{
		if (_vehicle_imu_sub.updated() || _sensor_mag_sub.updated()) {
			mavlink_scaled_imu3_t msg{};

			vehicle_imu_s imu;

			if (_vehicle_imu_sub.copy(&imu)) {
				msg.time_boot_ms = imu.timestamp / 1000;

				// Accelerometer in mG
				const float accel_dt_inv = 1.e6f / (float)imu.delta_velocity_dt;
				const Vector3f accel = Vector3f{imu.delta_velocity} * accel_dt_inv * 1000.0f / CONSTANTS_ONE_G;
				msg.xacc = (int16_t)accel(0);
				msg.yacc = (int16_t)accel(1);
				msg.zacc = (int16_t)accel(2);

				// Gyroscope in mrad/s
				const float gyro_dt_inv = 1.e6f / (float)imu.delta_velocity_dt;
				const Vector3f gyro = Vector3f{imu.delta_angle} * gyro_dt_inv * 1000.0f;
				msg.xgyro = gyro(0);
				msg.ygyro = gyro(1);
				msg.zgyro = gyro(2);
			}

			sensor_mag_s sensor_mag;

			if (_sensor_mag_sub.copy(&sensor_mag)) {
				if (msg.time_boot_ms == 0) {
					msg.time_boot_ms = sensor_mag.timestamp / 1000;
				}

				msg.xmag = sensor_mag.x * 1000.0f; // Gauss -> MilliGauss
				msg.ymag = sensor_mag.y * 1000.0f; // Gauss -> MilliGauss
				msg.zmag = sensor_mag.z * 1000.0f; // Gauss -> MilliGauss
				msg.temperature = sensor_mag.temperature;
			}

			mavlink_msg_scaled_imu3_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};
#endif /* SCALED_IMU3_HPP */
