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

#ifndef HIGHRES_IMU_HPP
#define HIGHRES_IMU_HPP

#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_magnetometer.h>

using matrix::Vector3f;

class MavlinkStreamHighresIMU : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHighresIMU(mavlink); }

	static constexpr const char *get_name_static() { return "HIGHRES_IMU"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HIGHRES_IMU; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamHighresIMU(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<vehicle_imu_s, 3> _vehicle_imu_subs{ORB_ID::vehicle_imu};
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure)};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	bool send() override
	{
		bool updated = false;

		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		vehicle_imu_s imu;

		for (auto &imu_sub : _vehicle_imu_subs) {
			if (imu_sub.update(&imu)) {
				if (imu.accel_device_id == sensor_selection.accel_device_id) {
					updated = true;
					break;
				}
			}
		}

		if (updated) {
			uint16_t fields_updated = 0;

			fields_updated |= (1 << 0) | (1 << 1) | (1 << 2); // accel
			fields_updated |= (1 << 3) | (1 << 4) | (1 << 5); // gyro

			vehicle_magnetometer_s magnetometer{};

			if (_magnetometer_sub.update(&magnetometer)) {
				// mark third group dimensions as changed
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);

			} else {
				_magnetometer_sub.copy(&magnetometer);
			}

			// find corresponding estimated sensor bias
			if (_estimator_selector_status_sub.updated()) {
				estimator_selector_status_s estimator_selector_status;

				if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
					_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
				}
			}

			Vector3f accel_bias{0.f, 0.f, 0.f};
			Vector3f gyro_bias{0.f, 0.f, 0.f};
			Vector3f mag_bias{0.f, 0.f, 0.f};

			{
				estimator_sensor_bias_s bias;

				if (_estimator_sensor_bias_sub.copy(&bias)) {
					if ((bias.accel_device_id != 0) && (bias.accel_device_id == imu.accel_device_id)) {
						accel_bias = Vector3f{bias.accel_bias};
					}

					if ((bias.gyro_device_id != 0) && (bias.gyro_device_id == imu.gyro_device_id)) {
						gyro_bias = Vector3f{bias.gyro_bias};
					}

					if ((bias.mag_device_id != 0) && (bias.mag_device_id == magnetometer.device_id)) {
						mag_bias = Vector3f{bias.mag_bias};

					} else {
						// find primary mag
						uORB::SubscriptionMultiArray<vehicle_magnetometer_s> mag_subs{ORB_ID::vehicle_magnetometer};

						for (int i = 0; i < mag_subs.size(); i++) {
							if (mag_subs[i].advertised() && mag_subs[i].copy(&magnetometer)) {
								if (magnetometer.device_id == bias.mag_device_id) {
									_magnetometer_sub.ChangeInstance(i);
									break;
								}
							}

						}
					}
				}
			}

			const Vector3f mag = Vector3f{magnetometer.magnetometer_ga} - mag_bias;

			vehicle_air_data_s air_data{};

			if (_air_data_sub.update(&air_data)) {
				/* mark fourth group (baro fields) dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);

			} else {
				_air_data_sub.copy(&air_data);
			}

			differential_pressure_s differential_pressure{};

			if (_differential_pressure_sub.update(&differential_pressure)) {
				/* mark fourth group (dpres field) dimensions as changed */
				fields_updated |= (1 << 10);

			} else {
				_differential_pressure_sub.copy(&differential_pressure);
			}

			const float accel_dt_inv = 1.e6f / (float)imu.delta_velocity_dt;
			const Vector3f accel = (Vector3f{imu.delta_velocity} * accel_dt_inv) - accel_bias;

			const float gyro_dt_inv = 1.e6f / (float)imu.delta_angle_dt;
			const Vector3f gyro = (Vector3f{imu.delta_angle} * gyro_dt_inv) - gyro_bias;

			mavlink_highres_imu_t msg{};

			msg.time_usec = imu.timestamp_sample;
			msg.xacc = accel(0);
			msg.yacc = accel(1);
			msg.zacc = accel(2);
			msg.xgyro = gyro(0);
			msg.ygyro = gyro(1);
			msg.zgyro = gyro(2);
			msg.xmag = mag(0);
			msg.ymag = mag(1);
			msg.zmag = mag(2);
			msg.abs_pressure = air_data.baro_pressure_pa;
			msg.diff_pressure = differential_pressure.differential_pressure_raw_pa;
			msg.pressure_alt = air_data.baro_alt_meter;
			msg.temperature = air_data.baro_temp_celcius;
			msg.fields_updated = fields_updated;

			mavlink_msg_highres_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};
#endif // HIGHRES_IMU_HPP
