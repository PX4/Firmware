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

#include "EKF2Selector.hpp"

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector2f;
using math::constrain;
using math::max;
using math::radians;

EKF2Selector::EKF2Selector() :
	ModuleParams(nullptr),
	ScheduledWorkItem("ekf2_selector", px4::wq_configurations::nav_and_controllers)
{
}

EKF2Selector::~EKF2Selector()
{
	Stop();
}

bool EKF2Selector::Start()
{
	ScheduleNow();
	return true;
}

void EKF2Selector::Stop()
{
	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
		_instance[i].estimator_attitude_sub.unregisterCallback();
		_instance[i].estimator_status_sub.unregisterCallback();
	}

	ScheduleClear();
}

bool EKF2Selector::SelectInstance(uint8_t ekf_instance)
{
	if ((ekf_instance != INVALID_INSTANCE) && (ekf_instance != _selected_instance)) {

		// update sensor_selection immediately
		sensor_selection_s sensor_selection{};
		sensor_selection.accel_device_id = _instance[ekf_instance].accel_device_id;
		sensor_selection.gyro_device_id = _instance[ekf_instance].gyro_device_id;
		sensor_selection.timestamp = hrt_absolute_time();
		_sensor_selection_pub.publish(sensor_selection);

		if (_selected_instance != INVALID_INSTANCE) {
			// switch callback registration
			_instance[_selected_instance].estimator_attitude_sub.unregisterCallback();
			_instance[_selected_instance].estimator_status_sub.unregisterCallback();

			if (!_instance[_selected_instance].healthy) {
				const char *reason = nullptr;

				if (_instance[_selected_instance].filter_fault) {
					reason = "filter fault";

				} else if (_instance[_selected_instance].timeout) {
					reason = "timeout";

				} else if (_gyro_fault_detected) {
					reason = "gyro fault";

				} else if (_accel_fault_detected) {
					reason = "accel fault";
				}

				if (reason) {
					PX4_WARN("primary EKF changed %d (%s) -> %d", _selected_instance, reason, ekf_instance);
				}
			}
		}

		_instance[ekf_instance].estimator_attitude_sub.registerCallback();
		_instance[ekf_instance].estimator_status_sub.registerCallback();

		_selected_instance = ekf_instance;
		_instance_changed_count++;
		_last_instance_change = sensor_selection.timestamp;
		_instance[ekf_instance].time_last_selected = _last_instance_change;

		// reset all relative test ratios
		for (uint8_t i = 0; i < _available_instances; i++) {
			_instance[i].relative_test_ratio = 0;
		}

		// publish new data immediately with resets
		PublishVehicleAttitude(true);
		PublishVehicleLocalPosition(true);
		PublishVehicleGlobalPosition(true);
		PublishWindEstimate(true);

		return true;
	}

	return false;
}

bool EKF2Selector::UpdateErrorScores()
{
	// first check imu inconsistencies
	_gyro_fault_detected = false;
	uint32_t faulty_gyro_id = 0;
	_accel_fault_detected = false;
	uint32_t faulty_accel_id = 0;

	if (_sensors_status_imu.updated()) {
		sensors_status_imu_s sensors_status_imu;

		if (_sensors_status_imu.copy(&sensors_status_imu)) {

			const float time_step_s = constrain((sensors_status_imu.timestamp - _last_update_us) * 1e-6f, 0.f, 0.02f);
			_last_update_us = sensors_status_imu.timestamp;

			{
				const float angle_rate_threshold = radians(_param_ekf2_sel_imu_angle_rate.get());
				const float angle_threshold = radians(_param_ekf2_sel_imu_angle.get());
				uint8_t n_gyros = 0;
				uint8_t n_gyro_exceedances = 0;
				float largest_accumulated_gyro_error = 0.0f;
				uint8_t largest_gyro_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for gyros with excessive difference to mean using accumulated error
					if (sensors_status_imu.gyro_device_ids[i] != 0) {
						n_gyros++;
						_accumulated_gyro_error[i] += (sensors_status_imu.gyro_inconsistency_rad_s[i] - angle_rate_threshold) * time_step_s;
						_accumulated_gyro_error[i] = fmaxf(_accumulated_gyro_error[i], 0.f);

						if (_accumulated_gyro_error[i] > angle_threshold) {
							n_gyro_exceedances++;
						}

						if (_accumulated_gyro_error[i] > largest_accumulated_gyro_error) {
							largest_accumulated_gyro_error = _accumulated_gyro_error[i];
							largest_gyro_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_gyro_error[i] = NAN;
					}
				}

				if (n_gyro_exceedances > 0) {
					if (n_gyros >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_gyro_fault_detected = true;
						faulty_gyro_id = sensors_status_imu.gyro_device_ids[largest_gyro_error_index];

					} else if (n_gyros == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_gyro_fault_detected = true;
					}
				}
			}

			{
				const float accel_threshold = _param_ekf2_sel_imu_accel.get();
				const float velocity_threshold = _param_ekf2_sel_imu_velocity.get();
				uint8_t n_accels = 0;
				uint8_t n_accel_exceedances = 0;
				float largest_accumulated_accel_error = 0.0f;
				uint8_t largest_accel_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for accelerometers with excessive difference to mean using accumulated error
					if (sensors_status_imu.accel_device_ids[i] != 0) {
						n_accels++;
						_accumulated_accel_error[i] += (sensors_status_imu.accel_inconsistency_m_s_s[i] - accel_threshold) * time_step_s;
						_accumulated_accel_error[i] = fmaxf(_accumulated_accel_error[i], 0.f);

						if (_accumulated_accel_error[i] > velocity_threshold) {
							n_accel_exceedances++;
						}

						if (_accumulated_accel_error[i] > largest_accumulated_accel_error) {
							largest_accumulated_accel_error = _accumulated_accel_error[i];
							largest_accel_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_accel_error[i] = NAN;
					}
				}

				if (n_accel_exceedances > 0) {
					if (n_accels >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_accel_fault_detected = true;
						faulty_accel_id = sensors_status_imu.accel_device_ids[largest_accel_error_index];

					} else if (n_accels == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_accel_fault_detected = true;
					}
				}
			}
		}
	}

	bool updated = false;
	bool primary_updated = false;

	// default estimator timeout
	hrt_abstime status_timeout = 50_ms;

	if (hrt_elapsed_time(&_attitude_last.timestamp) > FILTER_UPDATE_PERIOD) {
		// much lower timeout if current primary estimator attitude isn't publishing
		status_timeout = 2 * FILTER_UPDATE_PERIOD;
	}

	// calculate individual error scores
	for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) {
		const bool prev_healthy = _instance[i].healthy;

		estimator_status_s status;

		if (_instance[i].estimator_status_sub.update(&status)) {

			_instance[i].timestamp_sample_last = status.timestamp_sample;

			_instance[i].accel_device_id = status.accel_device_id;
			_instance[i].gyro_device_id = status.gyro_device_id;
			_instance[i].baro_device_id = status.baro_device_id;
			_instance[i].mag_device_id = status.mag_device_id;

			if ((i + 1) > _available_instances) {
				_available_instances = i + 1;
				updated = true;
			}

			if (i == _selected_instance) {
				primary_updated = true;
			}

			const bool tilt_align = status.control_mode_flags & (1 << estimator_status_s::CS_TILT_ALIGN);
			const bool yaw_align = status.control_mode_flags & (1 << estimator_status_s::CS_YAW_ALIGN);

			float combined_test_ratio = 0;

			if (tilt_align && yaw_align) {
				combined_test_ratio = fmaxf(0.f, 0.5f * (status.vel_test_ratio + status.pos_test_ratio));
				combined_test_ratio = fmaxf(combined_test_ratio, status.hgt_test_ratio);
			}

			_instance[i].combined_test_ratio = combined_test_ratio;
			_instance[i].healthy = tilt_align && yaw_align && (status.filter_fault_flags == 0);
			_instance[i].filter_fault = (status.filter_fault_flags != 0);
			_instance[i].timeout = false;

			if (!PX4_ISFINITE(_instance[i].relative_test_ratio)) {
				_instance[i].relative_test_ratio = 0;
			}

		} else if (hrt_elapsed_time(&_instance[i].timestamp_sample_last) > status_timeout) {
			_instance[i].healthy = false;
			_instance[i].timeout = true;
		}

		// if the gyro used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_gyro_fault_detected && (faulty_gyro_id != 0) && (_instance[i].gyro_device_id == faulty_gyro_id)) {
			_instance[i].healthy = false;
		}

		// if the accelerometer used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_accel_fault_detected && (faulty_accel_id != 0) && (_instance[i].accel_device_id == faulty_accel_id)) {
			_instance[i].healthy = false;
		}

		if (prev_healthy != _instance[i].healthy) {
			updated = true;
			_selector_status_publish = true;
		}
	}

	// update relative test ratios if primary has updated
	if (primary_updated) {
		for (uint8_t i = 0; i < _available_instances; i++) {
			if (i != _selected_instance) {

				const float error_delta = _instance[i].combined_test_ratio - _instance[_selected_instance].combined_test_ratio;

				// reduce error only if its better than the primary instance by at least EKF2_SEL_ERR_RED to prevent unnecessary selection changes
				const float threshold = _gyro_fault_detected ? 0.0f : fmaxf(_param_ekf2_sel_err_red.get(), 0.05f);

				if (error_delta > 0 || error_delta < -threshold) {
					_instance[i].relative_test_ratio += error_delta;
					_instance[i].relative_test_ratio = constrain(_instance[i].relative_test_ratio, -_rel_err_score_lim, _rel_err_score_lim);
				}
			}
		}
	}

	return (primary_updated || updated);
}

void EKF2Selector::PublishVehicleAttitude(bool reset)
{
	vehicle_attitude_s attitude;

	if (_instance[_selected_instance].estimator_attitude_sub.copy(&attitude)) {
		if (reset) {
			// on reset compute deltas from last published data
			++_quat_reset_counter;

			_delta_q_reset = (Quatf(attitude.q) * Quatf(_attitude_last.q).inversed()).normalized();

			// ensure monotonically increasing timestamp_sample through reset
			attitude.timestamp_sample = max(attitude.timestamp_sample, _attitude_last.timestamp_sample);

		} else {
			// otherwise propogate deltas from estimator data while maintaining the overall reset counts
			if (attitude.quat_reset_counter > _attitude_last.quat_reset_counter) {
				++_quat_reset_counter;
				_delta_q_reset = Quatf{attitude.delta_q_reset};
			}
		}

		// save last primary estimator_attitude
		_attitude_last = attitude;

		// republish with total reset count and current timestamp
		attitude.quat_reset_counter = _quat_reset_counter;
		_delta_q_reset.copyTo(attitude.delta_q_reset);

		attitude.timestamp = hrt_absolute_time();
		_vehicle_attitude_pub.publish(attitude);

		_instance[_selected_instance].timestamp_sample_last = attitude.timestamp_sample;
	}
}

void EKF2Selector::PublishVehicleLocalPosition(bool reset)
{
	// vehicle_local_position
	vehicle_local_position_s local_position;

	if (_instance[_selected_instance].estimator_local_position_sub.copy(&local_position)) {
		if (reset) {
			// on reset compute deltas from last published data
			++_xy_reset_counter;
			++_z_reset_counter;
			++_vxy_reset_counter;
			++_vz_reset_counter;
			++_heading_reset_counter;

			_delta_xy_reset = Vector2f{local_position.x, local_position.y} - Vector2f{_local_position_last.x, _local_position_last.y};
			_delta_z_reset = local_position.z - _local_position_last.z;
			_delta_vxy_reset = Vector2f{local_position.vx, local_position.vy} - Vector2f{_local_position_last.vx, _local_position_last.vy};
			_delta_vz_reset = local_position.vz - _local_position_last.vz;
			_delta_heading_reset = matrix::wrap_2pi(local_position.heading - _local_position_last.heading);

			// ensure monotonically increasing timestamp_sample through reset
			local_position.timestamp_sample = max(local_position.timestamp_sample, _local_position_last.timestamp_sample);

		} else {
			// otherwise propogate deltas from estimator data while maintaining the overall reset counts

			// XY reset
			if (local_position.xy_reset_counter > _local_position_last.xy_reset_counter) {
				++_xy_reset_counter;
				_delta_xy_reset = Vector2f{local_position.delta_xy};
			}

			// Z reset
			if (local_position.z_reset_counter > _local_position_last.z_reset_counter) {
				++_z_reset_counter;
				_delta_z_reset = local_position.delta_z;
			}

			// VXY reset
			if (local_position.vxy_reset_counter > _local_position_last.vxy_reset_counter) {
				++_vxy_reset_counter;
				_delta_vxy_reset = Vector2f{local_position.delta_vxy};
			}

			// VZ reset
			if (local_position.vz_reset_counter > _local_position_last.vz_reset_counter) {
				++_vz_reset_counter;
				_delta_vz_reset = local_position.delta_vz;
			}

			// heading reset
			if (local_position.heading_reset_counter > _local_position_last.heading_reset_counter) {
				++_heading_reset_counter;
				_delta_heading_reset = local_position.delta_heading;
			}
		}

		// save last primary estimator_local_position
		_local_position_last = local_position;

		// publish estimator's local position for system (vehicle_local_position) unless it's stale
		if (local_position.timestamp_sample >= _instance[_selected_instance].timestamp_sample_last) {
			// republish with total reset count and current timestamp
			local_position.xy_reset_counter = _xy_reset_counter;
			local_position.z_reset_counter = _z_reset_counter;
			local_position.vxy_reset_counter = _vxy_reset_counter;
			local_position.vz_reset_counter = _vz_reset_counter;
			local_position.heading_reset_counter = _heading_reset_counter;

			_delta_xy_reset.copyTo(local_position.delta_xy);
			local_position.delta_z = _delta_z_reset;
			_delta_vxy_reset.copyTo(local_position.delta_vxy);
			local_position.delta_vz = _delta_vz_reset;
			local_position.delta_heading = _delta_heading_reset;

			local_position.timestamp = hrt_absolute_time();
			_vehicle_local_position_pub.publish(local_position);
		}
	}
}

void EKF2Selector::PublishVehicleGlobalPosition(bool reset)
{
	vehicle_global_position_s global_position;

	if (_instance[_selected_instance].estimator_global_position_sub.copy(&global_position)) {
		if (reset) {
			// on reset compute deltas from last published data
			++_lat_lon_reset_counter;
			++_alt_reset_counter;

			_delta_lat_reset = global_position.lat - _global_position_last.lat;
			_delta_lon_reset = global_position.lon - _global_position_last.lon;
			_delta_alt_reset = global_position.delta_alt - _global_position_last.delta_alt;

			// ensure monotonically increasing timestamp_sample through reset
			global_position.timestamp_sample = max(global_position.timestamp_sample, _global_position_last.timestamp_sample);

		} else {
			// otherwise propogate deltas from estimator data while maintaining the overall reset counts

			// lat/lon reset
			if (global_position.lat_lon_reset_counter > _global_position_last.lat_lon_reset_counter) {
				++_lat_lon_reset_counter;

				// TODO: delta latitude/longitude
				//_delta_lat_reset = global_position.delta_lat;
				//_delta_lon_reset = global_position.delta_lon;
			}

			// alt reset
			if (global_position.alt_reset_counter > _global_position_last.alt_reset_counter) {
				++_alt_reset_counter;
				_delta_alt_reset = global_position.delta_alt;
			}
		}

		// save last primary estimator_global_position
		_global_position_last = global_position;

		// publish estimator's global position for system (vehicle_global_position) unless it's stale
		if (global_position.timestamp_sample >= _instance[_selected_instance].timestamp_sample_last) {
			// republish with total reset count and current timestamp
			global_position.lat_lon_reset_counter = _lat_lon_reset_counter;
			global_position.alt_reset_counter = _alt_reset_counter;
			global_position.delta_alt = _delta_alt_reset;

			global_position.timestamp = hrt_absolute_time();
			_vehicle_global_position_pub.publish(global_position);
		}
	}
}

void EKF2Selector::PublishWindEstimate(bool reset)
{
	wind_s wind;

	if (_instance[_selected_instance].estimator_wind_sub.copy(&wind)) {
		if (reset) {
			// ensure monotonically increasing timestamp_sample through reset
			wind.timestamp_sample = max(wind.timestamp_sample, _wind_last.timestamp_sample);
		}

		// save last primary wind
		_wind_last = wind;

		// republish with current timestamp
		wind.timestamp = hrt_absolute_time();
		_wind_pub.publish(wind);
	}
}

void EKF2Selector::Run()
{
	// re-schedule as backup timeout
	ScheduleDelayed(FILTER_UPDATE_PERIOD);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	// update combined test ratio for all estimators
	const bool updated = UpdateErrorScores();

	// if no valid instance then force select first instance with valid IMU
	if (_selected_instance == INVALID_INSTANCE) {
		for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) {
			if ((_instance[i].accel_device_id != 0)
			    && (_instance[i].gyro_device_id != 0)) {

				if (SelectInstance(i)) {
					break;
				}
			}
		}

		// if still invalid return early and check again on next scheduled run
		if (_selected_instance == INVALID_INSTANCE) {
			return;
		}
	}

	if (updated) {

		const uint8_t available_instances_prev = _available_instances;
		const uint8_t selected_instance_prev = _selected_instance;
		const uint32_t instance_changed_count_prev = _instance_changed_count;
		const hrt_abstime last_instance_change_prev = _last_instance_change;

		bool lower_error_available = false;
		float alternative_error = 0.f; // looking for instances that have error lower than the current primary
		float best_test_ratio = FLT_MAX;

		uint8_t best_ekf = _selected_instance;
		uint8_t best_ekf_alternate = INVALID_INSTANCE;
		uint8_t best_ekf_different_imu = INVALID_INSTANCE;

		// loop through all available instances to find if an alternative is available
		for (int i = 0; i < _available_instances; i++) {
			// Use an alternative instance if  -
			// (healthy and has updated recently)
			// AND
			// (has relative error less than selected instance and has not been the selected instance for at least 10 seconds
			// OR
			// selected instance has stopped updating
			if (_instance[i].healthy && (i != _selected_instance)) {
				const float test_ratio = _instance[i].combined_test_ratio;
				const float relative_error = _instance[i].relative_test_ratio;

				if (relative_error < alternative_error) {
					best_ekf_alternate = i;
					alternative_error = relative_error;

					// relative error less than selected instance and has not been the selected instance for at least 10 seconds
					if ((relative_error <= -_rel_err_thresh) && hrt_elapsed_time(&_instance[i].time_last_selected) > 10_s) {
						lower_error_available = true;
					}
				}

				if ((test_ratio > 0) && (test_ratio < best_test_ratio)) {
					best_ekf = i;
					best_test_ratio = test_ratio;

					// also check next best available ekf using a different IMU
					if (_instance[i].accel_device_id != _instance[_selected_instance].accel_device_id) {
						best_ekf_different_imu = i;
					}
				}
			}
		}

		if (!_instance[_selected_instance].healthy) {
			// prefer the best healthy instance using a different IMU
			if (!SelectInstance(best_ekf_different_imu)) {
				// otherwise switch to the healthy instance with best overall test ratio
				SelectInstance(best_ekf);
			}

		} else if (lower_error_available && (hrt_elapsed_time(&_last_instance_change) > 10_s)) {
			// if this instance has a significantly lower relative error to the active primary, we consider it as a
			// better instance and would like to switch to it even if the current primary is healthy
			SelectInstance(best_ekf_alternate);
		}

		// publish selector status at ~1 Hz or immediately on any change
		if (_selector_status_publish || (hrt_elapsed_time(&_last_status_publish) > 1_s)
		    || (available_instances_prev != _available_instances)
		    || (selected_instance_prev != _selected_instance)
		    || (instance_changed_count_prev != _instance_changed_count)
		    || (last_instance_change_prev != _last_instance_change)
		    || _accel_fault_detected || _gyro_fault_detected) {

			estimator_selector_status_s selector_status{};
			selector_status.primary_instance = _selected_instance;
			selector_status.instances_available = _available_instances;
			selector_status.instance_changed_count = _instance_changed_count;
			selector_status.last_instance_change = _last_instance_change;
			selector_status.accel_device_id = _instance[_selected_instance].accel_device_id;
			selector_status.baro_device_id = _instance[_selected_instance].baro_device_id;
			selector_status.gyro_device_id = _instance[_selected_instance].gyro_device_id;
			selector_status.mag_device_id = _instance[_selected_instance].mag_device_id;
			selector_status.gyro_fault_detected = _gyro_fault_detected;
			selector_status.accel_fault_detected = _accel_fault_detected;

			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				selector_status.combined_test_ratio[i] = _instance[i].combined_test_ratio;
				selector_status.relative_test_ratio[i] = _instance[i].relative_test_ratio;
				selector_status.healthy[i] = _instance[i].healthy;
			}

			for (int i = 0; i < IMU_STATUS_SIZE; i++) {
				selector_status.accumulated_gyro_error[i] = _accumulated_gyro_error[i];
				selector_status.accumulated_accel_error[i] = _accumulated_accel_error[i];
			}

			selector_status.timestamp = hrt_absolute_time();
			_estimator_selector_status_pub.publish(selector_status);
			_last_status_publish = selector_status.timestamp;
			_selector_status_publish = false;
		}
	}

	// republish selected estimator data for system

	// selected estimator_attitude -> vehicle_attitude
	if (_instance[_selected_instance].estimator_attitude_sub.updated()) {
		PublishVehicleAttitude();
	}

	// selected estimator_local_position -> vehicle_local_position
	if (_instance[_selected_instance].estimator_local_position_sub.updated()) {
		PublishVehicleLocalPosition();
	}

	// selected estimator_global_position -> vehicle_global_position
	if (_instance[_selected_instance].estimator_global_position_sub.updated()) {
		PublishVehicleGlobalPosition();
	}

	// selected estimator_wind -> wind
	if (_instance[_selected_instance].estimator_wind_sub.updated()) {
		PublishWindEstimate();
	}

	// selected estimator_odometry -> vehicle_odometry
	if (_instance[_selected_instance].estimator_odometry_sub.updated()) {
		vehicle_odometry_s vehicle_odometry;

		if (_instance[_selected_instance].estimator_odometry_sub.update(&vehicle_odometry)) {
			if (vehicle_odometry.timestamp_sample >= _instance[_selected_instance].timestamp_sample_last) {
				vehicle_odometry.timestamp = hrt_absolute_time();
				_vehicle_odometry_pub.publish(vehicle_odometry);
			}
		}
	}
}

void EKF2Selector::PrintStatus()
{
	PX4_INFO("available instances: %d", _available_instances);

	if (_selected_instance == INVALID_INSTANCE) {
		PX4_WARN("selected instance: None");
	}

	for (int i = 0; i < _available_instances; i++) {
		const EstimatorInstance &inst = _instance[i];

		PX4_INFO("%d: ACC: %d, GYRO: %d, MAG: %d, %s, test ratio: %.7f (%.5f) %s",
			 inst.instance, inst.accel_device_id, inst.gyro_device_id, inst.mag_device_id,
			 inst.healthy ? "healthy" : "unhealthy",
			 (double)inst.combined_test_ratio, (double)inst.relative_test_ratio,
			 (_selected_instance == i) ? "*" : "");
	}
}
