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

#include "VehicleMagnetometer.hpp"

#include <px4_platform_common/log.h>
#include <lib/ecl/geo/geo.h>

namespace sensors
{

using namespace matrix;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleMagnetometer::VehicleMagnetometer() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	param_find("CAL_MAG_SIDES");
	param_find("CAL_MAG_ROT_AUTO");

	_voter.set_timeout(SENSOR_TIMEOUT);
	_voter.set_equal_value_threshold(1000);

	ParametersUpdate(true);
}

VehicleMagnetometer::~VehicleMagnetometer()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleMagnetometer::Start()
{
	ScheduleNow();
	return true;
}

void VehicleMagnetometer::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleMagnetometer::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		// Mag compensation type
		MagCompensationType mag_comp_typ = static_cast<MagCompensationType>(_param_mag_comp_typ.get());

		if (mag_comp_typ != _mag_comp_type) {
			// check mag power compensation type (change battery current subscription instance if necessary)
			if (mag_comp_typ == MagCompensationType::Current_inst0 && _mag_comp_type != MagCompensationType::Current_inst0) {
				_battery_status_sub = uORB::Subscription{ORB_ID(battery_status), 0};
			}

			if (mag_comp_typ == MagCompensationType::Current_inst1 && _mag_comp_type != MagCompensationType::Current_inst1) {
				_battery_status_sub = uORB::Subscription{ORB_ID(battery_status), 1};
			}

			if (mag_comp_typ == MagCompensationType::Throttle) {
				_actuator_controls_0_sub = uORB::Subscription{ORB_ID(actuator_controls_0)};
			}
		}

		_mag_comp_type = mag_comp_typ;

		// update mag priority (CAL_MAGx_PRIO)
		for (int mag = 0; mag < MAX_SENSOR_COUNT; mag++) {
			const int32_t priority_old = _calibration[mag].priority();
			_calibration[mag].ParametersUpdate();
			const int32_t priority_new = _calibration[mag].priority();

			if (priority_old != priority_new) {
				if (_priority[mag] == priority_old) {
					_priority[mag] = priority_new;

				} else {
					// change relative priority to incorporate any sensor faults
					int priority_change = priority_new - priority_old;
					_priority[mag] = math::constrain(_priority[mag] + priority_change, 1, 100);
				}
			}
		}
	}
}

void VehicleMagnetometer::MagCalibrationUpdate()
{
	// State variance assumed for magnetometer bias storage.
	// This is a reference variance used to calculate the fraction of learned magnetometer bias that will be used to update the stored value.
	// Larger values cause a larger fraction of the learned biases to be used.
	static constexpr float magb_vref = 2.5e-7f;
	static constexpr float min_var_allowed = magb_vref * 0.01f;
	static constexpr float max_var_allowed = magb_vref * 100.f;

	if (_armed) {
		static constexpr uint8_t mag_cal_size = sizeof(_mag_cal) / sizeof(_mag_cal[0]);

		for (int i = 0; i < math::min(_estimator_sensor_bias_subs.size(), mag_cal_size); i++) {
			estimator_sensor_bias_s estimator_sensor_bias;

			if (_estimator_sensor_bias_subs[i].update(&estimator_sensor_bias)) {

				const Vector3f bias{estimator_sensor_bias.mag_bias};
				const Vector3f bias_variance{estimator_sensor_bias.mag_bias_variance};

				const bool valid = (hrt_elapsed_time(&estimator_sensor_bias.timestamp) < 1_s)
						   && (estimator_sensor_bias.mag_device_id != 0) && estimator_sensor_bias.mag_bias_valid
						   && (bias_variance.min() > min_var_allowed) && (bias_variance.max() < max_var_allowed);

				if (valid) {
					// find corresponding mag calibration
					for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
						if (_calibration[mag_index].device_id() == estimator_sensor_bias.mag_device_id) {

							const auto old_offset = _mag_cal[i].mag_offset;

							_mag_cal[i].device_id = estimator_sensor_bias.mag_device_id;
							_mag_cal[i].mag_offset = _calibration[mag_index].BiasCorrectedSensorOffset(bias);
							_mag_cal[i].mag_bias_variance = bias_variance;

							_mag_cal_available = true;

							if ((old_offset - _mag_cal[i].mag_offset).longerThan(0.01f)) {
								PX4_DEBUG("Mag %d (%d) est. offset saved: [% 05.3f % 05.3f % 05.3f] (bias [% 05.3f % 05.3f % 05.3f])",
									  mag_index, _mag_cal[i].device_id,
									  (double)_mag_cal[i].mag_offset(0), (double)_mag_cal[i].mag_offset(1), (double)_mag_cal[i].mag_offset(2),
									  (double)bias(0), (double)bias(1), (double)bias(2));
							}

							break;
						}
					}
				}
			}
		}

	} else if (_mag_cal_available) {
		// not armed and mag cal available
		bool calibration_param_save_needed = false;
		// iterate through available bias estimates and fuse them sequentially using a Kalman Filter scheme
		Vector3f state_variance{magb_vref, magb_vref, magb_vref};

		for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
			// apply all valid saved offsets
			for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				if ((_calibration[mag_index].device_id() != 0) && (_mag_cal[i].device_id == _calibration[mag_index].device_id())) {

					const Vector3f mag_cal_orig{_calibration[mag_index].offset()};
					Vector3f mag_cal_offset{_calibration[mag_index].offset()};

					// calculate weighting using ratio of variances and update stored bias values
					const Vector3f &observation = _mag_cal[i].mag_offset;
					const Vector3f &obs_variance = _mag_cal[i].mag_bias_variance;

					for (int axis_index = 0; axis_index < 3; axis_index++) {
						const float innovation_variance = state_variance(axis_index) + obs_variance(axis_index);
						const float innovation = mag_cal_offset(axis_index) - observation(axis_index);
						const float kalman_gain = state_variance(axis_index) / innovation_variance;
						mag_cal_offset(axis_index) -= innovation * kalman_gain;
						state_variance(axis_index) = fmaxf(state_variance(axis_index) * (1.f - kalman_gain), 0.f);
					}

					if (_calibration[mag_index].set_offset(mag_cal_offset)) {

						PX4_INFO("%d (%d) EST:%d offset committed: [%.2f %.2f %.2f]->[%.2f %.2f %.2f] (full [%.2f %.2f %.2f])",
							 mag_index, _calibration[mag_index].device_id(), i,
							 (double)mag_cal_orig(0), (double)mag_cal_orig(1), (double)mag_cal_orig(2),
							 (double)mag_cal_offset(0), (double)mag_cal_offset(1), (double)mag_cal_offset(2),
							 (double)_mag_cal[i].mag_offset(0), (double)_mag_cal[i].mag_offset(1), (double)_mag_cal[i].mag_offset(2));

						calibration_param_save_needed = true;
					}

					// clear
					_mag_cal[i].device_id = 0;
					_mag_cal[i].mag_offset.zero();
					_mag_cal[i].mag_bias_variance.zero();
				}
			}
		}

		if (calibration_param_save_needed) {
			for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
				if (_calibration[mag_index].device_id() != 0) {
					_calibration[mag_index].ParametersSave();
				}
			}

			_mag_cal_available = false;
		}
	}
}

void VehicleMagnetometer::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	// check vehicle status for changes to armed state
	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_armed = vehicle_control_mode.flag_armed;
		}
	}

	if (_mag_comp_type != MagCompensationType::Disabled) {
		// update power signal for mag compensation
		if (_armed) {
			if (_mag_comp_type == MagCompensationType::Throttle) {
				actuator_controls_s controls;

				if (_actuator_controls_0_sub.update(&controls)) {
					for (auto &cal : _calibration) {
						cal.UpdatePower(controls.control[actuator_controls_s::INDEX_THROTTLE]);
					}
				}

			} else if (_mag_comp_type == MagCompensationType::Current_inst0
				   || _mag_comp_type == MagCompensationType::Current_inst1) {

				battery_status_s bat_stat;

				if (_battery_status_sub.update(&bat_stat)) {
					float power = bat_stat.current_a * 0.001f; //current in [kA]

					for (auto &cal : _calibration) {
						cal.UpdatePower(power);
					}
				}
			}

		} else {
			for (auto &cal : _calibration) {
				cal.UpdatePower(0.f);
			}
		}
	}

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		if (!_calibration[uorb_index].enabled()) {
			continue;
		}

		if (!_advertised[uorb_index]) {
			// use data's timestamp to throttle advertisement checks
			if ((_last_data[uorb_index].timestamp == 0) || (hrt_elapsed_time(&_last_data[uorb_index].timestamp) > 1_s)) {
				if (_sensor_sub[uorb_index].advertised()) {
					if (uorb_index > 0) {
						/* the first always exists, but for each further sensor, add a new validator */
						if (!_voter.add_new_validator()) {
							PX4_ERR("failed to add validator for %s %i", "MAG", uorb_index);
						}
					}

					_advertised[uorb_index] = true;

					// advertise outputs in order if publishing all
					if (!_param_sens_mag_mode.get()) {
						for (int instance = 0; instance < uorb_index; instance++) {
							_vehicle_magnetometer_pub[instance].advertise();
						}
					}

					if (_selected_sensor_sub_index < 0) {
						_sensor_sub[uorb_index].registerCallback();
					}

				} else {
					_last_data[uorb_index].timestamp = hrt_absolute_time();
				}
			}

		}

		if (_advertised[uorb_index]) {
			sensor_mag_s report;

			while (_sensor_sub[uorb_index].update(&report)) {
				updated[uorb_index] = true;

				if (_calibration[uorb_index].device_id() != report.device_id) {
					_calibration[uorb_index].set_device_id(report.device_id, report.is_external);
					_priority[uorb_index] = _calibration[uorb_index].priority();
				}

				if (_calibration[uorb_index].enabled()) {
					const Vector3f vect = _calibration[uorb_index].Correct(Vector3f{report.x, report.y, report.z});

					float mag_array[3] {vect(0), vect(1), vect(2)};
					_voter.put(uorb_index, report.timestamp, mag_array, report.error_count, _priority[uorb_index]);

					_timestamp_sample_sum[uorb_index] += report.timestamp_sample;
					_mag_sum[uorb_index] += vect;
					_mag_sum_count[uorb_index]++;

					_last_data[uorb_index].timestamp_sample = report.timestamp_sample;
					_last_data[uorb_index].device_id = report.device_id;
					_last_data[uorb_index].x = vect(0);
					_last_data[uorb_index].y = vect(1);
					_last_data[uorb_index].z = vect(2);
				}
			}
		}
	}

	// check for the current best sensor
	int best_index = 0;
	_voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		if (_selected_sensor_sub_index != best_index) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			if (_param_sens_mag_mode.get()) {
				if (_selected_sensor_sub_index >= 0) {
					PX4_INFO("%s switch from #%u -> #%d", "MAG", _selected_sensor_sub_index, best_index);
				}
			}

			_selected_sensor_sub_index = best_index;
			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}
	}

	// Publish
	if (_param_sens_mag_mode.get()) {
		// publish only best mag
		if ((_selected_sensor_sub_index >= 0)
		    && (_voter.get_sensor_state(_selected_sensor_sub_index) == DataValidator::ERROR_FLAG_NO_ERROR)
		    && updated[_selected_sensor_sub_index]) {

			Publish(_selected_sensor_sub_index);
		}

	} else {
		// publish all
		for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {
			// publish all magnetometers as separate instances
			if (updated[uorb_index] && (_calibration[uorb_index].device_id() != 0)) {
				Publish(uorb_index, true);
			}
		}
	}


	// check failover and report
	if (_param_sens_mag_mode.get()) {
		if (_last_failover_count != _voter.failover_count()) {
			uint32_t flags = _voter.failover_state();
			int failover_index = _voter.failover_index();

			if (flags != DataValidator::ERROR_FLAG_NO_ERROR) {
				if (failover_index != -1) {
					const hrt_abstime now = hrt_absolute_time();

					if (now - _last_error_message > 3_s) {
						mavlink_log_emergency(&_mavlink_log_pub, "%s #%i failed: %s%s%s%s%s!",
								      "MAG",
								      failover_index,
								      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
								      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
								      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));
						_last_error_message = now;
					}

					// reduce priority of failed sensor to the minimum
					_priority[failover_index] = 1;
				}
			}

			_last_failover_count = _voter.failover_count();
		}
	}

	if (!_armed) {
		calcMagInconsistency();
	}

	MagCalibrationUpdate();

	// reschedule timeout
	ScheduleDelayed(20_ms);

	perf_end(_cycle_perf);
}

void VehicleMagnetometer::Publish(uint8_t instance, bool multi)
{
	if ((_param_sens_mag_rate.get() > 0) && ((_last_publication_timestamp[instance] == 0) ||
			(hrt_elapsed_time(&_last_publication_timestamp[instance]) >= (1e6f / _param_sens_mag_rate.get())))) {

		const Vector3f magnetometer_data = _mag_sum[instance] / _mag_sum_count[instance];
		const hrt_abstime timestamp_sample = _timestamp_sample_sum[instance] / _mag_sum_count[instance];

		// reset
		_timestamp_sample_sum[instance] = 0;
		_mag_sum[instance].zero();
		_mag_sum_count[instance] = 0;

		// populate vehicle_magnetometer with primary mag and publish
		vehicle_magnetometer_s out{};
		out.timestamp_sample = timestamp_sample;
		out.device_id = _calibration[instance].device_id();
		magnetometer_data.copyTo(out.magnetometer_ga);
		out.calibration_count = _calibration[instance].calibration_count();

		out.timestamp = hrt_absolute_time();

		if (multi) {
			_vehicle_magnetometer_pub[instance].publish(out);

		} else {
			// otherwise only ever publish the first instance
			_vehicle_magnetometer_pub[0].publish(out);
		}

		_last_publication_timestamp[instance] = out.timestamp;
	}
}

void VehicleMagnetometer::calcMagInconsistency()
{
	sensor_preflight_mag_s preflt{};

	const sensor_mag_s &primary_mag_report = _last_data[_selected_sensor_sub_index];
	const Vector3f primary_mag(primary_mag_report.x, primary_mag_report.y,
				   primary_mag_report.z); // primary mag field vector

	float mag_angle_diff_max = 0.0f; // the maximum angle difference
	unsigned check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		// check that the sensor we are checking against is not the same as the primary
		if (_advertised[i] && (_priority[i] > 0) && (i != _selected_sensor_sub_index)) {
			// calculate angle to 3D magnetic field vector of the primary sensor
			const sensor_mag_s &current_mag_report = _last_data[i];
			Vector3f current_mag{current_mag_report.x, current_mag_report.y, current_mag_report.z};

			float angle_error = AxisAnglef(Quatf(current_mag, primary_mag)).angle();

			// complementary filter to not fail/pass on single outliers
			_mag_angle_diff[check_index] *= 0.95f;
			_mag_angle_diff[check_index] += 0.05f * angle_error;

			mag_angle_diff_max = math::max(mag_angle_diff_max, _mag_angle_diff[check_index]);

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;
		}
	}

	// get the vector length of the largest difference and write to the combined sensor struct
	// will be zero if there is only one magnetometer and hence nothing to compare
	preflt.mag_inconsistency_angle = mag_angle_diff_max;

	preflt.timestamp = hrt_absolute_time();
	_sensor_preflight_mag_pub.publish(preflt);
}

void VehicleMagnetometer::PrintStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		PX4_INFO("selected magnetometer: %d (%d)", _last_data[_selected_sensor_sub_index].device_id,
			 _selected_sensor_sub_index);
	}

	_voter.print();

	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_advertised[i] && (_priority[i] > 0)) {
			_calibration[i].PrintStatus();
		}
	}
}

}; // namespace sensors
