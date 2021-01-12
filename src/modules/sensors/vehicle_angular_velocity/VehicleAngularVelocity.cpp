/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "VehicleAngularVelocity.hpp"

#include <px4_platform_common/log.h>

#include <uORB/topics/vehicle_imu_status.h>

using namespace matrix;

namespace sensors
{

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	CheckAndUpdateFilters();
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();
}

bool VehicleAngularVelocity::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("sensor_selection callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(true)) {
		_sensor_sub.registerCallback();
	}

	return true;
}

void VehicleAngularVelocity::Stop()
{
	// clear all registered callbacks
	_sensor_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

void VehicleAngularVelocity::CheckAndUpdateFilters()
{
	bool sample_rate_changed = false;

	// get sample rate from vehicle_imu_status publication
	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

		const float sample_rate_hz = imu_status.get().gyro_rate_hz;

		if ((imu_status.get().gyro_device_id != 0) && (imu_status.get().gyro_device_id == _calibration.device_id())
		    && PX4_ISFINITE(sample_rate_hz) && (sample_rate_hz > 0)) {
			// check if sample rate error is greater than 1%
			if ((fabsf(sample_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
				PX4_DEBUG("sample rate changed: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)sample_rate_hz);
				_filter_sample_rate = sample_rate_hz;
				sample_rate_changed = true;
				break;
			}
		}
	}

	// update software low pass filters
	if (sample_rate_changed || (fabsf(_lp_filter_velocity.get_cutoff_freq() - _param_imu_gyro_cutoff.get()) > 0.1f)) {
		_lp_filter_velocity.set_cutoff_frequency(_filter_sample_rate, _param_imu_gyro_cutoff.get());
		_lp_filter_velocity.reset(_angular_velocity_prev);
	}

	if (sample_rate_changed
	    || (fabsf(_notch_filter_velocity.getNotchFreq() - _param_imu_gyro_nf_freq.get()) > 0.1f)
	    || (fabsf(_notch_filter_velocity.getBandwidth() - _param_imu_gyro_nf_bw.get()) > 0.1f)
	   ) {
		_notch_filter_velocity.setParameters(_filter_sample_rate, _param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());
		_notch_filter_velocity.reset(_angular_velocity_prev);
	}

	if (sample_rate_changed || (fabsf(_lp_filter_acceleration.get_cutoff_freq() - _param_imu_dgyro_cutoff.get()) > 0.1f)) {
		_lp_filter_acceleration.set_cutoff_frequency(_filter_sample_rate, _param_imu_dgyro_cutoff.get());
		_lp_filter_acceleration.reset(_angular_acceleration_prev);
	}
}

void VehicleAngularVelocity::SensorBiasUpdate(bool force)
{
	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}
	}

	if (_estimator_sensor_bias_sub.updated() || force) {
		estimator_sensor_bias_s bias;

		if (_estimator_sensor_bias_sub.copy(&bias)) {
			if (bias.gyro_device_id == _calibration.device_id()) {
				_bias = Vector3f{bias.gyro_bias};

			} else {
				_bias.zero();
			}
		}
	}
}

bool VehicleAngularVelocity::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_calibration.device_id() == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if ((sensor_selection.gyro_device_id != 0) && (_calibration.device_id() != sensor_selection.gyro_device_id)) {
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				const uint32_t device_id = sensor_gyro_sub.get().device_id;

				if ((device_id != 0) && (device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_sub.ChangeInstance(i) && _sensor_sub.registerCallback()) {
						PX4_DEBUG("selected sensor changed %d -> %d", _calibration.device_id(), device_id);

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(device_id);

						CheckAndUpdateFilters();

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
			_calibration.set_device_id(0);
		}
	}

	return false;
}

void VehicleAngularVelocity::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		_calibration.ParametersUpdate();

		CheckAndUpdateFilters();
	}
}

void VehicleAngularVelocity::Run()
{
	// backup schedule
	ScheduleDelayed(10_ms);

	// update corrections first to set _selected_sensor
	bool selection_updated = SensorSelectionUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);
	ParametersUpdate();

	// process all outstanding messages
	sensor_gyro_s sensor_data;

	while (_sensor_sub.update(&sensor_data)) {

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((sensor_data.timestamp_sample - _timestamp_sample_prev) / 1e6f), 0.0002f, 0.02f);
		_timestamp_sample_prev = sensor_data.timestamp_sample;

		// get the sensor data and correct for thermal errors (apply offsets and scale)
		const Vector3f val{sensor_data.x, sensor_data.y, sensor_data.z};

		// correct for in-run bias errors
		const Vector3f angular_velocity_raw = _calibration.Correct(val) - _bias;

		// Gyro filtering:
		// - Apply general notch filter (IMU_GYRO_NF_FREQ)
		// - Apply general low-pass filter (IMU_GYRO_CUTOFF)
		// - Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)

		const Vector3f angular_velocity_notched{_notch_filter_velocity.apply(angular_velocity_raw)};

		const Vector3f angular_velocity{_lp_filter_velocity.apply(angular_velocity_notched)};

		const Vector3f angular_acceleration_raw = (angular_velocity - _angular_velocity_prev) / dt;
		_angular_velocity_prev = angular_velocity;
		_angular_acceleration_prev = angular_acceleration_raw;
		const Vector3f angular_acceleration{_lp_filter_acceleration.apply(angular_acceleration_raw)};


		// publish once all new samples are processed
		if (!_sensor_sub.updated()) {
			bool publish = true;

			if (_param_imu_gyro_rate_max.get() > 0) {
				const uint64_t interval = 1e6f / _param_imu_gyro_rate_max.get();

				if (hrt_elapsed_time(&_last_publish) < interval) {
					publish = false;
				}
			}

			if (publish) {
				// Publish vehicle_angular_acceleration
				vehicle_angular_acceleration_s v_angular_acceleration;
				v_angular_acceleration.timestamp_sample = sensor_data.timestamp_sample;
				angular_acceleration.copyTo(v_angular_acceleration.xyz);
				v_angular_acceleration.timestamp = hrt_absolute_time();
				_vehicle_angular_acceleration_pub.publish(v_angular_acceleration);

				// Publish vehicle_angular_velocity
				vehicle_angular_velocity_s v_angular_velocity;
				v_angular_velocity.timestamp_sample = sensor_data.timestamp_sample;
				angular_velocity.copyTo(v_angular_velocity.xyz);
				v_angular_velocity.timestamp = hrt_absolute_time();
				_vehicle_angular_velocity_pub.publish(v_angular_velocity);

				_last_publish = v_angular_velocity.timestamp_sample;
				return;
			}
		}
	}
}

void VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO("selected sensor: %d, rate: %.1f Hz, estimated bias: [%.4f %.4f %.4f]",
		 _calibration.device_id(), (double)_filter_sample_rate,
		 (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();
}

} // namespace sensors
