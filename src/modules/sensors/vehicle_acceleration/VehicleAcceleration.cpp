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

#include "VehicleAcceleration.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

namespace sensors
{

VehicleAcceleration::VehicleAcceleration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_lp_filter.set_cutoff_frequency(kInitialRateHz, _param_imu_accel_cutoff.get());
}

VehicleAcceleration::~VehicleAcceleration()
{
	Stop();
}

bool VehicleAcceleration::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("sensor_selection callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(true)) {
		ScheduleDelayed(10_ms);
	}

	return true;
}

void VehicleAcceleration::Stop()
{
	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}

	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

void VehicleAcceleration::CheckFilters()
{
	if (_interval_count > 1000) {
		bool reset_filters = false;

		// calculate sensor update rate
		const float sample_interval_avg = _interval_sum / _interval_count;

		if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.0f)) {

			_update_rate_hz = 1.e6f / sample_interval_avg;

			// check if sample rate error is greater than 1%
			if ((fabsf(_update_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
				reset_filters = true;
			}

			if (reset_filters || (_required_sample_updates == 0)) {
				if (_param_imu_integ_rate.get() > 0) {
					// determine number of sensor samples that will get closest to the desired rate
					const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
					const uint8_t samples = math::constrain(roundf(configured_interval_us / sample_interval_avg), 1.f,
										(float)sensor_accel_s::ORB_QUEUE_LENGTH);

					_sensor_sub[_selected_sensor_sub_index].set_required_updates(samples);
					_required_sample_updates = samples;

				} else {
					_sensor_sub[_selected_sensor_sub_index].set_required_updates(1);
					_required_sample_updates = 1;
				}
			}
		}

		if (!reset_filters) {
			// accel low pass cutoff frequency changed
			if (fabsf(_lp_filter.get_cutoff_freq() - _param_imu_accel_cutoff.get()) > 0.01f) {
				reset_filters = true;
			}
		}

		if (reset_filters) {
			PX4_DEBUG("resetting filters, sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)_update_rate_hz);
			_filter_sample_rate = _update_rate_hz;

			// update software low pass filters
			_lp_filter.set_cutoff_frequency(_filter_sample_rate, _param_imu_accel_cutoff.get());
			_lp_filter.reset(_acceleration_prev);
		}

		// reset sample interval accumulator
		_timestamp_sample_last = 0;
	}
}

void VehicleAcceleration::SensorBiasUpdate(bool force)
{
	if (_estimator_sensor_bias_sub.updated() || force) {
		estimator_sensor_bias_s bias;

		if (_estimator_sensor_bias_sub.copy(&bias)) {
			if (bias.accel_device_id == _selected_sensor_device_id) {
				_bias = Vector3f{bias.accel_bias};

			} else {
				_bias.zero();
			}
		}
	}
}

bool VehicleAcceleration::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if (_selected_sensor_device_id != sensor_selection.accel_device_id) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				sensor_accel_s report{};
				_sensor_sub[i].copy(&report);

				if ((report.device_id != 0) && (report.device_id == sensor_selection.accel_device_id)) {
					if (_sensor_sub[i].registerCallback()) {
						PX4_DEBUG("selected sensor changed %d -> %d", _selected_sensor_sub_index, i);

						// record selected sensor (array index)
						_selected_sensor_sub_index = i;
						_selected_sensor_device_id = sensor_selection.accel_device_id;

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(report.device_id);

						// reset sample interval accumulator on sensor change
						_timestamp_sample_last = 0;
						_required_sample_updates = 0;

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.accel_device_id);
			_selected_sensor_device_id = 0;
			_selected_sensor_sub_index = 0;
		}
	}

	return false;
}

void VehicleAcceleration::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_calibration.ParametersUpdate();
	}
}

void VehicleAcceleration::Run()
{
	// backup schedule
	ScheduleDelayed(10_ms);

	// update corrections first to set _selected_sensor
	bool selection_updated = SensorSelectionUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);
	ParametersUpdate();

	bool sensor_updated = _sensor_sub[_selected_sensor_sub_index].updated();

	// process all outstanding messages
	while (sensor_updated || selection_updated) {
		selection_updated = false;

		sensor_accel_s sensor_data;

		if (_sensor_sub[_selected_sensor_sub_index].copy(&sensor_data)) {

			if (sensor_updated) {
				// collect sample interval average for filters
				if ((_timestamp_sample_last > 0) && (sensor_data.timestamp_sample > _timestamp_sample_last)) {
					_interval_sum += (sensor_data.timestamp_sample - _timestamp_sample_last);
					_interval_count++;

				} else {
					_interval_sum = 0.f;
					_interval_count = 0.f;
				}

				_timestamp_sample_last = sensor_data.timestamp_sample;
			}

			CheckFilters();

			// Apply calibration and filter
			//  - calibration offsets, scale factors, and thermal scale (if available)
			//  - estimated in run bias (if available)
			//  - biquad low-pass filter
			const Vector3f accel_corrected = _calibration.Correct(Vector3f{sensor_data.x, sensor_data.y, sensor_data.z}) - _bias;
			const Vector3f accel_filtered = _lp_filter.apply(accel_corrected);

			_acceleration_prev = accel_corrected;

			// publish once all new samples are processed
			sensor_updated = _sensor_sub[_selected_sensor_sub_index].updated();

			if (!sensor_updated) {
				// Publish vehicle_acceleration
				vehicle_acceleration_s v_acceleration;
				v_acceleration.timestamp_sample = sensor_data.timestamp_sample;
				accel_filtered.copyTo(v_acceleration.xyz);
				v_acceleration.timestamp = hrt_absolute_time();
				_vehicle_acceleration_pub.publish(v_acceleration);

				return;
			}
		}
	}
}

void VehicleAcceleration::PrintStatus()
{
	PX4_INFO("selected sensor: %d (%d), rate: %.1f Hz",
		 _selected_sensor_device_id, _selected_sensor_sub_index, (double)_update_rate_hz);
	PX4_INFO("estimated bias: [%.4f %.4f %.4f]", (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();
}

} // namespace sensors
