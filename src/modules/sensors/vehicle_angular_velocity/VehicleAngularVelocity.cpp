/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();

	perf_free(_filter_reset_perf);
	perf_free(_selection_changed_perf);
#if !defined(CONSTRAINED_FLASH)
	perf_free(_dynamic_notch_filter_esc_rpm_update_perf);
	perf_free(_dynamic_notch_filter_fft_update_perf);
	perf_free(_dynamic_notch_filter_esc_rpm_perf);
	perf_free(_dynamic_notch_filter_fft_perf);
#endif // CONSTRAINED_FLASH
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
	_sensor_fifo_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

bool VehicleAngularVelocity::UpdateSampleRate()
{
	float sample_rate_hz = NAN;
	float publish_rate_hz = NAN;

	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

		if (imu_status.get().gyro_device_id == _selected_sensor_device_id) {
			sample_rate_hz = imu_status.get().gyro_raw_rate_hz;
			publish_rate_hz = imu_status.get().gyro_rate_hz;
			break;
		}
	}

	// calculate sensor update rate
	if ((sample_rate_hz > 0) && PX4_ISFINITE(sample_rate_hz) && (publish_rate_hz > 0) && PX4_ISFINITE(publish_rate_hz)) {
		// check if sample rate error is greater than 1%
		if ((_filter_sample_rate_hz <= FLT_EPSILON) || !PX4_ISFINITE(_filter_sample_rate_hz)
		    || (fabsf(sample_rate_hz - _filter_sample_rate_hz) / sample_rate_hz) > 0.01f) {

			PX4_DEBUG("updating sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate_hz, (double)sample_rate_hz);
			_reset_filters = true;
			_filter_sample_rate_hz = sample_rate_hz;

			if (_param_imu_gyro_ratemax.get() > 0.f) {
				// determine number of sensor samples that will get closest to the desired rate
				const float configured_interval_us = 1e6f / _param_imu_gyro_ratemax.get();
				const float publish_interval_us = 1e6f / publish_rate_hz;

				const uint8_t samples = roundf(configured_interval_us / publish_interval_us);

				if (_fifo_available) {
					_sensor_fifo_sub.set_required_updates(math::constrain(samples, (uint8_t)1, sensor_gyro_fifo_s::ORB_QUEUE_LENGTH));

				} else {
					_sensor_sub.set_required_updates(math::constrain(samples, (uint8_t)1, sensor_gyro_s::ORB_QUEUE_LENGTH));
				}

				// publish interval (constrained 100 Hz - 8 kHz)
				_publish_interval_min_us = math::constrain((int)roundf(configured_interval_us - (publish_interval_us / 2.f)), 125,
							   10000);

			} else {
				_sensor_sub.set_required_updates(1);
				_sensor_fifo_sub.set_required_updates(1);
				_publish_interval_min_us = 0;
			}
		}
	}

	return PX4_ISFINITE(_filter_sample_rate_hz) && (_filter_sample_rate_hz > 0);
}

void VehicleAngularVelocity::ResetFilters(float new_scale)
{
	if ((_filter_sample_rate_hz > 0) && PX4_ISFINITE(_filter_sample_rate_hz)) {

		const Vector3f angular_velocity{GetResetAngularVelocity(new_scale)};
		const Vector3f angular_acceleration{GetResetAngularAcceleration(new_scale)};

		for (int axis = 0; axis < 3; axis++) {
			// angular velocity low pass
			_lp_filter_velocity[axis].set_cutoff_frequency(_filter_sample_rate_hz, _param_imu_gyro_cutoff.get());
			_lp_filter_velocity[axis].reset(angular_velocity(axis));

			// angular velocity notch
			_notch_filter_velocity[axis].setParameters(_filter_sample_rate_hz, _param_imu_gyro_nf_freq.get(),
					_param_imu_gyro_nf_bw.get());
			_notch_filter_velocity[axis].reset(angular_velocity(axis));

			// angular acceleration low pass
			_lp_filter_acceleration[axis].set_cutoff_frequency(_filter_sample_rate_hz, _param_imu_dgyro_cutoff.get());
			_lp_filter_acceleration[axis].reset(angular_acceleration(axis));
		}

		// dynamic notch filters, first disable, then force update (if available)
		DisableDynamicNotchEscRpm();
		DisableDynamicNotchFFT();

		UpdateDynamicNotchEscRpm(new_scale, true);
		UpdateDynamicNotchFFT(new_scale, true);

		_angular_velocity_prev = angular_velocity;
		_last_scale = new_scale;

		_reset_filters = false;
		perf_count(_filter_reset_perf);
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

		if (_estimator_sensor_bias_sub.copy(&bias) && (bias.gyro_device_id == _selected_sensor_device_id)) {
			_bias = Vector3f{bias.gyro_bias};

		} else {
			_bias.zero();
		}
	}
}

bool VehicleAngularVelocity::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if (_selected_sensor_device_id != sensor_selection.gyro_device_id) {

			// see if the selected sensor publishes sensor_gyro_fifo
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

				if ((sensor_gyro_fifo_sub.get().device_id != 0)
				    && (sensor_gyro_fifo_sub.get().device_id == sensor_selection.gyro_device_id)) {
					if (_sensor_fifo_sub.ChangeInstance(i) && _sensor_fifo_sub.registerCallback()) {
						// make sure non-FIFO sub is unregistered
						_sensor_sub.unregisterCallback();

						// record selected sensor
						_selected_sensor_device_id = sensor_selection.gyro_device_id;
						_calibration.set_device_id(sensor_gyro_fifo_sub.get().device_id);

						_timestamp_sample_last = 0;
						_filter_sample_rate_hz = NAN;
						_reset_filters = true;
						_bias.zero();
						_fifo_available = true;
						_last_scale = 0.f;

						perf_count(_selection_changed_perf);

						PX4_DEBUG("selecting gyro FIFO %d %d", i, _selected_sensor_device_id);

						return true;
					}
				}
			}

			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if ((sensor_gyro_sub.get().device_id != 0)
				    && (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id)) {
					if (_sensor_sub.ChangeInstance(i) && _sensor_sub.registerCallback()) {
						// make sure FIFO sub is unregistered
						_sensor_fifo_sub.unregisterCallback();

						// record selected sensor
						_calibration.set_device_id(sensor_gyro_sub.get().device_id);
						_selected_sensor_device_id = sensor_selection.gyro_device_id;

						_timestamp_sample_last = 0;
						_filter_sample_rate_hz = NAN;
						_reset_filters = true;
						_bias.zero();
						_fifo_available = false;
						_last_scale = 1.f;

						perf_count(_selection_changed_perf);

						PX4_DEBUG("selecting gyro %d %d", i, _selected_sensor_device_id);

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
			_selected_sensor_device_id = 0;
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

		// gyro low pass cutoff frequency changed
		for (auto &lp : _lp_filter_velocity) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_gyro_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}

		// gyro notch filter frequency or bandwidth changed
		for (auto &nf : _notch_filter_velocity) {
			if ((fabsf(nf.getNotchFreq() - _param_imu_gyro_nf_freq.get()) > 0.01f)
			    || (fabsf(nf.getBandwidth() - _param_imu_gyro_nf_bw.get()) > 0.01f)) {

				_reset_filters = true;
				break;
			}
		}

		// gyro derivative low pass cutoff changed
		for (auto &lp : _lp_filter_acceleration) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_dgyro_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}

#if !defined(CONSTRAINED_FLASH)

		if (_param_imu_gyro_dyn_nf.get() & DynamicNotch::EscRpm) {
			if (_dynamic_notch_filter_esc_rpm_update_perf == nullptr) {
				_dynamic_notch_filter_esc_rpm_update_perf = perf_alloc(PC_COUNT,
						MODULE_NAME": gyro dynamic notch filter ESC RPM update");
			}

			if (_dynamic_notch_filter_esc_rpm_perf == nullptr) {
				_dynamic_notch_filter_esc_rpm_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": gyro dynamic notch ESC RPM filter");
			}

		} else {
			DisableDynamicNotchEscRpm();
		}

		if (_param_imu_gyro_dyn_nf.get() & DynamicNotch::FFT) {
			if (_dynamic_notch_filter_fft_update_perf == nullptr) {
				_dynamic_notch_filter_fft_update_perf = perf_alloc(PC_COUNT, MODULE_NAME": gyro dynamic notch filter FFT update");
			}

			if (_dynamic_notch_filter_fft_perf == nullptr) {
				_dynamic_notch_filter_fft_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": gyro dynamic notch FFT filter");
			}

		} else {
			DisableDynamicNotchFFT();
		}

#endif // !CONSTRAINED_FLASH
	}
}

Vector3f VehicleAngularVelocity::GetResetAngularVelocity(float new_scale) const
{
	if ((_last_publish != 0) && (new_scale > 0.f)) {
		// angular velocity filtering is performed on raw unscaled data
		//  start with last valid vehicle body frame angular velocity and compute equivalent raw data (for current sensor selection)
		Vector3f angular_velocity{_calibration.Uncorrect(_angular_velocity + _bias) / new_scale};

		if (PX4_ISFINITE(angular_velocity(0))
		    && PX4_ISFINITE(angular_velocity(1))
		    && PX4_ISFINITE(angular_velocity(2))) {

			return angular_velocity;
		}
	}

	return Vector3f{0.f, 0.f, 0.f};
}

Vector3f VehicleAngularVelocity::GetResetAngularAcceleration(float new_scale) const
{
	if ((_last_publish != 0) && (new_scale > 0.f)) {
		// angular acceleration filtering is performed on unscaled angular velocity data
		//  start with last valid vehicle body frame angular acceleration and compute equivalent raw data (for current sensor selection)
		Vector3f angular_acceleration{_calibration.rotation().I() *_angular_acceleration / new_scale};

		if (PX4_ISFINITE(angular_acceleration(0))
		    && PX4_ISFINITE(angular_acceleration(1))
		    && PX4_ISFINITE(angular_acceleration(2))) {

			return angular_acceleration;
		}
	}

	return Vector3f{0.f, 0.f, 0.f};
}

void VehicleAngularVelocity::DisableDynamicNotchEscRpm()
{
#if !defined(CONSTRAINED_FLASH)

	// device id mismatch, disable all
	for (auto &dnf : _dynamic_notch_filter_esc_rpm) {
		for (int harmonic = 0; harmonic < MAX_NUM_ESC_RPM_HARMONICS; harmonic++) {
			for (int axis = 0; axis < 3; axis++) {
				dnf[harmonic][axis].setParameters(0, 0, 0);
			}
		}
	}

	_dynamic_notch_esc_rpm_available = false;
#endif // !CONSTRAINED_FLASH
}

void VehicleAngularVelocity::DisableDynamicNotchFFT()
{
#if !defined(CONSTRAINED_FLASH)

	// device id mismatch, disable all
	for (auto &dnf : _dynamic_notch_filter_fft) {
		for (int axis = 0; axis < 3; axis++) {
			dnf[axis].setParameters(0, 0, 0);
		}
	}

	_dynamic_notch_fft_available = false;
#endif // !CONSTRAINED_FLASH
}

void VehicleAngularVelocity::UpdateDynamicNotchEscRpm(float new_scale, bool force)
{
#if !defined(CONSTRAINED_FLASH)
	const bool enabled = _param_imu_gyro_dyn_nf.get() & DynamicNotch::EscRpm;

	if (enabled && (_esc_status_sub.updated() || force)) {
		_dynamic_notch_esc_rpm_available = false;

		esc_status_s esc_status;

		if (_esc_status_sub.copy(&esc_status)) {
			for (size_t i = 0; i < MAX_NUM_ESC_RPM; i++) {
				static constexpr int32_t MIN_ESC_RPM = 20 * 60; // 20 Hz safe minimum limit TODO: configurable

				if ((esc_status.esc[i].timestamp != 0) && ((_timestamp_sample_last - esc_status.esc[i].timestamp) < 1_s)
				    && (esc_status.esc[i].esc_rpm > MIN_ESC_RPM)) {

					const float esc_hz = static_cast<float>(esc_status.esc[i].esc_rpm) / 60.f;

					for (int harmonic = 0; harmonic < MAX_NUM_ESC_RPM_HARMONICS; harmonic++) {
						const float frequency_hz = esc_hz * (harmonic + 1);

						auto &dnf0 = _dynamic_notch_filter_esc_rpm[i][harmonic][0];
						const float change_percent = fabsf(dnf0.getNotchFreq() - frequency_hz) / frequency_hz;

						if (change_percent > 0.001f) {
							// peak frequency changed by at least 0.1%
							for (int axis = 0; axis < 3; axis++) {
								auto &dnf = _dynamic_notch_filter_esc_rpm[i][harmonic][axis];
								dnf.setParameters(_filter_sample_rate_hz, frequency_hz, 1.f); // TODO: configurable bandwidth
							}

							// only reset if there's sufficient change (> 1%)
							if (force || (change_percent > 0.01f)) {
								const Vector3f reset_angular_velocity{GetResetAngularVelocity(new_scale)};

								for (int axis = 0; axis < 3; axis++) {
									auto &dnf = _dynamic_notch_filter_esc_rpm[i][harmonic][axis];
									dnf.reset(reset_angular_velocity(axis));
								}
							}

							perf_count(_dynamic_notch_filter_esc_rpm_update_perf);
						}
					}

					_dynamic_notch_esc_rpm_available = true;

				} else {
					// disable all notch filters for this ESC
					for (int harmonic = 0; harmonic < MAX_NUM_ESC_RPM_HARMONICS; harmonic++) {
						for (int axis = 0; axis < 3; axis++) {
							_dynamic_notch_filter_esc_rpm[i][harmonic][axis].setParameters(0, 0, 0);
						}
					}
				}
			}
		}
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAngularVelocity::UpdateDynamicNotchFFT(float new_scale, bool force)
{
#if !defined(CONSTRAINED_FLASH)
	const bool enabled = _param_imu_gyro_dyn_nf.get() & DynamicNotch::FFT;

	if (enabled && (_sensor_gyro_fft_sub.updated() || force)) {
		_dynamic_notch_fft_available = false;

		sensor_gyro_fft_s sensor_gyro_fft;

		if (_sensor_gyro_fft_sub.copy(&sensor_gyro_fft) && (sensor_gyro_fft.device_id == _selected_sensor_device_id)
		    && (fabsf(sensor_gyro_fft.sensor_sample_rate_hz - _filter_sample_rate_hz) < 10.f)) {

			float *peak_frequencies[] {sensor_gyro_fft.peak_frequencies_x, sensor_gyro_fft.peak_frequencies_y, sensor_gyro_fft.peak_frequencies_z};

			for (int axis = 0; axis < 3; axis++) {
				for (int i = 0; i < MAX_NUM_FFT_PEAKS; i++) {
					auto &dnf = _dynamic_notch_filter_fft[i][axis];
					const float &peak_freq = peak_frequencies[axis][i];

					if (PX4_ISFINITE(peak_freq) && (peak_freq > 1.f)) {
						const float peak_diff_abs = fabsf(dnf.getNotchFreq() - peak_freq);
						const float change_percent = peak_diff_abs / peak_freq;

						if (force || (change_percent > 0.001f)) {
							// peak frequency changed by at least 0.1%
							dnf.setParameters(_filter_sample_rate_hz, peak_freq, sensor_gyro_fft.resolution_hz);

							// only reset if there's sufficient change
							if (peak_diff_abs > sensor_gyro_fft.resolution_hz) {
								dnf.reset(GetResetAngularVelocity(new_scale)(axis));
							}

							perf_count(_dynamic_notch_filter_fft_update_perf);
						}

						_dynamic_notch_fft_available = true;

					} else {
						// disable this notch filter
						dnf.setParameters(0, 0, 0);
					}
				}
			}

		} else {
			DisableDynamicNotchFFT();
		}
	}

#endif // !CONSTRAINED_FLASH
}

float VehicleAngularVelocity::FilterAngularVelocity(int axis, float data[], int N)
{
#if !defined(CONSTRAINED_FLASH)

	// Apply dynamic notch filter from ESC RPM
	if (_dynamic_notch_esc_rpm_available) {
		perf_begin(_dynamic_notch_filter_esc_rpm_perf);

		for (auto &dnf : _dynamic_notch_filter_esc_rpm) {
			for (int harmonic = 0; harmonic < MAX_NUM_ESC_RPM_HARMONICS; harmonic++) {
				if (dnf[harmonic][axis].getNotchFreq() > 0.f) {
					dnf[harmonic][axis].applyDF1(data, N);

				} else {
					break;
				}
			}
		}

		perf_end(_dynamic_notch_filter_esc_rpm_perf);
	}

	// Apply dynamic notch filter from FFT
	if (_dynamic_notch_fft_available) {
		perf_begin(_dynamic_notch_filter_fft_perf);

		for (auto &dnf : _dynamic_notch_filter_fft) {
			if (dnf[axis].getNotchFreq() > 0.f) {
				dnf[axis].applyDF1(data, N);
			}
		}

		perf_end(_dynamic_notch_filter_fft_perf);
	}

#endif // !CONSTRAINED_FLASH

	// Apply general notch filter (IMU_GYRO_NF_FREQ)
	if (_notch_filter_velocity[axis].getNotchFreq() > 0.f) {
		_notch_filter_velocity[axis].apply(data, N);
	}

	// Apply general low-pass filter (IMU_GYRO_CUTOFF)
	_lp_filter_velocity[axis].apply(data, N);

	// return last filtered sample
	return data[N - 1];
}

float VehicleAngularVelocity::FilterAngularAcceleration(int axis, float dt_s, float data[], int N)
{
	// angular acceleration: Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
	float angular_acceleration_filtered = 0.f;

	for (int n = 0; n < N; n++) {
		const float angular_acceleration = (data[n] - _angular_velocity_prev(axis)) / dt_s;
		angular_acceleration_filtered = _lp_filter_acceleration[axis].apply(angular_acceleration);
		_angular_velocity_prev(axis) = data[n];
	}

	return angular_acceleration_filtered;
}

void VehicleAngularVelocity::Run()
{
	// backup schedule
	ScheduleDelayed(10_ms);

	ParametersUpdate();

	// update corrections first to set _selected_sensor
	const bool selection_updated = SensorSelectionUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);

	if (selection_updated || !PX4_ISFINITE(_filter_sample_rate_hz) || (_filter_sample_rate_hz <= FLT_EPSILON)) {
		if (!UpdateSampleRate()) {
			// sensor sample rate required to run
			return;
		}
	}

	if (_fifo_available) {
		// process all outstanding fifo messages
		sensor_gyro_fifo_s sensor_fifo_data;

		while (_sensor_fifo_sub.update(&sensor_fifo_data)) {
			const float dt_s = sensor_fifo_data.dt * 1e-6f;
			_timestamp_sample_last = sensor_fifo_data.timestamp_sample;

			// in FIFO mode the unscaled raw data is filtered, reset filters on any scale change
			if (_reset_filters || (fabsf(sensor_fifo_data.scale - _last_scale) > FLT_EPSILON)) {
				ResetFilters(sensor_fifo_data.scale);

				if (_reset_filters) {
					continue; // not safe to run until filters configured
				}
			}

			const int N = sensor_fifo_data.samples;
			static constexpr int FIFO_SIZE_MAX = sizeof(sensor_fifo_data.x) / sizeof(sensor_fifo_data.x[0]);

			if ((N > 0) && (N <= FIFO_SIZE_MAX)) {
				UpdateDynamicNotchEscRpm();
				UpdateDynamicNotchFFT();

				Vector3f angular_velocity_unscaled;
				Vector3f angular_acceleration_unscaled;

				int16_t *raw_data_array[] {sensor_fifo_data.x, sensor_fifo_data.y, sensor_fifo_data.z};

				for (int axis = 0; axis < 3; axis++) {
					// copy raw int16 sensor samples to float array for filtering
					float data[FIFO_SIZE_MAX];

					for (int n = 0; n < N; n++) {
						data[n] = raw_data_array[axis][n];
					}

					// save last filtered sample
					angular_velocity_unscaled(axis) = FilterAngularVelocity(axis, data, N);
					angular_acceleration_unscaled(axis) = FilterAngularAcceleration(axis, dt_s, data, N);
				}

				// Publish
				CalibrateAndPublish(!_sensor_fifo_sub.updated(), sensor_fifo_data.timestamp_sample, angular_velocity_unscaled,
						    angular_acceleration_unscaled, sensor_fifo_data.scale);
			}
		}

	} else {
		// process all outstanding messages
		sensor_gyro_s sensor_data;

		while (_sensor_sub.update(&sensor_data)) {
			if (_timestamp_sample_last == 0) {
				_timestamp_sample_last = sensor_data.timestamp_sample - 1e6f / _filter_sample_rate_hz;
			}

			const float dt_s = math::constrain(((sensor_data.timestamp_sample - _timestamp_sample_last) / 1e6f), 0.0002f, 0.02f);
			_timestamp_sample_last = sensor_data.timestamp_sample;

			if (_reset_filters) {
				ResetFilters();

				if (_reset_filters) {
					continue; // not safe to run until filters configured
				}
			}

			UpdateDynamicNotchEscRpm();
			UpdateDynamicNotchFFT();

			Vector3f angular_velocity;
			Vector3f angular_acceleration;

			float raw_data_array[] {sensor_data.x, sensor_data.y, sensor_data.z};

			for (int axis = 0; axis < 3; axis++) {
				// copy sensor sample to float array for filtering
				float data[1] {raw_data_array[axis]};

				// save last filtered sample
				angular_velocity(axis) = FilterAngularVelocity(axis, data);
				angular_acceleration(axis) = FilterAngularAcceleration(axis, dt_s, data);
			}

			// Publish
			CalibrateAndPublish(!_sensor_sub.updated(), sensor_data.timestamp_sample, angular_velocity, angular_acceleration);
		}
	}
}

void VehicleAngularVelocity::CalibrateAndPublish(bool publish, const hrt_abstime &timestamp_sample,
		const Vector3f &angular_velocity_unscaled,  const Vector3f &angular_acceleration_unscaled, float scale)
{
	// Angular velocity: rotate sensor frame to board, scale raw data to SI, apply calibration, and remove in-run estimated bias
	_angular_velocity = _calibration.Correct(angular_velocity_unscaled * scale) - _bias;

	// Angular acceleration: rotate sensor frame to board, scale raw data to SI, apply any additional configured rotation
	_angular_acceleration = _calibration.rotation() * angular_acceleration_unscaled * scale;

	if (publish && (timestamp_sample >= _last_publish + _publish_interval_min_us)) {

		// Publish vehicle_angular_acceleration
		vehicle_angular_acceleration_s v_angular_acceleration;
		v_angular_acceleration.timestamp_sample = timestamp_sample;
		_angular_acceleration.copyTo(v_angular_acceleration.xyz);
		v_angular_acceleration.timestamp = hrt_absolute_time();
		_vehicle_angular_acceleration_pub.publish(v_angular_acceleration);

		// Publish vehicle_angular_velocity
		vehicle_angular_velocity_s v_angular_velocity;
		v_angular_velocity.timestamp_sample = timestamp_sample;
		_angular_velocity.copyTo(v_angular_velocity.xyz);
		v_angular_velocity.timestamp = hrt_absolute_time();
		_vehicle_angular_velocity_pub.publish(v_angular_velocity);

		// shift last publish time forward, but don't let it get further behind than the interval
		_last_publish = math::constrain(_last_publish + _publish_interval_min_us,
						timestamp_sample - _publish_interval_min_us, timestamp_sample);
	}
}

void VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO("selected sensor: %d, rate: %.1f Hz %s",
		 _selected_sensor_device_id, (double)_filter_sample_rate_hz, _fifo_available ? "FIFO" : "");
	PX4_INFO("estimated bias: [%.4f %.4f %.4f]", (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();
}

} // namespace sensors
