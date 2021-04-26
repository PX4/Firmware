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

#include "GyroFFT.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

GyroFFT::GyroFFT() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

GyroFFT::~GyroFFT()
{
	perf_free(_cycle_perf);
	perf_free(_cycle_interval_perf);
	perf_free(_fft_perf);
	perf_free(_gyro_generation_gap_perf);
	perf_free(_gyro_fifo_generation_gap_perf);

	delete[] _gyro_data_buffer_x;
	delete[] _gyro_data_buffer_y;
	delete[] _gyro_data_buffer_z;
	delete[] _hanning_window;
	delete[] _fft_input_buffer;
	delete[] _fft_outupt_buffer;
}

bool GyroFFT::init()
{
	bool buffers_allocated = false;

	// arm_rfft_init_q15(&_rfft_q15, _imu_gyro_fft_len, 0, 1) manually inlined to save flash
	_rfft_q15.pTwiddleAReal = (q15_t *) realCoefAQ15;
	_rfft_q15.pTwiddleBReal = (q15_t *) realCoefBQ15;
	_rfft_q15.ifftFlagR = 0;
	_rfft_q15.bitReverseFlagR = 1;

	switch (_param_imu_gyro_fft_len.get()) {
	// case 128:
	// 	buffers_allocated = AllocateBuffers<128>();
	// 	_rfft_q15.fftLenReal = 128;
	// 	_rfft_q15.twidCoefRModifier = 64U;
	// 	_rfft_q15.pCfft = &arm_cfft_sR_q15_len64;
	// 	break;

	case 256:
		buffers_allocated = AllocateBuffers<256>();
		_rfft_q15.fftLenReal = 256;
		_rfft_q15.twidCoefRModifier = 32U;
		_rfft_q15.pCfft = &arm_cfft_sR_q15_len128;
		break;

	// case 512:
	// 	buffers_allocated = AllocateBuffers<512>();
	// 	_rfft_q15.fftLenReal = 512;
	// 	_rfft_q15.twidCoefRModifier = 16U;
	// 	_rfft_q15.pCfft = &arm_cfft_sR_q15_len256;
	// 	break;

	case 1024:
		buffers_allocated = AllocateBuffers<1024>();
		_rfft_q15.fftLenReal = 1024;
		_rfft_q15.twidCoefRModifier = 8U;
		_rfft_q15.pCfft = &arm_cfft_sR_q15_len512;
		break;

	// case 2048:
	// 	buffers_allocated = AllocateBuffers<2048>();
	// 	_rfft_q15.fftLenReal = 2048;
	// 	_rfft_q15.twidCoefRModifier = 4U;
	// 	_rfft_q15.pCfft = &arm_cfft_sR_q15_len1024;
	// 	break;

	case 4096:
		buffers_allocated = AllocateBuffers<4096>();
		_rfft_q15.fftLenReal = 4096;
		_rfft_q15.twidCoefRModifier = 2U;
		_rfft_q15.pCfft = &arm_cfft_sR_q15_len2048;
		break;

	// case 8192:
	// 	buffers_allocated = AllocateBuffers<8192>();
	// 	_rfft_q15.fftLenReal = 8192;
	// 	_rfft_q15.twidCoefRModifier = 1U;
	// 	_rfft_q15.pCfft = &arm_cfft_sR_q15_len4096;
	// 	break;

	default:
		// otherwise default to 256
		PX4_ERR("Invalid IMU_GYRO_FFT_LEN=%.3f, resetting", (double)_param_imu_gyro_fft_len.get());
		AllocateBuffers<256>();
		_param_imu_gyro_fft_len.set(256);
		_param_imu_gyro_fft_len.commit();
		break;
	}

	if (buffers_allocated) {
		_imu_gyro_fft_len = _param_imu_gyro_fft_len.get();

		// init Hanning window
		for (int n = 0; n < _imu_gyro_fft_len; n++) {
			const float hanning_value = 0.5f * (1.f - cosf(2.f * M_PI_F * n / (_imu_gyro_fft_len - 1)));
			arm_float_to_q15(&hanning_value, &_hanning_window[n], 1);
		}

		if (!SensorSelectionUpdate(true)) {
			ScheduleDelayed(500_ms);
		}

		return true;
	}

	PX4_ERR("failed to allocate buffers");
	delete[] _gyro_data_buffer_x;
	delete[] _gyro_data_buffer_y;
	delete[] _gyro_data_buffer_z;
	delete[] _hanning_window;
	delete[] _fft_input_buffer;
	delete[] _fft_outupt_buffer;

	return false;
}

bool GyroFFT::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if ((sensor_selection.gyro_device_id != 0) && (_selected_sensor_device_id != sensor_selection.gyro_device_id)) {
			// prefer sensor_gyro_fifo if available
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

				if (sensor_gyro_fifo_sub.get().device_id == sensor_selection.gyro_device_id) {
					if (_sensor_gyro_fifo_sub.ChangeInstance(i) && _sensor_gyro_fifo_sub.registerCallback()) {
						_sensor_gyro_sub.unregisterCallback();
						_sensor_gyro_fifo_sub.set_required_updates(sensor_gyro_fifo_s::ORB_QUEUE_LENGTH - 1);
						_selected_sensor_device_id = sensor_selection.gyro_device_id;
						_gyro_fifo = true;
						return true;
					}
				}
			}

			// otherwise use sensor_gyro
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id) {
					if (_sensor_gyro_sub.ChangeInstance(i) && _sensor_gyro_sub.registerCallback()) {
						_sensor_gyro_fifo_sub.unregisterCallback();
						_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH - 1);
						_selected_sensor_device_id = sensor_selection.gyro_device_id;
						_gyro_fifo = false;
						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
		}
	}

	return false;
}

void GyroFFT::VehicleIMUStatusUpdate(bool force)
{
	if (_vehicle_imu_status_sub.updated() || force) {
		vehicle_imu_status_s vehicle_imu_status;

		if (_vehicle_imu_status_sub.copy(&vehicle_imu_status)) {
			// find corresponding vehicle_imu_status instance if the device_id doesn't match
			if (vehicle_imu_status.gyro_device_id != _selected_sensor_device_id) {

				for (uint8_t imu_status = 0; imu_status < MAX_SENSOR_COUNT; imu_status++) {
					uORB::Subscription imu_status_sub{ORB_ID(vehicle_imu_status), imu_status};

					if (imu_status_sub.copy(&vehicle_imu_status)) {
						if (vehicle_imu_status.gyro_device_id == _selected_sensor_device_id) {
							_vehicle_imu_status_sub.ChangeInstance(imu_status);
							break;
						}
					}
				}
			}

			// update gyro sample rate
			if ((vehicle_imu_status.gyro_device_id == _selected_sensor_device_id) && (vehicle_imu_status.gyro_rate_hz > 0)) {
				if (_gyro_fifo) {
					_gyro_sample_rate_hz = vehicle_imu_status.gyro_raw_rate_hz;

				} else {
					_gyro_sample_rate_hz = vehicle_imu_status.gyro_rate_hz;
				}

				return;
			}
		}
	}
}

// helper function used for frequency estimation
static float tau(float x)
{
	float p1 = logf(3.f * powf(x, 2.f) + 6 * x + 1);
	float part1 = x + 1 - sqrtf(2.f / 3.f);
	float part2 = x + 1 + sqrtf(2.f / 3.f);
	float p2 = logf(part1 / part2);
	return (1.f / 4.f * p1 - sqrtf(6) / 24 * p2);
}

float GyroFFT::EstimatePeakFrequency(q15_t fft[], uint8_t peak_index)
{
	// find peak location using Quinn's Second Estimator (2020-06-14: http://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/)
	int16_t real[3] { fft[peak_index - 2],     fft[peak_index],     fft[peak_index + 2]     };
	int16_t imag[3] { fft[peak_index - 2 + 1], fft[peak_index + 1], fft[peak_index + 2 + 1] };

	const int k = 1;

	float divider = (real[k] * real[k] + imag[k] * imag[k]);

	// ap = (X[k + 1].r * X[k].r + X[k+1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
	float ap = (real[k + 1] * real[k] + imag[k + 1] * imag[k]) / divider;

	// am = (X[k – 1].r * X[k].r + X[k – 1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
	float am = (real[k - 1] * real[k] + imag[k - 1] * imag[k]) / divider;

	float dp = -ap  / (1.f - ap);
	float dm = am / (1.f - am);
	float d = (dp + dm) / 2 + tau(dp * dp) - tau(dm * dm);

	float adjusted_bin = peak_index + d;
	float peak_freq_adjusted = (_gyro_sample_rate_hz * adjusted_bin / (_imu_gyro_fft_len * 2.f));

	return peak_freq_adjusted;
}

void GyroFFT::Run()
{
	if (should_exit()) {
		_sensor_gyro_sub.unregisterCallback();
		_sensor_gyro_fifo_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// backup schedule
	ScheduleDelayed(500_ms);

	perf_begin(_cycle_perf);
	perf_count(_cycle_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	const bool selection_updated = SensorSelectionUpdate();
	VehicleIMUStatusUpdate(selection_updated);

	if (_gyro_fifo) {
		// run on sensor gyro fifo updates
		sensor_gyro_fifo_s sensor_gyro_fifo;

		while (_sensor_gyro_fifo_sub.update(&sensor_gyro_fifo)) {
			if (_sensor_gyro_fifo_sub.get_last_generation() != _gyro_last_generation + 1) {
				// force reset if we've missed a sample
				_fft_buffer_index[0] = 0;
				_fft_buffer_index[1] = 0;
				_fft_buffer_index[2] = 0;

				perf_count(_gyro_fifo_generation_gap_perf);
			}

			_gyro_last_generation = _sensor_gyro_fifo_sub.get_last_generation();

			if (fabsf(sensor_gyro_fifo.scale - _fifo_last_scale) > FLT_EPSILON) {
				// force reset if scale has changed
				_fft_buffer_index[0] = 0;
				_fft_buffer_index[1] = 0;
				_fft_buffer_index[2] = 0;

				_fifo_last_scale = sensor_gyro_fifo.scale;
			}

			int16_t *input[] {sensor_gyro_fifo.x, sensor_gyro_fifo.y, sensor_gyro_fifo.z};
			Update(sensor_gyro_fifo.timestamp_sample, input, sensor_gyro_fifo.samples);
		}

	} else {
		// run on sensor gyro fifo updates
		sensor_gyro_s sensor_gyro;

		while (_sensor_gyro_sub.update(&sensor_gyro)) {
			if (_sensor_gyro_sub.get_last_generation() != _gyro_last_generation + 1) {
				// force reset if we've missed a sample
				_fft_buffer_index[0] = 0;
				_fft_buffer_index[1] = 0;
				_fft_buffer_index[2] = 0;

				perf_count(_gyro_generation_gap_perf);
			}

			_gyro_last_generation = _sensor_gyro_sub.get_last_generation();

			const float gyro_scale = math::radians(1000.f); // arbitrary scaling float32 rad/s -> raw int16
			int16_t gyro_x[1] {(int16_t)roundf(sensor_gyro.x * gyro_scale)};
			int16_t gyro_y[1] {(int16_t)roundf(sensor_gyro.y * gyro_scale)};
			int16_t gyro_z[1] {(int16_t)roundf(sensor_gyro.z * gyro_scale)};

			int16_t *input[] {gyro_x, gyro_y, gyro_z};
			Update(sensor_gyro.timestamp_sample, input, 1);
		}
	}

	perf_end(_cycle_perf);
}

void GyroFFT::Update(const hrt_abstime &timestamp_sample, int16_t *input[], uint8_t N)
{
	bool publish = false;
	bool fft_updated = false;
	const float resolution_hz = _gyro_sample_rate_hz / _imu_gyro_fft_len;
	q15_t *gyro_data_buffer[] {_gyro_data_buffer_x, _gyro_data_buffer_y, _gyro_data_buffer_z};

	for (int axis = 0; axis < 3; axis++) {
		int &buffer_index = _fft_buffer_index[axis];

		for (int n = 0; n < N; n++) {
			if (buffer_index < _imu_gyro_fft_len) {
				// convert int16_t -> q15_t (scaling isn't relevant)
				gyro_data_buffer[axis][buffer_index] = input[axis][n] / 2;
				buffer_index++;
			}

			// if we have enough samples begin processing, but only one FFT per cycle
			if ((buffer_index >= _imu_gyro_fft_len) && !fft_updated) {
				arm_mult_q15(gyro_data_buffer[axis], _hanning_window, _fft_input_buffer, _imu_gyro_fft_len);

				perf_begin(_fft_perf);
				arm_rfft_q15(&_rfft_q15, _fft_input_buffer, _fft_outupt_buffer);
				perf_end(_fft_perf);
				fft_updated = true;

				static constexpr uint16_t MIN_SNR = 10; // TODO:

				bool peaks_detected = false;
				uint32_t peaks_magnitude[MAX_NUM_PEAKS] {};
				uint8_t peak_index[MAX_NUM_PEAKS] {};

				// start at 2 to skip DC
				// output is ordered [real[0], imag[0], real[1], imag[1], real[2], imag[2] ... real[(N/2)-1], imag[(N/2)-1]
				for (uint16_t bucket_index = 2; bucket_index < (_imu_gyro_fft_len / 2); bucket_index = bucket_index + 2) {
					const float freq_hz = (bucket_index / 2) * resolution_hz;

					if (freq_hz > _param_imu_gyro_fft_max.get()) {
						break;
					}

					if (freq_hz >= _param_imu_gyro_fft_min.get()) {
						const int16_t real = _fft_outupt_buffer[bucket_index];
						const int16_t complex = _fft_outupt_buffer[bucket_index + 1];

						const uint32_t fft_magnitude_squared = real * real + complex * complex;

						if (fft_magnitude_squared > MIN_SNR) {
							for (int i = 0; i < MAX_NUM_PEAKS; i++) {
								if (fft_magnitude_squared > peaks_magnitude[i]) {
									peaks_magnitude[i] = fft_magnitude_squared;
									peak_index[i] = bucket_index;
									peaks_detected = true;
									break;
								}
							}
						}
					}
				}

				if (peaks_detected) {
					float *peak_frequencies[] {_sensor_gyro_fft.peak_frequencies_x, _sensor_gyro_fft.peak_frequencies_y, _sensor_gyro_fft.peak_frequencies_z};
					uint32_t *peak_magnitude[] {_sensor_gyro_fft.peak_magnitude_x, _sensor_gyro_fft.peak_magnitude_y, _sensor_gyro_fft.peak_magnitude_z};

					int num_peaks_found = 0;

					for (int i = 0; i < MAX_NUM_PEAKS; i++) {
						if ((peak_index[i] > 0) && (peak_index[i] < _imu_gyro_fft_len) && (peaks_magnitude[i] > 0)) {
							const float freq = EstimatePeakFrequency(_fft_outupt_buffer, peak_index[i]);

							if (freq >= _param_imu_gyro_fft_min.get() && freq <= _param_imu_gyro_fft_max.get()) {

								if (fabsf(peak_frequencies[axis][num_peaks_found] - freq) > 0.1f) {
									publish = true;
									_sensor_gyro_fft.timestamp_sample = timestamp_sample;
								}

								peak_frequencies[axis][num_peaks_found] = freq;
								peak_magnitude[axis][num_peaks_found] = peaks_magnitude[i];

								num_peaks_found++;
							}
						}
					}

					// mark remaining slots empty
					for (int i = num_peaks_found; i < MAX_NUM_PEAKS; i++) {
						peak_frequencies[axis][i] = NAN;
						peak_magnitude[axis][i] = 0;
					}
				}

				// reset
				// shift buffer (3/4 overlap)
				const int overlap_start = _imu_gyro_fft_len / 4;
				memmove(&gyro_data_buffer[axis][0], &gyro_data_buffer[axis][overlap_start], sizeof(q15_t) * overlap_start * 3);
				buffer_index = overlap_start * 3;
			}
		}
	}

	if (publish) {
		_sensor_gyro_fft.device_id = _selected_sensor_device_id;
		_sensor_gyro_fft.sensor_sample_rate_hz = _gyro_sample_rate_hz;
		_sensor_gyro_fft.resolution_hz = resolution_hz;
		_sensor_gyro_fft.timestamp = hrt_absolute_time();
		_sensor_gyro_fft_pub.publish(_sensor_gyro_fft);

		publish = false;
	}
}

int GyroFFT::task_spawn(int argc, char *argv[])
{
	GyroFFT *instance = new GyroFFT();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int GyroFFT::print_status()
{
	PX4_INFO("gyro sample rate: %.3f Hz", (double)_gyro_sample_rate_hz);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_fft_perf);
	perf_print_counter(_gyro_generation_gap_perf);
	perf_print_counter(_gyro_fifo_generation_gap_perf);
	return 0;
}

int GyroFFT::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GyroFFT::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gyro_fft", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gyro_fft_main(int argc, char *argv[])
{
	return GyroFFT::main(argc, argv);
}
