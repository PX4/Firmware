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
using math::radians;

GyroFFT::GyroFFT() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	arm_rfft_init_q15(&_rfft_q15, FFT_LENGTH, 0, 1);

	// init Hanning window
	for (int n = 0; n < FFT_LENGTH; n++) {
		const float hanning_value = 0.5f * (1.f - cosf(2.f * M_PI_F * n / (FFT_LENGTH - 1)));
		arm_float_to_q15(&hanning_value, &_hanning_window[n], 1);
	}
}

GyroFFT::~GyroFFT()
{
	perf_free(_cycle_perf);
	perf_free(_cycle_interval_perf);
	perf_free(_fft_perf);
	perf_free(_gyro_fifo_generation_gap_perf);
}

bool GyroFFT::init()
{
	if (!SensorSelectionUpdate(true)) {
		PX4_WARN("sensor_gyro_fifo callback registration failed!");
		ScheduleDelayed(500_ms);
	}

	return true;
}

bool GyroFFT::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if (_selected_sensor_device_id != sensor_selection.gyro_device_id) {
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

				if ((sensor_gyro_fifo_sub.get().device_id != 0)
				    && (sensor_gyro_fifo_sub.get().device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_gyro_fifo_sub.ChangeInstance(i) && _sensor_gyro_fifo_sub.registerCallback()) {
						// find corresponding vehicle_imu_status instance
						for (uint8_t imu_status = 0; imu_status < MAX_SENSOR_COUNT; imu_status++) {
							uORB::Subscription imu_status_sub{ORB_ID(vehicle_imu_status), imu_status};

							vehicle_imu_status_s vehicle_imu_status;

							if (imu_status_sub.copy(&vehicle_imu_status)) {
								if (vehicle_imu_status.gyro_device_id == sensor_selection.gyro_device_id) {
									_vehicle_imu_status_sub.ChangeInstance(imu_status);
									return true;
								}
							}
						}

						PX4_WARN("unable to find IMU status for gyro %d", sensor_selection.gyro_device_id);
						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
		}
	}

	return false;
}

void GyroFFT::VehicleIMUStatusUpdate()
{
	vehicle_imu_status_s vehicle_imu_status;

	if (_vehicle_imu_status_sub.update(&vehicle_imu_status)) {
		if ((vehicle_imu_status.gyro_rate_hz > 0) && (fabsf(vehicle_imu_status.gyro_rate_hz - _gyro_sample_rate_hz) > 1.f)) {
			_gyro_sample_rate_hz = vehicle_imu_status.gyro_rate_hz;
		}
	}
}

// helper function used for frequency estimation
static constexpr float tau(float x)
{
	float p1 = logf(3.f * powf(x, 2.f) + 6 * x + 1);
	float part1 = x + 1 - sqrtf(2.f / 3.f);
	float part2 = x + 1 + sqrtf(2.f / 3.f);
	float p2 = logf(part1 / part2);
	return (1.f / 4.f * p1 - sqrtf(6) / 24 * p2);
}

float GyroFFT::EstimatePeakFrequency(q15_t fft[FFT_LENGTH * 2], uint8_t peak_index)
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
	float peak_freq_adjusted = (_gyro_sample_rate_hz * adjusted_bin / (FFT_LENGTH * 2.f));

	return peak_freq_adjusted;
}

static int float_cmp(const void *elem1, const void *elem2)
{
	if (*(const float *)elem1 < * (const float *)elem2) {
		return -1;
	}

	return *(const float *)elem1 > *(const float *)elem2;
}

void GyroFFT::Run()
{
	if (should_exit()) {
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

	SensorSelectionUpdate();

	const float resolution_hz = _gyro_sample_rate_hz / FFT_LENGTH;

	bool publish = false;
	bool fft_updated = false;

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

		const int N = sensor_gyro_fifo.samples;

		for (int axis = 0; axis < 3; axis++) {
			int16_t *input = nullptr;

			switch (axis) {
			case 0:
				input = sensor_gyro_fifo.x;
				break;

			case 1:
				input = sensor_gyro_fifo.y;
				break;

			case 2:
				input = sensor_gyro_fifo.z;
				break;
			}

			int &buffer_index = _fft_buffer_index[axis];

			for (int n = 0; n < N; n++) {
				if (buffer_index < FFT_LENGTH) {
					// convert int16_t -> q15_t (scaling isn't relevant)
					_gyro_data_buffer[axis][buffer_index] = input[n] / 2;
					buffer_index++;
				}

				// if we have enough samples begin processing, but only one FFT per cycle
				if ((buffer_index >= FFT_LENGTH) && !fft_updated) {

					arm_mult_q15(_gyro_data_buffer[axis], _hanning_window, _fft_input_buffer, FFT_LENGTH);

					perf_begin(_fft_perf);
					arm_rfft_q15(&_rfft_q15, _fft_input_buffer, _fft_outupt_buffer);
					perf_end(_fft_perf);
					fft_updated = true;

					static constexpr uint16_t MIN_SNR = 10; // TODO:

					uint32_t max_peak_magnitude = 0;
					uint8_t max_peak_index = 0;

					static constexpr int MAX_NUM_PEAKS = 4;
					uint32_t peaks_magnitude[MAX_NUM_PEAKS] {};
					uint8_t peak_index[MAX_NUM_PEAKS] {};

					// start at 2 to skip DC
					// output is ordered [real[0], imag[0], real[1], imag[1], real[2], imag[2] ... real[(N/2)-1], imag[(N/2)-1]
					for (uint8_t bucket_index = 2; bucket_index < (FFT_LENGTH / 2); bucket_index = bucket_index + 2) {
						const float freq_hz = (bucket_index / 2) * resolution_hz;

						if (freq_hz > _param_imu_gyro_fft_max.get()) {
							break;
						}

						if (freq_hz >= _param_imu_gyro_fft_min.get()) {
							const int16_t real = _fft_outupt_buffer[bucket_index];
							const int16_t complex = _fft_outupt_buffer[bucket_index + 1];

							const uint32_t fft_magnitude_squared = real * real + complex * complex;

							if (fft_magnitude_squared > MIN_SNR) {

								if (fft_magnitude_squared > max_peak_magnitude) {
									max_peak_magnitude = fft_magnitude_squared;
									max_peak_index = bucket_index;
								}

								for (int i = 0; i < MAX_NUM_PEAKS; i++) {
									if (fft_magnitude_squared > peaks_magnitude[i]) {
										peaks_magnitude[i] = fft_magnitude_squared;
										peak_index[i] = bucket_index;
										publish = true;
										break;
									}
								}
							}
						}
					}

					if (max_peak_index > 0) {
						_sensor_gyro_fft.peak_frequency[axis] = _median_filter[axis].apply(EstimatePeakFrequency(_fft_outupt_buffer,
											max_peak_index));
					}

					if (publish) {
						float *peak_frequencies;

						switch (axis) {
						case 0:
							peak_frequencies = _sensor_gyro_fft.peak_frequencies_x;
							break;

						case 1:
							peak_frequencies = _sensor_gyro_fft.peak_frequencies_y;
							break;

						case 2:
							peak_frequencies = _sensor_gyro_fft.peak_frequencies_z;
							break;
						}

						int peaks_found = 0;

						for (int i = 0; i < MAX_NUM_PEAKS; i++) {
							if ((peak_index[i] > 0) && (peak_index[i] < FFT_LENGTH) && (peaks_magnitude[i] > 0)) {
								const float freq = EstimatePeakFrequency(_fft_outupt_buffer, peak_index[i]);

								if (freq >= _param_imu_gyro_fft_min.get() && freq <= _param_imu_gyro_fft_max.get()) {
									peak_frequencies[peaks_found] = freq;
									peaks_found++;
								}
							}
						}

						// mark remaining slots empty
						for (int i = peaks_found; i < MAX_NUM_PEAKS; i++) {
							peak_frequencies[i] = NAN;
						}

						// publish in sorted order for convenience
						if (peaks_found > 0) {
							qsort(peak_frequencies, peaks_found, sizeof(float), float_cmp);
						}
					}

					// reset
					// shift buffer (75% overlap)
					int overlap_start = FFT_LENGTH / 4;
					memmove(&_gyro_data_buffer[axis][0], &_gyro_data_buffer[axis][overlap_start], sizeof(q15_t) * overlap_start * 3);
					buffer_index = overlap_start * 3;
				}
			}
		}

		if (publish) {
			_sensor_gyro_fft.dt = 1e6f / _gyro_sample_rate_hz;
			_sensor_gyro_fft.device_id = sensor_gyro_fifo.device_id;
			_sensor_gyro_fft.resolution_hz = resolution_hz;
			_sensor_gyro_fft.timestamp_sample = sensor_gyro_fifo.timestamp_sample;
			_sensor_gyro_fft.timestamp = hrt_absolute_time();
			_sensor_gyro_fft_pub.publish(_sensor_gyro_fft);

			publish = false;
		}
	}

	perf_end(_cycle_perf);
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
	perf_print_counter(_cycle_perf);
	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_fft_perf);
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
