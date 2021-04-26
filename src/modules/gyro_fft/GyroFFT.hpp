/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fft.h>
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_imu_status.h>

#include "arm_math.h"
#include "arm_const_structs.h"

using namespace time_literals;

class GyroFFT : public ModuleBase<GyroFFT>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	GyroFFT();
	~GyroFFT() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	float EstimatePeakFrequency(q15_t fft[], uint8_t peak_index);
	void Run() override;
	bool SensorSelectionUpdate(bool force = false);
	void Update(const hrt_abstime &timestamp_sample, int16_t *input[], uint8_t N);
	void VehicleIMUStatusUpdate(bool force = false);

	template<size_t N>
	bool AllocateBuffers()
	{
		_gyro_data_buffer_x = new q15_t[N];
		_gyro_data_buffer_y = new q15_t[N];
		_gyro_data_buffer_z = new q15_t[N];
		_hanning_window = new q15_t[N];
		_fft_input_buffer = new q15_t[N];
		_fft_outupt_buffer = new q15_t[N * 2];

		return (_gyro_data_buffer_x && _gyro_data_buffer_y && _gyro_data_buffer_z
			&& _hanning_window
			&& _fft_input_buffer
			&& _fft_outupt_buffer);
	}

	static constexpr int MAX_SENSOR_COUNT = 4;

	static constexpr int MAX_NUM_PEAKS = sizeof(sensor_gyro_fft_s::peak_frequencies_x) / sizeof(
			sensor_gyro_fft_s::peak_frequencies_x[0]);

	uORB::Publication<sensor_gyro_fft_s> _sensor_gyro_fft_pub{ORB_ID(sensor_gyro_fft)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};

	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub{this, ORB_ID(sensor_gyro)};
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_fifo_sub{this, ORB_ID(sensor_gyro_fifo)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _cycle_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
	perf_counter_t _fft_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": FFT")};
	perf_counter_t _gyro_generation_gap_perf{perf_alloc(PC_COUNT, MODULE_NAME": gyro data gap")};
	perf_counter_t _gyro_fifo_generation_gap_perf{perf_alloc(PC_COUNT, MODULE_NAME": gyro FIFO data gap")};

	uint32_t _selected_sensor_device_id{0};

	bool _gyro_fifo{false};

	arm_rfft_instance_q15 _rfft_q15;

	q15_t *_gyro_data_buffer_x{nullptr};
	q15_t *_gyro_data_buffer_y{nullptr};
	q15_t *_gyro_data_buffer_z{nullptr};
	q15_t *_hanning_window{nullptr};
	q15_t *_fft_input_buffer{nullptr};
	q15_t *_fft_outupt_buffer{nullptr};

	float _gyro_sample_rate_hz{8000}; // 8 kHz default

	float _fifo_last_scale{0};

	int _fft_buffer_index[3] {};

	unsigned _gyro_last_generation{0};

	sensor_gyro_fft_s _sensor_gyro_fft{};

	int32_t _imu_gyro_fft_len{256};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_GYRO_FFT_LEN>) _param_imu_gyro_fft_len,
		(ParamFloat<px4::params::IMU_GYRO_FFT_MIN>) _param_imu_gyro_fft_min,
		(ParamFloat<px4::params::IMU_GYRO_FFT_MAX>) _param_imu_gyro_fft_max
	)
};
