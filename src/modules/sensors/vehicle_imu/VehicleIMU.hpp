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

#pragma once

#include "Integrator.hpp"

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>

using namespace time_literals;

namespace sensors
{

class VehicleIMU : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleIMU() = delete;
	VehicleIMU(int instance, uint8_t accel_index, uint8_t gyro_index, const px4::wq_config_t &config);

	~VehicleIMU() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void ParametersUpdate(bool force = false);
	void Run() override;

	struct IntervalAverage {
		hrt_abstime timestamp_sample_last{0};
		uint32_t interval_sum{0};
		uint32_t interval_samples{0};
		uint32_t interval_count{0};
		float update_interval{0.f};
		float update_interval_raw{0.f};
	};

	bool UpdateIntervalAverage(IntervalAverage &intavg, const hrt_abstime &timestamp_sample, uint8_t samples = 1);
	void UpdateIntegratorConfiguration();
	void UpdateGyroVibrationMetrics(const matrix::Vector3f &delta_angle);
	void UpdateAccelVibrationMetrics(const matrix::Vector3f &delta_velocity);

	uORB::PublicationMulti<vehicle_imu_s> _vehicle_imu_pub{ORB_ID(vehicle_imu)};
	uORB::PublicationMulti<vehicle_imu_status_s> _vehicle_imu_status_pub{ORB_ID(vehicle_imu_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub;
	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub;

	calibration::Accelerometer _accel_calibration{};
	calibration::Gyroscope _gyro_calibration{};

	Integrator _accel_integrator{}; // 200 Hz default
	Integrator _gyro_integrator{true};   // 200 Hz default, coning compensation enabled

	hrt_abstime _last_timestamp_sample_accel{0};
	hrt_abstime _last_timestamp_sample_gyro{0};

	uint32_t _imu_integration_interval_us{4000};

	IntervalAverage _accel_interval{};
	IntervalAverage _gyro_interval{};

	unsigned _accel_last_generation{0};
	unsigned _gyro_last_generation{0};
	unsigned _consecutive_data_gap{0};

	matrix::Vector3f _accel_sum{};
	matrix::Vector3f _gyro_sum{};
	int _accel_sum_count{0};
	int _gyro_sum_count{0};
	float _accel_temperature{0};
	float _gyro_temperature{0};

	matrix::Vector3f _delta_angle_prev{0.f, 0.f, 0.f};	// delta angle from the previous IMU measurement
	matrix::Vector3f _delta_velocity_prev{0.f, 0.f, 0.f};	// delta velocity from the previous IMU measurement

	vehicle_imu_status_s _status{};

	uint8_t _delta_velocity_clipping{0};

	hrt_abstime _last_clipping_notify_time{0};
	uint64_t _last_clipping_notify_total_count{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	bool _intervals_configured{false};

	const uint8_t _instance;

	perf_counter_t _accel_update_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": accel update interval")};
	perf_counter_t _accel_generation_gap_perf{perf_alloc(PC_COUNT, MODULE_NAME": accel data gap")};
	perf_counter_t _gyro_update_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": gyro update interval")};
	perf_counter_t _gyro_generation_gap_perf{perf_alloc(PC_COUNT, MODULE_NAME": gyro data gap")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_INTEG_RATE>) _param_imu_integ_rate,
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_ratemax
	)
};

} // namespace sensors
