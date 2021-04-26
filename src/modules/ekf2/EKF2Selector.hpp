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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/wind.h>

#if CONSTRAINED_MEMORY
# define EKF2_MAX_INSTANCES 2
#else
# define EKF2_MAX_INSTANCES 9
#endif

using namespace time_literals;

class EKF2Selector : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2Selector();
	~EKF2Selector() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	static constexpr uint8_t INVALID_INSTANCE{UINT8_MAX};
	static constexpr uint64_t FILTER_UPDATE_PERIOD{10_ms};

	void Run() override;
	void PublishVehicleAttitude(bool reset = false);
	void PublishVehicleLocalPosition(bool reset = false);
	void PublishVehicleGlobalPosition(bool reset = false);
	void PublishWindEstimate(bool reset = false);
	bool SelectInstance(uint8_t instance);

	// Update the error scores for all available instances
	bool UpdateErrorScores();

	// Subscriptions (per estimator instance)
	struct EstimatorInstance {

		EstimatorInstance(EKF2Selector *selector, uint8_t i) :
			estimator_attitude_sub{selector, ORB_ID(estimator_attitude), i},
			estimator_status_sub{selector, ORB_ID(estimator_status), i},
			estimator_local_position_sub{ORB_ID(estimator_local_position), i},
			estimator_global_position_sub{ORB_ID(estimator_global_position), i},
			estimator_odometry_sub{ORB_ID(estimator_odometry), i},
			estimator_wind_sub{ORB_ID(estimator_wind), i},
			instance(i)
		{}

		uORB::SubscriptionCallbackWorkItem estimator_attitude_sub;
		uORB::SubscriptionCallbackWorkItem estimator_status_sub;

		uORB::Subscription estimator_local_position_sub;
		uORB::Subscription estimator_global_position_sub;
		uORB::Subscription estimator_odometry_sub;
		uORB::Subscription estimator_wind_sub;

		uint64_t timestamp_sample_last{0};

		uint32_t accel_device_id{0};
		uint32_t gyro_device_id{0};
		uint32_t baro_device_id{0};
		uint32_t mag_device_id{0};

		hrt_abstime time_last_selected{0};

		float combined_test_ratio{NAN};
		float relative_test_ratio{NAN};

		bool healthy{false};
		bool filter_fault{false};
		bool timeout{false};

		const uint8_t instance;
	};

	static constexpr float _rel_err_score_lim{1.0f}; // +- limit applied to the relative error score
	static constexpr float _rel_err_thresh{0.5f};    // the relative score difference needs to be greater than this to switch from an otherwise healthy instance

	EstimatorInstance _instance[EKF2_MAX_INSTANCES] {
		{this, 0},
		{this, 1},
#if EKF2_MAX_INSTANCES > 2
		{this, 2},
		{this, 3},
#if EKF2_MAX_INSTANCES > 4
		{this, 4},
		{this, 5},
		{this, 6},
		{this, 7},
		{this, 8},
#endif
#endif
	};

	static constexpr uint8_t IMU_STATUS_SIZE = (sizeof(sensors_status_imu_s::gyro_inconsistency_rad_s) / sizeof(
				sensors_status_imu_s::gyro_inconsistency_rad_s[0]));
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_gyro_error) / sizeof(
			      estimator_selector_status_s::accumulated_gyro_error[0]),
		      "increase estimator_selector_status_s::accumulated_gyro_error size");
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_accel_error) / sizeof(
			      estimator_selector_status_s::accumulated_accel_error[0]),
		      "increase estimator_selector_status_s::accumulated_accel_error size");
	static_assert(EKF2_MAX_INSTANCES <= sizeof(estimator_selector_status_s::combined_test_ratio) / sizeof(
			      estimator_selector_status_s::combined_test_ratio[0]),
		      "increase estimator_selector_status_s::combined_test_ratio size");

	float _accumulated_gyro_error[IMU_STATUS_SIZE] {};
	float _accumulated_accel_error[IMU_STATUS_SIZE] {};
	hrt_abstime _last_update_us{0};
	bool _gyro_fault_detected{false};
	bool _accel_fault_detected{false};

	uint8_t _available_instances{0};
	uint8_t _selected_instance{INVALID_INSTANCE};

	uint32_t _instance_changed_count{0};
	hrt_abstime _last_instance_change{0};

	hrt_abstime _last_status_publish{0};
	bool _selector_status_publish{false};

	// vehicle_attitude: reset counters
	vehicle_attitude_s _attitude_last{};
	matrix::Quatf _delta_q_reset{};
	uint8_t _quat_reset_counter{0};

	// vehicle_local_position: reset counters
	vehicle_local_position_s _local_position_last{};
	matrix::Vector2f _delta_xy_reset{};
	float _delta_z_reset{0.f};
	matrix::Vector2f _delta_vxy_reset{};
	float _delta_vz_reset{0.f};
	float _delta_heading_reset{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	// vehicle_global_position: reset counters
	vehicle_global_position_s _global_position_last{};
	double _delta_lat_reset{0};
	double _delta_lon_reset{0};
	float _delta_alt_reset{0.f};
	uint8_t _lat_lon_reset_counter{0};
	uint8_t _alt_reset_counter{0};

	// wind estimate
	wind_s _wind_last{};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _sensors_status_imu{ORB_ID(sensors_status_imu)};

	// Publications
	uORB::Publication<estimator_selector_status_s> _estimator_selector_status_pub{ORB_ID(estimator_selector_status)};
	uORB::Publication<sensor_selection_s>          _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<vehicle_attitude_s>          _vehicle_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s>   _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s>    _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_odometry_s>          _vehicle_odometry_pub{ORB_ID(vehicle_odometry)};
	uORB::Publication<wind_s>             _wind_pub{ORB_ID(wind)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::EKF2_SEL_ERR_RED>) _param_ekf2_sel_err_red,
		(ParamFloat<px4::params::EKF2_SEL_IMU_RAT>) _param_ekf2_sel_imu_angle_rate,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ANG>) _param_ekf2_sel_imu_angle,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ACC>) _param_ekf2_sel_imu_accel,
		(ParamFloat<px4::params::EKF2_SEL_IMU_VEL>) _param_ekf2_sel_imu_velocity
	)
};
