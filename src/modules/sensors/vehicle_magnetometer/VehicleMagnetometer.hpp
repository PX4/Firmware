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

#include "data_validator/DataValidatorGroup.hpp"

#include <lib/sensor_calibration/Magnetometer.hpp>
#include <lib/conversion/rotation.h>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_preflight_mag.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_magnetometer.h>

using namespace time_literals;

namespace sensors
{
class VehicleMagnetometer : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	VehicleMagnetometer();
	~VehicleMagnetometer() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);

	void Publish(uint8_t instance, bool multi = false);

	/**
	 * Calculates the magnitude in Gauss of the largest difference between the primary and any other magnetometers
	 */
	void calcMagInconsistency();
	void MagCalibrationUpdate();

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::Publication<sensor_preflight_mag_s> _sensor_preflight_mag_pub{ORB_ID(sensor_preflight_mag)};

	uORB::PublicationMulti<vehicle_magnetometer_s> _vehicle_magnetometer_pub[MAX_SENSOR_COUNT] {
		{ORB_ID(vehicle_magnetometer)},
		{ORB_ID(vehicle_magnetometer)},
		{ORB_ID(vehicle_magnetometer)},
		{ORB_ID(vehicle_magnetometer)},
	};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _actuator_controls_0_sub{ORB_ID(actuator_controls_0)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status), 0};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	// Used to check, save and use learned magnetometer biases
	uORB::SubscriptionMultiArray<estimator_sensor_bias_s> _estimator_sensor_bias_subs{ORB_ID::estimator_sensor_bias};

	bool _mag_cal_available{false};

	struct MagCal {
		uint32_t device_id{0};
		matrix::Vector3f mag_offset{};
		matrix::Vector3f mag_bias_variance{};
	} _mag_cal[ORB_MULTI_MAX_INSTANCES] {};

	uORB::SubscriptionCallbackWorkItem _sensor_sub[MAX_SENSOR_COUNT] {
		{this, ORB_ID(sensor_mag), 0},
		{this, ORB_ID(sensor_mag), 1},
		{this, ORB_ID(sensor_mag), 2},
		{this, ORB_ID(sensor_mag), 3}
	};

	calibration::Magnetometer _calibration[MAX_SENSOR_COUNT];

	// Magnetometer interference compensation
	enum class MagCompensationType {
		Disabled = 0,
		Throttle,
		Current_inst0,
		Current_inst1
	};
	MagCompensationType _mag_comp_type{MagCompensationType::Disabled};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	hrt_abstime _last_error_message{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	DataValidatorGroup _voter{1};
	unsigned _last_failover_count{0};

	uint64_t _timestamp_sample_sum[MAX_SENSOR_COUNT] {0};
	matrix::Vector3f _mag_sum[MAX_SENSOR_COUNT] {};
	int _mag_sum_count[MAX_SENSOR_COUNT] {};
	hrt_abstime _last_publication_timestamp[MAX_SENSOR_COUNT] {};

	sensor_mag_s _last_data[MAX_SENSOR_COUNT] {};
	bool _advertised[MAX_SENSOR_COUNT] {};

	float _mag_angle_diff[2] {};			/**< filtered mag angle differences between sensor instances (Ga) */

	uint8_t _priority[MAX_SENSOR_COUNT] {};

	int8_t _selected_sensor_sub_index{-1};

	bool _armed{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CAL_MAG_COMP_TYP>) _param_mag_comp_typ,
		(ParamBool<px4::params::SENS_MAG_MODE>) _param_sens_mag_mode,
		(ParamFloat<px4::params::SENS_MAG_RATE>) _param_sens_mag_rate
	)
};
}; // namespace sensors
