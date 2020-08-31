/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

/**
 * @file voted_sensors_update.h
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "data_validator/DataValidator.hpp"
#include "data_validator/DataValidatorGroup.hpp"

#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_preflight_imu.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/subsystem_info.h>

namespace sensors
{

static constexpr uint8_t GYRO_COUNT_MAX = 3;
static constexpr uint8_t ACCEL_COUNT_MAX = 3;

static constexpr uint8_t SENSOR_COUNT_MAX = math::max(GYRO_COUNT_MAX, ACCEL_COUNT_MAX);

/**
 ** class VotedSensorsUpdate
 *
 * Handling of sensor updates with voting
 */
class VotedSensorsUpdate : public ModuleParams
{
public:
	/**
	 * @param parameters parameter values. These do not have to be initialized when constructing this object.
	 * Only when calling init(), they have to be initialized.
	 */
	VotedSensorsUpdate(bool hil_enabled, uORB::SubscriptionCallbackWorkItem(&vehicle_imu_sub)[3]);

	/**
	 * initialize subscriptions etc.
	 * @return 0 on success, <0 otherwise
	 */
	int init(sensor_combined_s &raw);

	/**
	 * This tries to find new sensor instances. This is called from init(), then it can be called periodically.
	 */
	void initializeSensors();

	void printStatus();

	/**
	 * call this whenever parameters got updated. Make sure to have initializeSensors() called at least
	 * once before calling this.
	 */
	void parametersUpdate();

	/**
	 * read new sensor data
	 */
	void sensorsPoll(sensor_combined_s &raw);

	/**
	 * set the relative timestamps of each sensor timestamp, based on the last sensorsPoll,
	 * so that the data can be published.
	 */
	void setRelativeTimestamps(sensor_combined_s &raw);

	/**
	 * Calculates the magnitude in m/s/s of the largest difference between the primary and any other accel sensor
	 */
	void calcAccelInconsistency(sensor_preflight_imu_s &preflt);

	/**
	 * Calculates the magnitude in rad/s of the largest difference between the primary and any other gyro sensor
	 */
	void calcGyroInconsistency(sensor_preflight_imu_s &preflt);

private:

	static constexpr uint8_t ACCEL_COUNT_MAX = 3;
	static constexpr uint8_t GYRO_COUNT_MAX = 3;
	static constexpr uint8_t SENSOR_COUNT_MAX = math::max(ACCEL_COUNT_MAX, GYRO_COUNT_MAX);

	static constexpr uint8_t DEFAULT_PRIORITY = 50;

	struct SensorData {
		SensorData() = delete;
		explicit SensorData(ORB_ID meta) : subscription{{meta, 0}, {meta, 1}, {meta, 2}} {}

		uORB::Subscription subscription[SENSOR_COUNT_MAX]; /**< raw sensor data subscription */
		DataValidatorGroup voter{1};
		unsigned int last_failover_count{0};
		int32_t priority[SENSOR_COUNT_MAX] {};
		int32_t priority_configured[SENSOR_COUNT_MAX] {};
		uint8_t last_best_vote{0}; /**< index of the latest best vote */
		uint8_t subscription_count{0};
		bool advertised[SENSOR_COUNT_MAX] {false, false, false};
	};

	void initSensorClass(SensorData &sensor_data, uint8_t sensor_count_max);

	/**
	 * Poll IMU for updated data.
	 *
	 * @param raw	Combined sensor data structure into which
	 *		data should be returned.
	 */
	void imuPoll(sensor_combined_s &raw);

	/**
	 * Check & handle failover of a sensor
	 * @return true if a switch occured (could be for a non-critical reason)
	 */
	bool checkFailover(SensorData &sensor, const char *sensor_name, const uint64_t type);

	SensorData _accel{ORB_ID::sensor_accel};
	SensorData _gyro{ORB_ID::sensor_gyro};

	hrt_abstime _last_error_message{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};	/**< handle to the sensor selection uORB topic */
	uORB::PublicationQueued<subsystem_info_s> _info_pub{ORB_ID(subsystem_info)};	/* subsystem info publication */

	uORB::SubscriptionCallbackWorkItem(&_vehicle_imu_sub)[3];
	uORB::Subscription _vehicle_imu_status_sub[ACCEL_COUNT_MAX] {
		{ORB_ID(vehicle_imu_status), 0},
		{ORB_ID(vehicle_imu_status), 1},
		{ORB_ID(vehicle_imu_status), 2},
	};

	sensor_combined_s _last_sensor_data[SENSOR_COUNT_MAX] {};	/**< latest sensor data from all sensors instances */

	const bool _hil_enabled{false};			/**< is hardware-in-the-loop mode enabled? */

	bool _selection_changed{true};			/**< true when a sensor selection has changed and not been published */

	float _accel_diff[3][2] {};			/**< filtered accel differences between IMU units (m/s/s) */
	float _gyro_diff[3][2] {};			/**< filtered gyro differences between IMU uinits (rad/s) */

	uint32_t _accel_device_id[SENSOR_COUNT_MAX] {};	/**< accel driver device id for each uorb instance */
	uint32_t _gyro_device_id[SENSOR_COUNT_MAX] {};	/**< gyro driver device id for each uorb instance */

	uint64_t _last_accel_timestamp[ACCEL_COUNT_MAX] {};	/**< latest full timestamp */

	sensor_selection_s _selection {};		/**< struct containing the sensor selection to be published to the uORB */
	subsystem_info_s _info {};			/**< subsystem info publication */
};

} /* namespace sensors */
