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
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>

namespace sensors
{

static constexpr uint8_t MAX_SENSOR_COUNT = 4;

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
	VotedSensorsUpdate(bool hil_enabled, uORB::SubscriptionCallbackWorkItem(&vehicle_imu_sub)[MAX_SENSOR_COUNT]);

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

private:

	static constexpr uint8_t DEFAULT_PRIORITY = 50;

	struct SensorData {
		SensorData() = delete;
		explicit SensorData(ORB_ID meta) : subscription{{meta, 0}, {meta, 1}, {meta, 2}, {meta, 3}} {}

		uORB::Subscription subscription[MAX_SENSOR_COUNT]; /**< raw sensor data subscription */
		DataValidatorGroup voter{1};
		unsigned int last_failover_count{0};
		int32_t priority[MAX_SENSOR_COUNT] {};
		int32_t priority_configured[MAX_SENSOR_COUNT] {};
		uint8_t last_best_vote{0}; /**< index of the latest best vote */
		uint8_t subscription_count{0};
		bool advertised[MAX_SENSOR_COUNT] {false, false, false};
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
	bool checkFailover(SensorData &sensor, const char *sensor_name);

	/**
	 * Calculates the magnitude in m/s/s of the largest difference between each accelerometer vector and the mean of all vectors
	 */
	void calcAccelInconsistency();

	/**
	 * Calculates the magnitude in rad/s of the largest difference between each gyro vector and the mean of all vectors
	 */
	void calcGyroInconsistency();

	SensorData _accel{ORB_ID::sensor_accel};
	SensorData _gyro{ORB_ID::sensor_gyro};

	hrt_abstime _last_error_message{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};	/**< handle to the sensor selection uORB topic */
	uORB::Publication<sensors_status_imu_s> _sensors_status_imu_pub{ORB_ID(sensors_status_imu)};

	uORB::SubscriptionCallbackWorkItem(&_vehicle_imu_sub)[MAX_SENSOR_COUNT];
	uORB::SubscriptionMultiArray<vehicle_imu_status_s, MAX_SENSOR_COUNT> _vehicle_imu_status_subs{ORB_ID::vehicle_imu_status};

	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};

	sensor_combined_s _last_sensor_data[MAX_SENSOR_COUNT] {};	/**< latest sensor data from all sensors instances */

	const bool _hil_enabled{false};			/**< is hardware-in-the-loop mode enabled? */

	bool _selection_changed{true};			/**< true when a sensor selection has changed and not been published */

	matrix::Vector3f _accel_diff[MAX_SENSOR_COUNT] {};		/**< filtered accel differences between IMU units (m/s/s) */
	matrix::Vector3f _gyro_diff[MAX_SENSOR_COUNT] {};			/**< filtered gyro differences between IMU uinits (rad/s) */

	uint32_t _accel_device_id[MAX_SENSOR_COUNT] {};	/**< accel driver device id for each uorb instance */
	uint32_t _gyro_device_id[MAX_SENSOR_COUNT] {};	/**< gyro driver device id for each uorb instance */

	uint64_t _last_accel_timestamp[MAX_SENSOR_COUNT] {};	/**< latest full timestamp */

	sensor_selection_s _selection {};		/**< struct containing the sensor selection to be published to the uORB */

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SENS_IMU_MODE>) _param_sens_imu_mode
	)
};

} /* namespace sensors */
