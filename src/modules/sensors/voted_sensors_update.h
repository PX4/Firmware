/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "parameters.h"

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <lib/ecl/validation/data_validator.h>
#include <lib/ecl/validation/data_validator_group.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationQueued.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_accel_integrated.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro_integrated.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/subsystem_info.h>

#include "common.h"

namespace sensors
{

/**
 ** class VotedSensorsUpdate
 *
 * Handling of sensor updates with voting
 */
class VotedSensorsUpdate
{
public:
	/**
	 * @param parameters parameter values. These do not have to be initialized when constructing this object.
	 * Only when calling init(), they have to be initialized.
	 */
	VotedSensorsUpdate(const Parameters &parameters, bool hil_enabled);

	/**
	 * initialize subscriptions etc.
	 * @return 0 on success, <0 otherwise
	 */
	int init(sensor_combined_s &raw);

	/**
	 * This tries to find new sensor instances. This is called from init(), then it can be called periodically.
	 */
	void initializeSensors();

	/**
	 * deinitialize the object (we cannot use the destructor because it is called on the wrong thread)
	 */
	void deinit();

	void printStatus();

	/**
	 * call this whenever parameters got updated. Make sure to have initializeSensors() called at least
	 * once before calling this.
	 */
	void parametersUpdate();

	/**
	 * read new sensor data
	 */
	void sensorsPoll(sensor_combined_s &raw, vehicle_magnetometer_s &magnetometer);

	/**
	 * set the relative timestamps of each sensor timestamp, based on the last sensorsPoll,
	 * so that the data can be published.
	 */
	void setRelativeTimestamps(sensor_combined_s &raw);

	/**
	 * check if a failover event occured. if so, report it.
	 */
	void checkFailover();

	int bestGyroID() const { return _gyro_device_id[_gyro.last_best_vote]; }

	/**
	 * Calculates the magnitude in m/s/s of the largest difference between the primary and any other accel sensor
	 */
	void calcAccelInconsistency(sensor_preflight_s &preflt);

	/**
	 * Calculates the magnitude in rad/s of the largest difference between the primary and any other gyro sensor
	 */
	void calcGyroInconsistency(sensor_preflight_s &preflt);

	/**
	 * Calculates the magnitude in Gauss of the largest difference between the primary and any other magnetometers
	 */
	void calcMagInconsistency(sensor_preflight_s &preflt);

private:

	struct SensorData {
		SensorData()
			: last_best_vote(0),
			  subscription_count(0),
			  voter(1),
			  last_failover_count(0)
		{
			for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
				enabled[i] = true;
				subscription[i] = -1;
				priority[i] = 0;
			}
		}

		bool enabled[SENSOR_COUNT_MAX];

		int subscription[SENSOR_COUNT_MAX]; /**< raw sensor data subscription */
		uint8_t priority[SENSOR_COUNT_MAX]; /**< sensor priority */
		uint8_t last_best_vote; /**< index of the latest best vote */
		int subscription_count;
		DataValidatorGroup voter;
		unsigned int last_failover_count;
	};

	void initSensorClass(const orb_metadata *meta, SensorData &sensor_data, uint8_t sensor_count_max);

	/**
	 * Poll the accelerometer for updated data.
	 *
	 * @param raw	Combined sensor data structure into which
	 *		data should be returned.
	 */
	void accelPoll(sensor_combined_s &raw);

	/**
	 * Poll the gyro for updated data.
	 *
	 * @param raw	Combined sensor data structure into which
	 *		data should be returned.
	 */
	void gyroPoll(sensor_combined_s &raw);

	/**
	 * Poll the magnetometer for updated data.
	 *
	 * @param raw	Combined sensor data structure into which
	 *		data should be returned.
	 */
	void magPoll(vehicle_magnetometer_s &magnetometer);

	/**
	 * Check & handle failover of a sensor
	 * @return true if a switch occured (could be for a non-critical reason)
	 */
	bool checkFailover(SensorData &sensor, const char *sensor_name, const uint64_t type);

	SensorData _accel {};
	SensorData _gyro {};
	SensorData _mag {};

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};	/**< handle to the sensor selection uORB topic */
	uORB::PublicationQueued<subsystem_info_s> _info_pub{ORB_ID(subsystem_info)};	/* subsystem info publication */

	/* sensor thermal compensation */
	uORB::Subscription _corrections_sub{ORB_ID(sensor_correction)};

	sensor_combined_s _last_sensor_data[SENSOR_COUNT_MAX] {};	/**< latest sensor data from all sensors instances */
	vehicle_magnetometer_s _last_magnetometer[SENSOR_COUNT_MAX] {}; /**< latest sensor data from all sensors instances */

	matrix::Dcmf _board_rotation {};		/**< rotation matrix for the orientation that the board is mounted */
	matrix::Dcmf _mag_rotation[MAG_COUNT_MAX] {};	/**< rotation matrix for the orientation that the external mag0 is mounted */

	const Parameters &_parameters;
	const bool _hil_enabled{false};			/**< is hardware-in-the-loop mode enabled? */

	bool _selection_changed{false};			/**< true when a sensor selection has changed and not been published */

	float _accel_diff[3][2] {};			/**< filtered accel differences between IMU units (m/s/s) */
	float _gyro_diff[3][2] {};			/**< filtered gyro differences between IMU uinits (rad/s) */
	float _mag_angle_diff[2] {};			/**< filtered mag angle differences between sensor instances (Ga) */

	uint32_t _accel_device_id[SENSOR_COUNT_MAX] {};	/**< accel driver device id for each uorb instance */
	uint32_t _gyro_device_id[SENSOR_COUNT_MAX] {};	/**< gyro driver device id for each uorb instance */
	uint32_t _mag_device_id[SENSOR_COUNT_MAX] {};	/**< mag driver device id for each uorb instance */

	uint64_t _last_accel_timestamp[ACCEL_COUNT_MAX] {};	/**< latest full timestamp */

	sensor_correction_s _corrections {};		/**< struct containing the sensor corrections to be published to the uORB */
	sensor_selection_s _selection {};		/**< struct containing the sensor selection to be published to the uORB */
	subsystem_info_s _info {};			/**< subsystem info publication */
};

} /* namespace sensors */
