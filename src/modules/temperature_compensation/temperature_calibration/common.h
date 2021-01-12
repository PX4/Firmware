/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#define TC_PRINT_DEBUG 0
#if TC_PRINT_DEBUG
#define TC_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
#define TC_DEBUG(fmt, ...)
#endif

#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>
#include <lib/parameters/param.h>

#include "polyfit.hpp"

#define SENSOR_COUNT_MAX		3


#define TC_ERROR_INITIAL_TEMP_TOO_HIGH 110 ///< starting temperature was above the configured allowed temperature
#define TC_ERROR_COMMUNICATION         112 ///< no sensors found

/**
 * Base class for temperature calibration types with abstract methods (for all different sensor types)
 */
class TemperatureCalibrationBase
{
public:
	TemperatureCalibrationBase(float min_temperature_rise, float min_start_temperature, float max_start_temperature)
		: _min_temperature_rise(min_temperature_rise), _min_start_temperature(min_start_temperature),
		  _max_start_temperature(max_start_temperature) {}

	virtual ~TemperatureCalibrationBase() = default;

	/**
	 * check & update new sensor data.
	 * @return progress in range [0, 100], 110 when finished, <0 on error,
	 *         -TC_ERROR_INITIAL_TEMP_TOO_HIGH if starting temperature is too hot
	 *         -TC_ERROR_COMMUNICATION if no sensors found
	 */
	virtual int update() = 0;

	/**
	 * do final fitting & write the parameters. Call this exactly once after update() returned 110
	 * @return 0 on success, <0 otherwise
	 */
	virtual int finish() = 0;

protected:

	/**
	 * set a system parameter (without system notification) and print an error if it fails
	 * @param format_str for example "CAL_GYRO%u_XOFF"
	 * @param index which index (will replace %u in format_str)
	 * @param value
	 * @return 0 on success
	 */
	inline int set_parameter(const char *format_str, unsigned index, const void *value);

	float _min_temperature_rise; ///< minimum difference in temperature before the process finishes
	float _min_start_temperature; ///< minimum temperature before the process starts
	float _max_start_temperature; ///< maximum temperature above which the process does not start and an error is declared
};

int TemperatureCalibrationBase::set_parameter(const char *format_str, unsigned index, const void *value)
{
	char param_str[30] {};
	(void)sprintf(param_str, format_str, index);
	int result = param_set_no_notification(param_find(param_str), value);

	if (result != 0) {
		PX4_ERR("unable to reset %s (%i)", param_str, result);
	}

	return result;
}

/**
 ** class TemperatureCalibrationCommon
 * Common base class for all sensor types, contains shared code & data.
 */
template <int Dim, int PolyfitOrder>
class TemperatureCalibrationCommon : public TemperatureCalibrationBase
{
public:
	TemperatureCalibrationCommon(float min_temperature_rise, float min_start_temperature, float max_start_temperature)
		: TemperatureCalibrationBase(min_temperature_rise, min_start_temperature, max_start_temperature) {}

	virtual ~TemperatureCalibrationCommon() = default;

	/**
	 * @see TemperatureCalibrationBase::update()
	 */
	int update()
	{
		int num_not_complete = 0;

		if (_num_sensor_instances == 0) {
			return -TC_ERROR_COMMUNICATION;
		}

		for (unsigned uorb_index = 0; uorb_index < _num_sensor_instances; uorb_index++) {
			int status = update_sensor_instance(_data[uorb_index], _sensor_subs[uorb_index]);

			if (status < 0) {
				return status;
			}

			num_not_complete += status;
		}

		if (num_not_complete > 0) {
			// calculate progress
			float min_diff = _min_temperature_rise;

			for (unsigned uorb_index = 0; uorb_index < _num_sensor_instances; uorb_index++) {
				if (!_data[uorb_index].has_valid_temperature) {
					return 110;
				}

				float cur_diff = _data[uorb_index].high_temp - _data[uorb_index].low_temp;

				if (cur_diff < min_diff) {
					min_diff = cur_diff;
				}
			}

			return math::min(100, (int)(min_diff / _min_temperature_rise * 100.f));
		}

		return 110;
	}

protected:

	struct PerSensorData {
		float sensor_sample_filt[Dim + 1]; ///< last value is the temperature
		polyfitter < PolyfitOrder + 1 > P[Dim];
		unsigned hot_soak_sat = 0; /**< counter that increments every time the sensor temperature reduces
									from the last reading */
		uint32_t device_id = 0; ///< ID for the sensor being calibrated
		bool cold_soaked = false; ///< true when the sensor cold soak starting temperature condition had been
		/// verified and the starting temperature set
		bool hot_soaked = false; ///< true when the sensor has achieved the specified temperature increase
		bool tempcal_complete = false; ///< true when the calibration has been completed
		bool has_valid_temperature = false; ///< true if this sensor has temperature sensor
		float low_temp = 0.f; ///< low temperature recorded at start of calibration (deg C)
		float high_temp = 0.f; ///< highest temperature recorded during calibration (deg C)
		float ref_temp = 0.f; /**< calibration reference temperature, nominally in the middle of the
							calibration temperature range (deg C) */
	};

	PerSensorData _data[SENSOR_COUNT_MAX];

	/**
	 * update a single sensor instance
	 * @return 0 when done, 1 not finished yet, <0 for an error
	 */
	virtual int update_sensor_instance(PerSensorData &data, int sensor_sub) = 0;

	unsigned _num_sensor_instances{0};
	int _sensor_subs[SENSOR_COUNT_MAX];
};
