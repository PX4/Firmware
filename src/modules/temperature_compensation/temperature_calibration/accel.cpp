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

/**
 * @file accel.cpp
 * Implementation of the Accel Temperature Calibration for onboard sensors.
 *
 * @author Siddharth Bharat Purohit
 * @author Paul Riseborough
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "accel.h"
#include <uORB/topics/sensor_accel.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

TemperatureCalibrationAccel::TemperatureCalibrationAccel(float min_temperature_rise, float min_start_temperature,
		float max_start_temperature)
	: TemperatureCalibrationCommon(min_temperature_rise, min_start_temperature, max_start_temperature)
{
	// init subscriptions
	_num_sensor_instances = orb_group_count(ORB_ID(sensor_accel));

	if (_num_sensor_instances > SENSOR_COUNT_MAX) {
		_num_sensor_instances = SENSOR_COUNT_MAX;
	}

	for (unsigned i = 0; i < _num_sensor_instances; i++) {
		_sensor_subs[i] = orb_subscribe_multi(ORB_ID(sensor_accel), i);
	}
}

TemperatureCalibrationAccel::~TemperatureCalibrationAccel()
{
	for (unsigned i = 0; i < _num_sensor_instances; i++) {
		orb_unsubscribe(_sensor_subs[i]);
	}
}

int TemperatureCalibrationAccel::update_sensor_instance(PerSensorData &data, int sensor_sub)
{
	bool finished = data.hot_soaked;

	bool updated;
	orb_check(sensor_sub, &updated);

	if (!updated) {
		return finished ? 0 : 1;
	}

	sensor_accel_s accel_data;
	orb_copy(ORB_ID(sensor_accel), sensor_sub, &accel_data);

	if (finished) {
		// if we're done, return, but we need to return after orb_copy because of poll()
		return 0;
	}

	if (PX4_ISFINITE(accel_data.temperature)) {
		data.has_valid_temperature = true;

	} else {
		return 0;
	}

	data.device_id = accel_data.device_id;

	data.sensor_sample_filt[0] = accel_data.x;
	data.sensor_sample_filt[1] = accel_data.y;
	data.sensor_sample_filt[2] = accel_data.z;
	data.sensor_sample_filt[3] = accel_data.temperature;

	// wait for min start temp to be reached before starting calibration
	if (data.sensor_sample_filt[3] < _min_start_temperature) {
		return 1;
	}

	if (!data.cold_soaked) {
		// allow time for sensors and filters to settle
		if (hrt_absolute_time() > 10E6) {
			// If intial temperature exceeds maximum declare an error condition and exit
			if (data.sensor_sample_filt[3] > _max_start_temperature) {
				return -TC_ERROR_INITIAL_TEMP_TOO_HIGH;

			} else {
				data.cold_soaked = true;
				data.low_temp = data.sensor_sample_filt[3]; // Record the low temperature
				data.high_temp = data.low_temp; // Initialise the high temperature to the initial temperature
				data.ref_temp = data.sensor_sample_filt[3] + 0.5f * _min_temperature_rise;
				return 1;
			}

		} else {
			return 1;
		}
	}

	// check if temperature increased
	if (data.sensor_sample_filt[3] > data.high_temp) {
		data.high_temp = data.sensor_sample_filt[3];
		data.hot_soak_sat = 0;

	} else {
		return 1;
	}

	//TODO: Detect when temperature has stopped rising for more than TBD seconds
	if (data.hot_soak_sat == 10 || (data.high_temp - data.low_temp) > _min_temperature_rise) {
		data.hot_soaked = true;
	}

	if (sensor_sub == _sensor_subs[0]) { // debug output, but only for the first sensor
		TC_DEBUG("\nAccel: %.20f,%.20f,%.20f,%.20f, %.6f, %.6f, %.6f\n\n", (double)data.sensor_sample_filt[0],
			 (double)data.sensor_sample_filt[1],
			 (double)data.sensor_sample_filt[2], (double)data.sensor_sample_filt[3], (double)data.low_temp, (double)data.high_temp,
			 (double)(data.high_temp - data.low_temp));
	}

	//update linear fit matrices
	double relative_temperature = (double)data.sensor_sample_filt[3] - (double)data.ref_temp;
	data.P[0].update(relative_temperature, (double)data.sensor_sample_filt[0]);
	data.P[1].update(relative_temperature, (double)data.sensor_sample_filt[1]);
	data.P[2].update(relative_temperature, (double)data.sensor_sample_filt[2]);

	return 1;
}

int TemperatureCalibrationAccel::finish()
{
	for (unsigned uorb_index = 0; uorb_index < _num_sensor_instances; uorb_index++) {
		finish_sensor_instance(_data[uorb_index], uorb_index);
	}

	int32_t enabled = 1;
	int result = param_set_no_notification(param_find("TC_A_ENABLE"), &enabled);

	if (result != PX4_OK) {
		PX4_ERR("unable to reset TC_A_ENABLE (%i)", result);
	}

	return result;
}

int TemperatureCalibrationAccel::finish_sensor_instance(PerSensorData &data, int sensor_index)
{
	if (!data.has_valid_temperature) {
		PX4_WARN("Result Accel %d does not have a valid temperature sensor", sensor_index);

		uint32_t param = 0;
		set_parameter("TC_A%d_ID", sensor_index, &param);
		return 0;
	}

	if (!data.hot_soaked || data.tempcal_complete) {
		return 0;
	}

	double res[3][4] = {};
	data.P[0].fit(res[0]);
	res[0][3] = 0.0; // normalise the correction to be zero at the reference temperature
	PX4_INFO("Result Accel %d Axis 0: %.20f %.20f %.20f %.20f", sensor_index, (double)res[0][0], (double)res[0][1],
		 (double)res[0][2],
		 (double)res[0][3]);
	data.P[1].fit(res[1]);
	res[1][3] = 0.0; // normalise the correction to be zero at the reference temperature
	PX4_INFO("Result Accel %d Axis 1: %.20f %.20f %.20f %.20f", sensor_index, (double)res[1][0], (double)res[1][1],
		 (double)res[1][2],
		 (double)res[1][3]);
	data.P[2].fit(res[2]);
	res[2][3] = 0.0; // normalise the correction to be zero at the reference temperature
	PX4_INFO("Result Accel %d Axis 2: %.20f %.20f %.20f %.20f", sensor_index, (double)res[2][0], (double)res[2][1],
		 (double)res[2][2],
		 (double)res[2][3]);
	data.tempcal_complete = true;

	char str[30];
	float param = 0.0f;
	int result = PX4_OK;

	set_parameter("TC_A%d_ID", sensor_index, &data.device_id);

	for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
		for (unsigned coef_index = 0; coef_index <= 3; coef_index++) {
			sprintf(str, "TC_A%d_X%d_%d", sensor_index, 3 - coef_index, axis_index);
			param = (float)res[axis_index][coef_index];
			result = param_set_no_notification(param_find(str), &param);

			if (result != PX4_OK) {
				PX4_ERR("unable to reset %s", str);
			}
		}
	}

	set_parameter("TC_A%d_TMAX", sensor_index, &data.high_temp);
	set_parameter("TC_A%d_TMIN", sensor_index, &data.low_temp);
	set_parameter("TC_A%d_TREF", sensor_index, &data.ref_temp);
	return 0;
}
