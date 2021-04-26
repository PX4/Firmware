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

/**
 * @file MagnetometerBiasEstimator.cpp
 */

#include "MagnetometerBiasEstimator.hpp"

#include <px4_posix.h>
#include <drivers/drv_mag.h>
#include <mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;

MagnetometerBiasEstimator::MagnetometerBiasEstimator(const matrix::Dcmf &board_rotation) :
	ModuleParams(nullptr),
	_board_rotation(board_rotation)
{}

void MagnetometerBiasEstimator::extractBias(vehicle_magnetometer_s &mag_raw, const sensor_combined_s &gyro_raw)
{
	// fill in vectors
	Vector3f gyro(gyro_raw.gyro_rad);
	Vector3f mag(mag_raw.magnetometer_ga);

	// prepare delta time in seconds
	hrt_abstime time_stamp_current = hrt_absolute_time();
	float dt  = math::min((time_stamp_current - _time_stamp_last_loop), hrt_abstime(500_ms)) / 1e6f;
	_time_stamp_last_loop = time_stamp_current;

	_field_sensor_bias_estimator.updateEstimate(gyro, mag, dt);
	const Vector3f &bias = _field_sensor_bias_estimator.getBias();

	// save the bias to the parameters to apply it given the right circumstances
	const bool longer_than_10_sec = (time_stamp_current - _time_stamp_last_save) > hrt_abstime(10_s);
	const bool bias_significant = bias.norm_squared() > (0.01f * 0.01f);

	_actuator_armed_sub.update();

	if (!_actuator_armed_sub.get().armed && bias_significant && longer_than_10_sec) {
		saveBias(bias);
		_time_stamp_last_save = time_stamp_current;
	}

	(mag - bias).copyTo(mag_raw.magnetometer_ga);
}

void MagnetometerBiasEstimator::saveBias(const matrix::Vector3f &bias)
{
	// estimated bias is in body frame, but the driver needs sensor frame
	const Vector3f transformed_bias = _board_rotation.transpose() * bias;

	// get current calibration
	updateParams();
	Vector3f calibration_bias(_param_cal_mag0_xoff.get(),
				  _param_cal_mag0_yoff.get(),
				  _param_cal_mag0_zoff.get());

	// estimated bias comes on top of existing calibration
	calibration_bias += transformed_bias;

	// save new calibration
	_param_cal_mag0_xoff.set(calibration_bias(0));
	_param_cal_mag0_yoff.set(calibration_bias(1));
	_param_cal_mag0_zoff.set(calibration_bias(2));
	_param_cal_mag0_xoff.commit();
	_param_cal_mag0_yoff.commit();
	_param_cal_mag0_zoff.commit();

	// reset internal bias to zero because from now the driver will correct
	_field_sensor_bias_estimator.setBias(Vector3f());
}
