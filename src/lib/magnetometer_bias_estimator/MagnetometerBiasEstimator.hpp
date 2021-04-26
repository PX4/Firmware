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
 * @file MagnetometerBiasEstimator.hpp
 *
 * Runs the FieldSensorBiasEstimator with PX4 magnetometer and gyroscope data and saves the result to the calibration parameters.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <lib/conversion/rotation.h>
#include <px4_module_params.h>
#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_armed.h>

#include <FieldSensorBiasEstimator.hpp>

class MagnetometerBiasEstimator : public ModuleParams
{
public:
	MagnetometerBiasEstimator(const matrix::Dcmf &board_rotation);
	~MagnetometerBiasEstimator() = default;

	/**
	 * Update the estimator and extract updated magnetometer biases.
	 * @param mag_raw struct containing the magnetometer data to operate on (gets adjusted with current estimate)
	 * @param raw struct containing the gyroscope data to use
	 */
	void extractBias(vehicle_magnetometer_s &mag_raw, const sensor_combined_s &gyro_raw);

private:
	void updateEstimate(const matrix::Vector3f &gyro, const matrix::Vector3f &mag, const float dt);
	void saveBias(const matrix::Vector3f &bias);

	FieldSensorBiasEstimator _field_sensor_bias_estimator{};
	const matrix::Dcmf &_board_rotation;

	hrt_abstime _time_stamp_last_loop{0};
	hrt_abstime _time_stamp_last_save{0};

	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CAL_MAG0_XOFF>) _param_cal_mag0_xoff,
		(ParamFloat<px4::params::CAL_MAG0_YOFF>) _param_cal_mag0_yoff,
		(ParamFloat<px4::params::CAL_MAG0_ZOFF>) _param_cal_mag0_zoff
	)
};
