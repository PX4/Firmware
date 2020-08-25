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

#include "VehicleIMU.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

namespace sensors
{

VehicleIMU::VehicleIMU(uint8_t accel_index, uint8_t gyro_index) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_sensor_accel_integrated_sub(this, ORB_ID(sensor_accel_integrated), accel_index),
	_sensor_gyro_integrated_sub(this, ORB_ID(sensor_gyro_integrated), gyro_index),
	_accel_corrections(this, SensorCorrections::SensorType::Accelerometer),
	_gyro_corrections(this, SensorCorrections::SensorType::Gyroscope)
{
}

VehicleIMU::~VehicleIMU()
{
	Stop();
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	return _sensor_accel_integrated_sub.registerCallback() && _sensor_gyro_integrated_sub.registerCallback();
}

void VehicleIMU::Stop()
{
	// clear all registered callbacks
	_sensor_accel_integrated_sub.unregisterCallback();
	_sensor_gyro_integrated_sub.unregisterCallback();

	Deinit();
}

void VehicleIMU::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_accel_corrections.ParametersUpdate();
		_gyro_corrections.ParametersUpdate();
	}
}

void VehicleIMU::Run()
{
	if (_sensor_accel_integrated_sub.updated() && _sensor_gyro_integrated_sub.updated()) {
		sensor_accel_integrated_s accel;
		_sensor_accel_integrated_sub.copy(&accel);
		_accel_corrections.set_device_id(accel.device_id);

		sensor_gyro_integrated_s gyro;
		_sensor_gyro_integrated_sub.copy(&gyro);
		_gyro_corrections.set_device_id(gyro.device_id);

		ParametersUpdate();
		_accel_corrections.SensorCorrectionsUpdate();
		_gyro_corrections.SensorCorrectionsUpdate();

		// delta angle: apply offsets, scale, and board rotation
		const float gyro_dt = 1.e-6f * gyro.dt;
		Vector3f delta_angle = _gyro_corrections.Correct(Vector3f{gyro.delta_angle} * gyro_dt) / gyro_dt;

		// delta velocity: apply offsets, scale, and board rotation
		const float accel_dt = 1.e-6f * accel.dt;
		Vector3f delta_velocity = _accel_corrections.Correct(Vector3f{accel.delta_velocity} * accel_dt) / accel_dt;

		// publich vehicle_imu
		vehicle_imu_s imu;
		imu.timestamp_sample = accel.timestamp_sample;
		imu.accel_device_id = accel.device_id;
		imu.gyro_device_id = gyro.device_id;

		delta_angle.copyTo(imu.delta_angle);
		delta_velocity.copyTo(imu.delta_velocity);

		imu.dt = accel.dt;
		imu.integrated_samples = accel.samples;
		imu.clip_count = accel.clip_count;
		imu.timestamp = hrt_absolute_time();

		_vehicle_imu_pub.publish(imu);
	}
}

void VehicleIMU::PrintStatus()
{
	PX4_INFO("selected IMU: accel: %d gyro: %d", _accel_corrections.get_device_id(), _gyro_corrections.get_device_id());
	_accel_corrections.PrintStatus();
	_gyro_corrections.PrintStatus();
}

} // namespace sensors
