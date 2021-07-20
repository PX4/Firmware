/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
#include <lib/drivers/device/Device.hpp>

namespace sensors
{

struct GyroNoiseParameters {
	float noise{0.015f};        // Rate gyro noise for covariance prediction (rad/s)
	float bias_noise{0.001f};   // Process noise for IMU rate gyro bias prediction (rad/s^2)
	float switch_on_bias{0.1f}; // 1-sigma IMU gyro switch-on bias (rad/s)
	uint8_t type{DRV_DEVTYPE_UNUSED};
};

GyroNoiseParameters getGyroNoise(uint32_t sensor_device_id)
{
	device::Device::DeviceId device_id;
	device_id.devid = sensor_device_id;

	switch (device_id.devid_s.devtype) {
	case DRV_GYR_DEVTYPE_BMI055: return GyroNoiseParameters{};

	case DRV_GYR_DEVTYPE_BMI088: return GyroNoiseParameters{};

	case DRV_GYR_DEVTYPE_FXAS2100C: return GyroNoiseParameters{};

	case DRV_GYR_DEVTYPE_L3GD20: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_ADIS16448: return GyroNoiseParameters{ .noise = 0.012f, .bias_noise = 0.0005f, .switch_on_bias = 0.05f};

	case DRV_IMU_DEVTYPE_ADIS16470: return GyroNoiseParameters{ .noise = 0.010f, .bias_noise = 0.0002f, .switch_on_bias = 0.02f};

	case DRV_IMU_DEVTYPE_ADIS16477: return GyroNoiseParameters{ .noise = 0.010f, .bias_noise = 0.0002f, .switch_on_bias = 0.02f};

	case DRV_IMU_DEVTYPE_ADIS16497: return GyroNoiseParameters{ .noise = 0.010f, .bias_noise = 0.0002f, .switch_on_bias = 0.02f};

	case DRV_IMU_DEVTYPE_ICM20602: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM20608G: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM20649: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM20689: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM20948: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM40609D: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM42605: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM42688P: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_MPU6000: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_MPU6500: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_MPU9250: return GyroNoiseParameters{};

	case DRV_IMU_DEVTYPE_ST_LSM9DS1_AG : return GyroNoiseParameters{};
	}

	return GyroNoiseParameters{};
}

}; // namespace sensors
