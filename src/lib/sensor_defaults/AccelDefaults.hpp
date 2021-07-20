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

struct AccelNoiseParameters {
	float noise{0.35f};         // Accelerometer noise for covariance prediction (m/s^2)
	float bias_noise{0.003f};   // Process noise for IMU accelerometer bias prediction (m/s^2)
	float switch_on_bias{0.2f}; // 1-sigma IMU accelerometer switch-on bias (m/s^3)
	uint8_t type{DRV_DEVTYPE_UNUSED};
};

AccelNoiseParameters getAccelNoise(uint32_t sensor_device_id)
{
	device::Device::DeviceId device_id;
	device_id.devid = sensor_device_id;

	switch (device_id.devid_s.devtype) {
	case DRV_ACC_DEVTYPE_BMI055: return AccelNoiseParameters{};

	case DRV_ACC_DEVTYPE_BMI088: return AccelNoiseParameters{};

	case DRV_ACC_DEVTYPE_FXOS8701C: return AccelNoiseParameters{};

	case DRV_ACC_DEVTYPE_LSM303AGR: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_ADIS16448: return AccelNoiseParameters{ .noise = 0.30f, .bias_noise = 0.002f, .switch_on_bias = 0.1f};

	case DRV_IMU_DEVTYPE_ADIS16470: return AccelNoiseParameters{ .noise = 0.20f, .bias_noise = 0.001f, .switch_on_bias = 0.05f};

	case DRV_IMU_DEVTYPE_ADIS16477: return AccelNoiseParameters{ .noise = 0.20f, .bias_noise = 0.001f, .switch_on_bias = 0.05f};

	case DRV_IMU_DEVTYPE_ADIS16497: return AccelNoiseParameters{ .noise = 0.20f, .bias_noise = 0.001f, .switch_on_bias = 0.05f};

	case DRV_IMU_DEVTYPE_ICM20602: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM20608G: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM20649: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM20689: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM20948: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM40609D: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM42605: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_ICM42688P: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_MPU6000: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_MPU6500: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_MPU9250: return AccelNoiseParameters{};

	case DRV_IMU_DEVTYPE_ST_LSM9DS1_AG : return AccelNoiseParameters{};
	}

	return AccelNoiseParameters{};
}

}; // namespace sensors
