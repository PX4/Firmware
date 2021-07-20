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

static constexpr float DEFAULT_MAG_NOISE   = 0.005f;  // gauss

float getMagNoise(uint32_t sensor_device_id)
{
	device::Device::DeviceId device_id;
	device_id.devid = sensor_device_id;

	switch (device_id.devid_s.devtype) {
	case DRV_GYR_DEVTYPE_BMI055: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_HMC5883: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_AK8963: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_LIS3MDL: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_IST8310: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_RM3100: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_QMC5883L: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_AK09916: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_IST8308: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_LIS2MDL: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_BMM150: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_ST_LSM9DS1_M: return DEFAULT_MAG_NOISE;

	case DRV_MAG_DEVTYPE_LSM303AGR: return DEFAULT_MAG_NOISE;
	}

	return DEFAULT_MAG_NOISE;
}
}; // namespace sensors
