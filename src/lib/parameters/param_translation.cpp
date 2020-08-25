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

#include "param_translation.h"


#include <px4_platform_common/log.h>
#include <lib/drivers/device/Device.hpp>
#include <drivers/drv_sensor.h>
#include <lib/parameters/param.h>

bool param_modify_on_import(const char *name, bson_type_t type, void *value)
{

	// migrate MC_DTERM_CUTOFF -> IMU_DGYRO_CUTOFF (2020-03-12). This can be removed after the next release (current release=1.10)
	if (type == BSON_DOUBLE) {
		if (strcmp("MC_DTERM_CUTOFF", name) == 0) {
			param_t h = param_find("IMU_DGYRO_CUTOFF");

			if (h == PX4_OK) {
				float fvalue = *(float *)value;
				PX4_INFO("param migrating MC_DTERM_CUTOFF (removed) -> IMU_DGYRO_CUTOFF: value=%.3f", (double)fvalue);
				param_set_no_notification(h, value);
				return true;
			}

			return false;
		}
	}



	// translate (SPI) calibration ID parameters. This can be removed after the next release (current release=1.10)

	if (type != BSON_INT32) {
		return false;
	}

	int32_t *ivalue = (int32_t *)value;

	const char *cal_id_params[] = {
		"CAL_ACC0_ID",
		"CAL_GYRO0_ID",
		"CAL_MAG0_ID",
		"TC_A0_ID",
		"TC_B0_ID",
		"TC_G0_ID",
		"CAL_ACC1_ID",
		"CAL_GYRO1_ID",
		"CAL_MAG1_ID",
		"TC_A1_ID",
		"TC_B1_ID",
		"TC_G1_ID",
		"CAL_ACC2_ID",
		"CAL_GYRO2_ID",
		"CAL_MAG2_ID",
		"TC_A2_ID",
		"TC_B2_ID",
		"TC_G2_ID",
		"CAL_ACC_PRIME",
		"CAL_GYRO_PRIME",
		"CAL_MAG_PRIME",
	};
	bool found = false;

	for (int i = 0; i < sizeof(cal_id_params) / sizeof(cal_id_params[0]); ++i) {
		if (strcmp(cal_id_params[i], name) == 0) {
			found = true;
			break;
		}
	}

	if (!found) {
		return false;
	}


	device::Device::DeviceId device_id;
	device_id.devid = (uint32_t) * ivalue;

	// SPI board config translation
	if (device_id.devid_s.bus_type == device::Device::DeviceBusType_SPI) {
		device_id.devid_s.address = 0;
	}

	// deprecated ACC -> IMU translations
	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_MPU6000_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_MPU6000;
	}

	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_MPU9250_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_MPU9250;
	}

	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_ICM20602_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ICM20602;
	}

	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_ICM20608_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ICM20608;
	}

	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_ICM20689_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ICM20689;
	}

	int32_t new_value = (int32_t)device_id.devid;

	if (new_value != *ivalue) {
		PX4_INFO("param modify: %s, value=0x%x (old=0x%x)", name, new_value, *ivalue);
		*ivalue = new_value;
		return true;
	}

	return false;
}
