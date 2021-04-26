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

bool param_modify_on_import(bson_node_t node)
{
	// migrate COM_ARM_AUTH -> COM_ARM_AUTH_ID, COM_ARM_AUTH_MET and COM_ARM_AUTH_TO (2020-11-06). This can be removed after the next release (current release=1.11)
	if (node->type == BSON_INT32) {
		if (strcmp("COM_ARM_AUTH", node->name) == 0) {
			union {
				struct {
					uint8_t authorizer_system_id;
					uint16_t auth_method_arm_timeout_msec;
					uint8_t authentication_method;
				} __attribute__((packed)) struct_value;
				int32_t param_value;
			} old_param;
			old_param.param_value = node->i;

			int32_t method = old_param.struct_value.authentication_method;
			param_set_no_notification(param_find("COM_ARM_AUTH_MET"), &method);

			float timeout = old_param.struct_value.auth_method_arm_timeout_msec / 1000.f;
			param_set_no_notification(param_find("COM_ARM_AUTH_TO"), &timeout);

			strcpy(node->name, "COM_ARM_AUTH_ID");
			node->i = old_param.struct_value.authorizer_system_id;

			PX4_INFO("migrating COM_ARM_AUTH: %d -> COM_ARM_AUTH_ID:%d, COM_ARM_AUTH_MET: %d and COM_ARM_AUTH_TO: %f",
				 old_param.param_value,
				 old_param.struct_value.authorizer_system_id,
				 method,
				 (double)timeout);
		}
	}

	// migrate MPC_*_VEL_* -> MPC_*_VEL_*_ACC (2020-04-27). This can be removed after the next release (current release=1.10)
	if (node->type == BSON_DOUBLE) {
		param_migrate_velocity_gain(node, "MPC_XY_VEL_P");
		param_migrate_velocity_gain(node, "MPC_XY_VEL_I");
		param_migrate_velocity_gain(node, "MPC_XY_VEL_D");
		param_migrate_velocity_gain(node, "MPC_Z_VEL_P");
		param_migrate_velocity_gain(node, "MPC_Z_VEL_I");
		param_migrate_velocity_gain(node, "MPC_Z_VEL_D");
	}

	// migrate MC_DTERM_CUTOFF -> IMU_DGYRO_CUTOFF (2020-03-12). This can be removed after the next release (current release=1.10)
	if (node->type == BSON_DOUBLE) {
		if (strcmp("MC_DTERM_CUTOFF", node->name) == 0) {
			strcpy(node->name, "IMU_DGYRO_CUTOFF");
			PX4_INFO("param migrating MC_DTERM_CUTOFF (removed) -> IMU_DGYRO_CUTOFF: value=%.3f", node->d);
			return true;
		}
	}

	// 2020-06-29 (v1.11 beta): translate CAL_ACCx_EN/CAL_GYROx_EN/CAL_MAGx_EN -> CAL_ACCx_PRIO/CAL_GYROx_PRIO/CAL_MAGx_PRIO
	if (node->type == BSON_INT32) {

		const char *cal_sensor_en_params[] = {
			"CAL_ACC0_EN",
			"CAL_ACC1_EN",
			"CAL_ACC2_EN",
			"CAL_GYRO0_EN",
			"CAL_GYRO1_EN",
			"CAL_GYRO2_EN",
			"CAL_MAG0_EN",
			"CAL_MAG1_EN",
			"CAL_MAG2_EN",
			"CAL_MAG3_EN",
		};

		for (int i = 0; i < sizeof(cal_sensor_en_params) / sizeof(cal_sensor_en_params[0]); ++i) {
			if (strcmp(cal_sensor_en_params[i], node->name) == 0) {

				char new_parameter_name[17] {};
				strcpy(new_parameter_name, cal_sensor_en_params[i]);

				char *str_replace = strstr(new_parameter_name, "_EN");

				if (str_replace != nullptr) {
					strcpy(str_replace, "_PRIO");
					PX4_INFO("%s -> %s", cal_sensor_en_params[i], new_parameter_name);
					strcpy(node->name, new_parameter_name);
				}

				// if sensor wasn't disabled, reset to -1 so that it can be set to an appropriate default
				if (node->i != 0) {
					node->i = -1; // special value to process later
				}
			}
		}
	}

	// 2020-08-23 (v1.12 alpha): translate GPS blending parameters from EKF2 -> SENS
	{
		if (strcmp("EKF2_GPS_MASK", node->name) == 0) {
			strcpy(node->name, "SENS_GPS_MASK");
			PX4_INFO("copying %s -> %s", "EKF2_GPS_MASK", "SENS_GPS_MASK");
		}

		if (strcmp("EKF2_GPS_TAU", node->name) == 0) {
			strcpy(node->name, "SENS_GPS_TAU");
			PX4_INFO("copying %s -> %s", "EKF2_GPS_TAU", "SENS_GPS_TAU");
		}
	}

	// 2021-01-31 (v1.12 alpha): translate PWM_MIN/PWM_MAX/PWM_DISARMED to PWM_MAIN
	{
		if (strcmp("PWM_MIN", node->name) == 0) {
			strcpy(node->name, "PWM_MAIN_MIN");
			PX4_INFO("copying %s -> %s", "PWM_MIN", "PWM_MAIN_MIN");
		}

		if (strcmp("PWM_MAX", node->name) == 0) {
			strcpy(node->name, "PWM_MAIN_MAX");
			PX4_INFO("copying %s -> %s", "PWM_MAX", "PWM_MAIN_MAX");
		}

		if (strcmp("PWM_RATE", node->name) == 0) {
			strcpy(node->name, "PWM_MAIN_RATE");
			PX4_INFO("copying %s -> %s", "PWM_RATE", "PWM_MAIN_RATE");
		}

		if (strcmp("PWM_DISARMED", node->name) == 0) {
			strcpy(node->name, "PWM_MAIN_DISARM");
			PX4_INFO("copying %s -> %s", "PWM_DISARMED", "PWM_MAIN_DISARM");
		}
	}

	// translate (SPI) calibration ID parameters. This can be removed after the next release (current release=1.10)

	if (node->type != BSON_INT32) {
		return false;
	}

	int64_t *ivalue = &node->i;

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
		"TC_G2_ID"
	};
	bool found = false;

	for (int i = 0; i < sizeof(cal_id_params) / sizeof(cal_id_params[0]); ++i) {
		if (strcmp(cal_id_params[i], node->name) == 0) {
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
#ifdef __PX4_NUTTX // only on NuttX the address is 0

	if (device_id.devid_s.bus_type == device::Device::DeviceBusType_SPI) {
		device_id.devid_s.address = 0;
	}

#endif

	// deprecated ACC -> IMU translations
	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_MPU6000_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_MPU6000;
	}

	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_MPU6500_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_MPU6500;
	}

	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_MPU9250_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_MPU9250;
	}

	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_ICM20602_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ICM20602;
	}

	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_ICM20608_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ICM20608G;
	}

	if (device_id.devid_s.devtype == DRV_ACC_DEVTYPE_ICM20689_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ICM20689;
	}

	if (device_id.devid_s.devtype == DRV_MAG_DEVTYPE_LSM303D_LEGACY) {
		device_id.devid_s.devtype = DRV_IMU_DEVTYPE_LSM303D;
	}

	int32_t new_value = (int32_t)device_id.devid;

	if (new_value != *ivalue) {
		PX4_INFO("param modify: %s, value=0x%x (old=0x%x)", node->name, new_value, (int32_t)*ivalue);
		*ivalue = new_value;
		return true;
	}

	return false;
}

void param_migrate_velocity_gain(bson_node_t node, const char *parameter_name)
{
	if (strcmp(parameter_name, node->name) == 0) {
		strcat(node->name, "_ACC");
		node->d *= 20.0;
		PX4_INFO("migrating %s (removed) -> %s: new value=%.3f", parameter_name, node->name, node->d);
	}
}
