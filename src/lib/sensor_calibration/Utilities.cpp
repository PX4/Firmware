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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <lib/conversion/rotation.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>

using math::radians;
using matrix::Eulerf;
using matrix::Dcmf;
using matrix::Vector3f;

namespace calibration
{

int8_t FindCalibrationIndex(const char *sensor_type, uint32_t device_id)
{
	if (device_id == 0) {
		return -1;
	}

	for (unsigned i = 0; i < 4; ++i) {
		char str[20] {};
		sprintf(str, "CAL_%s%u_ID", sensor_type, i);

		int32_t device_id_val = 0;

		param_t param_handle = param_find_no_notification(str);

		if (param_handle == PARAM_INVALID) {
			continue;
		}

		// find again and get value, but this time mark it active
		if (param_get(param_find(str), &device_id_val) != OK) {
			continue;
		}

		if ((uint32_t)device_id_val == device_id) {
			return i;
		}
	}

	return -1;
}

int32_t GetCalibrationParam(const char *sensor_type, const char *cal_type, uint8_t instance)
{
	// eg CAL_MAGn_ID/CAL_MAGn_ROT
	char str[20] {};
	sprintf(str, "CAL_%s%u_%s", sensor_type, instance, cal_type);

	int32_t value = 0;

	if (param_get(param_find(str), &value) != 0) {
		PX4_ERR("failed to get %s", str);
	}

	return value;
}

bool SetCalibrationParam(const char *sensor_type, const char *cal_type, uint8_t instance, int32_t value)
{
	char str[20] {};

	// eg CAL_MAGn_ID/CAL_MAGn_ROT
	sprintf(str, "CAL_%s%u_%s", sensor_type, instance, cal_type);

	int ret = param_set_no_notification(param_find(str), &value);

	if (ret != PX4_OK) {
		PX4_ERR("failed to set %s = %d", str, value);
	}

	return ret == PX4_OK;
}

Vector3f GetCalibrationParamsVector3f(const char *sensor_type, const char *cal_type, uint8_t instance)
{
	Vector3f values{0.f, 0.f, 0.f};

	char str[20] {};

	for (int axis = 0; axis < 3; axis++) {
		char axis_char = 'X' + axis;

		// eg CAL_MAGn_{X,Y,Z}OFF
		sprintf(str, "CAL_%s%u_%c%s", sensor_type, instance, axis_char, cal_type);

		if (param_get(param_find(str), &values(axis)) != 0) {
			PX4_ERR("failed to get %s", str);
		}
	}

	return values;
}

bool SetCalibrationParamsVector3f(const char *sensor_type, const char *cal_type, uint8_t instance, Vector3f values)
{
	int ret = PX4_OK;
	char str[20] {};

	for (int axis = 0; axis < 3; axis++) {
		char axis_char = 'X' + axis;

		// eg CAL_MAGn_{X,Y,Z}OFF
		sprintf(str, "CAL_%s%u_%c%s", sensor_type, instance, axis_char, cal_type);

		if (param_set_no_notification(param_find(str), &values(axis)) != 0) {
			PX4_ERR("failed to set %s = %.4f", str, (double)values(axis));
			ret = PX4_ERROR;
		}
	}

	return ret == PX4_OK;
}

Eulerf GetSensorLevelAdjustment()
{
	float x_offset = 0.f;
	float y_offset = 0.f;
	float z_offset = 0.f;
	param_get(param_find("SENS_BOARD_X_OFF"), &x_offset);
	param_get(param_find("SENS_BOARD_Y_OFF"), &y_offset);
	param_get(param_find("SENS_BOARD_Z_OFF"), &z_offset);

	return Eulerf{radians(x_offset), radians(y_offset), radians(z_offset)};
}

enum Rotation GetBoardRotation()
{
	// get transformation matrix from sensor/board to body frame
	int32_t board_rot = -1;
	param_get(param_find("SENS_BOARD_ROT"), &board_rot);

	if (board_rot >= 0 && board_rot <= Rotation::ROTATION_MAX) {
		return static_cast<enum Rotation>(board_rot);

	} else {
		PX4_ERR("invalid SENS_BOARD_ROT: %d", board_rot);
	}

	return Rotation::ROTATION_NONE;
}

Dcmf GetBoardRotationMatrix()
{
	return get_rot_matrix(GetBoardRotation());
}

} // namespace calibration
