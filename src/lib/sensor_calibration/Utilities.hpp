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

#pragma once

#include <lib/conversion/rotation.h>
#include <matrix/math.hpp>

namespace calibration
{

/**
 * @brief Find sensor's calibration index if it exists.
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param device_id
 * @return int8_t Valid calibration index on success, -1 otherwise
 */
int8_t FindCalibrationIndex(const char *sensor_type, uint32_t device_id);

/**
 * @brief Get sensor calibration parameter value.
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Calibration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance
 * @return int32_t The calibration value.
 */
int32_t GetCalibrationParam(const char *sensor_type, const char *cal_type, uint8_t instance);

/**
 * @brief Set a single calibration paramter.
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Calibration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance Calibration index (0 - 3)
 * @param value int32_t parameter value
 * @return true if the parameter name was valid and value saved successfully, false otherwise.
 */
bool SetCalibrationParam(const char *sensor_type, const char *cal_type, uint8_t instance, int32_t value);

/**
 * @brief Get the Calibration Params Vector 3f object
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Calibration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance Calibration index (0 - 3)
 * @return matrix::Vector3f Vector of calibration values.
 */
matrix::Vector3f GetCalibrationParamsVector3f(const char *sensor_type, const char *cal_type, uint8_t instance);

/**
 * @brief Set the Calibration Params Vector 3f object
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Calibration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance Calibration index (0 - 3)
 * @param values Vector of calibration values x, y, z.
 * @return true if the parameter name was valid and all values saved successfully, false otherwise.
 */
bool SetCalibrationParamsVector3f(const char *sensor_type, const char *cal_type, uint8_t instance,
				  matrix::Vector3f values);

/**
 * @brief Get the board sensor level adjustment (SENS_BOARD_X_OFF, SENS_BOARD_Y_OFF, SENS_BOARD_Z_OFF).
 *
 * @return matrix::Eulerf
 */
matrix::Eulerf GetSensorLevelAdjustment();

/**
 * @brief Get the board rotation.
 *
 * @return enum Rotation
 */
Rotation GetBoardRotation();

/**
 * @brief Get the board rotation Dcm.
 *
 * @return matrix::Dcmf
 */
matrix::Dcmf GetBoardRotationMatrix();

} // namespace calibration
