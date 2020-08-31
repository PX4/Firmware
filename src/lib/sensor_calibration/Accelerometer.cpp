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

#include "Accelerometer.hpp"

#include "Utilities.hpp"

#include <lib/parameters/param.h>

using namespace matrix;
using namespace time_literals;

namespace calibration
{

Accelerometer::Accelerometer()
{
	Reset();
}

Accelerometer::Accelerometer(uint32_t device_id)
{
	Reset();
	set_device_id(device_id);
}

void Accelerometer::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		ParametersUpdate();
		SensorCorrectionsUpdate(true);
	}
}

void Accelerometer::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		// valid device id required
		if (_device_id == 0) {
			return;
		}

		sensor_correction_s corrections;

		if (_sensor_correction_sub.copy(&corrections)) {
			// find sensor_corrections index
			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (corrections.accel_device_ids[i] == _device_id) {
					switch (i) {
					case 0:
						_thermal_offset = Vector3f{corrections.accel_offset_0};
						return;
					case 1:
						_thermal_offset = Vector3f{corrections.accel_offset_1};
						return;
					case 2:
						_thermal_offset = Vector3f{corrections.accel_offset_2};
						return;
					}
				}
			}
		}

		// zero thermal offset if not found
		_thermal_offset.zero();
	}
}

void Accelerometer::ParametersUpdate()
{
	if (_device_id == 0) {
		Reset();
		return;
	}

	_calibration_index = FindCalibrationIndex(SensorString(), _device_id);

	if (_calibration_index >= 0) {

		if (!_external) {
			_rotation = GetBoardRotation();

		} else {
			// TODO: per sensor external rotation
			_rotation.setIdentity();
		}

		// CAL_ACCx_PRIO
		_priority = GetCalibrationParam(SensorString(), "PRIO", _calibration_index);

		if (_priority < 0 || _priority > 100) {
			// reset to default
			PX4_ERR("%s %d invalid priority %d, resetting to %d", SensorString(), _calibration_index, _priority, DEFAULT_PRIORITY);
			SetCalibrationParam(SensorString(), "PRIO", _calibration_index, DEFAULT_PRIORITY);
			_priority = DEFAULT_PRIORITY;
		}

		// CAL_ACCx_OFF{X,Y,Z}
		_offset = GetCalibrationParamsVector3f(SensorString(), "OFF", _calibration_index);

		// CAL_ACCx_SCALE{X,Y,Z}
		_scale = GetCalibrationParamsVector3f(SensorString(), "SCALE", _calibration_index);

	} else {
		Reset();
	}
}

void Accelerometer::Reset()
{
	_rotation.setIdentity();
	_offset.zero();
	_scale = Vector3f{1.f, 1.f, 1.f};
	_thermal_offset.zero();

	_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;

	_calibration_index = -1;
}

bool Accelerometer::ParametersSave()
{
	if (_calibration_index >= 0) {
		// save calibration
		SetCalibrationParam(SensorString(), "ID", _calibration_index, _device_id);
		SetCalibrationParam(SensorString(), "PRIO", _calibration_index, _priority);
		SetCalibrationParamsVector3f(SensorString(), "OFF", _calibration_index, _offset);
		SetCalibrationParamsVector3f(SensorString(), "SCALE", _calibration_index, _scale);

		// if (_external) {
		// 	SetCalibrationParam(SensorString(), "ROT", _calibration_index, (int32_t)_rotation_enum);

		// } else {
		// 	SetCalibrationParam(SensorString(), "ROT", _calibration_index, -1);
		// }

		return true;
	}

	return false;
}

void Accelerometer::PrintStatus()
{
	PX4_INFO("%s %d EN: %d, offset: [%.4f %.4f %.4f] scale: [%.4f %.4f %.4f]", SensorString(), device_id(), enabled(),
		 (double)_offset(0), (double)_offset(1), (double)_offset(2), (double)_scale(0), (double)_scale(1), (double)_scale(2));

	if (_thermal_offset.norm() > 0.f) {
		PX4_INFO("%s %d temperature offset: [%.4f %.4f %.4f]", SensorString(), _device_id,
			 (double)_thermal_offset(0), (double)_thermal_offset(1), (double)_thermal_offset(2));
	}
}

} // namespace calibration
