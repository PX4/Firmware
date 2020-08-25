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

#include "SensorCorrections.hpp"

using namespace matrix;
using namespace time_literals;
using math::radians;

namespace sensors
{

SensorCorrections::SensorCorrections(ModuleParams *parent, SensorType type) :
	ModuleParams(parent),
	_type(type)
{
}

void SensorCorrections::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		SensorCorrectionsUpdate(true);
	}
}

const char *SensorCorrections::SensorString() const
{
	switch (_type) {
	case SensorType::Accelerometer:
		return "ACC";

	case SensorType::Gyroscope:
		return "GYRO";
	}

	return nullptr;
}

void SensorCorrections::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		sensor_correction_s corrections;

		if (_sensor_correction_sub.copy(&corrections)) {

			// selected sensor has changed, find updated index
			if ((_corrections_selected_instance < 0) || force) {
				_corrections_selected_instance = -1;

				// find sensor_corrections index
				for (int i = 0; i < MAX_SENSOR_COUNT; i++) {

					switch (_type) {
					case SensorType::Accelerometer:
						if (corrections.accel_device_ids[i] == _device_id) {
							_corrections_selected_instance = i;
						}

						break;

					case SensorType::Gyroscope:
						if (corrections.gyro_device_ids[i] == _device_id) {
							_corrections_selected_instance = i;
						}

						break;
					}
				}
			}

			switch (_type) {
			case SensorType::Accelerometer:
				switch (_corrections_selected_instance) {
				case 0:
					_offset = Vector3f{corrections.accel_offset_0};
					_scale = Vector3f{corrections.accel_scale_0};
					return;
				case 1:
					_offset = Vector3f{corrections.accel_offset_1};
					_scale = Vector3f{corrections.accel_scale_1};
					return;
				case 2:
					_offset = Vector3f{corrections.accel_offset_2};
					_scale = Vector3f{corrections.accel_scale_2};
					return;
				}

				break;

			case SensorType::Gyroscope:
				switch (_corrections_selected_instance) {
				case 0:
					_offset = Vector3f{corrections.gyro_offset_0};
					_scale = Vector3f{corrections.gyro_scale_0};
					return;
				case 1:
					_offset = Vector3f{corrections.gyro_offset_1};
					_scale = Vector3f{corrections.gyro_scale_1};
					return;
				case 2:
					_offset = Vector3f{corrections.gyro_offset_2};
					_scale = Vector3f{corrections.gyro_scale_2};
					return;
				}

				break;
			}
		}
	}
}

void SensorCorrections::ParametersUpdate()
{
	// fine tune the rotation
	const Dcmf board_rotation_offset(Eulerf(
			radians(_param_sens_board_x_off.get()),
			radians(_param_sens_board_y_off.get()),
			radians(_param_sens_board_z_off.get())));

	// get transformation matrix from sensor/board to body frame
	_board_rotation = board_rotation_offset * get_rot_matrix((enum Rotation)_param_sens_board_rot.get());
}

void SensorCorrections::PrintStatus()
{
	if (_offset.norm() > 0.f) {
		PX4_INFO("%s %d offset: [%.3f %.3f %.3f]", SensorString(), _device_id, (double)_offset(0), (double)_offset(1),
			 (double)_offset(2));
	}

	if (fabsf(_scale.norm_squared() - 3.f) > FLT_EPSILON) {
		PX4_INFO("%s %d scale: [%.3f %.3f %.3f]", SensorString(), _device_id, (double)_scale(0), (double)_scale(1),
			 (double)_scale(2));
	}
}

} // namespace sensors
