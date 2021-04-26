/****************************************************************************
*
*   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

/**
 * @file output.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "output.h"
#include <errno.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/mount_orientation.h>
#include <px4_platform_common/defines.h>
#include <lib/ecl/geo/geo.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace vmount
{

OutputBase::OutputBase(const OutputConfig &output_config)
	: _config(output_config)
{
	_last_update = hrt_absolute_time();
}

void OutputBase::publish()
{
	mount_orientation_s mount_orientation{};

	for (unsigned i = 0; i < 3; ++i) {
		mount_orientation.attitude_euler_angle[i] = _angle_outputs[i];
	}

	mount_orientation.timestamp = hrt_absolute_time();
	_mount_orientation_pub.publish(mount_orientation);
}

float OutputBase::_calculate_pitch(double lon, double lat, float altitude,
				   const vehicle_global_position_s &global_position)
{
	if (!map_projection_initialized(&_projection_reference)) {
		map_projection_init(&_projection_reference, global_position.lat, global_position.lon);
	}

	float x1, y1, x2, y2;
	map_projection_project(&_projection_reference, lat, lon, &x1, &y1);
	map_projection_project(&_projection_reference, global_position.lat, global_position.lon, &x2, &y2);
	float dx = x1 - x2, dy = y1 - y2;
	float target_distance = sqrtf(dx * dx + dy * dy);
	float z = altitude - global_position.alt;

	return atan2f(z, target_distance);
}

void OutputBase::_set_angle_setpoints(const ControlData *control_data)
{
	_cur_control_data = control_data;

	switch (control_data->type) {
	case ControlData::Type::Angle:

		{
			for (int i = 0; i < 3; ++i) {
				switch (control_data->type_data.angle.frames[i]) {
				case ControlData::TypeData::TypeAngle::Frame::AngularRate:
					break;

				case ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame:
					_absolute_angle[i] = false;
					break;

				case ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame:
					_absolute_angle[i] = true;
					break;
				}

				_angle_velocity[i] = control_data->type_data.angle.angular_velocity[i];
			}

			for (int i = 0; i < 4; ++i) {
				_q_setpoint[i] = control_data->type_data.angle.q[i];
			}
		}

		break;

	case ControlData::Type::LonLat:
		_handle_position_update(true);
		break;

	case ControlData::Type::Neutral:
		_q_setpoint[0] = 1.f;
		_q_setpoint[1] = 0.f;
		_q_setpoint[2] = 0.f;
		_q_setpoint[3] = 0.f;
		_angle_velocity[0] = NAN;
		_angle_velocity[1] = NAN;
		_angle_velocity[2] = NAN;
		break;
	}

	for (int i = 0; i < 3; ++i) {
		_stabilize[i] = control_data->stabilize_axis[i];
	}
}

void OutputBase::_handle_position_update(bool force_update)
{
	if (!_cur_control_data || _cur_control_data->type != ControlData::Type::LonLat) {
		return;
	}

	vehicle_global_position_s vehicle_global_position{};
	vehicle_local_position_s vehicle_local_position{};

	if (force_update) {
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_vehicle_local_position_sub.copy(&vehicle_local_position);

	} else {
		if (!_vehicle_global_position_sub.update(&vehicle_global_position)) {
			return;
		}

		if (!_vehicle_local_position_sub.update(&vehicle_local_position)) {
			return;
		}
	}

	const double &vlat = vehicle_global_position.lat;
	const double &vlon = vehicle_global_position.lon;

	const double &lat = _cur_control_data->type_data.lonlat.lat;
	const double &lon = _cur_control_data->type_data.lonlat.lon;
	const float &alt = _cur_control_data->type_data.lonlat.altitude;

	float roll = _cur_control_data->type_data.lonlat.roll_angle;

	// interface: use fixed pitch value > -pi otherwise consider ROI altitude
	float pitch = (_cur_control_data->type_data.lonlat.pitch_fixed_angle >= -M_PI_F) ?
		      _cur_control_data->type_data.lonlat.pitch_fixed_angle :
		      _calculate_pitch(lon, lat, alt, vehicle_global_position);

	float yaw = get_bearing_to_next_waypoint(vlat, vlon, lat, lon) - vehicle_local_position.heading;

	// add offsets from VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET
	pitch += _cur_control_data->type_data.lonlat.pitch_angle_offset;
	yaw += _cur_control_data->type_data.lonlat.yaw_angle_offset;

	matrix::Quatf(matrix::Eulerf(roll, pitch, yaw)).copyTo(_q_setpoint);

	_angle_velocity[0] = NAN;
	_angle_velocity[1] = NAN;
	_angle_velocity[2] = NAN;
}

void OutputBase::_calculate_angle_output(const hrt_abstime &t)
{
	//get the output angles and stabilize if necessary
	vehicle_attitude_s vehicle_attitude{};
	matrix::Eulerf euler_vehicle;

	// We only need to apply additional compensation if the required angle is
	// absolute (world frame) as well as the gimbal is not capable of doing that
	// calculation. (Most gimbals stabilize at least roll and pitch
	// and only need compensation for yaw, if at all.)
	bool compensate[3];

	for (int i = 0; i < 3; ++i) {
		compensate[i] = _stabilize[i] && _absolute_angle[i];
	}

	if (compensate[0] || compensate[1] || compensate[2]) {
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		euler_vehicle = matrix::Quatf(vehicle_attitude.q);
	}

	float dt = (t - _last_update) / 1.e6f;

	matrix::Eulerf euler_gimbal = matrix::Quatf(_q_setpoint);

	for (int i = 0; i < 3; ++i) {

		if (PX4_ISFINITE(euler_gimbal(i))) {
			_angle_outputs[i] = euler_gimbal(i);
		}

		if (PX4_ISFINITE(_angle_velocity[i])) {
			_angle_outputs[i] += dt * _angle_velocity[i];
		}

		if (compensate[i]) {
			_angle_outputs[i] -= euler_vehicle(i);
		}

		if (PX4_ISFINITE(_angle_outputs[i])) {
			//bring angles into proper range [-pi, pi]
			_angle_outputs[i] = matrix::wrap_pi(_angle_outputs[i]);
		}
	}
}

} /* namespace vmount */

