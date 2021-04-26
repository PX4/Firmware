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
 * @file input_mavlink.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "input_mavlink.h"
#include <uORB/Publication.hpp>
#include <uORB/topics/gimbal_manager_information.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/gimbal_manager_set_attitude.h>
#include <uORB/topics/gimbal_manager_set_manual_control.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <errno.h>
#include <math.h>
#include <matrix/matrix/math.hpp>

namespace vmount
{

InputMavlinkROI::~InputMavlinkROI()
{
	if (_vehicle_roi_sub >= 0) {
		orb_unsubscribe(_vehicle_roi_sub);
	}

	if (_position_setpoint_triplet_sub >= 0) {
		orb_unsubscribe(_position_setpoint_triplet_sub);
	}
}

int InputMavlinkROI::initialize()
{
	_vehicle_roi_sub = orb_subscribe(ORB_ID(vehicle_roi));

	if (_vehicle_roi_sub < 0) {
		return -errno;
	}

	_position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

	if (_position_setpoint_triplet_sub < 0) {
		return -errno;
	}

	return 0;
}

int InputMavlinkROI::update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active)
{
	// already_active is unused, we don't care what happened previously.

	// Default to no change, set if we receive anything.
	*control_data = nullptr;

	const int num_poll = 2;
	px4_pollfd_struct_t polls[num_poll];
	polls[0].fd = 		_vehicle_roi_sub;
	polls[0].events = 	POLLIN;
	polls[1].fd = 		_position_setpoint_triplet_sub;
	polls[1].events = 	POLLIN;

	int ret = px4_poll(polls, num_poll, timeout_ms);

	if (ret < 0) {
		return -errno;
	}

	if (ret == 0) {
		// Timeout, _control_data is already null

	} else {
		if (polls[0].revents & POLLIN) {
			vehicle_roi_s vehicle_roi;
			orb_copy(ORB_ID(vehicle_roi), _vehicle_roi_sub, &vehicle_roi);

			_control_data.gimbal_shutter_retract = false;

			if (vehicle_roi.mode == vehicle_roi_s::ROI_NONE) {

				_control_data.type = ControlData::Type::Neutral;
				*control_data = &_control_data;
				_cur_roi_mode = vehicle_roi.mode;

			} else if (vehicle_roi.mode == vehicle_roi_s::ROI_WPNEXT) {
				_control_data.type = ControlData::Type::LonLat;
				_read_control_data_from_position_setpoint_sub();
				_control_data.type_data.lonlat.pitch_fixed_angle = -10.f;

				_control_data.type_data.lonlat.roll_angle = vehicle_roi.roll_offset;
				_control_data.type_data.lonlat.pitch_angle_offset = vehicle_roi.pitch_offset;
				_control_data.type_data.lonlat.yaw_angle_offset = vehicle_roi.yaw_offset;

				*control_data = &_control_data;
				_cur_roi_mode = vehicle_roi.mode;

			} else if (vehicle_roi.mode == vehicle_roi_s::ROI_LOCATION) {
				control_data_set_lon_lat(vehicle_roi.lon, vehicle_roi.lat, vehicle_roi.alt);

				*control_data = &_control_data;
				_cur_roi_mode = vehicle_roi.mode;

			} else if (vehicle_roi.mode == vehicle_roi_s::ROI_TARGET) {
				//TODO is this even suported?
			}
		}

		// check whether the position setpoint got updated
		if (polls[1].revents & POLLIN) {
			if (_cur_roi_mode == vehicle_roi_s::ROI_WPNEXT) {
				_read_control_data_from_position_setpoint_sub();
				*control_data = &_control_data;

			} else { // must do an orb_copy() in *every* case
				position_setpoint_triplet_s position_setpoint_triplet;
				orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub, &position_setpoint_triplet);
			}
		}
	}

	return 0;
}

void InputMavlinkROI::_read_control_data_from_position_setpoint_sub()
{
	position_setpoint_triplet_s position_setpoint_triplet;
	orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub, &position_setpoint_triplet);
	_control_data.type_data.lonlat.lon = position_setpoint_triplet.current.lon;
	_control_data.type_data.lonlat.lat = position_setpoint_triplet.current.lat;
	_control_data.type_data.lonlat.altitude = position_setpoint_triplet.current.alt;
}

void InputMavlinkROI::print_status()
{
	PX4_INFO("Input: Mavlink (ROI)");
}


InputMavlinkCmdMount::InputMavlinkCmdMount()
{
}

InputMavlinkCmdMount::~InputMavlinkCmdMount()
{
	if (_vehicle_command_sub >= 0) {
		orb_unsubscribe(_vehicle_command_sub);
	}
}

int InputMavlinkCmdMount::initialize()
{
	if ((_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command))) < 0) {
		return -errno;
	}

	// rate-limit inputs to 100Hz. If we don't do this and the output is configured to mavlink mode,
	// it will publish vehicle_command's as well, causing the input poll() in here to return
	// immediately, which in turn will cause an output update and thus a busy loop.
	orb_set_interval(_vehicle_command_sub, 10);

	return 0;
}


int InputMavlinkCmdMount::update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active)
{
	// Default to notify that there was no change.
	*control_data = nullptr;

	const int num_poll = 1;
	px4_pollfd_struct_t polls[num_poll];
	polls[0].fd = 		_vehicle_command_sub;
	polls[0].events = 	POLLIN;

	int poll_timeout = (int)timeout_ms;

	bool exit_loop = false;

	while (!exit_loop && poll_timeout >= 0) {
		hrt_abstime poll_start = hrt_absolute_time();

		int ret = px4_poll(polls, num_poll, poll_timeout);

		if (ret < 0) {
			return -errno;
		}

		poll_timeout -= (hrt_absolute_time() - poll_start) / 1000;

		// if we get a command that we need to handle, we exit the loop, otherwise we poll until we reach the timeout
		exit_loop = true;

		if (ret == 0) {
			// Timeout control_data already null.

		} else {
			if (polls[0].revents & POLLIN) {
				vehicle_command_s vehicle_command;
				orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &vehicle_command);

				// Process only if the command is for us or for anyone (component id 0).
				const bool sysid_correct = (vehicle_command.target_system == _mav_sys_id);
				const bool compid_correct = ((vehicle_command.target_component == _mav_comp_id) ||
							     (vehicle_command.target_component == 0));

				if (!sysid_correct || !compid_correct) {
					exit_loop = false;
					continue;
				}

				_control_data.gimbal_shutter_retract = false;

				if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL) {

					switch ((int)vehicle_command.param7) {
					case vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT:
						_control_data.gimbal_shutter_retract = true;

					/* FALLTHROUGH */

					case vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL:
						_control_data.type = ControlData::Type::Neutral;

						*control_data = &_control_data;
						break;

					case vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING: {
							_control_data.type = ControlData::Type::Angle;
							_control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
							_control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
							_control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

							// vmount spec has roll on channel 0, MAVLink spec has pitch on channel 0
							const float roll = math::radians(vehicle_command.param2);
							// vmount spec has pitch on channel 1, MAVLink spec has roll on channel 1
							const float pitch = math::radians(vehicle_command.param1);
							// both specs have yaw on channel 2
							float yaw = math::radians(vehicle_command.param3);

							matrix::Eulerf euler(roll, pitch, yaw);

							matrix::Quatf q(euler);
							q.copyTo(_control_data.type_data.angle.q);

							_control_data.type_data.angle.angular_velocity[0] = NAN;
							_control_data.type_data.angle.angular_velocity[1] = NAN;
							_control_data.type_data.angle.angular_velocity[2] = NAN;

							*control_data = &_control_data;
						}
						break;

					case vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
						break;

					case vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT:
						control_data_set_lon_lat((double)vehicle_command.param6, (double)vehicle_command.param5, vehicle_command.param4);

						*control_data = &_control_data;
						break;
					}

					_ack_vehicle_command(&vehicle_command);

				} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE) {

					_control_data.stabilize_axis[0] = (int)(vehicle_command.param2 + 0.5f) == 1;
					_control_data.stabilize_axis[1] = (int)(vehicle_command.param3 + 0.5f) == 1;
					_control_data.stabilize_axis[2] = (int)(vehicle_command.param4 + 0.5f) == 1;


					const int params[] = {
						(int)((float)vehicle_command.param5 + 0.5f),
						(int)((float)vehicle_command.param6 + 0.5f),
						(int)(vehicle_command.param7 + 0.5f)
					};

					for (int i = 0; i < 3; ++i) {

						if (params[i] == 0) {
							_control_data.type_data.angle.frames[i] =
								ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

						} else if (params[i] == 1) {
							_control_data.type_data.angle.frames[i] =
								ControlData::TypeData::TypeAngle::Frame::AngularRate;

						} else if (params[i] == 2) {
							_control_data.type_data.angle.frames[i] =
								ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;

						} else {
							// Not supported, fallback to body angle.
							_control_data.type_data.angle.frames[i] =
								ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
						}
					}

					_control_data.type = ControlData::Type::Neutral; //always switch to neutral position

					*control_data = &_control_data;
					_ack_vehicle_command(&vehicle_command);

				} else {
					exit_loop = false;
				}
			}

		}
	}

	return 0;
}

void InputMavlinkCmdMount::_ack_vehicle_command(vehicle_command_s *cmd)
{
	vehicle_command_ack_s vehicle_command_ack{};

	vehicle_command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack.command = cmd->command;
	vehicle_command_ack.result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
	vehicle_command_ack.target_system = cmd->source_system;
	vehicle_command_ack.target_component = cmd->source_component;

	uORB::Publication<vehicle_command_ack_s> cmd_ack_pub{ORB_ID(vehicle_command_ack)};
	cmd_ack_pub.publish(vehicle_command_ack);
}

void InputMavlinkCmdMount::print_status()
{
	PX4_INFO("Input: Mavlink (CMD_MOUNT)");
}

InputMavlinkGimbalV2::InputMavlinkGimbalV2(uint8_t mav_sys_id, uint8_t mav_comp_id,
		float &mnt_rate_pitch, float &mnt_rate_yaw) :
	_mav_sys_id(mav_sys_id),
	_mav_comp_id(mav_comp_id),
	_mnt_rate_pitch(mnt_rate_pitch),
	_mnt_rate_yaw(mnt_rate_yaw)
{
	_stream_gimbal_manager_information();
}

InputMavlinkGimbalV2::~InputMavlinkGimbalV2()
{
	if (_vehicle_roi_sub >= 0) {
		orb_unsubscribe(_vehicle_roi_sub);
	}

	if (_position_setpoint_triplet_sub >= 0) {
		orb_unsubscribe(_position_setpoint_triplet_sub);
	}

	if (_gimbal_manager_set_attitude_sub >= 0) {
		orb_unsubscribe(_gimbal_manager_set_attitude_sub);
	}

	if (_vehicle_command_sub >= 0) {
		orb_unsubscribe(_vehicle_command_sub);
	}

	if (_gimbal_manager_set_manual_control_sub >= 0) {
		orb_unsubscribe(_gimbal_manager_set_manual_control_sub);
	}
}


void InputMavlinkGimbalV2::print_status()
{
	PX4_INFO("Input: Mavlink (Gimbal V2)");
}

int InputMavlinkGimbalV2::initialize()
{
	_vehicle_roi_sub = orb_subscribe(ORB_ID(vehicle_roi));

	if (_vehicle_roi_sub < 0) {
		return -errno;
	}

	_position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

	if (_position_setpoint_triplet_sub < 0) {
		return -errno;
	}

	_gimbal_manager_set_attitude_sub  = orb_subscribe(ORB_ID(gimbal_manager_set_attitude));

	if (_gimbal_manager_set_attitude_sub < 0) {
		return -errno;
	}

	if ((_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command))) < 0) {
		return -errno;
	}

	if ((_gimbal_manager_set_manual_control_sub = orb_subscribe(ORB_ID(gimbal_manager_set_manual_control))) < 0) {
		return -errno;
	}

	// rate-limit inputs to 100Hz. If we don't do this and the output is configured to mavlink mode,
	// it will publish vehicle_command's as well, causing the input poll() in here to return
	// immediately, which in turn will cause an output update and thus a busy loop.
	orb_set_interval(_vehicle_command_sub, 10);

	return 0;
}

void InputMavlinkGimbalV2::_stream_gimbal_manager_status()
{
	gimbal_device_attitude_status_s gimbal_device_attitude_status{};

	if (_gimbal_device_attitude_status_sub.updated()) {
		_gimbal_device_attitude_status_sub.copy(&gimbal_device_attitude_status);
	}

	gimbal_manager_status_s gimbal_manager_status{};
	gimbal_manager_status.timestamp = hrt_absolute_time();
	gimbal_manager_status.flags = gimbal_device_attitude_status.device_flags;
	gimbal_manager_status.gimbal_device_id = 0;
	gimbal_manager_status.primary_control_sysid = _sys_id_primary_control;
	gimbal_manager_status.primary_control_compid = _comp_id_primary_control;
	gimbal_manager_status.secondary_control_sysid = 0; // TODO: support secondary control
	gimbal_manager_status.secondary_control_compid = 0; // TODO: support secondary control
	_gimbal_manager_status_pub.publish(gimbal_manager_status);
}

void InputMavlinkGimbalV2::_stream_gimbal_manager_information()
{
	// TODO: Take gimbal_device_information into account.

	gimbal_manager_information_s gimbal_manager_info;
	gimbal_manager_info.timestamp = hrt_absolute_time();

	gimbal_manager_info.cap_flags =
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL;

	gimbal_manager_info.pitch_max = M_PI_F / 2;
	gimbal_manager_info.pitch_min = -M_PI_F / 2;
	gimbal_manager_info.yaw_max = M_PI_F;
	gimbal_manager_info.yaw_min = -M_PI_F;

	_gimbal_manager_info_pub.publish(gimbal_manager_info);
}

int InputMavlinkGimbalV2::update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active)
{
	_stream_gimbal_manager_status();

	// Default to no change, set if we receive anything.
	*control_data = nullptr;

	const int num_poll = 5;
	px4_pollfd_struct_t polls[num_poll];
	polls[0].fd = 		_gimbal_manager_set_attitude_sub;
	polls[0].events = 	POLLIN;
	polls[1].fd = 		_vehicle_roi_sub;
	polls[1].events = 	POLLIN;
	polls[2].fd = 		_position_setpoint_triplet_sub;
	polls[2].events = 	POLLIN;
	polls[3].fd = 		_vehicle_command_sub;
	polls[3].events = 	POLLIN;
	polls[4].fd = 		_gimbal_manager_set_manual_control_sub;
	polls[4].events = 	POLLIN;

	int poll_timeout = (int)timeout_ms;

	bool exit_loop = false;

	while (!exit_loop && poll_timeout >= 0) {
		hrt_abstime poll_start = hrt_absolute_time();

		int ret = px4_poll(polls, num_poll, poll_timeout);

		if (ret < 0) {
			return -errno;
		}

		poll_timeout -= (hrt_absolute_time() - poll_start) / 1000;

		// if we get a command that we need to handle, we exit the loop, otherwise we poll until we reach the timeout
		exit_loop = true;

		if (ret == 0) {
			// Timeout control_data already null.

		} else {
			if (polls[0].revents & POLLIN) {
				gimbal_manager_set_attitude_s set_attitude;
				orb_copy(ORB_ID(gimbal_manager_set_attitude), _gimbal_manager_set_attitude_sub, &set_attitude);

				if (set_attitude.origin_sysid == _sys_id_primary_control &&
				    set_attitude.origin_compid == _comp_id_primary_control) {
					const matrix::Quatf q(set_attitude.q);
					const matrix::Vector3f angular_velocity(set_attitude.angular_velocity_x,
										set_attitude.angular_velocity_y,
										set_attitude.angular_velocity_z);

					_set_control_data_from_set_attitude(set_attitude.flags, q, angular_velocity);
					*control_data = &_control_data;

				} else {
					PX4_DEBUG("Ignoring gimbal_manager_set_attitude from %d/%d",
						  set_attitude.origin_sysid, set_attitude.origin_compid);
				}
			}

			if (polls[1].revents & POLLIN) {
				vehicle_roi_s vehicle_roi;
				orb_copy(ORB_ID(vehicle_roi), _vehicle_roi_sub, &vehicle_roi);

				_control_data.gimbal_shutter_retract = false;

				if (vehicle_roi.mode == vehicle_roi_s::ROI_NONE) {

					_control_data.type = ControlData::Type::Neutral;
					*control_data = &_control_data;
					_cur_roi_mode = vehicle_roi.mode;

				} else if (vehicle_roi.mode == vehicle_roi_s::ROI_WPNEXT) {
					_control_data.type = ControlData::Type::LonLat;
					_read_control_data_from_position_setpoint_sub();
					_control_data.type_data.lonlat.pitch_fixed_angle = -10.f;

					_control_data.type_data.lonlat.roll_angle = vehicle_roi.roll_offset;
					_control_data.type_data.lonlat.pitch_angle_offset = vehicle_roi.pitch_offset;
					_control_data.type_data.lonlat.yaw_angle_offset = vehicle_roi.yaw_offset;

					*control_data = &_control_data;
					_cur_roi_mode = vehicle_roi.mode;

				} else if (vehicle_roi.mode == vehicle_roi_s::ROI_LOCATION) {
					control_data_set_lon_lat(vehicle_roi.lon, vehicle_roi.lat, vehicle_roi.alt);

					*control_data = &_control_data;
					_cur_roi_mode = vehicle_roi.mode;

				} else if (vehicle_roi.mode == vehicle_roi_s::ROI_TARGET) {
					//TODO is this even suported?
					exit_loop = false;

				} else {
					exit_loop = false;
				}
			}

			// check whether the position setpoint got updated
			if (polls[2].revents & POLLIN) {
				if (_cur_roi_mode == vehicle_roi_s::ROI_WPNEXT) {
					_read_control_data_from_position_setpoint_sub();
					*control_data = &_control_data;

				} else { // must do an orb_copy() in *every* case
					position_setpoint_triplet_s position_setpoint_triplet;
					orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub, &position_setpoint_triplet);
					exit_loop = false;
				}
			}

			if (polls[3].revents & POLLIN) {
				vehicle_command_s vehicle_command;
				orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &vehicle_command);

				// Process only if the command is for us or for anyone (component id 0).
				const bool sysid_correct = (vehicle_command.target_system == _mav_sys_id) || (vehicle_command.target_system == 0);
				const bool compid_correct = ((vehicle_command.target_component == _mav_comp_id) ||
							     (vehicle_command.target_component == 0));

				if (!sysid_correct || !compid_correct) {
					exit_loop = false;
					continue;
				}

				if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL) {
					// FIXME: Remove me later. For now, we support this for legacy missions
					//        using gimbal v1 protocol.

					switch ((int)vehicle_command.param7) {
					case vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT:
						_control_data.gimbal_shutter_retract = true;

					/* FALLTHROUGH */

					case vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL:
						_control_data.type = ControlData::Type::Neutral;

						*control_data = &_control_data;
						break;

					case vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING: {
							_control_data.type = ControlData::Type::Angle;
							_control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
							_control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
							_control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

							// vmount spec has roll on channel 0, MAVLink spec has pitch on channel 0
							const float roll = math::radians(vehicle_command.param2);
							// vmount spec has pitch on channel 1, MAVLink spec has roll on channel 1
							const float pitch = math::radians(vehicle_command.param1);
							// both specs have yaw on channel 2
							float yaw = math::radians(vehicle_command.param3);

							// We expect angle of [-pi..+pi]. If the input range is [0..2pi] we can fix that.
							if (yaw > M_PI_F) {
								yaw -= 2 * M_PI_F;
							}

							matrix::Eulerf euler(roll, pitch, yaw);

							matrix::Quatf q(euler);
							q.copyTo(_control_data.type_data.angle.q);

							_control_data.type_data.angle.angular_velocity[0] = NAN;
							_control_data.type_data.angle.angular_velocity[1] = NAN;
							_control_data.type_data.angle.angular_velocity[2] = NAN;

							*control_data = &_control_data;
						}
						break;

					case vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
						break;

					case vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT:
						control_data_set_lon_lat((double)vehicle_command.param6, (double)vehicle_command.param5, vehicle_command.param4);

						*control_data = &_control_data;
						break;
					}

					_ack_vehicle_command(&vehicle_command, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

				} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE) {

					_control_data.stabilize_axis[0] = (int)(vehicle_command.param2 + 0.5f) == 1;
					_control_data.stabilize_axis[1] = (int)(vehicle_command.param3 + 0.5f) == 1;
					_control_data.stabilize_axis[2] = (int)(vehicle_command.param4 + 0.5f) == 1;


					const int params[] = {
						(int)((float)vehicle_command.param5 + 0.5f),
						(int)((float)vehicle_command.param6 + 0.5f),
						(int)(vehicle_command.param7 + 0.5f)
					};

					for (int i = 0; i < 3; ++i) {

						if (params[i] == 0) {
							_control_data.type_data.angle.frames[i] =
								ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

						} else if (params[i] == 1) {
							_control_data.type_data.angle.frames[i] =
								ControlData::TypeData::TypeAngle::Frame::AngularRate;

						} else if (params[i] == 2) {
							_control_data.type_data.angle.frames[i] =
								ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;

						} else {
							// Not supported, fallback to body angle.
							_control_data.type_data.angle.frames[i] =
								ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
						}
					}

					_control_data.type = ControlData::Type::Neutral; //always switch to neutral position

					*control_data = &_control_data;
					_ack_vehicle_command(&vehicle_command, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

				} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE) {

					const int param_sys_id = roundf(vehicle_command.param1);
					const int param_comp_id = roundf(vehicle_command.param2);

					uint8_t new_sys_id_primary_control = [&]() {
						if (param_sys_id >= 0 && param_sys_id < 256) {
							// Valid new sysid.
							return (uint8_t)param_sys_id;

						} else if (param_sys_id == -1) {
							// leave unchanged
							return _sys_id_primary_control;

						} else if (param_sys_id == -2) {
							// set itself
							return _mav_sys_id;

						} else if (param_sys_id == -3) {
							// release control if in control
							if (_sys_id_primary_control == vehicle_command.source_system) {
								return (uint8_t)0;

							} else {
								return _sys_id_primary_control;
							}

						} else {
							PX4_WARN("Unknown param1 value for DO_GIMBAL_MANAGER_CONFIGURE");
							return _sys_id_primary_control;
						}
					}();

					uint8_t new_comp_id_primary_control = [&]() {
						if (param_comp_id >= 0 && param_comp_id < 256) {
							// Valid new compid.
							return (uint8_t)param_comp_id;

						} else if (param_comp_id == -1) {
							// leave unchanged
							return _comp_id_primary_control;

						} else if (param_comp_id == -2) {
							// set itself
							return _mav_comp_id;

						} else if (param_comp_id == -3) {
							// release control if in control
							if (_comp_id_primary_control == vehicle_command.source_component) {
								return (uint8_t)0;

							} else {
								return _comp_id_primary_control;
							}

						} else {
							PX4_WARN("Unknown param2 value for DO_GIMBAL_MANAGER_CONFIGURE");
							return _comp_id_primary_control;
						}
					}();


					if (new_sys_id_primary_control != _sys_id_primary_control ||
					    new_comp_id_primary_control != _comp_id_primary_control) {
						PX4_INFO("Configured primary gimbal control sysid/compid from %d/%d to %d/%d",
							 _sys_id_primary_control, _comp_id_primary_control,
							 new_sys_id_primary_control, new_comp_id_primary_control);
						_sys_id_primary_control = new_sys_id_primary_control;
						_comp_id_primary_control = new_comp_id_primary_control;
					}

					// TODO: support secondary control
					// TODO: support gimbal device id for multiple gimbals

					_ack_vehicle_command(&vehicle_command, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

				} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW) {

					if (vehicle_command.source_system == _sys_id_primary_control &&
					    vehicle_command.source_component == _comp_id_primary_control) {

						const matrix::Eulerf euler(0.0f, math::radians(vehicle_command.param1), math::radians(vehicle_command.param2));
						const matrix::Quatf q(euler);
						const matrix::Vector3f angular_velocity(0.0f, vehicle_command.param3, vehicle_command.param4);
						const uint32_t flags = vehicle_command.param5;

						// TODO: support gimbal device id for multiple gimbals

						_set_control_data_from_set_attitude(flags, q, angular_velocity);
						*control_data = &_control_data;
						_ack_vehicle_command(&vehicle_command, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);

					} else {
						PX4_ERR("GIMBAL_MANAGER_PITCHYAW denied, not in control");
						_ack_vehicle_command(&vehicle_command, vehicle_command_s::VEHICLE_CMD_RESULT_DENIED);
					}

				} else {
					exit_loop = false;
				}

			}

			if (polls[4].revents & POLLIN) {
				gimbal_manager_set_manual_control_s set_manual_control;
				orb_copy(ORB_ID(gimbal_manager_set_manual_control), _gimbal_manager_set_manual_control_sub, &set_manual_control);

				if (set_manual_control.origin_sysid == _sys_id_primary_control &&
				    set_manual_control.origin_compid == _comp_id_primary_control) {

					const matrix::Quatf q =
						(PX4_ISFINITE(set_manual_control.pitch) && PX4_ISFINITE(set_manual_control.yaw)) ?
						matrix::Quatf(matrix::Eulerf(0.0f, set_manual_control.pitch, set_manual_control.yaw)) :
						matrix::Quatf(NAN, NAN, NAN, NAN);

					const matrix::Vector3f angular_velocity =
						(PX4_ISFINITE(set_manual_control.pitch_rate) && PX4_ISFINITE(set_manual_control.yaw_rate)) ?
						matrix::Vector3f(0.0f,
								 math::radians(_mnt_rate_pitch) * set_manual_control.pitch_rate,
								 math::radians(_mnt_rate_yaw) * set_manual_control.yaw_rate) :
						matrix::Vector3f(NAN, NAN, NAN);

					_set_control_data_from_set_attitude(set_manual_control.flags, q, angular_velocity);
					*control_data = &_control_data;

				} else {
					PX4_DEBUG("Ignoring gimbal_manager_set_manual_control from %d/%d",
						  set_manual_control.origin_sysid, set_manual_control.origin_compid);
				}
			}
		}
	}

	return 0;
}

void InputMavlinkGimbalV2::_set_control_data_from_set_attitude(const uint32_t flags, const matrix::Quatf &q,
		const matrix::Vector3f &angular_velocity)
{
	if ((flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_RETRACT) != 0) {
		// not implemented in ControlData
	} else if ((flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_NEUTRAL) != 0) {
		_control_data.type = ControlData::Type::Neutral;

	} else {
		_control_data.type = ControlData::Type::Angle;

		q.copyTo(_control_data.type_data.angle.q);

		_control_data.type_data.angle.angular_velocity[0] = angular_velocity(0);
		_control_data.type_data.angle.angular_velocity[1] = angular_velocity(1);
		_control_data.type_data.angle.angular_velocity[2] = angular_velocity(2);

		_control_data.type_data.angle.frames[0] = (flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_ROLL_LOCK)
				? ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame
				: ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

		_control_data.type_data.angle.frames[1] = (flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_PITCH_LOCK)
				? ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame
				: ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

		_control_data.type_data.angle.frames[2] = (flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_YAW_LOCK)
				? ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame
				: ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
	}
}

//TODO move this one to input.cpp such that it can be shared across functions
void InputMavlinkGimbalV2::_ack_vehicle_command(vehicle_command_s *cmd, uint8_t result)
{
	vehicle_command_ack_s vehicle_command_ack{};

	vehicle_command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack.command = cmd->command;
	vehicle_command_ack.result = result;
	vehicle_command_ack.target_system = cmd->source_system;
	vehicle_command_ack.target_component = cmd->source_component;

	uORB::Publication<vehicle_command_ack_s> cmd_ack_pub{ORB_ID(vehicle_command_ack)};
	cmd_ack_pub.publish(vehicle_command_ack);
}

void InputMavlinkGimbalV2::_read_control_data_from_position_setpoint_sub()
{
	position_setpoint_triplet_s position_setpoint_triplet;
	orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub, &position_setpoint_triplet);
	_control_data.type_data.lonlat.lon = position_setpoint_triplet.current.lon;
	_control_data.type_data.lonlat.lat = position_setpoint_triplet.current.lat;
	_control_data.type_data.lonlat.altitude = position_setpoint_triplet.current.alt;
}

} /* namespace vmount */
