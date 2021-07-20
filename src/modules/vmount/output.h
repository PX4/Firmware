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
 * @file output.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "common.h"
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

namespace vmount
{

struct OutputConfig {
	int32_t gimbal_as_uav_yaw;		/**< Use mixer output as UAV yaw */
	float gimbal_retracted_mode_value;	/**< Mixer output value for selecting gimbal retracted mode */
	float gimbal_normal_mode_value;		/**< Mixer output value for selecting gimbal normal mode */

	/** Scale factor for pitch channel (maps from angle in radians to actuator output in [-1,1]). OutputRC only. */
	float pitch_scale;
	/** Scale factor for roll channel (maps from angle in radians to actuator output in [-1,1]). OutputRC only. */
	float roll_scale;
	/** Scale factor for yaw channel (maps from angle in radians to actuator output in [-1,1]). OutputRC only. */
	float yaw_scale;

	float pitch_offset;	/**< Offset for pitch channel in radians */
	float roll_offset;	/**< Offset for roll channel in radians */
	float yaw_offset;	/**< Offset for yaw channel in radians */

	uint32_t mavlink_sys_id_v1;	/**< Mavlink target system id for mavlink output only for v1 */
	uint32_t mavlink_comp_id_v1;
};


/**
 ** class OutputBase
 * Base class for all driver output classes
 */
class OutputBase
{
public:
	OutputBase(const OutputConfig &output_config);
	virtual ~OutputBase() = default;

	virtual int initialize() { return 0; }

	/**
	 * Update the output.
	 * @param data new command if non null
	 * @return 0 on success, <0 otherwise
	 */
	virtual int update(const ControlData *control_data) = 0;

	/** report status to stdout */
	virtual void print_status() = 0;

	/** Publish _angle_outputs as a mount_orientation message. */
	void publish();

protected:
	float _calculate_pitch(double lon, double lat, float altitude,
			       const vehicle_global_position_s &global_position);

	map_projection_reference_s _projection_reference = {}; ///< reference to convert (lon, lat) to local [m]

	const OutputConfig &_config;

	/** set angle setpoints, speeds & stabilize flags */
	void _set_angle_setpoints(const ControlData *control_data);

	/** check if vehicle position changed and update the setpoint angles if in gps mode */
	void _handle_position_update(bool force_update = false);

	const ControlData *_cur_control_data = nullptr;

	float _q_setpoint[4] = { NAN, NAN, NAN, NAN }; ///< can be NAN if not specifically set
	float _angle_velocity[3] = { NAN, NAN, NAN }; //< [rad/s], can be NAN if not specifically set

	bool _stabilize[3] = { false, false, false };

	// Pitch and role are by default aligned with the horizon.
	// Yaw follows the vehicle (not lock/absolute mode).
	bool _absolute_angle[3] = {true, true, false };

	/** calculate the _angle_outputs (with speed) and stabilize if needed */
	void _calculate_angle_output(const hrt_abstime &t);

	float _angle_outputs[3] = { 0.f, 0.f, 0.f }; ///< calculated output angles (roll, pitch, yaw) [rad]
	hrt_abstime _last_update;

private:
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	uORB::Publication<mount_orientation_s> _mount_orientation_pub{ORB_ID(mount_orientation)};
};


} /* namespace vmount */
