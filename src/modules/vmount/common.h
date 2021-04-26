/****************************************************************************
*
*   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file common.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include <stdint.h>

namespace vmount
{

/**
 * @struct ControlData
 * This defines the common API between an input and an output of the vmount driver.
 * Each output must support the (full) set of the commands, and an input can create all
 * or a subset of the types.
 */
struct ControlData {

	enum class Type : uint8_t {
		Neutral = 0,      /**< move to neutral position */
		Angle,            /**< control the roll, pitch & yaw angle directly */
		LonLat            /**< control via lon, lat */
	};


	Type type = Type::Neutral;

	union TypeData {
		struct TypeAngle {
			float q[4]; /**< attitude quaternion */
			float angular_velocity[3]; // angular velocity

			// according to DO_MOUNT_CONFIGURE
			enum class Frame : uint8_t {
				AngleBodyFrame = 0, /**< Follow mode, angle relative to vehicle (usually default for yaw axis). */
				AngularRate = 1, /**< Angular rate set only, for compatibility with MAVLink v1 protocol. */
				AngleAbsoluteFrame = 2/**< Lock mode, angle relative to horizon/world, lock mode. (usually default for roll and pitch). */
			} frames[3]; /**< Mode. */
		} angle;

		struct TypeLonLat {
			double lon;              /**< longitude in [deg] */
			double lat;              /**< latitude in [deg] */
			float altitude;          /**< altitude in [m] */
			float roll_angle;        /**< roll is set to a fixed angle. Set to 0 for level horizon [rad] */
			float pitch_angle_offset; /**< angular offset for pitch [rad] */
			float yaw_angle_offset;  /**< angular offset for yaw [rad] */
			float pitch_fixed_angle; /**< ignored if < -pi, otherwise use a fixed pitch angle instead of the altitude */
		} lonlat;
	} type_data;


	bool stabilize_axis[3] = { false, false, false }; /**< whether the vmount driver should stabilize an axis
	 	 	 	 	 	 	 	 	 	 (if the output supports it, this can also be done externally) */
	bool gimbal_shutter_retract = false; /**< whether to lock the gimbal (only in RC output mode) */

};

} /* namespace vmount */
