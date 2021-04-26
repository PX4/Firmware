/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file navigation.h
 * Definition of a mission consisting of mission items.
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#if defined(MEMORY_CONSTRAINED_SYSTEM)
#  define NUM_MISSIONS_SUPPORTED 50
#elif defined(__PX4_POSIX)
#  define NUM_MISSIONS_SUPPORTED (UINT16_MAX-1) // This is allocated as needed.
#else
#  define NUM_MISSIONS_SUPPORTED 2000 // This allocates a file of around 181 kB on the SD card.
#endif

#define NAV_EPSILON_POSITION	0.001f	/**< Anything smaller than this is considered zero */

/* compatible to mavlink MAV_CMD */
enum NAV_CMD {
	NAV_CMD_IDLE = 0,
	NAV_CMD_WAYPOINT = 16,
	NAV_CMD_LOITER_UNLIMITED = 17,
	NAV_CMD_LOITER_TIME_LIMIT = 19,
	NAV_CMD_RETURN_TO_LAUNCH = 20,
	NAV_CMD_LAND = 21,
	NAV_CMD_TAKEOFF = 22,
	NAV_CMD_LOITER_TO_ALT = 31,
	NAV_CMD_DO_FOLLOW_REPOSITION = 33,
	NAV_CMD_VTOL_TAKEOFF = 84,
	NAV_CMD_VTOL_LAND = 85,
	NAV_CMD_DELAY = 93,
	NAV_CMD_DO_JUMP = 177,
	NAV_CMD_DO_CHANGE_SPEED = 178,
	NAV_CMD_DO_SET_HOME = 179,
	NAV_CMD_DO_SET_SERVO = 183,
	NAV_CMD_DO_LAND_START = 189,
	NAV_CMD_DO_SET_ROI_LOCATION = 195,
	NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196,
	NAV_CMD_DO_SET_ROI_NONE = 197,
	NAV_CMD_DO_CONTROL_VIDEO = 200,
	NAV_CMD_DO_SET_ROI = 201,
	NAV_CMD_DO_DIGICAM_CONTROL = 203,
	NAV_CMD_DO_MOUNT_CONFIGURE = 204,
	NAV_CMD_DO_MOUNT_CONTROL = 205,
	NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
	NAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
	NAV_CMD_OBLIQUE_SURVEY = 260,
	NAV_CMD_SET_CAMERA_MODE = 530,
	NAV_CMD_SET_CAMERA_ZOOM = 531,
	NAV_CMD_SET_CAMERA_FOCUS = 532,
	NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000,
	NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001,
	NAV_CMD_IMAGE_START_CAPTURE = 2000,
	NAV_CMD_IMAGE_STOP_CAPTURE = 2001,
	NAV_CMD_DO_TRIGGER_CONTROL = 2003,
	NAV_CMD_VIDEO_START_CAPTURE = 2500,
	NAV_CMD_VIDEO_STOP_CAPTURE = 2501,
	NAV_CMD_DO_VTOL_TRANSITION = 3000,
	NAV_CMD_FENCE_RETURN_POINT = 5000,
	NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
	NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
	NAV_CMD_FENCE_CIRCLE_INCLUSION = 5003,
	NAV_CMD_FENCE_CIRCLE_EXCLUSION = 5004,
	NAV_CMD_CONDITION_GATE = 4501,
	NAV_CMD_INVALID = UINT16_MAX /* ensure that casting a large number results in a specific error */
};

enum ORIGIN {
	ORIGIN_MAVLINK = 0,
	ORIGIN_ONBOARD
};

/* compatible to mavlink MAV_FRAME */
enum NAV_FRAME {
	NAV_FRAME_GLOBAL = 0,
	NAV_FRAME_LOCAL_NED = 1,
	NAV_FRAME_MISSION = 2,
	NAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
	NAV_FRAME_LOCAL_ENU = 4,
	NAV_FRAME_GLOBAL_INT = 5,
	NAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
	NAV_FRAME_LOCAL_OFFSET_NED = 7,
	NAV_FRAME_BODY_NED = 8,
	NAV_FRAME_BODY_OFFSET_NED = 9,
	NAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
	NAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
};

/**
 * @addtogroup topics
 * @{
 */

/**
 * Global position setpoint in WGS84 coordinates.
 *
 * This is the position the MAV is heading towards. If it is of type loiter,
 * the MAV is circling around it with the given loiter radius in meters.
 *
 * Corresponds to one of the DM_KEY_WAYPOINTS_OFFBOARD_* dataman items
 */

// Mission Item structure
//  We explicitly handle struct padding to ensure consistency between in memory and on disk formats across different platforms, toolchains, etc
//  The use of #pragma pack is avoided to prevent the possibility of unaligned memory accesses.

#if (__GNUC__ >= 5) || __clang__
//  Disabled in GCC 4.X as the warning doesn't seem to "pop" correctly
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wpadded"
#endif // GCC >= 5 || Clang

struct mission_item_s {
	double lat;					/**< latitude in degrees				*/
	double lon;					/**< longitude in degrees				*/
	union {
		struct {
			union {
				float time_inside;		/**< time that the MAV should stay inside the radius before advancing in seconds */
				float circle_radius;		/**< geofence circle radius in meters (only used for NAV_CMD_NAV_FENCE_CIRCLE*) */
			};
			float acceptance_radius;		/**< default radius in which the mission is accepted as reached in meters */
			float loiter_radius;			/**< loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise */
			float yaw;				/**< in radians NED -PI..+PI, NAN means don't change yaw		*/
			float ___lat_float;			/**< padding */
			float ___lon_float;			/**< padding */
			float altitude;				/**< altitude in meters	(AMSL)			*/
		};
		float params[7];				/**< array to store mission command values for MAV_FRAME_MISSION ***/
	};

	uint16_t nav_cmd;				/**< navigation command					*/

	int16_t do_jump_mission_index;			/**< index where the do jump will go to                 */
	uint16_t do_jump_repeat_count;			/**< how many times do jump needs to be done            */

	union {
		uint16_t do_jump_current_count;			/**< count how many times the jump has been done	*/
		uint16_t vertex_count;				/**< Polygon vertex count (geofence)	*/
		uint16_t land_precision;			/**< Defines if landing should be precise: 0 = normal landing, 1 = opportunistic precision landing, 2 = required precision landing (with search)	*/
	};

	struct {
		uint16_t frame : 4,				/**< mission frame */
			 origin : 3,				/**< how the mission item was generated */
			 loiter_exit_xtrack : 1,		/**< exit xtrack location: 0 for center of loiter wp, 1 for exit location */
			 force_heading : 1,			/**< heading needs to be reached */
			 altitude_is_relative : 1,		/**< true if altitude is relative from start point	*/
			 autocontinue : 1,			/**< true if next waypoint should follow after this one */
			 vtol_back_transition : 1,		/**< part of the vtol back transition sequence */
			 _padding0 : 4;				/**< padding remaining unused bits  */
	};

	uint8_t _padding1[2];				/**< padding struct size to alignment boundary  */
};

/**
 * dataman housekeeping information for a specific item.
 * Corresponds to the first dataman entry of DM_KEY_FENCE_POINTS and DM_KEY_SAFE_POINTS
 */
struct mission_stats_entry_s {
	uint16_t num_items;			/**< total number of items stored (excluding this one) */
	uint16_t update_counter;			/**< This counter is increased when (some) items change (this can wrap) */
};

/**
 * Geofence vertex point.
 * Corresponds to the DM_KEY_FENCE_POINTS dataman item
 */
struct mission_fence_point_s {
	double lat;
	double lon;
	float alt;

	union {
		uint16_t vertex_count;			/**< number of vertices in this polygon */
		float circle_radius;			/**< geofence circle radius in meters (only used for NAV_CMD_NAV_FENCE_CIRCLE*) */
	};

	uint16_t nav_cmd;				/**< navigation command (one of MAV_CMD_NAV_FENCE_*) */
	uint8_t frame;					/**< MAV_FRAME */

	uint8_t _padding0[5];				/**< padding struct size to alignment boundary  */
};

/**
 * Safe Point (Rally Point).
 * Corresponds to the DM_KEY_SAFE_POINTS dataman item
 */
struct mission_safe_point_s {
	double lat;
	double lon;
	float alt;
	uint8_t frame;					/**< MAV_FRAME */

	uint8_t _padding0[3];				/**< padding struct size to alignment boundary  */
};

#if (__GNUC__ >= 5) || __clang__
#pragma GCC diagnostic pop
#endif // GCC >= 5 || Clang

#include <uORB/topics/mission.h>

/**
 * @}
 */
