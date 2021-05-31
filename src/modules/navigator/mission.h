/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file mission.h
 *
 * Navigator mode to access missions
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include "mission_block.h"
#include "mission_feasibility_checker.h"
#include "navigator_mode.h"

#include <float.h>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/uORB.h>

class Navigator;

class Mission : public MissionBlock, public ModuleParams
{
public:
	Mission(Navigator *navigator, NavigatorCore &navigator_core);
	~Mission() override = default;

	void on_inactive() override;
	void on_inactivation() override;
	void on_activation() override;
	void on_active() override;

	bool set_current_mission_index(uint16_t index);

	bool land_start();
	bool landing();

	uint16_t get_land_start_index() const { return _land_start_index; }
	bool get_land_start_available() const { return _land_start_available; }
	bool get_mission_finished() const { return _mission_type == MISSION_TYPE_NONE; }
	bool get_mission_changed() const { return _mission_changed ; }
	bool get_mission_waypoints_changed() const { return _mission_waypoints_changed ; }
	double get_landing_start_lat() { return _landing_start_lat; }
	double get_landing_start_lon() { return _landing_start_lon; }
	float get_landing_start_alt() { return _landing_start_alt; }

	double get_landing_lat() { return _landing_lat; }
	double get_landing_lon() { return _landing_lon; }
	float get_landing_alt() { return _landing_alt; }

	void set_closest_item_as_current();

	/**
	 * Set a new mission mode and handle the switching between the different modes
	 *
	 * For a list of the different modes refer to mission_result.msg
	 */
	void set_execution_mode(const uint8_t mode);
private:

	/**
	 * Update mission topic
	 */
	void update_mission();

	/**
	 * Move on to next mission item or switch to loiter
	 */
	void advance_mission();

	/**
	 * Set new mission items
	 */
	void set_mission_items();

	/**
	 * Returns true if we need to do a takeoff at the current state
	 */
	bool do_need_vertical_takeoff();

	/**
	 * Returns true if we need to move to waypoint location before starting descent
	 */
	bool do_need_move_to_land();

	/**
	 * Returns true if we need to move to waypoint location after vtol takeoff
	 */
	bool do_need_move_to_takeoff();

	/**
	 * Copies position from setpoint if valid, otherwise copies current position
	 */
	void copy_position_if_valid(struct mission_item_s *mission_item, struct position_setpoint_s *setpoint);

	/**
	 * Create mission item to align towards next waypoint
	 */
	void set_align_mission_item(struct mission_item_s *mission_item, struct mission_item_s *mission_item_next);

	/**
	 * Calculate takeoff height for mission item considering ground clearance
	 */
	float calculate_takeoff_altitude(struct mission_item_s *mission_item);

	/**
	 * Updates the heading of the vehicle. Rotary wings only.
	 */
	void heading_sp_update();

	/**
	 * Update the cruising speed setpoint.
	 */
	void cruising_speed_sp_update();

	/**
	 * Abort landing
	 */
	void do_abort_landing();

	/**
	 * Read the current and the next mission item. The next mission item read is the
	 * next mission item that contains a position.
	 *
	 * @return true if current mission item available
	 */
	bool prepare_mission_items(mission_item_s *mission_item,
				   mission_item_s *next_position_mission_item, bool *has_next_position_item,
				   mission_item_s *next_next_position_mission_item = nullptr, bool *has_next_next_position_item = nullptr);

	/**
	 * Read current (offset == 0) or a specific (offset > 0) mission item
	 * from the dataman and watch out for DO_JUMPS
	 *
	 * @return true if successful
	 */
	bool read_mission_item(int offset, struct mission_item_s *mission_item);

	/**
	 * Save current mission state to dataman
	 */
	void save_mission_state();

	/**
	 * Inform about a changed mission item after a DO_JUMP
	 */
	void report_do_jump_mission_changed(int index, int do_jumps_remaining);

	/**
	 * Set a mission item as reached
	 */
	void set_mission_item_reached();

	/**
	 * Set the current mission item
	 */
	void set_current_mission_item();

	/**
	 * Check whether a mission is ready to go
	 */
	void check_mission_valid(bool force);

	/**
	 * Reset mission
	 */
	void reset_mission(struct mission_s &mission);

	/**
	 * Returns true if we need to reset the mission (call this only when inactive)
	 */
	bool need_to_reset_mission();

	/**
	 * Project current location with heading to far away location and fill setpoint.
	 */
	void generate_waypoint_from_heading(struct position_setpoint_s *setpoint, float yaw);

	/**
	 * Find and store the index of the landing sequence (DO_LAND_START)
	 */
	bool find_mission_land_start();

	/**
	 * Return the index of the closest mission item to the current global position.
	 */
	int32_t index_closest_mission_item() const;

	bool position_setpoint_equal(const position_setpoint_s *p1, const position_setpoint_s *p2) const;

	void publish_navigator_mission_item();

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MIS_DIST_1WP>) _param_mis_dist_1wp,
		(ParamFloat<px4::params::MIS_DIST_WPS>) _param_mis_dist_wps,
		(ParamInt<px4::params::MIS_MNT_YAW_CTL>) _param_mis_mnt_yaw_ctl
	)

	uORB::Publication<navigator_mission_item_s> _navigator_mission_item_pub{ORB_ID::navigator_mission_item};

	uORB::Subscription	_mission_sub{ORB_ID(mission)};		/**< mission subscription */
	mission_s		_mission {};

	int32_t _current_mission_index{-1};

	// track location of planned mission landing
	bool	_land_start_available{false};
	uint16_t _land_start_index{UINT16_MAX};		/**< index of DO_LAND_START, INVALID_DO_LAND_START if no planned landing */
	double _landing_start_lat{0.0};
	double _landing_start_lon{0.0};
	float _landing_start_alt{0.0f};

	double _landing_lat{0.0};
	double _landing_lon{0.0};
	float _landing_alt{0.0f};

	bool _need_takeoff{true};					/**< if true, then takeoff must be performed before going to the first waypoint (if needed) */

	hrt_abstime _time_mission_deactivated{0};

	enum {
		MISSION_TYPE_NONE,
		MISSION_TYPE_MISSION
	} _mission_type{MISSION_TYPE_NONE};

	bool _inited{false};
	bool _home_inited{false};
	bool _need_mission_reset{false};
	bool _mission_waypoints_changed{false};
	bool _mission_changed{false}; /** < true if the mission changed since the mission mode was active */

	enum work_item_type {
		WORK_ITEM_TYPE_DEFAULT,		/**< default mission item */
		WORK_ITEM_TYPE_TAKEOFF,		/**< takeoff before moving to waypoint */
		WORK_ITEM_TYPE_MOVE_TO_LAND,	/**< move to land waypoint before descent */
		WORK_ITEM_TYPE_ALIGN,		/**< align for next waypoint */
		WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF,
		WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION,
		WORK_ITEM_TYPE_PRECISION_LAND
	} _work_item_type{WORK_ITEM_TYPE_DEFAULT};	/**< current type of work to do (sub mission item) */

	uint8_t _mission_execution_mode{mission_result_s::MISSION_EXECUTION_MODE_NORMAL};	/**< the current mode of how the mission is executed,look at mission_result.msg for the definition */
	bool _execution_mode_changed{false};
};
