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
 * @file navigator_mission.cpp
 *
 * Helper class to access missions
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Simon Wilks <simon@uaventure.com>
 * @author Andreas Antener <andreas@uaventure.com>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "mission.h"
#include "navigator.h"

#include <string.h>
#include <drivers/drv_hrt.h>
#include <dataman/dataman.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/ecl/geo/geo.h>
#include <navigator/navigation.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

Mission::Mission(Navigator *navigator, NavigatorCore &navigator_core) :
	MissionBlock(navigator, navigator_core),
	ModuleParams(navigator)
{
}

void
Mission::on_inactive()
{
	/* We need to reset the mission cruising speed, otherwise the
	 * mission velocity which might have been set using mission items
	 * is used for missions such as RTL. */
	_navigator->set_cruising_speed();

	// if we were executing an landing but have been inactive for 2 seconds, then make the landing invalid
	// this prevents RTL to just continue at the current mission index
	if (_navigator->getMissionLandingInProgress() && (hrt_absolute_time() - _time_mission_deactivated) > 2_s) {
		_navigator->setMissionLandingInProgress(false);
	}

	/* Without home a mission can't be valid yet anyway, let's wait. */
	if (!_navigator_core.isHomeValid()) {
		return;
	}

	if (_inited) {
		if (_mission_sub.updated()) {
			update_mission();

			if (_mission_type == MISSION_TYPE_NONE && _mission.count > 0) {
				_mission_type = MISSION_TYPE_MISSION;
			}
		}

		/* reset the current mission if needed */
		if (need_to_reset_mission()) {
			reset_mission(_mission);
			update_mission();
			_navigator->reset_cruising_speed();
		}

	} else {

		/* load missions from storage */
		mission_s mission_state = {};

		dm_lock(DM_KEY_MISSION_STATE);

		/* read current state */
		int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

		dm_unlock(DM_KEY_MISSION_STATE);

		if (read_res == sizeof(mission_s)) {
			_mission.dataman_id = mission_state.dataman_id;
			_mission.count = mission_state.count;
			_current_mission_index = mission_state.current_seq;

			// find and store landing start marker (if available)
			find_mission_land_start();
		}

		/* On init let's check the mission, maybe there is already one available. */
		check_mission_valid(false);

		_inited = true;
	}

	/* require takeoff after non-loiter or landing */
	if (!_navigator->get_can_loiter_at_sp() || _navigator_core.getLanded()) {
		_need_takeoff = true;
	}

	/* reset so current mission item gets restarted if mission was paused */
	_work_item_type = WORK_ITEM_TYPE_DEFAULT;

	/* reset so MISSION_ITEM_REACHED isn't published */
	_navigator->get_mission_result()->seq_reached = -1;
}

void
Mission::on_inactivation()
{
	// Disable camera trigger
	vehicle_command_s cmd {};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// Pause trigger
	cmd.param1 = -1.0f;
	cmd.param3 = 1.0f;
	_navigator->publish_vehicle_cmd(&cmd);

	_navigator->release_gimbal_control();

	if (_navigator->get_precland()->is_activated()) {
		_navigator->get_precland()->on_inactivation();
	}

	_time_mission_deactivated = hrt_absolute_time();

	/* reset so current mission item gets restarted if mission was paused */
	_work_item_type = WORK_ITEM_TYPE_DEFAULT;
}

void
Mission::on_activation()
{
	if (_mission_waypoints_changed) {
		// do not set the closest mission item in the normal mission mode
		if (_mission_execution_mode != mission_result_s::MISSION_EXECUTION_MODE_NORMAL) {
			_current_mission_index = index_closest_mission_item();
		}

		_mission_waypoints_changed = false;
	}

	// we already reset the mission items
	_execution_mode_changed = false;

	set_mission_items();

	// unpause triggering if it was paused
	vehicle_command_s cmd = {};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// unpause trigger
	cmd.param1 = -1.0f;
	cmd.param3 = 0.0f;
	_navigator->publish_vehicle_cmd(&cmd);
}

void
Mission::on_active()
{
	check_mission_valid(false);

	/* check if anything has changed */
	bool mission_sub_updated = _mission_sub.updated();

	if (mission_sub_updated) {
		_navigator->reset_triplets();
		update_mission();
	}

	/* mission is running (and we are armed), need reset after disarm */
	_need_mission_reset = true;

	_mission_changed = false;

	/* reset mission items if needed */
	if (mission_sub_updated || _mission_waypoints_changed || _execution_mode_changed) {
		if (_mission_waypoints_changed) {
			// do not set the closest mission item in the normal mission mode
			if (_mission_execution_mode != mission_result_s::MISSION_EXECUTION_MODE_NORMAL) {
				_current_mission_index = index_closest_mission_item();
			}

			_mission_waypoints_changed = false;
		}

		_execution_mode_changed = false;
		set_mission_items();
	}

	/* lets check if we reached the current mission item */
	if (_mission_type != MISSION_TYPE_NONE && is_mission_item_reached()) {
		/* If we just completed a takeoff which was inserted before the right waypoint,
		   there is no need to report that we reached it because we didn't. */
		if (_work_item_type != WORK_ITEM_TYPE_TAKEOFF) {
			set_mission_item_reached();
		}

		if (_mission_item.autocontinue) {
			/* switch to next waypoint if 'autocontinue' flag set */
			advance_mission();
			set_mission_items();
		}

	} else {
		/* if waypoint position reached allow loiter on the setpoint */
		if (_waypoint_position_reached && _mission_item.nav_cmd != NAV_CMD_IDLE) {
			_navigator->set_can_loiter_at_sp(true);
		}
	}

	/* check if a cruise speed change has been commanded */
	if (_mission_type != MISSION_TYPE_NONE) {
		cruising_speed_sp_update();
	}

	/* see if we need to update the current yaw heading */
	if (!_param_mis_mnt_yaw_ctl.get()
	    && (_navigator_core.isRotaryWing())
	    && (_navigator->get_vroi().mode != vehicle_roi_s::ROI_NONE)
	    && !(_mission_item.nav_cmd == NAV_CMD_TAKEOFF
		 || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
		 || _mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
		 || _mission_item.nav_cmd == NAV_CMD_LAND
		 || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND
		 || _work_item_type == WORK_ITEM_TYPE_ALIGN)) {
		// Mount control is disabled If the vehicle is in ROI-mode, the vehicle
		// needs to rotate such that ROI is in the field of view.
		// ROI only makes sense for multicopters.
		heading_sp_update();
	}

	// TODO: Add vtol heading update method if required.
	// Question: Why does vtol ever have to update heading?

	/* check if landing needs to be aborted */
	if ((_mission_item.nav_cmd == NAV_CMD_LAND)
	    && (_navigator->abort_landing())) {

		do_abort_landing();
	}

	if (_work_item_type == WORK_ITEM_TYPE_PRECISION_LAND) {
		_navigator->get_precland()->on_active();

	} else if (_navigator->get_precland()->is_activated()) {
		_navigator->get_precland()->on_inactivation();
	}
}

bool
Mission::set_current_mission_index(uint16_t index)
{
	if (_navigator->get_mission_result()->valid &&
	    (index != _current_mission_index) && (index < _mission.count)) {

		_current_mission_index = index;

		// a mission index is set manually which has the higher priority than the closest mission item
		// as it is set by the user
		_mission_waypoints_changed = false;

		// update mission items if already in active mission
		if (_navigator->is_planned_mission()) {
			// prevent following "previous - current" line
			_navigator->get_position_setpoint_triplet()->previous.valid = false;
			_navigator->get_position_setpoint_triplet()->current.valid = false;
			_navigator->get_position_setpoint_triplet()->next.valid = false;
			set_mission_items();
		}

		return true;
	}

	return false;
}

void
Mission::set_closest_item_as_current()
{
	_current_mission_index = index_closest_mission_item();
}

void
Mission::set_execution_mode(const uint8_t mode)
{
	if (_mission_execution_mode != mode) {
		_execution_mode_changed = true;
		_navigator->get_mission_result()->execution_mode = mode;


		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD:
			if (mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
				// command a transition if in vtol mc mode
				if (_navigator_core.isRotaryWing() &&
				    _navigator_core.isVTOL() &&
				    !_navigator_core.getLanded()) {

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

					position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
					pos_sp_triplet->previous = pos_sp_triplet->current;
					generate_waypoint_from_heading(&pos_sp_triplet->current, _mission_item.yaw);
					publish_navigator_mission_item(); // for logging
					_navigator->set_position_setpoint_triplet_updated();
					issue_command(_mission_item);
				}

				if (_mission_type == MISSION_TYPE_NONE && _mission.count > 0) {
					_mission_type = MISSION_TYPE_MISSION;
				}

				if (_current_mission_index > _mission.count - 1) {
					_current_mission_index = _mission.count - 1;

				} else if (_current_mission_index > 0) {
					--_current_mission_index;
				}

				_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			}

			break;

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE:
			if ((mode == mission_result_s::MISSION_EXECUTION_MODE_NORMAL) ||
			    (mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD)) {
				// handle switch from reverse to forward mission
				if (_current_mission_index < 0) {
					_current_mission_index = 0;

				} else if (_current_mission_index < _mission.count - 1) {
					++_current_mission_index;
				}

				_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			}

			break;

		}

		_mission_execution_mode = mode;
	}
}

bool
Mission::find_mission_land_start()
{
	/* return true if a MAV_CMD_DO_LAND_START, NAV_CMD_VTOL_LAND or NAV_CMD_LAND is found and internally save the index
	 *  return false if not found
	 */

	const dm_item_t dm_current = (dm_item_t)_mission.dataman_id;
	struct mission_item_s missionitem = {};
	struct mission_item_s missionitem_prev = {}; //to store mission item before currently checked on, needed to get pos of wp before NAV_CMD_DO_LAND_START

	_land_start_available = false;

	bool found_land_start_marker = false;

	for (size_t i = 1; i < _mission.count; i++) {
		const ssize_t len = sizeof(missionitem);
		missionitem_prev = missionitem; // store the last mission item before reading a new one

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			PX4_ERR("dataman read failure");
			break;
		}

		if (missionitem.nav_cmd == NAV_CMD_DO_LAND_START) {
			found_land_start_marker = true;
			_land_start_index = i;
		}

		if (found_land_start_marker && !_land_start_available && i > _land_start_index
		    && item_contains_position(missionitem)) {
			// use the position of any waypoint after the land start marker which specifies a position.
			_landing_start_lat = missionitem.lat;
			_landing_start_lon = missionitem.lon;
			_landing_start_alt = missionitem.altitude_is_relative ?	missionitem.altitude +
					     _navigator_core.getHomeAltAMSLMeter() : missionitem.altitude;
			_land_start_available = true;
		}

		if (((missionitem.nav_cmd == NAV_CMD_VTOL_LAND) && _navigator_core.isVTOL()) ||
		    (missionitem.nav_cmd == NAV_CMD_LAND)) {

			_landing_lat = missionitem.lat;
			_landing_lon = missionitem.lon;
			_landing_alt = missionitem.altitude_is_relative ?	missionitem.altitude + _navigator_core.getHomeAltAMSLMeter() :
				       missionitem.altitude;

			// don't have a valid land start yet, use the landing item itself then
			if (!_land_start_available) {
				_land_start_index = i;
				_landing_start_lat = _landing_lat;
				_landing_start_lon = _landing_lon;
				_landing_start_alt = _landing_alt;
				_land_start_available = true;
			}

		}
	}

	return _land_start_available;
}

bool
Mission::land_start()
{
	// if not currently landing, jump to do_land_start
	if (_land_start_available) {
		if (_navigator->getMissionLandingInProgress()) {
			return true;

		} else {
			set_current_mission_index(get_land_start_index());

			const bool can_land_now = landing();
			_navigator->setMissionLandingInProgress(can_land_now);
			return can_land_now;
		}
	}

	return false;
}

bool
Mission::landing()
{
	// vehicle is currently landing if
	//  mission valid, still flying, and in the landing portion of mission

	const bool mission_valid = _navigator->get_mission_result()->valid;
	const bool on_landing_stage = _land_start_available && (_current_mission_index >= get_land_start_index());

	return mission_valid && on_landing_stage;
}

void
Mission::update_mission()
{

	bool failed = true;

	/* Reset vehicle_roi
	 * Missions that do not explicitly configure ROI would not override
	 * an existing ROI setting from previous missions */
	_navigator->reset_vroi();

	const mission_s old_mission = _mission;

	if (_mission_sub.copy(&_mission)) {
		/* determine current index */
		if (_mission.current_seq >= 0 && _mission.current_seq < (int)_mission.count) {
			_current_mission_index = _mission.current_seq;

		} else {
			/* if less items available, reset to first item */
			if (_current_mission_index >= (int)_mission.count) {
				_current_mission_index = 0;

			} else if (_current_mission_index < 0) {
				/* if not initialized, set it to 0 */
				_current_mission_index = 0;
			}

			/* otherwise, just leave it */
		}

		check_mission_valid(true);

		failed = !_navigator->get_mission_result()->valid;

		if (!failed) {
			/* reset mission failure if we have an updated valid mission */
			_navigator->get_mission_result()->failure = false;

			/* reset sequence info as well */
			_navigator->get_mission_result()->seq_reached = -1;
			_navigator->get_mission_result()->seq_total = _mission.count;

			/* reset work item if new mission has been accepted */
			_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			_mission_changed = true;
		}

		/* check if the mission waypoints changed while the vehicle is in air
		 * TODO add a flag to mission_s which actually tracks if the position of the waypoint changed */
		if (((_mission.count != old_mission.count) ||
		     (_mission.dataman_id != old_mission.dataman_id)) &&
		    !_navigator_core.getLanded()) {
			_mission_waypoints_changed = true;
		}

	} else {
		PX4_ERR("mission update failed");
	}

	if (failed) {
		// only warn if the check failed on merit
		if ((int)_mission.count > 0) {
			PX4_WARN("mission check failed");
		}

		// reset the mission
		_mission.count = 0;
		_mission.current_seq = 0;
		_current_mission_index = 0;
	}

	// find and store landing start marker (if available)
	find_mission_land_start();

	set_current_mission_item();
}


void
Mission::advance_mission()
{
	/* do not advance mission item if we're processing sub mission work items */
	if (_work_item_type != WORK_ITEM_TYPE_DEFAULT) {
		return;
	}

	switch (_mission_type) {
	case MISSION_TYPE_MISSION:
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				_current_mission_index++;
				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: {
				// find next position item in reverse order
				dm_item_t dm_current = (dm_item_t)(_mission.dataman_id);

				for (int32_t i = _current_mission_index - 1; i >= 0; i--) {
					struct mission_item_s missionitem = {};
					const ssize_t len = sizeof(missionitem);

					if (dm_read(dm_current, i, &missionitem, len) != len) {
						/* not supposed to happen unless the datamanager can't access the SD card, etc. */
						PX4_ERR("dataman read failure");
						break;
					}

					if (item_contains_position(missionitem)) {
						_current_mission_index = i;
						return;
					}
				}

				// finished flying back the mission
				_current_mission_index = -1;
				break;
			}

		default:
			_current_mission_index++;
		}

		break;

	case MISSION_TYPE_NONE:
	default:
		break;
	}
}

void
Mission::set_mission_items()
{
	/* the home dist check provides user feedback, so we initialize it to this */
	bool user_feedback_done = false;

	/* mission item that comes after current if available */
	struct mission_item_s mission_item_next_position;
	struct mission_item_s mission_item_after_next_position;
	bool has_next_position_item = false;
	bool has_after_next_position_item = false;

	work_item_type new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

	if (prepare_mission_items(&_mission_item, &mission_item_next_position, &has_next_position_item,
				  &mission_item_after_next_position, &has_after_next_position_item)) {
		/* if mission type changed, notify */
		if (_mission_type != MISSION_TYPE_MISSION) {
			mavlink_log_info(_navigator->get_mavlink_log_pub(),
					 _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE ? "Executing Reverse Mission" :
					 "Executing Mission");
			user_feedback_done = true;
		}

		_mission_type = MISSION_TYPE_MISSION;

	} else {
		/* no mission available or mission finished, switch to loiter */
		if (_mission_type != MISSION_TYPE_NONE) {

			if (_navigator_core.getLanded()) {
				mavlink_log_info(_navigator->get_mavlink_log_pub(),
						 _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE ? "Reverse Mission finished, landed" :
						 "Mission finished, landed.");

			} else {
				/* https://en.wikipedia.org/wiki/Loiter_(aeronautics) */
				mavlink_log_info(_navigator->get_mavlink_log_pub(),
						 _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE ? "Reverse Mission finished, loitering" :
						 "Mission finished, loitering.");

				/* use last setpoint for loiter */
				_navigator->set_can_loiter_at_sp(true);
			}

			user_feedback_done = true;
		}

		_mission_type = MISSION_TYPE_NONE;

		/* set loiter mission item and ensure that there is a minimum clearance from home */
		set_loiter_item(&_mission_item, _navigator_core.getRelativeTakeoffMinAltitudeMeter());

		/* update position setpoint triplet  */
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->previous.valid = false;
		mission_apply_limitation(_mission_item);
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		pos_sp_triplet->next.valid = false;

		/* reuse setpoint for LOITER only if it's not IDLE */
		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

		// set mission finished
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();

		if (!user_feedback_done) {
			/* only tell users that we got no mission if there has not been any
			 * better, more specific feedback yet
			 * https://en.wikipedia.org/wiki/Loiter_(aeronautics)
			 */

			if (_navigator_core.getLanded()) {
				/* landed, refusing to take off without a mission */
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "No valid mission available, refusing takeoff.");

			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "No valid mission available, loitering.");
			}

			user_feedback_done = true;
		}

		publish_navigator_mission_item(); // for logging
		_navigator->set_position_setpoint_triplet_updated();

		return;
	}

	/*********************************** handle mission item *********************************************/

	/* handle mission items depending on the mode */

	const position_setpoint_s current_setpoint_copy = _navigator->get_position_setpoint_triplet()->current;

	if (item_contains_position(_mission_item)) {
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				/* force vtol land */
				if (_navigator_core.forceVTOL() && _mission_item.nav_cmd == NAV_CMD_LAND) {
					_mission_item.nav_cmd = NAV_CMD_VTOL_LAND;
				}

				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				/* do takeoff before going to setpoint if needed and not already in takeoff */
				/* in fixed-wing this whole block will be ignored and a takeoff item is always propagated */
				if (do_need_vertical_takeoff() &&
				    _work_item_type == WORK_ITEM_TYPE_DEFAULT &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					mission_item_next_position.nav_cmd = NAV_CMD_WAYPOINT;
					has_next_position_item = true;

					float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

					mavlink_log_info(_navigator->get_mavlink_log_pub(), "Takeoff to %.1f meters above home.",
							 (double)(takeoff_alt - _navigator_core.getHomeAltAMSLMeter()));

					_mission_item.nav_cmd = NAV_CMD_TAKEOFF;
					_mission_item.lat = _navigator_core.getLatRad();
					_mission_item.lon = _navigator_core.getLonRad();
					/* hold heading for takeoff items */
					_mission_item.yaw = _navigator_core.getTrueHeadingRad();
					_mission_item.altitude = takeoff_alt;
					_mission_item.altitude_is_relative = false;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
					   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && _navigator_core.isRotaryWing()) {

					/* if there is no need to do a takeoff but we have a takeoff item, treat is as waypoint */
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;

				} else if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
					   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					if (_navigator_core.isRotaryWing()) {
						/* haven't transitioned yet, trigger vtol takeoff logic below */
						_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

					} else {
						/* already in fixed-wing, go to waypoint */
						_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					}

					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
				}

				/* if we just did a normal takeoff navigate to the actual waypoint now */
				if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
				}

				/* if we just did a VTOL takeoff, prepare transition */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT &&
				    _navigator_core.isRotaryWing() &&
				    !_navigator_core.getLanded()) {

					/* disable weathervane before front transition for allowing yaw to align */
					pos_sp_triplet->current.disable_weather_vane = true;

					/* set yaw setpoint to heading of VTOL_TAKEOFF wp against current position */
					_mission_item.yaw = get_bearing_to_next_waypoint(
								    _navigator_core.getLatRad(), _navigator_core.getLonRad(),
								    _mission_item.lat, _mission_item.lon);

					_mission_item.force_heading = true;

					new_work_item_type = WORK_ITEM_TYPE_ALIGN;

					/* set position setpoint to current while aligning */
					_mission_item.lat = _navigator_core.getLatRad();
					_mission_item.lon = _navigator_core.getLonRad();
				}

				/* heading is aligned now, prepare transition */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_ALIGN &&
				    _navigator_core.isRotaryWing() &&
				    !_navigator_core.getLanded()) {

					/* re-enable weather vane again after alignment */
					pos_sp_triplet->current.disable_weather_vane = false;

					/* check if the vtol_takeoff waypoint is on top of us */
					if (do_need_move_to_takeoff()) {
						new_work_item_type = WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF;
					}

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
					_mission_item.yaw = _navigator_core.getTrueHeadingRad();

					/* set position setpoint to target during the transition */
					generate_waypoint_from_heading(&pos_sp_triplet->current, _mission_item.yaw);
				}

				/* takeoff completed and transitioned, move to takeoff wp as fixed wing */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
				    && _work_item_type == WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				/* move to land wp as fixed wing */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
				    && (_work_item_type == WORK_ITEM_TYPE_DEFAULT || _work_item_type == WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF)
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && !_navigator_core.getLanded()) {

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					has_next_position_item = true;

					float altitude = _navigator_core.getAltitudeAMSLMeters();

					if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
						altitude = pos_sp_triplet->current.alt;
					}

					_mission_item.altitude = altitude;
					_mission_item.altitude_is_relative = false;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
					_mission_item.vtol_back_transition = true;
				}

				/* transition to MC */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
				    && _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && _navigator_core.isFixedWing()
				    && !_navigator_core.getLanded()) {

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
					_mission_item.altitude = _navigator_core.getAltitudeAMSLMeters();
					_mission_item.altitude_is_relative = false;

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION;

					// make previous setpoint invalid, such that there will be no prev-current line following
					// if the vehicle drifted off the path during back-transition it should just go straight to the landing point
					pos_sp_triplet->previous.valid = false;
				}

				/* move to landing waypoint before descent if necessary */
				if (do_need_move_to_land() &&
				    (_work_item_type == WORK_ITEM_TYPE_DEFAULT ||
				     _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION) &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					has_next_position_item = true;

					/*
					 * Ignoring waypoint altitude:
					 * Set altitude to the same as we have now to prevent descending too fast into
					 * the ground. Actual landing will descend anyway until it touches down.
					 * XXX: We might want to change that at some point if it is clear to the user
					 * what the altitude means on this waypoint type.
					 */
					float altitude = _navigator_core.getAltitudeAMSLMeters();

					if (pos_sp_triplet->current.valid
					    && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
						altitude = pos_sp_triplet->current.alt;
					}

					_mission_item.altitude = altitude;
					_mission_item.altitude_is_relative = false;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

					// make previous setpoint invalid, such that there will be no prev-current line following.
					// if the vehicle drifted off the path during back-transition it should just go straight to the landing point
					pos_sp_triplet->previous.valid = false;

				} else if (_mission_item.nav_cmd == NAV_CMD_LAND && _work_item_type == WORK_ITEM_TYPE_DEFAULT) {
					if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
						new_work_item_type = WORK_ITEM_TYPE_PRECISION_LAND;

						if (_mission_item.land_precision == 1) {
							_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

						} else { //_mission_item.land_precision == 2
							_navigator->get_precland()->set_mode(PrecLandMode::Required);
						}

						_navigator->get_precland()->on_activation();

					}
				}

				/* we just moved to the landing waypoint, now descend */
				if (_work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
						new_work_item_type = WORK_ITEM_TYPE_PRECISION_LAND;

						if (_mission_item.land_precision == 1) {
							_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

						} else { //_mission_item.land_precision == 2
							_navigator->get_precland()->set_mode(PrecLandMode::Required);
						}

						_navigator->get_precland()->on_activation();

					}

				}

				/* ignore yaw for landing items */
				/* XXX: if specified heading for landing is desired we could add another step before the descent
				 * that aligns the vehicle first */
				if (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND) {
					_mission_item.yaw = NAN;
				}


				// for fast forward convert certain types to simple waypoint
				// XXX: add other types which should be ignored in fast forward
				if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD &&
				    ((_mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED) ||
				     (_mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT))) {
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: {
				if (item_contains_position(_mission_item)) {
					// convert mission item to a simple waypoint
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else {
					mavlink_log_critical(_navigator->get_mavlink_log_pub(), "MissionReverse: Got a non-position mission item, ignoring it");
				}

				break;
			}
		}

	} else {
		/* handle non-position mission items such as commands */
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				/* turn towards next waypoint before MC to FW transition */
				if (_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
				    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && _navigator_core.isRotaryWing()
				    && !_navigator_core.getLanded()
				    && has_next_position_item) {

					/* disable weathervane before front transition for allowing yaw to align */
					pos_sp_triplet->current.disable_weather_vane = true;

					new_work_item_type = WORK_ITEM_TYPE_ALIGN;

					set_align_mission_item(&_mission_item, &mission_item_next_position);

					/* set position setpoint to target during the transition */
					mission_apply_limitation(_mission_item);
					mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->current);
				}

				/* yaw is aligned now */
				if (_work_item_type == WORK_ITEM_TYPE_ALIGN &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

					/* re-enable weather vane again after alignment */
					pos_sp_triplet->current.disable_weather_vane = false;

					/* set position setpoint to target during the transition */
					pos_sp_triplet->previous = pos_sp_triplet->current;
					generate_waypoint_from_heading(&pos_sp_triplet->current, pos_sp_triplet->current.yaw);
				}

				// ignore certain commands in mission fast forward
				if ((_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD) &&
				    (_mission_item.nav_cmd == NAV_CMD_DELAY)) {
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: {
				// nothing to do, all commands are ignored
				break;
			}
		}

		if (_mission_item.nav_cmd == NAV_CMD_CONDITION_GATE) {
			_mission_item.autocontinue = true;
			_mission_item.time_inside = 0;
		}
	}

	/*********************************** set setpoints and check next *********************************************/

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// The logic in this section establishes the tracking between the current waypoint
	// which we are approaching and the next waypoint, which will tell us in which direction
	// we will change our trajectory right after reaching it.

	// Because actions, gates and jump labels can be interleaved with waypoints,
	// we are searching around the current mission item in the list to find the closest
	// gate and the closest waypoint. We then store them separately.

	// Check if the mission item is a gate
	// along the current trajectory
	if (item_contains_gate(_mission_item)) {

		// The mission item is a gate, let's check if the next item in the list provides
		// a position to go towards.

		// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
		if (has_next_position_item) {
			// We have a position, convert it to the setpoint and update setpoint triplet
			mission_apply_limitation(mission_item_next_position);
			mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->current);
		}

		// ELSE: The current position setpoint stays unchanged.

	} else {
		// The mission item is not a gate, set the current position setpoint from mission item (is protected against non-position items)
		// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
		if (new_work_item_type != WORK_ITEM_TYPE_PRECISION_LAND) {
			mission_apply_limitation(_mission_item);
			mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		}

		// ELSE: The current position setpoint stays unchanged.
	}

	// Only set the previous position item if the current one really changed
	// TODO Precision land needs to be refactored: https://github.com/PX4/Firmware/issues/14320
	if ((_work_item_type != WORK_ITEM_TYPE_MOVE_TO_LAND) &&
	    !position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
		pos_sp_triplet->previous = current_setpoint_copy;
	}

	/* issue command if ready (will do nothing for position mission items) */
	issue_command(_mission_item);

	/* set current work item type */
	_work_item_type = new_work_item_type;

	/* require takeoff after landing or idle */
	if (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_need_takeoff = true;
	}

	_navigator->set_can_loiter_at_sp(false);
	reset_mission_item_reached();

	if (_mission_type == MISSION_TYPE_MISSION) {
		set_current_mission_item();
	}

	// If the mission item under evaluation contains a gate,
	// then we need to check if we have a next position item so
	// the controller can fly the correct line between the
	// current and next setpoint
	if (item_contains_gate(_mission_item)) {
		if (has_after_next_position_item) {
			/* got next mission item, update setpoint triplet */
			mission_apply_limitation(mission_item_next_position);
			mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->next);

		} else {
			pos_sp_triplet->next.valid = false;
		}

	} else {
		const bool brake_for_hold = _navigator_core.isRotaryWing()
					    && get_time_inside(_mission_item) > FLT_EPSILON; //allow the vehicle to decelerate before reaching a wp with a hold time

		if (_mission_item.autocontinue && !brake_for_hold) {
			/* try to process next mission item */
			if (has_next_position_item) {
				/* got next mission item, update setpoint triplet */
				mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->next);

			} else {
				/* next mission item is not available */
				pos_sp_triplet->next.valid = false;
			}

		} else {
			/* vehicle will be paused on current waypoint, don't set next item */
			pos_sp_triplet->next.valid = false;
		}
	}

	publish_navigator_mission_item(); // for logging
	_navigator->set_position_setpoint_triplet_updated();
}

bool
Mission::do_need_vertical_takeoff()
{
	if (_navigator_core.isRotaryWing()) {

		float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

		if (_navigator_core.getLanded()) {
			/* force takeoff if landed (additional protection) */
			_need_takeoff = true;

		} else if (_navigator_core.getAltitudeAMSLMeters() > takeoff_alt - _navigator_core.getAltAcceptanceRadiusMeter()) {
			/* if in-air and already above takeoff height, don't do takeoff */
			_need_takeoff = false;

		} else if (_navigator_core.getAltitudeAMSLMeters() <= takeoff_alt - _navigator_core.getAltAcceptanceRadiusMeter()
			   && (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
			       || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)) {
			/* if in-air but below takeoff height and we have a takeoff item */
			_need_takeoff = true;
		}

		/* check if current mission item is one that requires takeoff before */
		if (_need_takeoff && (
			    _mission_item.nav_cmd == NAV_CMD_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_WAYPOINT ||
			    _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED)) {

			_need_takeoff = false;
			return true;
		}
	}

	return false;
}

bool
Mission::do_need_move_to_land()
{
	if (_navigator_core.isRotaryWing()
	    && (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND)) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _navigator_core.getLatRad(), _navigator_core.getLonRad());

		return d_current > _navigator_core.getAcceptanceRadiusMeter();
	}

	return false;
}

bool
Mission::do_need_move_to_takeoff()
{
	if (_navigator_core.isRotaryWing()
	    && _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _navigator_core.getLatRad(), _navigator_core.getLonRad());

		return d_current > _navigator_core.getAcceptanceRadiusMeter();
	}

	return false;
}

void
Mission::copy_position_if_valid(struct mission_item_s *mission_item, struct position_setpoint_s *setpoint)
{
	if (setpoint->valid && setpoint->type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
		mission_item->lat = setpoint->lat;
		mission_item->lon = setpoint->lon;
		mission_item->altitude = setpoint->alt;

	} else {
		mission_item->lat = _navigator_core.getLatRad();
		mission_item->lon = _navigator_core.getLonRad();
		mission_item->altitude = _navigator_core.getAltitudeAMSLMeters();
	}

	mission_item->altitude_is_relative = false;
}

void
Mission::set_align_mission_item(struct mission_item_s *mission_item, struct mission_item_s *mission_item_next)
{
	mission_item->nav_cmd = NAV_CMD_WAYPOINT;
	copy_position_if_valid(mission_item, &(_navigator->get_position_setpoint_triplet()->current));
	mission_item->altitude_is_relative = false;
	mission_item->autocontinue = true;
	mission_item->time_inside = 0.0f;
	mission_item->yaw = get_bearing_to_next_waypoint(
				    _navigator_core.getLatRad(), _navigator_core.getLonRad(),
				    mission_item_next->lat, mission_item_next->lon);
	mission_item->force_heading = true;
}

float
Mission::calculate_takeoff_altitude(struct mission_item_s *mission_item)
{
	/* calculate takeoff altitude */
	float takeoff_alt = get_absolute_altitude_for_item(*mission_item);

	/* takeoff to at least MIS_TAKEOFF_ALT above home/ground, even if first waypoint is lower */
	if (_navigator_core.getLanded()) {
		takeoff_alt = fmaxf(takeoff_alt, _navigator_core.getAltitudeAMSLMeters() +
				    _navigator_core.getRelativeTakeoffMinAltitudeMeter());

	} else {
		takeoff_alt = fmaxf(takeoff_alt, _navigator_core.getHomeAltAMSLMeter() +
				    _navigator_core.getRelativeTakeoffMinAltitudeMeter());
	}

	return takeoff_alt;
}

void
Mission::heading_sp_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet =
		_navigator->get_position_setpoint_triplet();

	// Only update if current triplet is valid
	if (pos_sp_triplet->current.valid) {

		double point_from_latlon[2] = { _navigator_core.getLatRad(),
						_navigator_core.getLonRad()
					      };
		double point_to_latlon[2] = { _navigator_core.getLatRad(),
					      _navigator_core.getLonRad()
					    };
		float yaw_offset = 0.0f;

		// Depending on ROI-mode, update heading
		switch (_navigator->get_vroi().mode) {
		case vehicle_roi_s::ROI_LOCATION: {
				// ROI is a fixed location. Vehicle needs to point towards that location
				point_to_latlon[0] = _navigator->get_vroi().lat;
				point_to_latlon[1] = _navigator->get_vroi().lon;
				// No yaw offset required
				yaw_offset = 0.0f;
				break;
			}

		case vehicle_roi_s::ROI_WPNEXT: {
				// ROI is current waypoint. Vehcile needs to point towards current waypoint
				point_to_latlon[0] = pos_sp_triplet->current.lat;
				point_to_latlon[1] = pos_sp_triplet->current.lon;
				// Add the gimbal's yaw offset
				yaw_offset = _navigator->get_vroi().yaw_offset;
				break;
			}

		case vehicle_roi_s::ROI_NONE:
		case vehicle_roi_s::ROI_WPINDEX:
		case vehicle_roi_s::ROI_TARGET:
		case vehicle_roi_s::ROI_ENUM_END:
		default: {
				return;
			}
		}

		// Get desired heading and update it.
		// However, only update if distance to desired heading is
		// larger than acceptance radius to prevent excessive yawing
		float d_current = get_distance_to_next_waypoint(point_from_latlon[0],
				  point_from_latlon[1], point_to_latlon[0], point_to_latlon[1]);

		if (d_current > _navigator_core.getAcceptanceRadiusMeter()) {
			float yaw = matrix::wrap_pi(
					    get_bearing_to_next_waypoint(point_from_latlon[0],
							    point_from_latlon[1], point_to_latlon[0],
							    point_to_latlon[1]) + yaw_offset);

			_mission_item.yaw = yaw;
			pos_sp_triplet->current.yaw = _mission_item.yaw;
			pos_sp_triplet->current.yaw_valid = true;

		} else {
			if (!pos_sp_triplet->current.yaw_valid) {
				_mission_item.yaw = _navigator_core.getTrueHeadingRad();
				pos_sp_triplet->current.yaw = _mission_item.yaw;
				pos_sp_triplet->current.yaw_valid = true;
			}
		}

		// we set yaw directly so we can run this in parallel to the FOH update
		publish_navigator_mission_item();
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
Mission::cruising_speed_sp_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float cruising_speed = _navigator->get_cruising_speed();

	/* Don't change setpoint if the current waypoint is not valid */
	if (!pos_sp_triplet->current.valid ||
	    fabsf(pos_sp_triplet->current.cruising_speed - cruising_speed) < FLT_EPSILON) {
		return;
	}

	pos_sp_triplet->current.cruising_speed = cruising_speed;

	publish_navigator_mission_item();
	_navigator->set_position_setpoint_triplet_updated();
}

void
Mission::do_abort_landing()
{
	// Abort FW landing
	//  first climb out then loiter over intended landing location

	if (_mission_item.nav_cmd != NAV_CMD_LAND) {
		return;
	}

	// loiter at the larger of MIS_LTRMIN_ALT above the landing point
	//  or 2 * FW_CLMBOUT_DIFF above the current altitude
	const float alt_landing = get_absolute_altitude_for_item(_mission_item);
	const float alt_sp = math::max(alt_landing + _navigator_core.getRelativeLoiterMinAltitudeMeter(),
				       _navigator_core.getAltitudeAMSLMeters() + 20.0f);

	// turn current landing waypoint into an indefinite loiter
	_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	_mission_item.altitude_is_relative = false;
	_mission_item.altitude = alt_sp;
	_mission_item.loiter_radius = _navigator_core.getLoiterRadiusMeter();
	_mission_item.acceptance_radius = _navigator_core.getAcceptanceRadiusMeter();
	_mission_item.autocontinue = false;
	_mission_item.origin = ORIGIN_ONBOARD;

	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &_navigator->get_position_setpoint_triplet()->current);
	publish_navigator_mission_item(); // for logging
	_navigator->set_position_setpoint_triplet_updated();

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "Holding at %d m above landing.",
			 (int)(alt_sp - alt_landing));

	// reset mission index to start of landing
	if (_land_start_available) {
		_current_mission_index = get_land_start_index();

	} else {
		// move mission index back (landing approach point)
		_current_mission_index -= 1;
	}

	// send reposition cmd to get out of mission
	vehicle_command_s vcmd = {};

	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
	vcmd.param1 = -1;
	vcmd.param2 = 1;
	vcmd.param5 = _mission_item.lat;
	vcmd.param6 = _mission_item.lon;
	vcmd.param7 = alt_sp;

	_navigator->publish_vehicle_cmd(&vcmd);
}

bool
Mission::prepare_mission_items(struct mission_item_s *mission_item,
			       struct mission_item_s *next_position_mission_item, bool *has_next_position_item,
			       struct mission_item_s *after_next_position_mission_item, bool *has_after_next_position_item)
{
	*has_next_position_item = false;
	bool first_res = false;
	int offset = 1;

	if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
		offset = -1;
	}

	if (read_mission_item(0, mission_item)) {

		first_res = true;

		/* trying to find next position mission item */
		while (read_mission_item(offset, next_position_mission_item)) {
			if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
				offset--;

			} else {
				offset++;
			}

			if (item_contains_position(*next_position_mission_item)) {
				*has_next_position_item = true;
				break;
			}
		}

		if (_mission_execution_mode != mission_result_s::MISSION_EXECUTION_MODE_REVERSE &&
		    after_next_position_mission_item && has_after_next_position_item) {
			/* trying to find next next position mission item */
			while (read_mission_item(offset, after_next_position_mission_item)) {
				offset++;

				if (item_contains_position(*after_next_position_mission_item)) {
					*has_after_next_position_item = true;
					break;
				}
			}
		}
	}

	return first_res;
}

bool
Mission::read_mission_item(int offset, struct mission_item_s *mission_item)
{
	/* select mission */
	const int current_index = _current_mission_index;
	int index_to_read = current_index + offset;

	int *mission_index_ptr = (offset == 0) ? &_current_mission_index : &index_to_read;
	const dm_item_t dm_item = (dm_item_t)_mission.dataman_id;

	/* do not work on empty missions */
	if (_mission.count == 0) {
		return false;
	}

	/* Repeat this several times in case there are several DO JUMPS that we need to follow along, however, after
	 * 10 iterations we have to assume that the DO JUMPS are probably cycling and give up. */
	for (int i = 0; i < 10; i++) {
		if (*mission_index_ptr < 0 || *mission_index_ptr >= (int)_mission.count) {
			/* mission item index out of bounds - if they are equal, we just reached the end */
			if ((*mission_index_ptr != (int)_mission.count) && (*mission_index_ptr != -1)) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission item index out of bound, index: %d, max: %d.",
						     *mission_index_ptr, _mission.count);
			}

			return false;
		}

		const ssize_t len = sizeof(struct mission_item_s);

		/* read mission item to temp storage first to not overwrite current mission item if data damaged */
		struct mission_item_s mission_item_tmp;

		/* read mission item from datamanager */
		if (dm_read(dm_item, *mission_index_ptr, &mission_item_tmp, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Waypoint could not be read.");
			return false;
		}

		/* check for DO_JUMP item, and whether it hasn't not already been repeated enough times */
		if (mission_item_tmp.nav_cmd == NAV_CMD_DO_JUMP) {
			const bool execute_jumps = _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_NORMAL;

			/* do DO_JUMP as many times as requested if not in reverse mode */
			if ((mission_item_tmp.do_jump_current_count < mission_item_tmp.do_jump_repeat_count) && execute_jumps) {

				/* only raise the repeat count if this is for the current mission item
				 * but not for the read ahead mission item */
				if (offset == 0) {
					(mission_item_tmp.do_jump_current_count)++;

					/* save repeat count */
					if (dm_write(dm_item, *mission_index_ptr, DM_PERSIST_POWER_ON_RESET, &mission_item_tmp, len) != len) {
						/* not supposed to happen unless the datamanager can't access the dataman */
						mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP waypoint could not be written.");
						return false;
					}

					report_do_jump_mission_changed(*mission_index_ptr, mission_item_tmp.do_jump_repeat_count);
				}

				/* set new mission item index and repeat
				 * we don't have to validate here, if it's invalid, we should realize this later .*/
				*mission_index_ptr = mission_item_tmp.do_jump_mission_index;

			} else {
				if (offset == 0 && execute_jumps) {
					mavlink_log_info(_navigator->get_mavlink_log_pub(), "DO JUMP repetitions completed.");
				}

				/* no more DO_JUMPS, therefore just try to continue with next mission item */
				if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
					(*mission_index_ptr)--;

				} else {
					(*mission_index_ptr)++;
				}
			}

		} else {
			/* if it's not a DO_JUMP, then we were successful */
			memcpy(mission_item, &mission_item_tmp, sizeof(struct mission_item_s));
			return true;
		}
	}

	/* we have given up, we don't want to cycle forever */
	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP is cycling, giving up.");
	return false;
}

void
Mission::save_mission_state()
{
	mission_s mission_state = {};

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM_KEY_MISSION_STATE lock failed");
	}

	/* read current state */
	int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

	if (read_res == sizeof(mission_s)) {
		/* data read successfully, check dataman ID and items count */
		if (mission_state.dataman_id == _mission.dataman_id && mission_state.count == _mission.count) {
			/* navigator may modify only sequence, write modified state only if it changed */
			if (mission_state.current_seq != _current_mission_index) {
				mission_state.current_seq = _current_mission_index;
				mission_state.timestamp = hrt_absolute_time();

				if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state,
					     sizeof(mission_s)) != sizeof(mission_s)) {

					PX4_ERR("Can't save mission state");
				}
			}
		}

	} else {
		/* invalid data, this must not happen and indicates error in mission publisher */
		mission_state.timestamp = hrt_absolute_time();
		mission_state.dataman_id = _mission.dataman_id;
		mission_state.count = _mission.count;
		mission_state.current_seq = _current_mission_index;

		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Invalid mission state.");

		/* write modified state only if changed */
		if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state,
			     sizeof(mission_s)) != sizeof(mission_s)) {

			PX4_ERR("Can't save mission state");
		}
	}

	/* unlock MISSION_STATE item */
	if (dm_lock_ret == 0) {
		dm_unlock(DM_KEY_MISSION_STATE);
	}
}

void
Mission::report_do_jump_mission_changed(int index, int do_jumps_remaining)
{
	/* inform about the change */
	_navigator->get_mission_result()->item_do_jump_changed = true;
	_navigator->get_mission_result()->item_changed_index = index;
	_navigator->get_mission_result()->item_do_jump_remaining = do_jumps_remaining;

	_navigator->set_mission_result_updated();
}

void
Mission::set_mission_item_reached()
{
	_navigator->get_mission_result()->seq_reached = _current_mission_index;
	_navigator->set_mission_result_updated();

	// let the navigator know that we are currently executing the mission landing.
	// Using the method landing() itself is not accurate as it only give information about the mission index
	// but the vehicle could still be very far from the actual landing items
	_navigator->setMissionLandingInProgress(landing());

	reset_mission_item_reached();
}

void
Mission::set_current_mission_item()
{
	_navigator->get_mission_result()->finished = false;
	_navigator->get_mission_result()->seq_current = _current_mission_index;

	_navigator->set_mission_result_updated();

	save_mission_state();
}

void
Mission::check_mission_valid(bool force)
{
	if ((!_home_inited && _navigator_core.isHomeValid()) || force) {

		MissionFeasibilityChecker _missionFeasibilityChecker(_navigator);

		_navigator->get_mission_result()->valid =
			_missionFeasibilityChecker.checkMissionFeasible(_mission,
					_param_mis_dist_1wp.get(),
					_param_mis_dist_wps.get(),
					_navigator->mission_landing_required());

		_navigator->get_mission_result()->seq_total = _mission.count;
		_navigator->increment_mission_instance_count();
		_navigator->set_mission_result_updated();
		_home_inited = _navigator_core.isHomeValid();

		// find and store landing start marker (if available)
		find_mission_land_start();
	}
}

void
Mission::reset_mission(struct mission_s &mission)
{
	dm_lock(DM_KEY_MISSION_STATE);

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
		if (mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) {
			/* set current item to 0 */
			mission.current_seq = 0;

			/* reset jump counters */
			if (mission.count > 0) {
				const dm_item_t dm_current = (dm_item_t)mission.dataman_id;

				for (unsigned index = 0; index < mission.count; index++) {
					struct mission_item_s item;
					const ssize_t len = sizeof(struct mission_item_s);

					if (dm_read(dm_current, index, &item, len) != len) {
						PX4_WARN("could not read mission item during reset");
						break;
					}

					if (item.nav_cmd == NAV_CMD_DO_JUMP) {
						item.do_jump_current_count = 0;

						if (dm_write(dm_current, index, DM_PERSIST_POWER_ON_RESET, &item, len) != len) {
							PX4_WARN("could not save mission item during reset");
							break;
						}
					}
				}
			}

		} else {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Could not read mission.");

			/* initialize mission state in dataman */
			mission.timestamp = hrt_absolute_time();
			mission.dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
			mission.count = 0;
			mission.current_seq = 0;
		}

		dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));
	}

	dm_unlock(DM_KEY_MISSION_STATE);
}

bool
Mission::need_to_reset_mission()
{
	/* reset mission state when disarmed */
	if (_navigator_core.isNotArmed() && _need_mission_reset) {
		_need_mission_reset = false;
		return true;
	}

	return false;
}

void
Mission::generate_waypoint_from_heading(struct position_setpoint_s *setpoint, float yaw)
{
	waypoint_from_heading_and_distance(
		_navigator_core.getLatRad(), _navigator_core.getLonRad(),
		yaw, 1000000.0f,
		&(setpoint->lat), &(setpoint->lon));
	setpoint->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	setpoint->yaw = yaw;
}

int32_t
Mission::index_closest_mission_item() const
{
	int32_t min_dist_index(0);
	float min_dist(FLT_MAX), dist_xy(FLT_MAX), dist_z(FLT_MAX);

	dm_item_t dm_current = (dm_item_t)(_mission.dataman_id);

	for (size_t i = 0; i < _mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(missionitem);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			PX4_ERR("dataman read failure");
			break;
		}

		if (item_contains_position(missionitem)) {
			// do not consider land waypoints for a fw
			if (!((missionitem.nav_cmd == NAV_CMD_LAND) &&
			      (_navigator_core.isFixedWing()) &&
			      (!_navigator_core.isVTOL()))) {
				float dist = get_distance_to_point_global_wgs84(missionitem.lat, missionitem.lon,
						get_absolute_altitude_for_item(missionitem),
						_navigator_core.getLatRad(),
						_navigator_core.getLonRad(),
						_navigator_core.getAltitudeAMSLMeters(),
						&dist_xy, &dist_z);

				if (dist < min_dist) {
					min_dist = dist;
					min_dist_index = i;
				}
			}
		}
	}

	// for mission reverse also consider the home position
	if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
		float dist = get_distance_to_point_global_wgs84(
				     _navigator_core.getHomeLatRad(),
				     _navigator_core.getHomeLonRad(),
				     _navigator_core.getHomeAltAMSLMeter(),
				     _navigator_core.getLatRad(),
				     _navigator_core.getLonRad(),
				     _navigator_core.getAltitudeAMSLMeters(),
				     &dist_xy, &dist_z);

		if (dist < min_dist) {
			min_dist = dist;
			min_dist_index = -1;
		}
	}

	return min_dist_index;
}

bool Mission::position_setpoint_equal(const position_setpoint_s *p1, const position_setpoint_s *p2) const
{
	return ((p1->valid == p2->valid) &&
		(p1->type == p2->type) &&
		(fabsf(p1->vx - p2->vx) < FLT_EPSILON) &&
		(fabsf(p1->vy - p2->vy) < FLT_EPSILON) &&
		(fabsf(p1->vz - p2->vz) < FLT_EPSILON) &&
		(p1->velocity_valid == p2->velocity_valid) &&
		(p1->velocity_frame == p2->velocity_frame) &&
		(p1->alt_valid == p2->alt_valid) &&
		(fabs(p1->lat - p2->lat) < DBL_EPSILON) &&
		(fabs(p1->lon - p2->lon) < DBL_EPSILON) &&
		(fabsf(p1->alt - p2->alt) < FLT_EPSILON) &&
		((fabsf(p1->yaw - p2->yaw) < FLT_EPSILON) || (!PX4_ISFINITE(p1->yaw) && !PX4_ISFINITE(p2->yaw))) &&
		(p1->yaw_valid == p2->yaw_valid) &&
		(fabsf(p1->yawspeed - p2->yawspeed) < FLT_EPSILON) &&
		(p1->yawspeed_valid == p2->yawspeed_valid) &&
		(fabsf(p1->loiter_radius - p2->loiter_radius) < FLT_EPSILON) &&
		(p1->loiter_direction == p2->loiter_direction) &&
		(fabsf(p1->acceptance_radius - p2->acceptance_radius) < FLT_EPSILON) &&
		(fabsf(p1->cruising_speed - p2->cruising_speed) < FLT_EPSILON) &&
		((fabsf(p1->cruising_throttle - p2->cruising_throttle) < FLT_EPSILON) || (!PX4_ISFINITE(p1->cruising_throttle)
				&& !PX4_ISFINITE(p2->cruising_throttle))));

}

void Mission::publish_navigator_mission_item()
{
	navigator_mission_item_s navigator_mission_item{};

	navigator_mission_item.instance_count = _navigator->mission_instance_count();
	navigator_mission_item.sequence_current = _current_mission_index;
	navigator_mission_item.nav_cmd = _mission_item.nav_cmd;
	navigator_mission_item.latitude = _mission_item.lat;
	navigator_mission_item.longitude = _mission_item.lon;
	navigator_mission_item.altitude = _mission_item.altitude;

	navigator_mission_item.time_inside = get_time_inside(_mission_item);
	navigator_mission_item.acceptance_radius = _mission_item.acceptance_radius;
	navigator_mission_item.loiter_radius = _mission_item.loiter_radius;
	navigator_mission_item.yaw = _mission_item.yaw;

	navigator_mission_item.frame = _mission_item.frame;
	navigator_mission_item.frame = _mission_item.origin;

	navigator_mission_item.loiter_exit_xtrack = _mission_item.loiter_exit_xtrack;
	navigator_mission_item.force_heading = _mission_item.force_heading;
	navigator_mission_item.altitude_is_relative = _mission_item.altitude_is_relative;
	navigator_mission_item.autocontinue = _mission_item.autocontinue;
	navigator_mission_item.vtol_back_transition = _mission_item.vtol_back_transition;

	navigator_mission_item.timestamp = hrt_absolute_time();

	_navigator_mission_item_pub.publish(navigator_mission_item);
}
