/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file rtl.cpp
 *
 * Helper class to access RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Kent <julian@auterion.com>
 */

#include "rtl.h"
#include "navigator.h"
#include <dataman/dataman.h>

#include <lib/ecl/geo/geo.h>


static constexpr float DELAY_SIGMA = 0.01f;

using namespace time_literals;
using namespace math;

RTL::RTL(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void RTL::on_inactivation()
{
	if (_navigator->get_precland()->is_activated()) {
		_navigator->get_precland()->on_inactivation();
	}
}

void RTL::on_inactive()
{
	// Reset RTL state.
	_rtl_state = RTL_STATE_NONE;

	find_RTL_destination();
}

void RTL::find_RTL_destination()
{
	// don't update RTL destination faster than 1 Hz
	if (hrt_elapsed_time(&_destination_check_time) < 1_s) {
		return;
	}

	if (!_navigator->home_position_valid()) {
		return;
	}

	_destination_check_time = hrt_absolute_time();

	// get home position:
	home_position_s &home_landing_position = *_navigator->get_home_position();

	// get global position
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	// set destination to home per default, then check if other valid landing spot is closer
	_destination.set(home_landing_position);

	// get distance to home position
	double dlat = home_landing_position.lat - global_position.lat;
	double dlon = home_landing_position.lon - global_position.lon;

	double lon_scale = cos(radians(global_position.lat));

	auto coord_dist_sq = [lon_scale](double lat_diff, double lon_diff) -> double {
		double lon_diff_scaled =  lon_scale * matrix::wrap(lon_diff, -180., 180.);
		return lat_diff * lat_diff + lon_diff_scaled * lon_diff_scaled;
	};

	double min_dist_squared = coord_dist_sq(dlat, dlon);

	_destination.type = RTL_DESTINATION_HOME;

	const bool vtol_in_rw_mode = _navigator->get_vstatus()->is_vtol
				     && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;


	// consider the mission landing if not RTL_HOME type set
	if (rtl_type() != RTL_HOME && _navigator->get_mission_start_land_available()) {
		double mission_landing_lat;
		double mission_landing_lon;
		float mission_landing_alt;
		RTLDestinationType destination_type = RTL_DESTINATION_MISSION_LANDING;

		if (vtol_in_rw_mode) {
			mission_landing_lat = _navigator->get_mission_landing_lat();
			mission_landing_lon = _navigator->get_mission_landing_lon();
			mission_landing_alt = _navigator->get_mission_landing_alt();
			destination_type = RTL_DESTINATION_HOME;

		} else {
			mission_landing_lat = _navigator->get_mission_landing_start_lat();
			mission_landing_lon = _navigator->get_mission_landing_start_lon();
			mission_landing_alt = _navigator->get_mission_landing_start_alt();
		}

		// compare home position to landing position to decide which is closer
		dlat = mission_landing_lat - global_position.lat;
		dlon = mission_landing_lon - global_position.lon;
		double dist_squared = coord_dist_sq(dlat, dlon);

		// set destination to mission landing if closest or in RTL_LAND or RTL_MISSION (so not in RTL_CLOSEST)
		if (dist_squared < min_dist_squared || rtl_type() != RTL_CLOSEST) {
			min_dist_squared = dist_squared;
			_destination.lat = mission_landing_lat;
			_destination.lon = mission_landing_lon;
			_destination.alt = mission_landing_alt;
			_destination.type = destination_type;


		}
	}

	// do not consider rally point if RTL type is set to RTL_MISSION, so exit function and use either home or mission landing
	if (rtl_type() == RTL_MISSION) {
		return;
	}

	// compare to safe landing positions
	mission_safe_point_s closest_safe_point {};
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_safe_points = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_safe_points = stats.num_items;
	}

	// check if a safe point is closer than home or landing
	int closest_index = 0;

	for (int current_seq = 1; current_seq <= num_safe_points; ++current_seq) {
		mission_safe_point_s mission_safe_point;

		if (dm_read(DM_KEY_SAFE_POINTS, current_seq, &mission_safe_point, sizeof(mission_safe_point_s)) !=
		    sizeof(mission_safe_point_s)) {
			PX4_ERR("dm_read failed");
			continue;
		}

		// TODO: take altitude into account for distance measurement
		dlat = mission_safe_point.lat - global_position.lat;
		dlon = mission_safe_point.lon - global_position.lon;
		double dist_squared = coord_dist_sq(dlat, dlon);

		if (dist_squared < min_dist_squared) {
			closest_index = current_seq;
			min_dist_squared = dist_squared;
			closest_safe_point = mission_safe_point;
		}
	}

	if (closest_index > 0) {
		_destination.type = RTL_DESTINATION_SAFE_POINT;

		// There is a safe point closer than home/mission landing
		// TODO: handle all possible mission_safe_point.frame cases
		switch (closest_safe_point.frame) {
		case 0: // MAV_FRAME_GLOBAL
			_destination.lat = closest_safe_point.lat;
			_destination.lon = closest_safe_point.lon;
			_destination.alt = closest_safe_point.alt;
			_destination.yaw = home_landing_position.yaw;
			break;

		case 3: // MAV_FRAME_GLOBAL_RELATIVE_ALT
			_destination.lat = closest_safe_point.lat;
			_destination.lon = closest_safe_point.lon;
			_destination.alt = closest_safe_point.alt + home_landing_position.alt; // alt of safe point is rel to home
			_destination.yaw = home_landing_position.yaw;
			break;

		default:
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unsupported MAV_FRAME");
			break;
		}
	}

	// figure out how long the RTL will take
	float rtl_xy_speed, rtl_z_speed;
	get_rtl_xy_z_speed(rtl_xy_speed, rtl_z_speed);

	matrix::Vector3f to_destination_vec;
	get_vector_to_next_waypoint(global_position.lat, global_position.lon, _destination.lat, _destination.lon,
				    &to_destination_vec(0), &to_destination_vec(1));
	to_destination_vec(2) = _destination.alt - global_position.alt;

	float time_to_home_s = time_to_home(to_destination_vec, get_wind(), rtl_xy_speed, rtl_z_speed);

	float rtl_flight_time_ratio = time_to_home_s / (60 * _param_rtl_flt_time.get());
	rtl_flight_time_s rtl_flight_time{};
	rtl_flight_time.timestamp = hrt_absolute_time();
	rtl_flight_time.rtl_limit_fraction = rtl_flight_time_ratio;
	rtl_flight_time.rtl_time_s = time_to_home_s;
	_rtl_flight_time_pub.publish(rtl_flight_time);
}

void RTL::on_activation()
{

	_deny_mission_landing = _navigator->get_vstatus()->is_vtol
				&& _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	// output the correct message, depending on where the RTL destination is
	switch (_destination.type) {
	case RTL_DESTINATION_HOME:
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: landing at home position.");
		break;

	case RTL_DESTINATION_MISSION_LANDING:
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: landing at mission landing.");
		break;

	case RTL_DESTINATION_SAFE_POINT:
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: landing at safe landing point.");
		break;
	}

	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	_rtl_loiter_rad = _param_rtl_loiter_rad.get();

	if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_rtl_alt = calculate_return_alt_from_cone_half_angle((float)_param_rtl_cone_half_angle_deg.get());

	} else {
		_rtl_alt = max(global_position.alt, max(_destination.alt,
							_navigator->get_home_position()->alt + _param_rtl_return_alt.get()));
	}


	if (_navigator->get_land_detected()->landed) {
		// For safety reasons don't go into RTL if landed.
		_rtl_state = RTL_STATE_LANDED;

	} else if ((_destination.type == RTL_DESTINATION_MISSION_LANDING) && _navigator->getMissionLandingInProgress()) {
		// we were just on a mission landing, set _rtl_state past RTL_STATE_RETURN such that navigator will engage mission mode,
		// which will continue executing the landing
		_rtl_state = RTL_STATE_DESCEND;


	} else if ((global_position.alt < _destination.alt + _param_rtl_return_alt.get()) || _rtl_alt_min) {

		// If lower than return altitude, climb up first.
		// If rtl_alt_min is true then forcing altitude change even if above.
		_rtl_state = RTL_STATE_CLIMB;

	} else {
		// Otherwise go straight to return
		_rtl_state = RTL_STATE_RETURN;
	}

	setClimbAndReturnDone(_rtl_state > RTL_STATE_RETURN);

	set_rtl_item();

}

void RTL::on_active()
{
	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		advance_rtl();
		set_rtl_item();
	}

	if (_rtl_state == RTL_STATE_LAND && _param_rtl_pld_md.get() > 0) {
		_navigator->get_precland()->on_active();

	} else if (_navigator->get_precland()->is_activated()) {
		_navigator->get_precland()->on_inactivation();
	}
}

void RTL::set_rtl_item()
{
	// RTL_TYPE: mission landing.
	// Landing using planned mission landing, fly to DO_LAND_START instead of returning _destination.
	// After reaching DO_LAND_START, do nothing, let navigator takeover with mission landing.
	if (_destination.type == RTL_DESTINATION_MISSION_LANDING) {
		if (_rtl_state > RTL_STATE_RETURN) {
			if (_navigator->start_mission_landing()) {
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: using mission landing");
				return;

			} else {
				// Otherwise use regular RTL.
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unable to use mission landing");
			}
		}
	}

	_navigator->set_can_loiter_at_sp(false);

	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon, gpos.lat, gpos.lon);
	const float descend_altitude_target = min(_destination.alt + _param_rtl_descend_alt.get(), gpos.alt);
	const float loiter_altitude = min(descend_altitude_target, _rtl_alt);

	switch (_rtl_state) {
	case RTL_STATE_CLIMB: {

			// do not use LOITER_TO_ALT for rotary wing mode as it would then always climb to at least MIS_LTRMIN_ALT,
			// even if current climb altitude is below (e.g. RTL immediately after take off)
			if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

			} else {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
			}

			_mission_item.lat = gpos.lat;
			_mission_item.lon = gpos.lon;
			_mission_item.altitude = _rtl_alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = _navigator->get_local_position()->heading;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above destination)",
					 (int)ceilf(_rtl_alt), (int)ceilf(_rtl_alt - _destination.alt));
			break;
		}

	case RTL_STATE_RETURN: {
			// Don't change altitude.
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = _rtl_alt;
			_mission_item.altitude_is_relative = false;

			// Use destination yaw if close to _destination.
			// Check if we are pretty close to the destination already.
			if (destination_dist < _param_rtl_min_dist.get()) {
				_mission_item.yaw = _destination.yaw;

			} else {
				// Use current heading to _destination.
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _destination.lat, _destination.lon);
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: return at %d m (%d m above destination)",
					 (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - _destination.alt));

			break;
		}

	case RTL_STATE_DESCEND: {
			_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;

			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;

			// Except for vtol which might be still off here and should point towards this location.
			const float d_current = get_distance_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			if (_navigator->get_vstatus()->is_vtol && (d_current > _navigator->get_acceptance_radius())) {
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			} else {
				_mission_item.yaw = _destination.yaw;
			}

			if (_navigator->get_vstatus()->is_vtol) {
				_mission_item.loiter_radius = _rtl_loiter_rad;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: descend to %d m (%d m above destination)",
					 (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - _destination.alt));
			break;
		}

	case RTL_STATE_LOITER: {
			const bool autoland = (_param_rtl_land_delay.get() > FLT_EPSILON);

			// Don't change altitude.
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = _destination.yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = max(_param_rtl_land_delay.get(), 0.0f);
			_mission_item.autocontinue = autoland;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);

			if (autoland) {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: loiter %.1fs",
						 (double)get_time_inside(_mission_item));

			} else {
				_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loitering");
			}

			break;
		}

	case RTL_STATE_HEAD_TO_CENTER: {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;

			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;
			break;
		}

	case RTL_STATE_TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
			break;
		}

	case RTL_MOVE_TO_LAND_HOVER_VTOL: {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.origin = ORIGIN_ONBOARD;
			break;
		}

	case RTL_STATE_LAND: {
			// Land at destination.
			_mission_item.nav_cmd = NAV_CMD_LAND;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.yaw = _destination.yaw;
			_mission_item.altitude = _destination.alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.land_precision = _param_rtl_pld_md.get();

			if (_mission_item.land_precision == 1) {
				_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);
				_navigator->get_precland()->on_activation();

			} else if (_mission_item.land_precision == 2) {
				_navigator->get_precland()->set_mode(PrecLandMode::Required);
				_navigator->get_precland()->on_activation();
			}

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at destination");
			break;
		}

	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);
			set_return_alt_min(false);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	// Execute command if set. This is required for commands like VTOL transition.
	if (!item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	// Convert mission item to current position setpoint and make it valid.
	mission_apply_limitation(_mission_item);

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void RTL::advance_rtl()
{
	// determines if the vehicle should loiter above land
	const bool descend_and_loiter = _param_rtl_land_delay.get() < -DELAY_SIGMA || _param_rtl_land_delay.get() > DELAY_SIGMA;

	// vehicle is a vtol and currently in fixed wing mode
	const bool vtol_in_fw_mode = _navigator->get_vstatus()->is_vtol
				     && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:
		setClimbAndReturnDone(true);

		if (vtol_in_fw_mode || descend_and_loiter) {
			_rtl_state = RTL_STATE_DESCEND;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_DESCEND:

		if (descend_and_loiter) {
			_rtl_state = RTL_STATE_LOITER;

		} else if (vtol_in_fw_mode) {
			_rtl_state = RTL_STATE_HEAD_TO_CENTER;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_LOITER:
		if (vtol_in_fw_mode) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		_rtl_state = RTL_STATE_LAND;
		break;

	case RTL_STATE_HEAD_TO_CENTER:

		_rtl_state = RTL_STATE_TRANSITION_TO_MC;

		break;

	case RTL_STATE_TRANSITION_TO_MC:

		_rtl_state = RTL_MOVE_TO_LAND_HOVER_VTOL;

		break;

	case RTL_MOVE_TO_LAND_HOVER_VTOL:

		_rtl_state = RTL_STATE_LAND;

		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;
		break;

	default:
		break;
	}
}

float RTL::calculate_return_alt_from_cone_half_angle(float cone_half_angle_deg)
{
	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	// horizontal distance to destination
	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon, gpos.lat, gpos.lon);

	// minium rtl altitude to use when outside of horizontal acceptance radius of target position.
	// We choose the minimum height to be two times the distance from the land position in order to
	// avoid the vehicle touching the ground while still moving horizontally.
	const float return_altitude_min_outside_acceptance_rad_amsl = _destination.alt + 2.0f *
			_navigator->get_acceptance_radius();

	float return_altitude_amsl = _destination.alt + _param_rtl_return_alt.get();

	if (destination_dist <= _navigator->get_acceptance_radius()) {
		return_altitude_amsl = _destination.alt + 2.0f * destination_dist;

	} else {

		if (cone_half_angle_deg > 0.0f && destination_dist <= _param_rtl_min_dist.get()) {

			// constrain cone half angle to meaningful values. All other cases are already handled above.
			const float cone_half_angle_rad = radians(constrain(cone_half_angle_deg, 1.0f, 89.0f));

			// minimum altitude we need in order to be within the user defined cone
			const float cone_intersection_altitude_amsl = destination_dist / tanf(cone_half_angle_rad) + _destination.alt;

			return_altitude_amsl = min(cone_intersection_altitude_amsl, return_altitude_amsl);
		}

		return_altitude_amsl = max(return_altitude_amsl, return_altitude_min_outside_acceptance_rad_amsl);
	}

	return max(return_altitude_amsl, gpos.alt);
}

void RTL::get_rtl_xy_z_speed(float &xy, float &z)
{
	uint8_t vehicle_type = _navigator->get_vstatus()->vehicle_type;
	// Caution: here be dragons!
	// Use C API to allow this code to be compiled with builds that don't have FW/MC/Rover

	if (vehicle_type != _rtl_vehicle_type) {
		_rtl_vehicle_type = vehicle_type;

		switch (vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
			_param_rtl_xy_speed = param_find("MPC_XY_CRUISE");
			_param_rtl_descent_speed = param_find("MPC_Z_VEL_MAX_DN");
			break;

		case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
			_param_rtl_xy_speed = param_find("FW_AIRSPD_TRIM");
			_param_rtl_descent_speed = param_find("FW_T_SINK_MIN");
			break;

		case vehicle_status_s::VEHICLE_TYPE_ROVER:
			_param_rtl_xy_speed = param_find("GND_SPEED_THR_SC");
			_param_rtl_descent_speed = PARAM_INVALID;
			break;
		}
	}

	if ((_param_rtl_xy_speed == PARAM_INVALID) || param_get(_param_rtl_xy_speed, &xy) != PX4_OK) {
		xy = 1e6f;
	}

	if ((_param_rtl_descent_speed == PARAM_INVALID) || param_get(_param_rtl_descent_speed, &z) != PX4_OK) {
		z = 1e6f;
	}

}

matrix::Vector2f RTL::get_wind()
{
	_wind_sub.update();
	matrix::Vector2f wind;

	if (hrt_absolute_time() - _wind_sub.get().timestamp < 1_s) {
		wind(0) = _wind_sub.get().windspeed_north;
		wind(1) = _wind_sub.get().windspeed_east;
	}

	return wind;
}

float time_to_home(const matrix::Vector3f &to_home_vec,
		   const matrix::Vector2f &wind_velocity, float vehicle_speed_m_s, float vehicle_descent_speed_m_s)
{
	const matrix::Vector2f to_home = to_home_vec.xy();
	const float alt_change = to_home_vec(2);
	const matrix::Vector2f to_home_dir = to_home.unit_or_zero();
	const float dist_to_home = to_home.norm();

	const float wind_towards_home = wind_velocity.dot(to_home_dir);
	const float wind_across_home = matrix::Vector2f(wind_velocity - to_home_dir * wind_towards_home).norm();

	// Note: use fminf so that we don't _rely_ on wind towards home to make RTL more efficient
	const float cruise_speed = sqrtf(vehicle_speed_m_s * vehicle_speed_m_s - wind_across_home * wind_across_home) + fminf(
					   0.f, wind_towards_home);

	if (!PX4_ISFINITE(cruise_speed) || cruise_speed <= 0) {
		return INFINITY; // we never reach home if the wind is stronger than vehicle speed
	}

	// assume horizontal and vertical motions happen serially, so their time adds
	float horiz = dist_to_home / cruise_speed;
	float descent = fabsf(alt_change) / vehicle_descent_speed_m_s;
	return horiz + descent;
}
