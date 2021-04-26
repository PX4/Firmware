/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mission_block.cpp
 *
 * Helper class to use mission items
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include "mission_block.h"
#include "navigator.h"

#include <math.h>
#include <float.h>

#include <lib/ecl/geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vtol_vehicle_status.h>

using matrix::wrap_pi;

MissionBlock::MissionBlock(Navigator *navigator) :
	NavigatorMode(navigator)
{
	_mission_item.lat = (double)NAN;
	_mission_item.lon = (double)NAN;
	_mission_item.yaw = NAN;
	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.time_inside = 0.0f;
	_mission_item.autocontinue = true;
	_mission_item.origin = ORIGIN_ONBOARD;
}

bool
MissionBlock::is_mission_item_reached()
{
	/* handle non-navigation or indefinite waypoints */

	switch (_mission_item.nav_cmd) {
	case NAV_CMD_DO_SET_SERVO:
		return true;

	case NAV_CMD_LAND: /* fall through */
	case NAV_CMD_VTOL_LAND:
		return _navigator->get_land_detected()->landed;

	case NAV_CMD_IDLE: /* fall through */
	case NAV_CMD_LOITER_UNLIMITED:
		return false;

	case NAV_CMD_DO_LAND_START:
	case NAV_CMD_DO_TRIGGER_CONTROL:
	case NAV_CMD_DO_DIGICAM_CONTROL:
	case NAV_CMD_IMAGE_START_CAPTURE:
	case NAV_CMD_IMAGE_STOP_CAPTURE:
	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
	case NAV_CMD_DO_CONTROL_VIDEO:
	case NAV_CMD_DO_MOUNT_CONFIGURE:
	case NAV_CMD_DO_MOUNT_CONTROL:
	case NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
	case NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
	case NAV_CMD_DO_SET_ROI:
	case NAV_CMD_DO_SET_ROI_LOCATION:
	case NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
	case NAV_CMD_DO_SET_ROI_NONE:
	case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
	case NAV_CMD_OBLIQUE_SURVEY:
	case NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
	case NAV_CMD_SET_CAMERA_MODE:
	case NAV_CMD_SET_CAMERA_ZOOM:
	case NAV_CMD_SET_CAMERA_FOCUS:
		return true;

	case NAV_CMD_DO_VTOL_TRANSITION:

		/*
		 * We wait half a second to give the transition command time to propagate.
		 * Then monitor the transition status for completion.
		 */
		// TODO: check desired transition state achieved and drop _action_start
		if (hrt_absolute_time() - _action_start > 500000 &&
		    !_navigator->get_vstatus()->in_transition_mode) {

			_action_start = 0;
			return true;

		} else {
			return false;
		}

	case NAV_CMD_DO_CHANGE_SPEED:
	case NAV_CMD_DO_SET_HOME:
		return true;

	default:
		/* do nothing, this is a 3D waypoint */
		break;
	}

	hrt_abstime now = hrt_absolute_time();

	if (!_navigator->get_land_detected()->landed && !_waypoint_position_reached) {

		float dist = -1.0f;
		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		const float mission_item_altitude_amsl = get_absolute_altitude_for_item(_mission_item);

		dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, mission_item_altitude_amsl,
				_navigator->get_global_position()->lat,
				_navigator->get_global_position()->lon,
				_navigator->get_global_position()->alt,
				&dist_xy, &dist_z);

		if ((_mission_item.nav_cmd == NAV_CMD_TAKEOFF || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)
		    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

			/* We want to avoid the edge case where the acceptance radius is bigger or equal than
			 * the altitude of the takeoff waypoint above home. Otherwise, we do not really follow
			 * take-off procedures like leaving the landing gear down. */

			float takeoff_alt = _mission_item.altitude_is_relative ?
					    _mission_item.altitude :
					    (_mission_item.altitude - _navigator->get_home_position()->alt);

			float altitude_acceptance_radius = _navigator->get_altitude_acceptance_radius();

			/* It should be safe to just use half of the takoeff_alt as an acceptance radius. */
			if (takeoff_alt > 0 && takeoff_alt < altitude_acceptance_radius) {
				altitude_acceptance_radius = takeoff_alt / 2.0f;
			}

			/* require only altitude for takeoff for multicopter */
			if (_navigator->get_global_position()->alt >
			    mission_item_altitude_amsl - altitude_acceptance_radius) {
				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
			   && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
			/* for takeoff mission items use the parameter for the takeoff acceptance radius */
			if (dist_xy >= 0.0f && dist_xy <= _navigator->get_acceptance_radius()) {
				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF) {
			/* for takeoff mission items use the parameter for the takeoff acceptance radius */
			if (dist >= 0.0f && dist <= _navigator->get_acceptance_radius()
			    && dist_z <= _navigator->get_altitude_acceptance_radius()) {
				_waypoint_position_reached = true;
			}

		} else if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING &&
			   (_mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT)) {

			/* Loiter mission item on a non rotary wing: the aircraft is going to circle the
			 * coordinates with a radius equal to the loiter_radius field. It is not flying
			 * through the waypoint center.
			 * Therefore the item is marked as reached once the system reaches the loiter
			 * radius + L1 distance. Time inside and turn count is handled elsewhere.
			 */

			// check if within loiter radius around wp, if yes then set altitude sp to mission item
			if (dist >= 0.0f && dist_xy <= (_navigator->get_acceptance_radius() + fabsf(_mission_item.loiter_radius))
			    && dist_z <= _navigator->get_altitude_acceptance_radius()) {

				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
			// NAV_CMD_LOITER_TO_ALT only uses mission item altitude once it's in the loiter.
			// First check if the altitude setpoint is the mission setpoint (that means that the loiter is not yet reached)
			struct position_setpoint_s *curr_sp = &_navigator->get_position_setpoint_triplet()->current;

			if (fabsf(curr_sp->alt - mission_item_altitude_amsl) >= FLT_EPSILON) {
				dist_xy = -1.0f;
				dist_z = -1.0f;

				dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, curr_sp->alt,
						_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_navigator->get_global_position()->alt,
						&dist_xy, &dist_z);

				// check if within loiter radius around wp, if yes then set altitude sp to mission item
				if (dist >= 0.0f && dist_xy <= (_navigator->get_acceptance_radius() + fabsf(_mission_item.loiter_radius))
				    && dist_z <= _navigator->get_default_altitude_acceptance_radius()) {

					curr_sp->alt = mission_item_altitude_amsl;
					curr_sp->type = position_setpoint_s::SETPOINT_TYPE_LOITER;
					_navigator->set_position_setpoint_triplet_updated();
				}

			} else if (dist >= 0.f && dist_xy <= (_navigator->get_acceptance_radius() + fabsf(_mission_item.loiter_radius))
				   && dist_z <= _navigator->get_altitude_acceptance_radius()) {
				// loitering, check if new altitude is reached, while still also having check on position

				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_CONDITION_GATE) {

			struct position_setpoint_s *curr_sp = &_navigator->get_position_setpoint_triplet()->current;

			// if the setpoint is valid we are checking if we reached the gate
			// in the case of an invalid setpoint we are defaulting to
			// assuming that we have already reached the gate to not block
			// the further execution of the mission.
			if (curr_sp->valid) {

				// location of gate (mission item)
				struct map_projection_reference_s ref_pos;
				map_projection_init(&ref_pos, _mission_item.lat, _mission_item.lon);

				// current setpoint
				matrix::Vector2f gate_to_curr_sp;
				map_projection_project(&ref_pos, curr_sp->lat, curr_sp->lon, &gate_to_curr_sp(0), &gate_to_curr_sp(1));

				// system position
				matrix::Vector2f vehicle_pos;
				map_projection_project(&ref_pos, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
						       &vehicle_pos(0), &vehicle_pos(1));
				const float dot_product = vehicle_pos.dot(gate_to_curr_sp.normalized());

				// if the dot product (projected vector) is positive, then
				// the current position is between the gate position and the
				// next waypoint
				if (dot_product >= 0) {
					_waypoint_position_reached = true;
					_waypoint_yaw_reached = true;
					_time_wp_reached = now;
				}
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_DELAY) {
			_waypoint_position_reached = true;
			_waypoint_yaw_reached = true;
			_time_wp_reached = now;

		} else {
			/*normal mission items */

			float mission_acceptance_radius = _navigator->get_acceptance_radius();

			/* for vtol back transition calculate acceptance radius based on time and ground speed */
			if (_mission_item.vtol_back_transition
			    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

				float velocity = sqrtf(_navigator->get_local_position()->vx * _navigator->get_local_position()->vx +
						       _navigator->get_local_position()->vy * _navigator->get_local_position()->vy);

				const float back_trans_dec = _navigator->get_vtol_back_trans_deceleration();
				const float reverse_delay = _navigator->get_vtol_reverse_delay();

				if (back_trans_dec > FLT_EPSILON && velocity > FLT_EPSILON) {
					mission_acceptance_radius = ((velocity / back_trans_dec / 2) * velocity) + reverse_delay * velocity;

				}

			}

			if (dist_xy >= 0.0f && dist_xy <= mission_acceptance_radius
			    && dist_z <= _navigator->get_altitude_acceptance_radius()) {
				_waypoint_position_reached = true;
			}
		}

		if (_waypoint_position_reached) {
			// reached just now
			_time_wp_reached = now;
		}

		// consider yaw reached for non-rotary wing vehicles (such as fixed-wing)
		if (_navigator->get_vstatus()->vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			_waypoint_yaw_reached = true;
		}
	}

	/* Check if the requested yaw setpoint is reached (only for rotary wing flight). */

	if (_waypoint_position_reached && !_waypoint_yaw_reached) {

		if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && PX4_ISFINITE(_navigator->get_yaw_acceptance(_mission_item.yaw))) {

			const float yaw_err = wrap_pi(_mission_item.yaw - _navigator->get_local_position()->heading);

			/* accept yaw if reached or if timeout is set in which case we ignore not forced headings */
			if (fabsf(yaw_err) < _navigator->get_yaw_threshold()
			    || (_navigator->get_yaw_timeout() >= FLT_EPSILON && !_mission_item.force_heading)) {

				_waypoint_yaw_reached = true;
			}

			/* if heading needs to be reached, the timeout is enabled and we don't make it, abort mission */
			if (!_waypoint_yaw_reached && _mission_item.force_heading &&
			    (_navigator->get_yaw_timeout() >= FLT_EPSILON) &&
			    (now - _time_wp_reached >= (hrt_abstime)_navigator->get_yaw_timeout() * 1e6f)) {

				_navigator->set_mission_failure("unable to reach heading within timeout");
			}

		} else {
			_waypoint_yaw_reached = true;
		}
	}

	/* Once the waypoint and yaw setpoint have been reached we can start the loiter time countdown */
	if (_waypoint_position_reached && _waypoint_yaw_reached) {

		bool time_inside_reached = false;

		/* check if the MAV was long enough inside the waypoint orbit */
		if ((get_time_inside(_mission_item) < FLT_EPSILON) ||
		    (now - _time_wp_reached >= (hrt_abstime)(get_time_inside(_mission_item) * 1e6f))) {
			time_inside_reached = true;
		}

		// check if heading for exit is reached (only applies for fixed-wing flight)
		bool exit_heading_reached = false;

		if (time_inside_reached) {

			struct position_setpoint_s *curr_sp_new = &_navigator->get_position_setpoint_triplet()->current;
			const position_setpoint_s &next_sp = _navigator->get_position_setpoint_triplet()->next;

			/* enforce exit heading if in FW, the next wp is valid, the vehicle is currently loitering and either having force_heading set,
			   or if loitering to achieve altitdue at a NAV_CMD_WAYPOINT */
			const bool enforce_exit_heading = _navigator->get_vstatus()->vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
							  &&
							  next_sp.valid &&
							  curr_sp_new->type == position_setpoint_s::SETPOINT_TYPE_LOITER &&
							  (_mission_item.force_heading || _mission_item.nav_cmd == NAV_CMD_WAYPOINT);

			if (enforce_exit_heading) {


				const float dist_current_next = get_distance_to_next_waypoint(curr_sp_new->lat, curr_sp_new->lon, next_sp.lat,
								next_sp.lon);

				float yaw_err = 0.0f;

				if (dist_current_next >  1.2f * _navigator->get_loiter_radius()) {
					// set required yaw from bearing to the next mission item
					_mission_item.yaw = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
							    _navigator->get_global_position()->lon,
							    next_sp.lat, next_sp.lon);
					const float cog = atan2f(_navigator->get_local_position()->vy, _navigator->get_local_position()->vx);
					yaw_err = wrap_pi(_mission_item.yaw - cog);



				}


				if (fabsf(yaw_err) < _navigator->get_yaw_threshold()) {
					exit_heading_reached = true;
				}

			} else {
				exit_heading_reached = true;
			}
		}

		// set exit flight course to next waypoint
		if (exit_heading_reached) {
			position_setpoint_s &curr_sp = _navigator->get_position_setpoint_triplet()->current;
			const position_setpoint_s &next_sp = _navigator->get_position_setpoint_triplet()->next;

			const float range = get_distance_to_next_waypoint(curr_sp.lat, curr_sp.lon, next_sp.lat, next_sp.lon);

			// exit xtrack location
			// reset lat/lon of loiter waypoint so vehicle follows a tangent
			if (_mission_item.loiter_exit_xtrack && next_sp.valid && PX4_ISFINITE(range) &&
			    (_mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			     _mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT)) {

				float bearing = get_bearing_to_next_waypoint(curr_sp.lat, curr_sp.lon, next_sp.lat, next_sp.lon);
				// We should not use asinf outside of [-1..1].
				const float ratio = math::constrain(_mission_item.loiter_radius / range, -1.0f, 1.0f);
				float inner_angle = M_PI_2_F - asinf(ratio);

				// Compute "ideal" tangent origin
				if (curr_sp.loiter_direction > 0) {
					bearing -= inner_angle;

				} else {
					bearing += inner_angle;
				}

				// Replace current setpoint lat/lon with tangent coordinate
				waypoint_from_heading_and_distance(curr_sp.lat, curr_sp.lon,
								   bearing, fabsf(curr_sp.loiter_radius),
								   &curr_sp.lat, &curr_sp.lon);
			}

			return true; // mission item is reached
		}
	}

	return false;
}

void
MissionBlock::reset_mission_item_reached()
{
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;
	_time_wp_reached = 0;
}

void
MissionBlock::issue_command(const mission_item_s &item)
{
	if (item_contains_position(item)
	    || item_contains_gate(item)
	    || item_contains_marker(item)) {
		return;
	}

	if (item.nav_cmd == NAV_CMD_DO_SET_SERVO) {
		PX4_INFO("DO_SET_SERVO command");

		// XXX: we should issue a vehicle command and handle this somewhere else
		actuator_controls_s actuators = {};
		actuators.timestamp = hrt_absolute_time();

		// params[0] actuator number to be set 0..5 (corresponds to AUX outputs 1..6)
		// params[1] new value for selected actuator in ms 900...2000
		actuators.control[(int)item.params[0]] = 1.0f / 2000 * -item.params[1];

		_actuator_pub.publish(actuators);

	} else {
		_action_start = hrt_absolute_time();

		// This is to support legacy DO_MOUNT_CONTROL as part of a mission.
		if (item.nav_cmd == NAV_CMD_DO_MOUNT_CONTROL) {
			_navigator->acquire_gimbal_control();
		}

		// we're expecting a mission command item here so assign the "raw" inputs to the command
		// (MAV_FRAME_MISSION mission item)
		vehicle_command_s vcmd = {};
		vcmd.command = item.nav_cmd;
		vcmd.param1 = item.params[0];
		vcmd.param2 = item.params[1];
		vcmd.param3 = item.params[2];
		vcmd.param4 = item.params[3];

		if (item.nav_cmd == NAV_CMD_DO_SET_ROI_LOCATION && item.altitude_is_relative) {
			vcmd.param5 = item.lat;
			vcmd.param6 = item.lon;
			vcmd.param7 = item.altitude + _navigator->get_home_position()->alt;

		} else {
			vcmd.param5 = (double)item.params[4];
			vcmd.param6 = (double)item.params[5];
			vcmd.param7 = item.params[6];
		}

		_navigator->publish_vehicle_cmd(&vcmd);
	}
}

float
MissionBlock::get_time_inside(const mission_item_s &item) const
{
	if ((item.nav_cmd == NAV_CMD_WAYPOINT
	     && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) ||
	    item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
	    item.nav_cmd == NAV_CMD_DELAY) {

		// a negative time inside would be invalid
		return math::max(item.time_inside, 0.0f);
	}

	return 0.0f;
}

bool
MissionBlock::item_contains_position(const mission_item_s &item)
{
	return item.nav_cmd == NAV_CMD_WAYPOINT ||
	       item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
	       item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
	       item.nav_cmd == NAV_CMD_LAND ||
	       item.nav_cmd == NAV_CMD_TAKEOFF ||
	       item.nav_cmd == NAV_CMD_LOITER_TO_ALT ||
	       item.nav_cmd == NAV_CMD_VTOL_TAKEOFF ||
	       item.nav_cmd == NAV_CMD_VTOL_LAND ||
	       item.nav_cmd == NAV_CMD_DO_FOLLOW_REPOSITION;
}

bool
MissionBlock::item_contains_gate(const mission_item_s &item)
{
	return item.nav_cmd == NAV_CMD_CONDITION_GATE;
}

bool
MissionBlock::item_contains_marker(const mission_item_s &item)
{
	return item.nav_cmd == NAV_CMD_DO_LAND_START;
}

bool
MissionBlock::mission_item_to_position_setpoint(const mission_item_s &item, position_setpoint_s *sp)
{
	/* don't change the setpoint for non-position items */
	if (!item_contains_position(item)) {
		return false;
	}

	sp->lat = item.lat;
	sp->lon = item.lon;
	sp->alt = get_absolute_altitude_for_item(item);
	sp->yaw = item.yaw;
	sp->yaw_valid = PX4_ISFINITE(item.yaw);
	sp->loiter_radius = (fabsf(item.loiter_radius) > NAV_EPSILON_POSITION) ? fabsf(item.loiter_radius) :
			    _navigator->get_loiter_radius();
	sp->loiter_direction = (item.loiter_radius > 0) ? 1 : -1;

	if (item.acceptance_radius > 0.0f && PX4_ISFINITE(item.acceptance_radius)) {
		// if the mission item has a specified acceptance radius, overwrite the default one from parameters
		sp->acceptance_radius = item.acceptance_radius;

	} else {
		sp->acceptance_radius = _navigator->get_default_acceptance_radius();
	}

	sp->cruising_speed = _navigator->get_cruising_speed();
	sp->cruising_throttle = _navigator->get_cruising_throttle();

	switch (item.nav_cmd) {
	case NAV_CMD_IDLE:
		sp->type = position_setpoint_s::SETPOINT_TYPE_IDLE;
		break;

	case NAV_CMD_TAKEOFF:

		// if already flying (armed and !landed) treat TAKEOFF like regular POSITION
		if ((_navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED)
		    && !_navigator->get_land_detected()->landed && !_navigator->get_land_detected()->maybe_landed) {

			sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;

		} else {
			sp->type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
		}

		break;

	case NAV_CMD_VTOL_TAKEOFF:
		sp->type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
		break;

	case NAV_CMD_LAND:
	case NAV_CMD_VTOL_LAND:
		sp->type = position_setpoint_s::SETPOINT_TYPE_LAND;
		break;

	case NAV_CMD_LOITER_TO_ALT:

		// initially use current altitude, and switch to mission item altitude once in loiter position
		if (_navigator->get_loiter_min_alt() > 0.f) { // ignore _param_loiter_min_alt if smaller than 0
			sp->alt = math::max(_navigator->get_global_position()->alt,
					    _navigator->get_home_position()->alt + _navigator->get_loiter_min_alt());

		} else {
			sp->alt = _navigator->get_global_position()->alt;
		}

	// FALLTHROUGH
	case NAV_CMD_LOITER_TIME_LIMIT:
	case NAV_CMD_LOITER_UNLIMITED:

		sp->type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		break;

	default:
		sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		break;
	}

	sp->valid = true;
	sp->timestamp = hrt_absolute_time();

	return sp->valid;
}

void
MissionBlock::set_loiter_item(struct mission_item_s *item, float min_clearance)
{
	if (_navigator->get_land_detected()->landed) {
		/* landed, don't takeoff, but switch to IDLE mode */
		item->nav_cmd = NAV_CMD_IDLE;

	} else {
		item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;

		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

		if (_navigator->get_can_loiter_at_sp() && pos_sp_triplet->current.valid) {
			/* use current position setpoint */
			item->lat = pos_sp_triplet->current.lat;
			item->lon = pos_sp_triplet->current.lon;
			item->altitude = pos_sp_triplet->current.alt;

		} else {
			/* use current position and use return altitude as clearance */
			item->lat = _navigator->get_global_position()->lat;
			item->lon = _navigator->get_global_position()->lon;
			item->altitude = _navigator->get_global_position()->alt;

			if (min_clearance > 0.0f && item->altitude < _navigator->get_home_position()->alt + min_clearance) {
				item->altitude = _navigator->get_home_position()->alt + min_clearance;
			}
		}

		item->altitude_is_relative = false;
		item->yaw = NAN;
		item->loiter_radius = _navigator->get_loiter_radius();
		item->acceptance_radius = _navigator->get_acceptance_radius();
		item->time_inside = 0.0f;
		item->autocontinue = false;
		item->origin = ORIGIN_ONBOARD;
	}
}

void
MissionBlock::set_takeoff_item(struct mission_item_s *item, float abs_altitude)
{
	item->nav_cmd = NAV_CMD_TAKEOFF;

	/* use current position */
	item->lat = _navigator->get_global_position()->lat;
	item->lon = _navigator->get_global_position()->lon;
	item->yaw = _navigator->get_local_position()->heading;

	item->altitude = abs_altitude;
	item->altitude_is_relative = false;

	item->loiter_radius = _navigator->get_loiter_radius();
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_land_item(struct mission_item_s *item, bool at_current_location)
{
	/* VTOL transition to RW before landing */
	if (_navigator->force_vtol()) {

		vehicle_command_s vcmd = {};
		vcmd.command = NAV_CMD_DO_VTOL_TRANSITION;
		vcmd.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		_navigator->publish_vehicle_cmd(&vcmd);
	}

	/* set the land item */
	item->nav_cmd = NAV_CMD_LAND;

	/* use current position */
	if (at_current_location) {
		item->lat = (double)NAN; //descend at current position
		item->lon = (double)NAN; //descend at current position
		item->yaw = _navigator->get_local_position()->heading;

	} else {
		/* use home position */
		item->lat = _navigator->get_home_position()->lat;
		item->lon = _navigator->get_home_position()->lon;
		item->yaw = _navigator->get_home_position()->yaw;
	}

	item->altitude = 0;
	item->altitude_is_relative = false;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = true;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_idle_item(struct mission_item_s *item)
{
	item->nav_cmd = NAV_CMD_IDLE;
	item->lat = _navigator->get_home_position()->lat;
	item->lon = _navigator->get_home_position()->lon;
	item->altitude_is_relative = false;
	item->altitude = _navigator->get_home_position()->alt;
	item->yaw = NAN;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = true;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_vtol_transition_item(struct mission_item_s *item, const uint8_t new_mode)
{
	item->nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	item->params[0] = (float) new_mode;
	item->yaw = _navigator->get_local_position()->heading;
	item->autocontinue = true;
}

void
MissionBlock::mission_apply_limitation(mission_item_s &item)
{
	/*
	 * Limit altitude
	 */

	/* do nothing if altitude max is negative */
	if (_navigator->get_land_detected()->alt_max > 0.0f) {

		/* absolute altitude */
		float altitude_abs = item.altitude_is_relative
				     ? item.altitude + _navigator->get_home_position()->alt
				     : item.altitude;

		/* limit altitude to maximum allowed altitude */
		if ((_navigator->get_land_detected()->alt_max + _navigator->get_home_position()->alt) < altitude_abs) {
			item.altitude = item.altitude_is_relative ?
					_navigator->get_land_detected()->alt_max :
					_navigator->get_land_detected()->alt_max + _navigator->get_home_position()->alt;

		}
	}

	/*
	 * Add other limitations here
	 */
}

float
MissionBlock::get_absolute_altitude_for_item(const mission_item_s &mission_item) const
{
	if (mission_item.altitude_is_relative) {
		return mission_item.altitude + _navigator->get_home_position()->alt;

	} else {
		return mission_item.altitude;
	}
}
