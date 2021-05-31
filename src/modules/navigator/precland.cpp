/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file precland.cpp
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "precland.h"
#include "navigator.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#define SEC2USEC 1000000.0f

#define STATE_TIMEOUT 10000000 // [us] Maximum time to spend in any state

PrecLand::PrecLand(Navigator *navigator, NavigatorCore &navigator_core) :
	MissionBlock(navigator, _navigator_core),
	ModuleParams(navigator)
{
	_handle_param_acceleration_hor = param_find("MPC_ACC_HOR");
	_handle_param_xy_vel_cruise = param_find("MPC_XY_CRUISE");

	updateParams();
}

void
PrecLand::on_activation()
{
	_state = PrecLandState::Start;
	_search_cnt = 0;
	_last_slewrate_time = 0;

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (!map_projection_initialized(&_map_ref)) {
		map_projection_init(&_map_ref, vehicle_local_position->ref_lat, vehicle_local_position->ref_lon);
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->previous.valid = false;

	// Check that the current position setpoint is valid, otherwise land at current position
	if (!pos_sp_triplet->current.valid) {
		PX4_WARN("Resetting landing position to current position");
		pos_sp_triplet->current.lat = _navigator_core.getLatRad();
		pos_sp_triplet->current.lon = _navigator_core.getLonRad();
		pos_sp_triplet->current.alt = _navigator_core.getAltitudeAMSLMeters();
		pos_sp_triplet->current.valid = true;
		pos_sp_triplet->current.timestamp = hrt_absolute_time();
	}

	_sp_pev = matrix::Vector2f(0, 0);
	_sp_pev_prev = matrix::Vector2f(0, 0);
	_last_slewrate_time = 0;

	switch_to_state_start();

	_is_activated = true;
}

void
PrecLand::on_active()
{
	// get new target measurement
	_target_pose_updated = _target_pose_sub.update(&_target_pose);

	if (_target_pose_updated) {
		_target_pose_valid = true;
	}

	if ((hrt_elapsed_time(&_target_pose.timestamp) / 1e6f) > _param_pld_btout.get()) {
		_target_pose_valid = false;
	}

	// stop if we are landed
	if (_navigator_core.getLanded()) {
		switch_to_state_done();
	}

	switch (_state) {
	case PrecLandState::Start:
		run_state_start();
		break;

	case PrecLandState::HorizontalApproach:
		run_state_horizontal_approach();
		break;

	case PrecLandState::DescendAboveTarget:
		run_state_descend_above_target();
		break;

	case PrecLandState::FinalApproach:
		run_state_final_approach();
		break;

	case PrecLandState::Search:
		run_state_search();
		break;

	case PrecLandState::Fallback:
		run_state_fallback();
		break;

	case PrecLandState::Done:
		// nothing to do
		break;

	default:
		// unknown state
		break;
	}

}

void
PrecLand::on_inactivation()
{
	_is_activated = false;
}

void
PrecLand::updateParams()
{
	ModuleParams::updateParams();

	if (_handle_param_acceleration_hor != PARAM_INVALID) {
		param_get(_handle_param_acceleration_hor, &_param_acceleration_hor);
	}

	if (_handle_param_xy_vel_cruise != PARAM_INVALID) {
		param_get(_handle_param_xy_vel_cruise, &_param_xy_vel_cruise);
	}
}

void
PrecLand::run_state_start()
{
	// check if target visible and go to horizontal approach
	if (switch_to_state_horizontal_approach()) {
		return;
	}

	if (_mode == PrecLandMode::Opportunistic) {
		// could not see the target immediately, so just fall back to normal landing
		if (!switch_to_state_fallback()) {
			PX4_ERR("Can't switch to search or fallback landing");
		}
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	float dist = get_distance_to_next_waypoint(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
			_navigator_core.getLatRad(), _navigator_core.getLonRad());

	// check if we've reached the start point
	if (dist < _navigator_core.getAcceptanceRadiusMeter()) {
		if (!_point_reached_time) {
			_point_reached_time = hrt_absolute_time();
		}

		// if we don't see the target after 1 second, search for it
		if (_param_pld_srch_tout.get() > 0) {

			if (hrt_absolute_time() - _point_reached_time > 2000000) {
				if (!switch_to_state_search()) {
					if (!switch_to_state_fallback()) {
						PX4_ERR("Can't switch to search or fallback landing");
					}
				}
			}

		} else {
			if (!switch_to_state_fallback()) {
				PX4_ERR("Can't switch to search or fallback landing");
			}
		}
	}
}

void
PrecLand::run_state_horizontal_approach()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// check if target visible, if not go to start
	if (!check_state_conditions(PrecLandState::HorizontalApproach)) {
		PX4_WARN("Lost landing target while landing (horizontal approach).");

		// Stay at current position for searching for the landing target
		pos_sp_triplet->current.lat = _navigator_core.getLatRad();
		pos_sp_triplet->current.lon = _navigator_core.getLonRad();
		pos_sp_triplet->current.alt = _navigator_core.getAltitudeAMSLMeters();

		if (!switch_to_state_start()) {
			if (!switch_to_state_fallback()) {
				PX4_ERR("Can't switch to fallback landing");
			}
		}

		return;
	}

	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!_point_reached_time) {
			_point_reached_time = hrt_absolute_time();
		}

		if (hrt_absolute_time() - _point_reached_time > 2000000) {
			// if close enough for descent above target go to descend above target
			if (switch_to_state_descend_above_target()) {
				return;
			}
		}

	}

	if (hrt_absolute_time() - _state_start_time > STATE_TIMEOUT) {
		PX4_ERR("Precision landing took too long during horizontal approach phase.");

		if (switch_to_state_fallback()) {
			return;
		}

		PX4_ERR("Can't switch to fallback landing");
	}

	float x = _target_pose.x_abs;
	float y = _target_pose.y_abs;

	slewrate(x, y);

	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	double lat, lon;
	map_projection_reproject(&_map_ref, x, y, &lat, &lon);

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;
	pos_sp_triplet->current.alt = _approach_alt;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	_navigator->set_position_setpoint_triplet_updated();
}

void
PrecLand::run_state_descend_above_target()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// check if target visible
	if (!check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!switch_to_state_final_approach()) {
			PX4_WARN("Lost landing target while landing (descending).");

			// Stay at current position for searching for the target
			pos_sp_triplet->current.lat = _navigator_core.getLatRad();
			pos_sp_triplet->current.lon = _navigator_core.getLonRad();
			pos_sp_triplet->current.alt = _navigator_core.getAltitudeAMSLMeters();

			if (!switch_to_state_start()) {
				if (!switch_to_state_fallback()) {
					PX4_ERR("Can't switch to fallback landing");
				}
			}
		}

		return;
	}

	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	double lat, lon;
	map_projection_reproject(&_map_ref, _target_pose.x_abs, _target_pose.y_abs, &lat, &lon);

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;

	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

	_navigator->set_position_setpoint_triplet_updated();
}

void
PrecLand::run_state_final_approach()
{
	// nothing to do, will land
}

void
PrecLand::run_state_search()
{
	// check if we can see the target
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		if (!_target_acquired_time) {
			// target just became visible. Stop climbing, but give it some margin so we don't stop too apruptly
			_target_acquired_time = hrt_absolute_time();
			position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
			float new_alt = _navigator_core.getAltitudeAMSLMeters() + 1.0f;
			pos_sp_triplet->current.alt = new_alt < pos_sp_triplet->current.alt ? new_alt : pos_sp_triplet->current.alt;
			_navigator->set_position_setpoint_triplet_updated();
		}

	}

	// stay at that height for a second to allow the vehicle to settle
	if (_target_acquired_time && (hrt_absolute_time() - _target_acquired_time) > 1000000) {
		// try to switch to horizontal approach
		if (switch_to_state_horizontal_approach()) {
			return;
		}
	}

	// check if search timed out and go to fallback
	if (hrt_absolute_time() - _state_start_time > _param_pld_srch_tout.get()*SEC2USEC) {
		PX4_WARN("Search timed out");

		if (!switch_to_state_fallback()) {
			PX4_ERR("Can't switch to fallback landing");
		}
	}
}

void
PrecLand::run_state_fallback()
{
	// nothing to do, will land
}

bool
PrecLand::switch_to_state_start()
{
	if (check_state_conditions(PrecLandState::Start)) {
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		_navigator->set_position_setpoint_triplet_updated();
		_search_cnt++;

		_point_reached_time = 0;

		_state = PrecLandState::Start;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_horizontal_approach()
{
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		_approach_alt = _navigator_core.getAltitudeAMSLMeters();

		_point_reached_time = 0;

		_state = PrecLandState::HorizontalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_descend_above_target()
{
	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		_state = PrecLandState::DescendAboveTarget;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_final_approach()
{
	if (check_state_conditions(PrecLandState::FinalApproach)) {
		_state = PrecLandState::FinalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_search()
{
	PX4_INFO("Climbing to search altitude.");
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.alt = vehicle_local_position->ref_alt + _param_pld_srch_alt.get();
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_navigator->set_position_setpoint_triplet_updated();

	_target_acquired_time = 0;

	_state = PrecLandState::Search;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_fallback()
{
	PX4_WARN("Falling back to normal land.");
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.lat = _navigator_core.getLatRad();
	pos_sp_triplet->current.lon = _navigator_core.getLonRad();
	pos_sp_triplet->current.alt = _navigator_core.getAltitudeAMSLMeters();
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	_navigator->set_position_setpoint_triplet_updated();

	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_done()
{
	_state = PrecLandState::Done;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool PrecLand::check_state_conditions(PrecLandState state)
{
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	switch (state) {
	case PrecLandState::Start:
		return _search_cnt <= _param_pld_max_srch.get();

	case PrecLandState::HorizontalApproach:

		// if we're already in this state, only want to make it invalid if we reached the target but can't see it anymore
		if (_state == PrecLandState::HorizontalApproach) {
			if (fabsf(_target_pose.x_abs - vehicle_local_position->x) < _param_pld_hacc_rad.get()
			    && fabsf(_target_pose.y_abs - vehicle_local_position->y) < _param_pld_hacc_rad.get()) {
				// we've reached the position where we last saw the target. If we don't see it now, we need to do something
				return _target_pose_valid && _target_pose.abs_pos_valid;

			} else {
				// We've seen the target sometime during horizontal approach.
				// Even if we don't see it as we're moving towards it, continue approaching last known location
				return true;
			}
		}

		// If we're trying to switch to this state, the target needs to be visible
		return _target_pose_updated && _target_pose_valid && _target_pose.abs_pos_valid;

	case PrecLandState::DescendAboveTarget:

		// if we're already in this state, only leave it if target becomes unusable, don't care about horizontall offset to target
		if (_state == PrecLandState::DescendAboveTarget) {
			// if we're close to the ground, we're more critical of target timeouts so we quickly go into descend
			if (check_state_conditions(PrecLandState::FinalApproach)) {
				return hrt_absolute_time() - _target_pose.timestamp < 500000; // 0.5s

			} else {
				return _target_pose_valid && _target_pose.abs_pos_valid;
			}

		} else {
			// if not already in this state, need to be above target to enter it
			return _target_pose_updated && _target_pose.abs_pos_valid
			       && fabsf(_target_pose.x_abs - vehicle_local_position->x) < _param_pld_hacc_rad.get()
			       && fabsf(_target_pose.y_abs - vehicle_local_position->y) < _param_pld_hacc_rad.get();
		}

	case PrecLandState::FinalApproach:
		return _target_pose_valid && _target_pose.abs_pos_valid
		       && (_target_pose.z_abs - vehicle_local_position->z) < _param_pld_fappr_alt.get();

	case PrecLandState::Search:
		return true;

	case PrecLandState::Fallback:
		return true;

	default:
		return false;
	}
}

void PrecLand::slewrate(float &sp_x, float &sp_y)
{
	matrix::Vector2f sp_curr(sp_x, sp_y);
	uint64_t now = hrt_absolute_time();

	float dt = (now - _last_slewrate_time);

	if (dt < 1) {
		// bad dt, can't divide by it
		return;
	}

	dt /= SEC2USEC;

	if (!_last_slewrate_time) {
		// running the first time since switching to precland

		// assume dt will be about 50000us
		dt = 50000 / SEC2USEC;

		// set a best guess for previous setpoints for smooth transition
		map_projection_project(&_map_ref, _navigator->get_position_setpoint_triplet()->current.lat,
				       _navigator->get_position_setpoint_triplet()->current.lon, &_sp_pev(0), &_sp_pev(1));
		_sp_pev_prev(0) = _sp_pev(0) - _navigator_core.getVelNorthMPS() * dt;
		_sp_pev_prev(1) = _sp_pev(1) - _navigator_core.getVelEastMPS() * dt;
	}

	_last_slewrate_time = now;

	// limit the setpoint speed to the maximum cruise speed
	matrix::Vector2f sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > _param_xy_vel_cruise) {
		sp_vel = sp_vel.normalized() * _param_xy_vel_cruise;
		sp_curr = _sp_pev + sp_vel * dt;
	}

	// limit the setpoint acceleration to the maximum acceleration
	matrix::Vector2f sp_acc = (sp_curr - _sp_pev * 2 + _sp_pev_prev) / (dt * dt); // acceleration of the setpoints

	if (sp_acc.length() > _param_acceleration_hor) {
		sp_acc = sp_acc.normalized() * _param_acceleration_hor;
		sp_curr = _sp_pev * 2 - _sp_pev_prev + sp_acc * (dt * dt);
	}

	// limit the setpoint speed such that we can stop at the setpoint given the maximum acceleration/deceleration
	float max_spd = sqrtf(_param_acceleration_hor * ((matrix::Vector2f)(_sp_pev - matrix::Vector2f(sp_x,
			      sp_y))).length());
	sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > max_spd) {
		sp_vel = sp_vel.normalized() * max_spd;
		sp_curr = _sp_pev + sp_vel * dt;
	}

	_sp_pev_prev = _sp_pev;
	_sp_pev = sp_curr;

	sp_x = sp_curr(0);
	sp_y = sp_curr(1);
}
