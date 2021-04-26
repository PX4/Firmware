/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file MulticopterLandDetector.cpp
 *
 *The MC land-detector goes through 3 states before it will detect landed:
 *
 *State 1 (=ground_contact):
 *ground_contact is detected once the vehicle is not moving along the NED-z direction and has
 *a thrust value below 0.3 of the thrust_range (thrust_hover - thrust_min). The condition has to be true
 *for GROUND_CONTACT_TRIGGER_TIME_US in order to detect ground_contact
 *
 *State 2 (=maybe_landed):
 *maybe_landed can only occur if the internal ground_contact hysteresis state is true. maybe_landed criteria requires to have no motion in x and y,
 *no rotation and a thrust below 0.1 of the thrust_range (thrust_hover - thrust_min). In addition, the mc_pos_control turns off the thrust_sp in
 *body frame along x and y which helps to detect maybe_landed. The criteria for maybe_landed needs to be true for MAYBE_LAND_DETECTOR_TRIGGER_TIME_US.
 *
 *State 3 (=landed)
 *landed can only be detected if maybe_landed is true for LAND_DETECTOR_TRIGGER_TIME_US. No farther criteria is tested, but the mc_pos_control goes into
 *idle (thrust_sp = 0) which helps to detect landed. By doing this the thrust-criteria of State 2 will always be met, however the remaining criteria of no rotation and no motion still
 *have to be valid.

 *It is to note that if one criteria is not met, then vehicle exits the state directly without blocking.
 *
 *If the land-detector does not detect ground_contact, then the vehicle is either flying or falling, where free fall detection heavily relies
 *on the acceleration. TODO: verify that free fall is reliable
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include "MulticopterLandDetector.h"

using matrix::Vector2f;
using matrix::Vector3f;

namespace land_detector
{

MulticopterLandDetector::MulticopterLandDetector()
{
	_paramHandle.landSpeed = param_find("MPC_LAND_SPEED");
	_paramHandle.minManThrottle = param_find("MPC_MANTHR_MIN");
	_paramHandle.minThrottle = param_find("MPC_THR_MIN");
	_paramHandle.useHoverThrustEstimate = param_find("MPC_USE_HTE");
	_paramHandle.hoverThrottle = param_find("MPC_THR_HOVER");

	// Use Trigger time when transitioning from in-air (false) to landed (true) / ground contact (true).
	_freefall_hysteresis.set_hysteresis_time_from(false, FREEFALL_TRIGGER_TIME_US);
	_ground_contact_hysteresis.set_hysteresis_time_from(false, GROUND_CONTACT_TRIGGER_TIME_US);
	_maybe_landed_hysteresis.set_hysteresis_time_from(false, MAYBE_LAND_DETECTOR_TRIGGER_TIME_US);
	_landed_hysteresis.set_hysteresis_time_from(false, LAND_DETECTOR_TRIGGER_TIME_US);
}

void MulticopterLandDetector::_update_topics()
{
	actuator_controls_s actuator_controls;

	if (_actuator_controls_sub.update(&actuator_controls)) {
		_actuator_controls_throttle = actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
	}

	vehicle_control_mode_s vehicle_control_mode;

	if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
		_flag_control_climb_rate_enabled = vehicle_control_mode.flag_control_climb_rate_enabled;
	}

	if (_params.useHoverThrustEstimate) {
		hover_thrust_estimate_s hte;

		if (_hover_thrust_estimate_sub.update(&hte)) {
			if (hte.valid) {
				_params.hoverThrottle = hte.hover_thrust;
				_hover_thrust_estimate_last_valid = hte.timestamp;
			}
		}
	}

	takeoff_status_s takeoff_status;

	if (_takeoff_status_sub.update(&takeoff_status)) {
		_takeoff_state = takeoff_status.takeoff_state;
	}
}

void MulticopterLandDetector::_update_params()
{
	param_get(_paramHandle.minThrottle, &_params.minThrottle);
	param_get(_paramHandle.minManThrottle, &_params.minManThrottle);
	param_get(_paramHandle.landSpeed, &_params.landSpeed);

	if (_param_lndmc_z_vel_max.get() > _params.landSpeed) {
		PX4_ERR("LNDMC_Z_VEL_MAX > MPC_LAND_SPEED, updating %.3f -> %.3f",
			(double)_param_lndmc_z_vel_max.get(), (double)_params.landSpeed);

		_param_lndmc_z_vel_max.set(_params.landSpeed);
		_param_lndmc_z_vel_max.commit_no_notification();
	}

	int32_t use_hover_thrust_estimate = 0;
	param_get(_paramHandle.useHoverThrustEstimate, &use_hover_thrust_estimate);
	_params.useHoverThrustEstimate = (use_hover_thrust_estimate == 1);

	if (!_params.useHoverThrustEstimate || !_hover_thrust_initialized) {
		param_get(_paramHandle.hoverThrottle, &_params.hoverThrottle);

		// HTE runs based on the position controller so, even if we wish to use
		// the estimate, it is only available in altitude and position modes.
		// Therefore, we need to always initialize the hoverThrottle using the hover
		// thrust parameter in case we fly in stabilized
		// TODO: this can be removed once HTE runs in all modes
		_hover_thrust_initialized = true;
	}
}

bool MulticopterLandDetector::_get_freefall_state()
{
	// norm of specific force. Should be close to 9.8 m/s^2 when landed.
	return _acceleration.norm() < 2.f;
}

bool MulticopterLandDetector::_get_ground_contact_state()
{
	const hrt_abstime time_now_us = hrt_absolute_time();

	const bool lpos_available = ((time_now_us - _vehicle_local_position.timestamp) < 1_s);

	// land speed threshold, 90% of MPC_LAND_SPEED
	const float land_speed_threshold = 0.9f * math::max(_params.landSpeed, 0.1f);

	bool vertical_movement = true;

	if (lpos_available && _vehicle_local_position.v_z_valid) {
		// Check if we are moving vertically - this might see a spike after arming due to
		// throttle-up vibration. If accelerating fast the throttle thresholds will still give
		// an accurate in-air indication.
		float max_climb_rate = math::min(land_speed_threshold * 0.5f, _param_lndmc_z_vel_max.get());

		if ((time_now_us - _landed_time) < LAND_DETECTOR_LAND_PHASE_TIME_US) {
			// Widen acceptance thresholds for landed state right after arming
			// so that motor spool-up and other effects do not trigger false negatives.
			max_climb_rate = _param_lndmc_z_vel_max.get() * 2.5f;
		}

		vertical_movement = (fabsf(_vehicle_local_position.vz) > max_climb_rate);
	}


	// Check if we are moving horizontally.
	if (lpos_available && _vehicle_local_position.v_xy_valid) {
		const Vector2f v_xy{_vehicle_local_position.vx, _vehicle_local_position.vy};
		_horizontal_movement = v_xy.longerThan(_param_lndmc_xy_vel_max.get());

	} else {
		_horizontal_movement = false; // not known
	}

	if (lpos_available && _vehicle_local_position.dist_bottom_valid) {
		_below_gnd_effect_hgt = _vehicle_local_position.dist_bottom < _get_gnd_effect_altitude();

	} else {
		_below_gnd_effect_hgt = false;
	}

	const bool hover_thrust_estimate_valid = ((time_now_us - _hover_thrust_estimate_last_valid) < 1_s);

	if (!_in_descend || hover_thrust_estimate_valid) {
		// continue using valid hover thrust if it became invalid during descent
		_hover_thrust_estimate_valid = hover_thrust_estimate_valid;
	}

	// low thrust: 30% of throttle range between min and hover, relaxed to 60% if hover thrust estimate available
	const float thr_pct_hover = _hover_thrust_estimate_valid ? 0.6f : 0.3f;
	const float sys_low_throttle = _params.minThrottle + (_params.hoverThrottle - _params.minThrottle) * thr_pct_hover;
	bool ground_contact = (_actuator_controls_throttle <= sys_low_throttle);

	// if we have a valid velocity setpoint and the vehicle is demanded to go down but no vertical movement present,
	// we then can assume that the vehicle hit ground
	if (_flag_control_climb_rate_enabled) {
		vehicle_local_position_setpoint_s vehicle_local_position_setpoint;

		if (_vehicle_local_position_setpoint_sub.update(&vehicle_local_position_setpoint)) {
			// setpoints can briefly be NAN to signal resets, TODO: fix in multicopter position controller
			const bool descend_vel_sp = PX4_ISFINITE(vehicle_local_position_setpoint.vz)
						    && (vehicle_local_position_setpoint.vz >= land_speed_threshold);

			const bool descend_acc_sp = PX4_ISFINITE(vehicle_local_position_setpoint.acceleration[2])
						    && (vehicle_local_position_setpoint.acceleration[2] >= 100.f);

			_in_descend = descend_vel_sp || descend_acc_sp;
		}

		// ground contact requires commanded descent until landed
		if (!_maybe_landed_hysteresis.get_state() && !_landed_hysteresis.get_state()) {
			ground_contact &= _in_descend;
		}

	} else {
		_in_descend = false;
	}


	// When not armed, consider to have ground-contact
	if (!_armed) {
		return true;
	}


	// TODO: we need an accelerometer based check for vertical movement for flying without GPS
	return ground_contact && !_horizontal_movement && !vertical_movement;
}

bool MulticopterLandDetector::_get_maybe_landed_state()
{
	// When not armed, consider to be maybe-landed
	if (!_armed) {
		return true;
	}

	const hrt_abstime time_now_us = hrt_absolute_time();

	// minimal throttle: initially 10% of throttle range between min and hover
	float sys_min_throttle = _params.minThrottle + (_params.hoverThrottle - _params.minThrottle) * 0.1f;

	// Determine the system min throttle based on flight mode
	if (!_flag_control_climb_rate_enabled) {
		sys_min_throttle = (_params.minManThrottle + 0.01f);
	}

	// Check if thrust output is less than the minimum throttle.
	if (_actuator_controls_throttle <= sys_min_throttle) {
		if (_min_thrust_start == 0) {
			_min_thrust_start = time_now_us;
		}

	} else {
		_min_thrust_start = 0;
		return false;
	}


	if (_freefall_hysteresis.get_state()) {
		return false;
	}


	float landThresholdFactor = 1.f;

	// Widen acceptance thresholds for landed state right after landed
	if ((time_now_us - _landed_time) < LAND_DETECTOR_LAND_PHASE_TIME_US) {
		landThresholdFactor = 2.5f;
	}

	// Next look if all rotation angles are not moving.
	vehicle_angular_velocity_s vehicle_angular_velocity{};
	_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
	const Vector2f angular_velocity{vehicle_angular_velocity.xyz[0], vehicle_angular_velocity.xyz[1]};
	const float max_rotation_scaled = math::radians(_param_lndmc_rot_max.get()) * landThresholdFactor;

	if (angular_velocity.norm() > max_rotation_scaled) {
		return false;
	}

	// If vertical velocity is available: ground contact, no thrust, no movement -> landed
	if (((time_now_us - _vehicle_local_position.timestamp) < 1_s) && _vehicle_local_position.v_z_valid) {
		return _ground_contact_hysteresis.get_state();
	}

	// Otherwise, landed if the system has minimum thrust (manual or in failsafe) and no rotation for at least 8 seconds
	return (_min_thrust_start > 0) && ((time_now_us - _min_thrust_start) > 8_s);
}

bool MulticopterLandDetector::_get_landed_state()
{
	// reset the landed_time
	if (!_maybe_landed_hysteresis.get_state()) {
		_landed_time = 0;

	} else if (_landed_time == 0) {
		_landed_time = hrt_absolute_time();
	}

	// When not armed, consider to be landed
	if (!_armed) {
		return true;
	}

	// if we have maybe_landed, the mc_pos_control goes into idle (thrust_sp = 0.0)
	// therefore check if all other condition of the landed state remain true
	return _maybe_landed_hysteresis.get_state();
}

float MulticopterLandDetector::_get_max_altitude()
{
	if (_param_lndmc_alt_max.get() < 0.0f) {
		return INFINITY;

	} else {
		return _param_lndmc_alt_max.get();
	}
}

float MulticopterLandDetector::_get_gnd_effect_altitude()
{
	if (_param_lndmc_alt_gnd_effect.get() < 0.0f) {
		return INFINITY;

	} else {
		return _param_lndmc_alt_gnd_effect.get();
	}
}

bool MulticopterLandDetector::_get_ground_effect_state()
{
	return (_in_descend && !_horizontal_movement) ||
	       (_below_gnd_effect_hgt && _takeoff_state == takeoff_status_s::TAKEOFF_STATE_FLIGHT) ||
	       _takeoff_state == takeoff_status_s::TAKEOFF_STATE_RAMPUP;
}

} // namespace land_detector
