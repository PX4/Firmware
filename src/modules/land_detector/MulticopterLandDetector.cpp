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


namespace land_detector
{

MulticopterLandDetector::MulticopterLandDetector()
{
	_paramHandle.landSpeed      = param_find("MPC_LAND_SPEED");
	_paramHandle.minManThrottle = param_find("MPC_MANTHR_MIN");
	_paramHandle.minThrottle    = param_find("MPC_THR_MIN");
	_paramHandle.useHoverThrustEstimate = param_find("MPC_USE_HTE");
	_paramHandle.hoverThrottle  = param_find("MPC_THR_HOVER");

	// Use Trigger time when transitioning from in-air (false) to landed (true) / ground contact (true).
	_ground_contact_hysteresis.set_hysteresis_time_from(false, GROUND_CONTACT_TRIGGER_TIME_US);
	_landed_hysteresis.set_hysteresis_time_from(false, LAND_DETECTOR_TRIGGER_TIME_US);
	_maybe_landed_hysteresis.set_hysteresis_time_from(false, MAYBE_LAND_DETECTOR_TRIGGER_TIME_US);
}

void MulticopterLandDetector::_update_topics()
{
	LandDetector::_update_topics();

	_actuator_controls_sub.update(&_actuator_controls);
	_battery_sub.update(&_battery_status);
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);
	_vehicle_local_position_setpoint_sub.update(&_vehicle_local_position_setpoint);

	if (_params.useHoverThrustEstimate) {
		hover_thrust_estimate_s hte;

		if (_hover_thrust_estimate_sub.update(&hte)) {
			_params.hoverThrottle = hte.hover_thrust;
		}
	}
}

void MulticopterLandDetector::_update_params()
{
	LandDetector::_update_params();

	_freefall_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1e6f * _param_lndmc_ffall_ttri.get()));

	param_get(_paramHandle.minThrottle, &_params.minThrottle);
	param_get(_paramHandle.minManThrottle, &_params.minManThrottle);
	param_get(_paramHandle.landSpeed, &_params.landSpeed);

	int32_t use_hover_thrust_estimate = 0;
	param_get(_paramHandle.useHoverThrustEstimate, &use_hover_thrust_estimate);
	_params.useHoverThrustEstimate = (use_hover_thrust_estimate == 1);

	if (!_params.useHoverThrustEstimate) {
		param_get(_paramHandle.hoverThrottle, &_params.hoverThrottle);
	}
}

bool MulticopterLandDetector::_get_freefall_state()
{
	if (_param_lndmc_ffall_thr.get() < 0.1f ||
	    _param_lndmc_ffall_thr.get() > 10.0f) {	//if parameter is set to zero or invalid, disable free-fall detection.
		return false;
	}

	if (_vehicle_acceleration.timestamp == 0) {
		// _sensors is not valid yet, we have to assume we're not falling.
		return false;
	}

	// norm of specific force. Should be close to 9.8 m/s^2 when landed.
	const matrix::Vector3f accel{_vehicle_acceleration.xyz};

	return (accel.norm() < _param_lndmc_ffall_thr.get());	// true if we are currently falling
}

bool MulticopterLandDetector::_get_ground_contact_state()
{
	// When not armed, consider to have ground-contact
	if (!_actuator_armed.armed) {
		return true;
	}

	// land speed threshold
	float land_speed_threshold = 0.9f * math::max(_params.landSpeed, 0.1f);

	// Check if we are moving vertically - this might see a spike after arming due to
	// throttle-up vibration. If accelerating fast the throttle thresholds will still give
	// an accurate in-air indication.
	bool vertical_movement = false;

	if (hrt_elapsed_time(&_landed_time) < LAND_DETECTOR_LAND_PHASE_TIME_US) {

		// Widen acceptance thresholds for landed state right after arming
		// so that motor spool-up and other effects do not trigger false negatives.
		vertical_movement = fabsf(_vehicle_local_position.vz) > _param_lndmc_z_vel_max.get()  * 2.5f;

	} else {
		// Adjust max_climb_rate if land_speed is lower than 2x max_climb_rate
		float max_climb_rate = ((land_speed_threshold * 0.5f) < _param_lndmc_z_vel_max.get()) ? (0.5f * land_speed_threshold) :
				       _param_lndmc_z_vel_max.get();
		vertical_movement = fabsf(_vehicle_local_position.vz) > max_climb_rate;
	}

	// Check if we are moving horizontally.
	_horizontal_movement = sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx
				     + _vehicle_local_position.vy * _vehicle_local_position.vy) > _param_lndmc_xy_vel_max.get();

	// if we have a valid velocity setpoint and the vehicle is demanded to go down but no vertical movement present,
	// we then can assume that the vehicle hit ground
	_in_descend = _is_climb_rate_enabled()
		      && (_vehicle_local_position_setpoint.vz >= land_speed_threshold);
	bool hit_ground = _in_descend && !vertical_movement;

	// TODO: we need an accelerometer based check for vertical movement for flying without GPS
	return (_has_low_thrust() || hit_ground) && (!_horizontal_movement || !_has_position_lock())
	       && (!vertical_movement || !_has_altitude_lock());
}

bool MulticopterLandDetector::_get_maybe_landed_state()
{
	// When not armed, consider to be maybe-landed
	if (!_actuator_armed.armed) {
		return true;
	}

	if (_has_minimal_thrust()) {
		if (_min_trust_start == 0) {
			_min_trust_start = hrt_absolute_time();
		}

	} else {
		_min_trust_start = 0;
	}

	float landThresholdFactor = 1.0f;

	// Widen acceptance thresholds for landed state right after landed
	if (hrt_elapsed_time(&_landed_time) < LAND_DETECTOR_LAND_PHASE_TIME_US) {
		landThresholdFactor = 2.5f;
	}

	// Next look if all rotation angles are not moving.
	float max_rotation_scaled = math::radians(_param_lndmc_rot_max.get()) * landThresholdFactor;

	bool rotating = (fabsf(_vehicle_angular_velocity.xyz[0]) > max_rotation_scaled) ||
			(fabsf(_vehicle_angular_velocity.xyz[1]) > max_rotation_scaled) ||
			(fabsf(_vehicle_angular_velocity.xyz[2]) > max_rotation_scaled);

	// Return status based on armed state and throttle if no position lock is available.
	if (!_has_altitude_lock() && !rotating) {
		// The system has minimum trust set (manual or in failsafe)
		// if this persists for 8 seconds AND the drone is not
		// falling consider it to be landed. This should even sustain
		// quite acrobatic flight.
		return (_min_trust_start > 0) && (hrt_elapsed_time(&_min_trust_start) > 8_s);
	}

	// Ground contact, no thrust and no movement -> landed
	return _ground_contact_hysteresis.get_state() && _has_minimal_thrust() && !rotating;
}

bool MulticopterLandDetector::_get_landed_state()
{
	// When not armed, consider to be landed
	if (!_actuator_armed.armed) {
		return true;
	}

	// reset the landed_time
	if (!_maybe_landed_hysteresis.get_state()) {
		_landed_time = 0;

	} else if (_landed_time == 0) {
		_landed_time = hrt_absolute_time();
	}

	// if we have maybe_landed, the mc_pos_control goes into idle (thrust_sp = 0.0)
	// therefore check if all other condition of the landed state remain true
	return _maybe_landed_hysteresis.get_state();
}

float MulticopterLandDetector::_get_max_altitude()
{
	/* TODO: add a meaningful altitude */
	float valid_altitude_max = _param_lndmc_alt_max.get();

	if (valid_altitude_max < 0.0f) {
		return INFINITY;
	}

	if (_battery_status.warning == battery_status_s::BATTERY_WARNING_LOW) {
		valid_altitude_max = _param_lndmc_alt_max.get() * 0.75f;
	}

	if (_battery_status.warning == battery_status_s::BATTERY_WARNING_CRITICAL) {
		valid_altitude_max = _param_lndmc_alt_max.get() * 0.5f;
	}

	if (_battery_status.warning == battery_status_s::BATTERY_WARNING_EMERGENCY) {
		valid_altitude_max = _param_lndmc_alt_max.get() * 0.25f;
	}

	return valid_altitude_max;
}

bool MulticopterLandDetector::_has_altitude_lock()
{
	return (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 1_s) && _vehicle_local_position.z_valid;
}

bool MulticopterLandDetector::_has_position_lock()
{
	return _has_altitude_lock() && _vehicle_local_position.xy_valid;
}

bool MulticopterLandDetector::_is_climb_rate_enabled()
{
	bool has_updated = (hrt_elapsed_time(&_vehicle_local_position_setpoint.timestamp) < 1_s);

	return (_vehicle_control_mode.flag_control_climb_rate_enabled && has_updated
		&& PX4_ISFINITE(_vehicle_local_position_setpoint.vz));
}

bool MulticopterLandDetector::_has_low_thrust()
{
	// 30% of throttle range between min and hover
	float sys_min_throttle = _params.minThrottle + (_params.hoverThrottle - _params.minThrottle) *
				 _param_lndmc_low_t_thr.get();

	// Check if thrust output is less than the minimum auto throttle param.
	return _actuator_controls.control[actuator_controls_s::INDEX_THROTTLE] <= sys_min_throttle;
}

bool MulticopterLandDetector::_has_minimal_thrust()
{
	// 10% of throttle range between min and hover once we entered ground contact
	float sys_min_throttle = _params.minThrottle + (_params.hoverThrottle - _params.minThrottle) * 0.1f;

	// Determine the system min throttle based on flight mode
	if (!_vehicle_control_mode.flag_control_climb_rate_enabled) {
		sys_min_throttle = (_params.minManThrottle + 0.01f);
	}

	// Check if thrust output is less than the minimum auto throttle param.
	return _actuator_controls.control[actuator_controls_s::INDEX_THROTTLE] <= sys_min_throttle;
}

bool MulticopterLandDetector::_get_ground_effect_state()
{
	return _in_descend && !_horizontal_movement;
}

} // namespace land_detector
