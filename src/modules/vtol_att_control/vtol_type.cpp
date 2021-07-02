/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file vtol_type.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Andreas Antener	<andreas@uaventure.com>
*
*/

#include "vtol_type.h"
#include "vtol_att_control_main.h"

#include <float.h>
#include <px4_platform_common/defines.h>
#include <matrix/math.hpp>

using namespace matrix;


VtolType::VtolType(VtolAttitudeControl *att_controller) :
	_attc(att_controller),
	_vtol_mode(mode::ROTARY_WING)
{
	_v_att = _attc->get_att();
	_v_att_sp = _attc->get_att_sp();
	_mc_virtual_att_sp = _attc->get_mc_virtual_att_sp();
	_fw_virtual_att_sp = _attc->get_fw_virtual_att_sp();
	_v_control_mode = _attc->get_control_mode();
	_vtol_vehicle_status = _attc->get_vtol_vehicle_status();
	_actuators_out_0 = _attc->get_actuators_out0();
	_actuators_out_1 = _attc->get_actuators_out1();
	_actuators_mc_in = _attc->get_actuators_mc_in();
	_actuators_fw_in = _attc->get_actuators_fw_in();
	_local_pos = _attc->get_local_pos();
	_local_pos_sp = _attc->get_local_pos_sp();
	_airspeed_validated = _attc->get_airspeed();
	_tecs_status = _attc->get_tecs_status();
	_land_detected = _attc->get_land_detected();
	_params = _attc->get_params();

	for (auto &pwm_max : _max_mc_pwm_values.values) {
		pwm_max = PWM_DEFAULT_MAX;
	}

	for (auto &pwm_min : _min_mc_pwm_values.values) {
		pwm_min = PWM_DEFAULT_MIN;
	}

	for (auto &pwm_disarmed : _disarmed_pwm_values.values) {
		pwm_disarmed = PWM_MOTOR_OFF;
	}

}

bool VtolType::init()
{
	const char *dev = _params->vt_mc_on_fmu ? PWM_OUTPUT1_DEVICE_PATH : PWM_OUTPUT0_DEVICE_PATH;

	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return false;
	}

	int ret = px4_ioctl(fd, PWM_SERVO_GET_MAX_PWM, (long unsigned int)&_max_mc_pwm_values);


	if (ret != PX4_OK) {
		PX4_ERR("failed getting max values");
		px4_close(fd);
		return false;
	}

	ret = px4_ioctl(fd, PWM_SERVO_GET_MIN_PWM, (long unsigned int)&_min_mc_pwm_values);

	if (ret != PX4_OK) {
		PX4_ERR("failed getting min values");
		px4_close(fd);
		return false;
	}

	ret = px4_ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, (long unsigned int)&_disarmed_pwm_values);

	if (ret != PX4_OK) {
		PX4_ERR("failed getting disarmed values");
		px4_close(fd);
		return false;
	}

	px4_close(fd);

	_main_motor_channel_bitmap = generate_bitmap_from_channel_numbers(_params->vtol_motor_id);
	_alternate_motor_channel_bitmap = generate_bitmap_from_channel_numbers(_params->fw_motors_off);


	// in order to get the main motors we take all motors and clear the alternate motor bits
	for (int i = 0; i < 8; i++) {
		if (_alternate_motor_channel_bitmap & (1 << i)) {
			_main_motor_channel_bitmap &= ~(1 << i);
		}
	}

	_current_max_pwm_values = _max_mc_pwm_values;
	_current_min_pwm_values = _min_mc_pwm_values;

	return true;
}

void VtolType::update_mc_state()
{
	// Set the min/max PWM values for MC mode
	if (!_flag_idle_mc) {
		_flag_idle_mc = set_limits_mc();
	}

	resetAccelToPitchPitchIntegrator();

	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
	_mc_throttle_weight = 1.0f;
}

void VtolType::update_fw_state()
{
	// Set the min/max PWM values for FW mode
	if (_flag_idle_mc) {
		_flag_idle_mc = !set_limits_fw();
	}

	resetAccelToPitchPitchIntegrator();

	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
	_mc_roll_weight = 0.0f;
	_mc_pitch_weight = 0.0f;
	_mc_yaw_weight = 0.0f;

	// tecs didn't publish an update yet after the transition
	if (_tecs_status->timestamp < _trans_finished_ts) {
		_tecs_running = false;

	} else if (!_tecs_running) {
		_tecs_running = true;
		_tecs_running_ts = hrt_absolute_time();
	}

	// TECS didn't publish yet or the position controller didn't publish yet AFTER tecs
	// only wait on TECS we're in a mode where it is actually running
	if ((!_tecs_running || (_tecs_running && _fw_virtual_att_sp->timestamp <= _tecs_running_ts))
	    && _v_control_mode->flag_control_altitude_enabled) {

		waiting_on_tecs();
	}

	check_quadchute_condition();
}

void VtolType::update_transition_state()
{
	hrt_abstime t_now = hrt_absolute_time();
	_transition_dt = (float)(t_now - _last_loop_ts) / 1e6f;
	_transition_dt = math::constrain(_transition_dt, 0.0001f, 0.02f);
	_last_loop_ts = t_now;

	check_quadchute_condition();
}

float VtolType::update_and_get_backtransition_pitch_sp()
{
	// maximum up or down pitch the controller is allowed to demand
	const float pitch_lim = 0.3f;
	const Eulerf euler(Quatf(_v_att->q));

	const float track = atan2f(_local_pos->vy, _local_pos->vx);
	const float accel_body_forward = cosf(track) * _local_pos->ax + sinf(track) * _local_pos->ay;

	// get accel error, positive means decelerating too slow, need to pitch up (must reverse dec_max, as it is a positive number)
	const float accel_error_forward = _params->back_trans_dec_sp + accel_body_forward;

	const float pitch_sp_new = _params->dec_to_pitch_ff * _params->back_trans_dec_sp + _accel_to_pitch_integ;

	float integrator_input = _params->dec_to_pitch_i * accel_error_forward;

	if ((pitch_sp_new >= pitch_lim && accel_error_forward > 0.0f) ||
	    (pitch_sp_new <= -pitch_lim && accel_error_forward < 0.0f)) {
		integrator_input = 0.0f;
	}

	_accel_to_pitch_integ += integrator_input * _transition_dt;


	return math::constrain(pitch_sp_new, -pitch_lim, pitch_lim);
}

bool VtolType::can_transition_on_ground()
{
	return !_v_control_mode->flag_armed || _land_detected->landed;
}

void VtolType::check_quadchute_condition()
{
	if (_attc->get_transition_command() == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC && _attc->get_immediate_transition()
	    && !_quadchute_command_treated) {
		_attc->quadchute("QuadChute by external command");
		_quadchute_command_treated = true;
		_attc->reset_immediate_transition();

	} else {
		_quadchute_command_treated = false;
	}

	if (!_tecs_running) {
		// reset the filtered height rate and heigh rate setpoint if TECS is not running
		_ra_hrate = 0.0f;
		_ra_hrate_sp = 0.0f;
	}

	if (_v_control_mode->flag_armed && !_land_detected->landed) {
		Eulerf euler = Quatf(_v_att->q);

		// fixed-wing minimum altitude
		if (_params->fw_min_alt > FLT_EPSILON) {

			if (-(_local_pos->z) < _params->fw_min_alt) {
				_attc->quadchute("QuadChute: Minimum altitude breached");
			}
		}

		// adaptive quadchute
		if (_params->fw_alt_err > FLT_EPSILON && _v_control_mode->flag_control_altitude_enabled) {

			// We use tecs for tracking in FW and local_pos_sp during transitions
			if (_tecs_running) {
				// 1 second rolling average
				_ra_hrate = (49 * _ra_hrate + _tecs_status->height_rate) / 50;
				_ra_hrate_sp = (49 * _ra_hrate_sp + _tecs_status->height_rate_setpoint) / 50;

				// are we dropping while requesting significant ascend?
				if (((_tecs_status->altitude_sp - _tecs_status->altitude_filtered) > _params->fw_alt_err) &&
				    (_ra_hrate < -1.0f) &&
				    (_ra_hrate_sp > 1.0f)) {

					_attc->quadchute("QuadChute: loss of altitude");
				}

			} else {
				const bool height_error = _local_pos->z_valid && ((-_local_pos_sp->z - -_local_pos->z) > _params->fw_alt_err);
				const bool height_rate_error = _local_pos->v_z_valid && (_local_pos->vz > 1.0f) && (_local_pos->z_deriv > 1.0f);

				if (height_error && height_rate_error) {
					_attc->quadchute("QuadChute: large altitude error");
				}
			}
		}

		// fixed-wing maximum pitch angle
		if (_params->fw_qc_max_pitch > 0) {

			if (fabsf(euler.theta()) > fabsf(math::radians(_params->fw_qc_max_pitch))) {
				_attc->quadchute("Maximum pitch angle exceeded");
			}
		}

		// fixed-wing maximum roll angle
		if (_params->fw_qc_max_roll > 0) {

			if (fabsf(euler.phi()) > fabsf(math::radians(_params->fw_qc_max_roll))) {
				_attc->quadchute("Maximum roll angle exceeded");
			}
		}
	}
}

bool VtolType::set_limits_mc()
{
	// Enabale all VTOL motors for rotary wing flight
	set_all_motor_state(motor_state::ENABLED);

	return true;
}

bool VtolType::set_limits_fw()
{
	// Enable all "main" motors, and set all "alt" motors to FW idle
	set_main_motor_state(motor_state::ENABLED);
	set_alternate_motor_state(motor_state::IDLE);

	return true;
}

bool VtolType::apply_pwm_limits(struct pwm_output_values &pwm_values, pwm_limit_type type)
{
	const char *dev = _params->vt_mc_on_fmu ? PWM_OUTPUT1_DEVICE_PATH : PWM_OUTPUT0_DEVICE_PATH;

	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
		return false;
	}

	int ret;

	if (type == pwm_limit_type::TYPE_MINIMUM) {
		ret = px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);

	} else {
		ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);
	}

	px4_close(fd);


	if (ret != OK) {
		PX4_DEBUG("failed setting max values");
		return false;
	}

	return true;
}

void VtolType::set_all_motor_state(const motor_state target_state, const int value)
{
	set_main_motor_state(target_state, value);
	set_alternate_motor_state(target_state, value);
}

void VtolType::set_main_motor_state(const motor_state target_state, const int value)
{
	if (_main_motor_state != target_state) {

		if (set_motor_state(target_state, _main_motor_channel_bitmap, value)) {
			_main_motor_state = target_state;
		}
	}
}

void VtolType::set_alternate_motor_state(const motor_state target_state, const int value)
{
	if (_alternate_motor_state != target_state) {

		if (set_motor_state(target_state, _alternate_motor_channel_bitmap, value)) {
			_alternate_motor_state = target_state;
		}
	}
}

bool VtolType::set_motor_state(const motor_state target_state, const int32_t channel_bitmap,  const int value)
{
	switch (target_state) {
	case motor_state::ENABLED:

		// Motors are fully enabled: Use the PWM_{MAIN,AUX}_{MIN,MAX}{i} param values
		for (int i = 0; i < num_outputs_max; i++) {
			if (is_channel_set(i, channel_bitmap)) {
				_current_max_pwm_values.values[i] = _max_mc_pwm_values.values[i];
				_current_min_pwm_values.values[i] = _min_mc_pwm_values.values[i];
			}
		}

		break;

	case motor_state::DISABLED:

		// Motors are fully disabled: Use the PWM_{MAIN,AUX}_DIS{i} param values
		for (int i = 0; i < num_outputs_max; i++) {
			if (is_channel_set(i, channel_bitmap)) {
				_current_max_pwm_values.values[i] = _disarmed_pwm_values.values[i];
				_current_min_pwm_values.values[i] = _disarmed_pwm_values.values[i];
			}
		}

		break;

	case motor_state::IDLE:

		// Motors are off, but not disarmed: Use the VT_IDLE_PWM_FW param value
		for (int i = 0; i < num_outputs_max; i++) {
			if (is_channel_set(i, channel_bitmap)) {
				_current_max_pwm_values.values[i] = _params->idle_pwm_fw;
				_current_min_pwm_values.values[i] = _params->idle_pwm_fw;
			}
		}

		break;

	case motor_state::VALUE:

		// Motors are enabled, but set the max PWM to <value>
		for (int i = 0; i < num_outputs_max; i++) {
			if (is_channel_set(i, channel_bitmap)) {
				_current_max_pwm_values.values[i] = value;
				_current_min_pwm_values.values[i] = _min_mc_pwm_values.values[i];
			}
		}

		break;
	}

	_current_max_pwm_values.channel_count = num_outputs_max;
	_current_min_pwm_values.channel_count = num_outputs_max;

	const int res_max = apply_pwm_limits(_current_max_pwm_values, pwm_limit_type::TYPE_MAXIMUM);
	const int res_min = apply_pwm_limits(_current_min_pwm_values, pwm_limit_type::TYPE_MINIMUM);

	return res_max && res_min;
}

int VtolType::generate_bitmap_from_channel_numbers(const int channels)
{
	int channel_bitmap = 0;
	int channel_numbers = channels;

	int tmp;

	for (int i = 0; i < num_outputs_max; ++i) {
		tmp = channel_numbers % 10;

		if (tmp == 0) {
			break;
		}

		channel_bitmap |= 1 << (tmp - 1);
		channel_numbers = channel_numbers / 10;
	}

	return channel_bitmap;
}

bool VtolType::is_channel_set(const int channel, const int bitmap)
{
	return bitmap & (1 << channel);
}


float VtolType::pusher_assist()
{
	// Altitude above ground is distance sensor altitude if available, otherwise local z-position
	float dist_to_ground = -_local_pos->z;

	if (_local_pos->dist_bottom_valid) {
		dist_to_ground = _local_pos->dist_bottom;
	}

	// disable pusher assist depending on setting of forward_thrust_enable_mode:
	switch (_params->vt_forward_thrust_enable_mode) {
	case DISABLE: // disable in all modes
		return 0.0f;
		break;

	case ENABLE_WITHOUT_LAND: // disable in land mode
		if (_attc->get_pos_sp_triplet()->current.valid
		    && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
		    && _v_control_mode->flag_control_auto_enabled) {
			return 0.0f;
		}

		break;

	case ENABLE_ABOVE_MPC_LAND_ALT1: // disable if below MPC_LAND_ALT1
		if (!PX4_ISFINITE(dist_to_ground) || (dist_to_ground < _params->mpc_land_alt1)) {
			return 0.0f;
		}

		break;

	case ENABLE_ABOVE_MPC_LAND_ALT2: // disable if below MPC_LAND_ALT2
		if (!PX4_ISFINITE(dist_to_ground) || (dist_to_ground < _params->mpc_land_alt2)) {
			return 0.0f;
		}

		break;

	case ENABLE_ABOVE_MPC_LAND_ALT1_WITHOUT_LAND: // disable if below MPC_LAND_ALT1 or in land mode
		if ((_attc->get_pos_sp_triplet()->current.valid
		     && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
		     && _v_control_mode->flag_control_auto_enabled) ||
		    (!PX4_ISFINITE(dist_to_ground) || (dist_to_ground < _params->mpc_land_alt1))) {
			return 0.0f;
		}

		break;

	case ENABLE_ABOVE_MPC_LAND_ALT2_WITHOUT_LAND: // disable if below MPC_LAND_ALT2 or in land mode
		if ((_attc->get_pos_sp_triplet()->current.valid
		     && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
		     && _v_control_mode->flag_control_auto_enabled) ||
		    (!PX4_ISFINITE(dist_to_ground) || (dist_to_ground < _params->mpc_land_alt2))) {
			return 0.0f;
		}

		break;
	}

	// if the thrust scale param is zero or the drone is not in some position or altitude control mode,
	// then the pusher-for-pitch strategy is disabled and we can return
	if (_params->forward_thrust_scale < FLT_EPSILON || !(_v_control_mode->flag_control_position_enabled
			|| _v_control_mode->flag_control_altitude_enabled)) {
		return 0.0f;
	}

	// Do not engage pusher assist during a failsafe event (could be a problem with the fixed wing drive)
	if (_attc->get_vtol_vehicle_status()->vtol_transition_failsafe) {
		return 0.0f;
	}

	const Dcmf R(Quatf(_v_att->q));
	const Dcmf R_sp(Quatf(_v_att_sp->q_d));
	const Eulerf euler(R);
	const Eulerf euler_sp(R_sp);

	// direction of desired body z axis represented in earth frame
	Vector3f body_z_sp(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	// rotate desired body z axis into new frame which is rotated in z by the current
	// heading of the vehicle. we refer to this as the heading frame.
	Dcmf R_yaw = Eulerf(0.0f, 0.0f, -euler(2));
	body_z_sp = R_yaw * body_z_sp;
	body_z_sp.normalize();

	// calculate the desired pitch seen in the heading frame
	// this value corresponds to the amount the vehicle would try to pitch down
	float pitch_down = atan2f(body_z_sp(0), body_z_sp(2));

	// normalized pusher support throttle (standard VTOL) or tilt (tiltrotor), initialize to 0
	float forward_thrust = 0.0f;

	// only allow pitching down up to threshold, the rest of the desired
	// forward acceleration will be compensated by the pusher/tilt
	if (pitch_down < -_params->down_pitch_max) {
		// desired roll angle in heading frame stays the same
		float roll_new = -asinf(body_z_sp(1));

		forward_thrust = (sinf(-pitch_down) - sinf(_params->down_pitch_max)) * _params->forward_thrust_scale;
		// limit forward actuation to [0, 0.9]
		forward_thrust = math::constrain(forward_thrust, 0.0f, 0.9f);

		// return the vehicle to level position
		float pitch_new = 0.0f;

		// create corrected desired body z axis in heading frame
		const Dcmf R_tmp = Eulerf(roll_new, pitch_new, 0.0f);
		Vector3f tilt_new(R_tmp(0, 2), R_tmp(1, 2), R_tmp(2, 2));

		// rotate the vector into a new frame which is rotated in z by the desired heading
		// with respect to the earh frame.
		const float yaw_error = wrap_pi(euler_sp(2) - euler(2));
		const Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
		tilt_new = R_yaw_correction * tilt_new;

		// now extract roll and pitch setpoints
		_v_att_sp->pitch_body = atan2f(tilt_new(0), tilt_new(2));
		_v_att_sp->roll_body = -asinf(tilt_new(1));

		const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, euler_sp(2)));
		q_sp.copyTo(_v_att_sp->q_d);
	}

	return forward_thrust;

}
