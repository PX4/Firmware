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
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

using namespace matrix;
using namespace time_literals;

#define ARSP_YAW_CTRL_DISABLE 7.0f	// airspeed at which we stop controlling yaw during a front transition

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tiltrotor.tilt_mc = param_find("VT_TILT_MC");
	_params_handles_tiltrotor.tilt_transition = param_find("VT_TILT_TRANS");
	_params_handles_tiltrotor.tilt_fw = param_find("VT_TILT_FW");
	_params_handles_tiltrotor.tilt_spinup = param_find("VT_TILT_SPINUP");
	_params_handles_tiltrotor.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR");
}

void
Tiltrotor::parameters_update()
{
	float v;

	/* vtol tilt mechanism position in mc mode */
	param_get(_params_handles_tiltrotor.tilt_mc, &v);
	_params_tiltrotor.tilt_mc = v;

	/* vtol tilt mechanism position in transition mode */
	param_get(_params_handles_tiltrotor.tilt_transition, &v);
	_params_tiltrotor.tilt_transition = v;

	/* vtol tilt mechanism position in fw mode */
	param_get(_params_handles_tiltrotor.tilt_fw, &v);
	_params_tiltrotor.tilt_fw = v;

	/* vtol tilt mechanism position during motor spinup */
	param_get(_params_handles_tiltrotor.tilt_spinup, &v);
	_params_tiltrotor.tilt_spinup = v;

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tiltrotor.front_trans_dur_p2, &v);
	_params_tiltrotor.front_trans_dur_p2 = v;
}

void Tiltrotor::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting rotors, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors are tilted
	 * forward completely. For the backtransition the motors simply rotate back.
	*/

	if (_vtol_vehicle_status->vtol_transition_failsafe) {
		// Failsafe event, switch to MC mode immediately
		_vtol_schedule.flight_mode = vtol_mode::MC_MODE;

		//reset failsafe when FW is no longer requested
		if (!_attc->is_fixed_wing_requested()) {
			_vtol_vehicle_status->vtol_transition_failsafe = false;
		}

	} else 	if (!_attc->is_fixed_wing_requested()) {

		// plane is in multicopter mode
		switch (_vtol_schedule.flight_mode) {
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_FRONT_P2:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			const float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;
			const float ground_speed = sqrtf(_local_pos->vx * _local_pos->vx + _local_pos->vy * _local_pos->vy);
			const bool ground_speed_below_cruise = _local_pos->v_xy_valid && (ground_speed <= _params->mpc_xy_cruise);

			if (_tilt_control <= _params_tiltrotor.tilt_mc && (time_since_trans_start > _params->back_trans_duration
					|| ground_speed_below_cruise)) {
				_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {

		switch (_vtol_schedule.flight_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {

				float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

				const bool airspeed_triggers_transition = PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)
						&& !_params->airspeed_disabled;

				bool transition_to_p2 = false;

				if (time_since_trans_start > _params->front_trans_time_min) {
					if (airspeed_triggers_transition) {
						transition_to_p2 = _airspeed_validated->calibrated_airspeed_m_s >= _params->transition_airspeed;

					} else {
						transition_to_p2 = _tilt_control >= _params_tiltrotor.tilt_transition &&
								   time_since_trans_start > _params->front_trans_time_openloop;;
					}
				}

				transition_to_p2 |= can_transition_on_ground();

				if (transition_to_p2) {
					_vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P2;
					_vtol_schedule.transition_start = hrt_absolute_time();
				}

				break;
			}

		case vtol_mode::TRANSITION_FRONT_P2:

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _params_tiltrotor.tilt_fw) {
				_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
				_tilt_control = _params_tiltrotor.tilt_fw;
			}

			break;

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
			break;
		}
	}

	// map tiltrotor specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case vtol_mode::MC_MODE:
		_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
	case vtol_mode::TRANSITION_FRONT_P2:
		_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	/*Motor spin up: define the first second after arming as motor spin up time, during which
	* the tilt is set to the value of VT_TILT_SPINUP. This allowes the user to set a spin up
	* tilt angle in case the propellers don't spin up smootly in full upright (MC mode) position.
	*/

	const int spin_up_duration_p1 = 1000_ms; // duration of 1st phase of spinup (at fixed tilt)
	const int spin_up_duration_p2 = 700_ms; // duration of 2nd phase of spinup (transition from spinup tilt to mc tilt)

	// reset this timestamp while disarmed
	if (!_v_control_mode->flag_armed) {
		_last_timestamp_disarmed = hrt_absolute_time();
		_tilt_motors_for_startup = _params_tiltrotor.tilt_spinup > 0.01f; // spinup phase only required if spinup tilt > 0

	} else if (_tilt_motors_for_startup) {
		// leave motors tilted forward after arming to allow them to spin up easier
		if (hrt_absolute_time() - _last_timestamp_disarmed > (spin_up_duration_p1 + spin_up_duration_p2)) {
			_tilt_motors_for_startup = false;
		}
	}

	if (_tilt_motors_for_startup) {
		if (hrt_absolute_time() - _last_timestamp_disarmed < spin_up_duration_p1) {
			_tilt_control = _params_tiltrotor.tilt_spinup;

		} else {
			// duration phase 2: begin to adapt tilt to multicopter tilt
			float delta_tilt = (_params_tiltrotor.tilt_mc - _params_tiltrotor.tilt_spinup);
			_tilt_control = _params_tiltrotor.tilt_spinup + delta_tilt / spin_up_duration_p2 * (hrt_absolute_time() -
					(_last_timestamp_disarmed + spin_up_duration_p1));
		}

		_mc_yaw_weight = 0.0f; //disable yaw control during spinup

	} else {
		// normal operation
		_tilt_control = VtolType::pusher_assist();
		_mc_yaw_weight = 1.0f;
		_v_att_sp->thrust_body[2] = Tiltrotor::thrust_compensation_for_tilt();
	}

}

void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state();

	// make sure motors are tilted forward
	_tilt_control = _params_tiltrotor.tilt_fw;
}

void Tiltrotor::update_transition_state()
{
	VtolType::update_transition_state();

	// copy virtual attitude setpoint to real attitude setpoint (we use multicopter att sp)
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	_v_att_sp->roll_body = _fw_virtual_att_sp->roll_body;

	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_flag_was_in_trans_mode = true;
	}

	if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P1) {
		// for the first part of the transition all rotors are enabled
		set_all_motor_state(motor_state::ENABLED);

		// tilt rotors forward up to certain angle
		if (_tilt_control <= _params_tiltrotor.tilt_transition) {
			_tilt_control = _params_tiltrotor.tilt_mc +
					fabsf(_params_tiltrotor.tilt_transition - _params_tiltrotor.tilt_mc) * time_since_trans_start /
					_params->front_trans_duration;
		}


		// at low speeds give full weight to MC
		_mc_roll_weight = 1.0f;
		_mc_yaw_weight = 1.0f;

		// reduce MC controls once the plane has picked up speed
		if (!_params->airspeed_disabled && PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s) &&
		    _airspeed_validated->calibrated_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;
		}

		if (!_params->airspeed_disabled && PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s) &&
		    _airspeed_validated->calibrated_airspeed_m_s >= _params->airspeed_blend) {
			_mc_roll_weight = 1.0f - (_airspeed_validated->calibrated_airspeed_m_s - _params->airspeed_blend) /
					  (_params->transition_airspeed - _params->airspeed_blend);
		}

		// without airspeed do timed weight changes
		if ((_params->airspeed_disabled || !PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) &&
		    time_since_trans_start > _params->front_trans_time_min) {
			_mc_roll_weight = 1.0f - (time_since_trans_start - _params->front_trans_time_min) /
					  (_params->front_trans_time_openloop - _params->front_trans_time_min);
			_mc_yaw_weight = _mc_roll_weight;
		}

		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

		// in stabilized, acro or manual mode, set the MC thrust to the throttle stick position (coming from the FW attitude setpoint)
		if (!_v_control_mode->flag_control_climb_rate_enabled) {
			_v_att_sp->thrust_body[2] = -_fw_virtual_att_sp->thrust_body[0];
		}

	} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = _params_tiltrotor.tilt_transition +
				fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_transition) * time_since_trans_start /
				_params_tiltrotor.front_trans_dur_p2;

		_mc_roll_weight = 0.0f;
		_mc_yaw_weight = 0.0f;

		// ramp down motors not used in fixed-wing flight (setting MAX_PWM down scales the given output into the new range)
		int ramp_down_value = (1.0f - time_since_trans_start / _params_tiltrotor.front_trans_dur_p2) *
				      (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + PWM_DEFAULT_MIN;


		set_alternate_motor_state(motor_state::VALUE, ramp_down_value);


		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

		// in stabilized, acro or manual mode, set the MC thrust to the throttle stick position (coming from the FW attitude setpoint)
		if (!_v_control_mode->flag_control_climb_rate_enabled) {
			_v_att_sp->thrust_body[2] = -_fw_virtual_att_sp->thrust_body[0];
		}

	} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_BACK) {
		// turn on all MC motors
		set_all_motor_state(motor_state::ENABLED);


		// set idle speed for rotary wing mode
		if (!_flag_idle_mc) {
			_flag_idle_mc = set_idle_mc();
		}

		// tilt rotors back
		if (_tilt_control > _params_tiltrotor.tilt_mc) {
			_tilt_control = _params_tiltrotor.tilt_fw -
					fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_mc) * time_since_trans_start / 1.0f;
		}

		_mc_yaw_weight = 1.0f;

		// control backtransition deceleration using pitch.
		if (_v_control_mode->flag_control_climb_rate_enabled) {
			_v_att_sp->pitch_body = update_and_get_backtransition_pitch_sp();
		}

		// while we quickly rotate back the motors keep throttle at idle
		if (time_since_trans_start < 1.0f) {
			_mc_throttle_weight = 0.0f;
			_mc_roll_weight = 0.0f;
			_mc_pitch_weight = 0.0f;

		} else {
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			// slowly ramp up throttle to avoid step inputs
			_mc_throttle_weight = (time_since_trans_start - 1.0f) / 1.0f;
		}

		// in stabilized, acro or manual mode, set the MC thrust to the throttle stick position (coming from the FW attitude setpoint)
		if (!_v_control_mode->flag_control_climb_rate_enabled) {
			_v_att_sp->thrust_body[2] = -_fw_virtual_att_sp->thrust_body[0];
		}
	}

	const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
	q_sp.copyTo(_v_att_sp->q_d);

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_throttle_weight = math::constrain(_mc_throttle_weight, 0.0f, 1.0f);
}

void Tiltrotor::waiting_on_tecs()
{
	// keep multicopter thrust until we get data from TECS
	_v_att_sp->thrust_body[0] = _thrust_transition;
}

/**
* Write data to actuator output topic.
*/
void Tiltrotor::fill_actuator_outputs()
{
	auto &mc_in = _actuators_mc_in->control;
	auto &fw_in = _actuators_fw_in->control;

	auto &mc_out = _actuators_out_0->control;
	auto &fw_out = _actuators_out_1->control;

	// Multirotor output
	mc_out[actuator_controls_s::INDEX_ROLL]  = mc_in[actuator_controls_s::INDEX_ROLL]  * _mc_roll_weight;
	mc_out[actuator_controls_s::INDEX_PITCH] = mc_in[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	mc_out[actuator_controls_s::INDEX_YAW]   = mc_in[actuator_controls_s::INDEX_YAW]   * _mc_yaw_weight;

	if (_vtol_schedule.flight_mode == vtol_mode::FW_MODE) {
		mc_out[actuator_controls_s::INDEX_THROTTLE] = fw_in[actuator_controls_s::INDEX_THROTTLE];

		/* allow differential thrust if enabled */
		if (_params->diff_thrust == 1) {
			mc_out[actuator_controls_s::INDEX_ROLL] = fw_in[actuator_controls_s::INDEX_YAW] * _params->diff_thrust_scale;
		}

	} else {
		mc_out[actuator_controls_s::INDEX_THROTTLE] = mc_in[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;
	}

	// Fixed wing output
	fw_out[4] = _tilt_control;

	if (_params->elevons_mc_lock && _vtol_schedule.flight_mode == vtol_mode::MC_MODE) {
		fw_out[actuator_controls_s::INDEX_ROLL]  = 0;
		fw_out[actuator_controls_s::INDEX_PITCH] = 0;
		fw_out[actuator_controls_s::INDEX_YAW]   = 0;

	} else {
		fw_out[actuator_controls_s::INDEX_ROLL]  = fw_in[actuator_controls_s::INDEX_ROLL];
		fw_out[actuator_controls_s::INDEX_PITCH] = fw_in[actuator_controls_s::INDEX_PITCH];
		fw_out[actuator_controls_s::INDEX_YAW]   = fw_in[actuator_controls_s::INDEX_YAW];
	}

	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	_actuators_out_0->timestamp = _actuators_out_1->timestamp = hrt_absolute_time();
}

/*
 * Increase combined thrust of MC propellers if motors are tilted. Assumes that all MC motors are tilted equally.
 */

float Tiltrotor::thrust_compensation_for_tilt()
{
	// only compensate for tilt angle up to 0.5 * max tilt
	float compensated_tilt = math::constrain(_tilt_control, 0.0f, 0.5f);

	// increase vertical thrust by 1/cos(tilt), limmit to [-1,0]
	return math::constrain(_v_att_sp->thrust_body[2] / cosf(compensated_tilt * M_PI_2_F), -1.0f, 0.0f);
}
