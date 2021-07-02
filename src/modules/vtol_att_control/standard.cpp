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
 * @file standard.cpp
 *
 * @author Simon Wilks		<simon@uaventure.com>
 * @author Roman Bapst		<bapstroman@gmail.com>
 * @author Andreas Antener	<andreas@uaventure.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
*/

#include "standard.h"
#include "vtol_att_control_main.h"

#include <float.h>

using namespace matrix;

Standard::Standard(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
	_vtol_schedule.transition_start = 0;
	_pusher_active = false;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
	_mc_throttle_weight = 1.0f;

	_params_handles_standard.pusher_ramp_dt = param_find("VT_PSHER_RMP_DT");
	_params_handles_standard.back_trans_ramp = param_find("VT_B_TRANS_RAMP");
	_params_handles_standard.pitch_setpoint_offset = param_find("FW_PSP_OFF");
	_params_handles_standard.reverse_output = param_find("VT_B_REV_OUT");
	_params_handles_standard.reverse_delay = param_find("VT_B_REV_DEL");
}

void
Standard::parameters_update()
{
	float v;

	/* duration of a forwards transition to fw mode */
	param_get(_params_handles_standard.pusher_ramp_dt, &v);
	_params_standard.pusher_ramp_dt = math::constrain(v, 0.0f, 20.0f);

	/* MC ramp up during back transition to mc mode */
	param_get(_params_handles_standard.back_trans_ramp, &v);
	_params_standard.back_trans_ramp = math::constrain(v, 0.0f, _params->back_trans_duration);

	_airspeed_trans_blend_margin = _params->transition_airspeed - _params->airspeed_blend;

	/* pitch setpoint offset */
	param_get(_params_handles_standard.pitch_setpoint_offset, &v);
	_params_standard.pitch_setpoint_offset = math::radians(v);

	/* reverse output */
	param_get(_params_handles_standard.reverse_output, &v);
	_params_standard.reverse_output = math::constrain(v, 0.0f, 1.0f);

	/* reverse output */
	param_get(_params_handles_standard.reverse_delay, &v);
	_params_standard.reverse_delay = math::constrain(v, 0.0f, 10.0f);

}

void Standard::update_vtol_state()
{
	/* After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors shutdown.
	 * For the back transition the pusher motor is immediately stopped and rotors reactivated.
	 */

	float mc_weight = _mc_roll_weight;
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	if (_vtol_vehicle_status->vtol_transition_failsafe) {
		// Failsafe event, engage mc motors immediately
		_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
		_pusher_throttle = 0.0f;
		_reverse_output = 0.0f;

		//reset failsafe when FW is no longer requested
		if (!_attc->is_fixed_wing_requested()) {
			_vtol_vehicle_status->vtol_transition_failsafe = false;
		}

	} else if (!_attc->is_fixed_wing_requested()) {

		// the transition to fw mode switch is off
		if (_vtol_schedule.flight_mode == vtol_mode::MC_MODE) {
			// in mc mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			mc_weight = 1.0f;
			_pusher_throttle = 0.0f;
			_reverse_output = 0.0f;

		} else if (_vtol_schedule.flight_mode == vtol_mode::FW_MODE) {
			// Regular backtransition
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_TO_MC;
			_vtol_schedule.transition_start = hrt_absolute_time();
			_reverse_output = _params_standard.reverse_output;

		} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			mc_weight = 1.0f;
			_pusher_throttle = 0.0f;
			_reverse_output = 0.0f;

		} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_TO_MC) {
			// transition to MC mode if transition time has passed or forward velocity drops below MPC cruise speed

			const Dcmf R_to_body(Quatf(_v_att->q).inversed());
			const Vector3f vel = R_to_body * Vector3f(_local_pos->vx, _local_pos->vy, _local_pos->vz);

			float x_vel = vel(0);

			if (time_since_trans_start > _params->back_trans_duration ||
			    (_local_pos->v_xy_valid && x_vel <= _params->mpc_xy_cruise) ||
			    can_transition_on_ground()) {
				_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			}
		}

	} else {
		// the transition to fw mode switch is on
		if (_vtol_schedule.flight_mode == vtol_mode::MC_MODE || _vtol_schedule.flight_mode == vtol_mode::TRANSITION_TO_MC) {
			// start transition to fw mode
			/* NOTE: The failsafe transition to fixed-wing was removed because it can result in an
			 * unsafe flying state. */
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_TO_FW;
			_vtol_schedule.transition_start = hrt_absolute_time();

		} else if (_vtol_schedule.flight_mode == vtol_mode::FW_MODE) {
			// in fw mode
			_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
			mc_weight = 0.0f;

		} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_TO_FW) {
			// continue the transition to fw mode while monitoring airspeed for a final switch to fw mode

			const bool airspeed_triggers_transition = PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)
					&& !_params->airspeed_disabled;
			const bool minimum_trans_time_elapsed = time_since_trans_start > _params->front_trans_time_min;

			bool transition_to_fw = false;

			if (minimum_trans_time_elapsed) {
				if (airspeed_triggers_transition) {
					transition_to_fw = _airspeed_validated->calibrated_airspeed_m_s >= _params->transition_airspeed;

				} else {
					transition_to_fw = true;
				}
			}

			transition_to_fw |= can_transition_on_ground();

			if (transition_to_fw) {
				_vtol_schedule.flight_mode = vtol_mode::FW_MODE;

				// don't set pusher throttle here as it's being ramped up elsewhere
				_trans_finished_ts = hrt_absolute_time();
			}
		}
	}

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;

	// map specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case vtol_mode::MC_MODE:
		_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_TO_FW:
		_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_TO_MC:
		_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Standard::update_transition_state()
{
	float mc_weight = 1.0f;
	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	VtolType::update_transition_state();

	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_TO_FW) {
		if (_params_standard.pusher_ramp_dt <= 0.0f) {
			// just set the final target throttle value
			_pusher_throttle = _params->front_trans_throttle;

		} else if (_pusher_throttle <= _params->front_trans_throttle) {
			// ramp up throttle to the target throttle value
			_pusher_throttle = _params->front_trans_throttle * time_since_trans_start / _params_standard.pusher_ramp_dt;
		}

		// do blending of mc and fw controls if a blending airspeed has been provided and the minimum transition time has passed
		if (_airspeed_trans_blend_margin > 0.0f &&
		    PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s) &&
		    _airspeed_validated->calibrated_airspeed_m_s > 0.0f &&
		    _airspeed_validated->calibrated_airspeed_m_s >= _params->airspeed_blend &&
		    time_since_trans_start > _params->front_trans_time_min) {

			mc_weight = 1.0f - fabsf(_airspeed_validated->calibrated_airspeed_m_s - _params->airspeed_blend) /
				    _airspeed_trans_blend_margin;
			// time based blending when no airspeed sensor is set

		} else if (_params->airspeed_disabled || !PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) {
			mc_weight = 1.0f - time_since_trans_start / _params->front_trans_time_min;
			mc_weight = math::constrain(2.0f * mc_weight, 0.0f, 1.0f);

		}

		// ramp up FW_PSP_OFF
		_v_att_sp->pitch_body = _params_standard.pitch_setpoint_offset * (1.0f - mc_weight);

		_v_att_sp->roll_body = _fw_virtual_att_sp->roll_body;

		// in stabilized, acro or manual mode, set the MC thrust to the throttle stick position (coming from the FW attitude setpoint)
		if (!_v_control_mode->flag_control_climb_rate_enabled) {
			_v_att_sp->thrust_body[2] = -_fw_virtual_att_sp->thrust_body[0];
		}

		const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);

		// check front transition timeout
		if (_params->front_trans_timeout > FLT_EPSILON) {
			if (time_since_trans_start > _params->front_trans_timeout) {
				// transition timeout occured, abort transition
				_attc->quadchute("Transition timeout");
			}
		}

	} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_TO_MC) {

		_v_att_sp->roll_body = _fw_virtual_att_sp->roll_body;

		if (_v_control_mode->flag_control_climb_rate_enabled) {
			// control backtransition deceleration using pitch.
			_v_att_sp->pitch_body = update_and_get_backtransition_pitch_sp();
		}

		// in stabilized, acro or manual mode, set the MC thrust to the throttle stick position (coming from the FW attitude setpoint)
		if (!_v_control_mode->flag_control_climb_rate_enabled) {
			_v_att_sp->thrust_body[2] = -_fw_virtual_att_sp->thrust_body[0];
		}

		const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);

		_pusher_throttle = 0.0f;

		if (time_since_trans_start >= _params_standard.reverse_delay) {
			// Handle throttle reversal for active breaking
			float thrscale = (time_since_trans_start - _params_standard.reverse_delay) / (_params_standard.pusher_ramp_dt);
			thrscale = math::constrain(thrscale, 0.0f, 1.0f);
			_pusher_throttle = thrscale * _params->back_trans_throttle;
		}

		// continually increase mc attitude control as we transition back to mc mode
		if (_params_standard.back_trans_ramp > FLT_EPSILON) {
			mc_weight = time_since_trans_start / _params_standard.back_trans_ramp;

		}

		set_all_motor_state(motor_state::ENABLED);

		// reset min/max PWM values for MC actuators for MC flight
		if (!_flag_idle_mc) {
			_flag_idle_mc = set_limits_mc();
		}
	}

	mc_weight = math::constrain(mc_weight, 0.0f, 1.0f);

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;
}

void Standard::update_mc_state()
{
	VtolType::update_mc_state();

	_pusher_throttle = VtolType::pusher_assist();
}

void Standard::update_fw_state()
{
	VtolType::update_fw_state();
}

/**
 * Prepare message to acutators with data from mc and fw attitude controllers. An mc attitude weighting will determine
 * what proportion of control should be applied to each of the control groups (mc and fw).
 */
void Standard::fill_actuator_outputs()
{
	auto &mc_in = _actuators_mc_in->control;
	auto &fw_in = _actuators_fw_in->control;

	auto &mc_out = _actuators_out_0->control;
	auto &fw_out = _actuators_out_1->control;

	const bool elevon_lock = (_params->elevons_mc_lock == 1);

	switch (_vtol_schedule.flight_mode) {
	case vtol_mode::MC_MODE:

		// MC out = MC in
		mc_out[actuator_controls_s::INDEX_ROLL]         = mc_in[actuator_controls_s::INDEX_ROLL];
		mc_out[actuator_controls_s::INDEX_PITCH]        = mc_in[actuator_controls_s::INDEX_PITCH];
		mc_out[actuator_controls_s::INDEX_YAW]          = mc_in[actuator_controls_s::INDEX_YAW];
		mc_out[actuator_controls_s::INDEX_THROTTLE]     = mc_in[actuator_controls_s::INDEX_THROTTLE];
		mc_out[actuator_controls_s::INDEX_LANDING_GEAR] = mc_in[actuator_controls_s::INDEX_LANDING_GEAR];

		// FW out = 0, other than roll and pitch depending on elevon lock
		fw_out[actuator_controls_s::INDEX_ROLL]         = elevon_lock ? 0 : fw_in[actuator_controls_s::INDEX_ROLL];
		fw_out[actuator_controls_s::INDEX_PITCH]        = elevon_lock ? 0 : fw_in[actuator_controls_s::INDEX_PITCH];
		fw_out[actuator_controls_s::INDEX_YAW]          = 0;
		fw_out[actuator_controls_s::INDEX_THROTTLE]     = _pusher_throttle;
		fw_out[actuator_controls_s::INDEX_FLAPS]        = 0;
		fw_out[actuator_controls_s::INDEX_AIRBRAKES]    = 0;

		break;

	case vtol_mode::TRANSITION_TO_FW:

	// FALLTHROUGH
	case vtol_mode::TRANSITION_TO_MC:
		// MC out = MC in (weighted)
		mc_out[actuator_controls_s::INDEX_ROLL]         = mc_in[actuator_controls_s::INDEX_ROLL]     * _mc_roll_weight;
		mc_out[actuator_controls_s::INDEX_PITCH]        = mc_in[actuator_controls_s::INDEX_PITCH]    * _mc_pitch_weight;
		mc_out[actuator_controls_s::INDEX_YAW]          = mc_in[actuator_controls_s::INDEX_YAW]      * _mc_yaw_weight;
		mc_out[actuator_controls_s::INDEX_THROTTLE]     = mc_in[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;
		mc_out[actuator_controls_s::INDEX_LANDING_GEAR] = 0;

		// FW out = FW in, with VTOL transition controlling throttle and airbrakes
		fw_out[actuator_controls_s::INDEX_ROLL]         = fw_in[actuator_controls_s::INDEX_ROLL];
		fw_out[actuator_controls_s::INDEX_PITCH]        = fw_in[actuator_controls_s::INDEX_PITCH];
		fw_out[actuator_controls_s::INDEX_YAW]          = fw_in[actuator_controls_s::INDEX_YAW];
		fw_out[actuator_controls_s::INDEX_THROTTLE]     = _pusher_throttle;
		fw_out[actuator_controls_s::INDEX_FLAPS]        = fw_in[actuator_controls_s::INDEX_FLAPS];
		fw_out[actuator_controls_s::INDEX_AIRBRAKES]    = _reverse_output;

		break;

	case vtol_mode::FW_MODE:
		// MC out = 0
		mc_out[actuator_controls_s::INDEX_ROLL]         = 0;
		mc_out[actuator_controls_s::INDEX_PITCH]        = 0;
		mc_out[actuator_controls_s::INDEX_YAW]          = 0;
		mc_out[actuator_controls_s::INDEX_THROTTLE]     = 0;
		mc_out[actuator_controls_s::INDEX_LANDING_GEAR] = 0;

		// FW out = FW in
		fw_out[actuator_controls_s::INDEX_ROLL]         = fw_in[actuator_controls_s::INDEX_ROLL];
		fw_out[actuator_controls_s::INDEX_PITCH]        = fw_in[actuator_controls_s::INDEX_PITCH];
		fw_out[actuator_controls_s::INDEX_YAW]          = fw_in[actuator_controls_s::INDEX_YAW];
		fw_out[actuator_controls_s::INDEX_THROTTLE]     = fw_in[actuator_controls_s::INDEX_THROTTLE];
		fw_out[actuator_controls_s::INDEX_FLAPS]        = fw_in[actuator_controls_s::INDEX_FLAPS];
		fw_out[actuator_controls_s::INDEX_AIRBRAKES]    = 0;
		break;
	}

	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	_actuators_out_0->timestamp = _actuators_out_1->timestamp = hrt_absolute_time();
}

void
Standard::waiting_on_tecs()
{
	// keep thrust from transition
	_v_att_sp->thrust_body[0] = _pusher_throttle;
};
