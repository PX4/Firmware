/****************************************************************************
 *
 *   Copyright (c) 2015, 2021 PX4 Development Team. All rights reserved.
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
* @file vtol_type.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Sander Smeets		<sander@droneslab.com>
* @author Andreas Antener	<andreas@uaventure.com>
*
*/

#ifndef VTOL_TYPE_H
#define VTOL_TYPE_H

#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

struct Params {
	int32_t idle_pwm_fw;			// pwm value for idle in fw mode
	int32_t vtol_motor_id;
	int32_t vtol_type;
	bool elevons_mc_lock;		// lock elevons in multicopter mode
	float fw_min_alt;			// minimum relative altitude for FW mode (QuadChute)
	float fw_alt_err;			// maximum negative altitude error for FW mode (Adaptive QuadChute)
	float fw_qc_max_pitch;		// maximum pitch angle FW mode (QuadChute)
	float fw_qc_max_roll;		// maximum roll angle FW mode (QuadChute)
	float front_trans_time_openloop;
	float front_trans_time_min;
	float front_trans_duration;
	float back_trans_duration;
	float transition_airspeed;
	float front_trans_throttle;
	float back_trans_throttle;
	float airspeed_blend;
	bool airspeed_disabled;
	float front_trans_timeout;
	float mpc_xy_cruise;
	int32_t fw_motors_off;			/**< bitmask of all motors that should be off in fixed wing mode */
	int32_t diff_thrust;
	float diff_thrust_scale;
	float down_pitch_max;
	float forward_thrust_scale;
	float dec_to_pitch_ff;
	float dec_to_pitch_i;
	float back_trans_dec_sp;
	bool vt_mc_on_fmu;
	int32_t vt_forward_thrust_enable_mode;
	float mpc_land_alt1;
	float mpc_land_alt2;
};

// Has to match 1:1 msg/vtol_vehicle_status.msg
enum class mode {
	TRANSITION_TO_FW = 1,
	TRANSITION_TO_MC = 2,
	ROTARY_WING = 3,
	FIXED_WING = 4
};

enum class vtol_type {
	TAILSITTER = 0,
	TILTROTOR,
	STANDARD
};

enum VtolForwardActuationMode {
	DISABLE = 0,
	ENABLE_WITHOUT_LAND,
	ENABLE_ABOVE_MPC_LAND_ALT1,
	ENABLE_ABOVE_MPC_LAND_ALT2,
	ENABLE_ALL_MODES,
	ENABLE_ABOVE_MPC_LAND_ALT1_WITHOUT_LAND,
	ENABLE_ABOVE_MPC_LAND_ALT2_WITHOUT_LAND
};

// these are states that can be applied to a selection of multirotor motors.
// e.g. if we need to shut off some motors after transitioning to fixed wing mode
// we can individually disable them while others might still need to be enabled to produce thrust.
// we can select the target motors via VT_FW_MOT_OFFID
enum class motor_state {
	ENABLED = 0,		// motor max pwm will be set to the standard max pwm value
	DISABLED,		// motor max pwm will be set to a value that shuts the motor off
	IDLE,			// motor max pwm will be set to VT_IDLE_PWM_FW
	VALUE 			// motor max pwm will be set to a specific value provided, see set_motor_state()
};

/**
 * @brief      Used to specify if min or max pwm values should be altered
 */
enum class pwm_limit_type {
	TYPE_MINIMUM = 0,
	TYPE_MAXIMUM
};

class VtolAttitudeControl;

class VtolType
{
public:

	VtolType(VtolAttitudeControl *att_controller);
	VtolType(const VtolType &) = delete;
	VtolType &operator=(const VtolType &) = delete;

	virtual ~VtolType() = default;

	/**
	 * Initialise.
	 */
	bool init();

	/**
	 * Update vtol state.
	 */
	virtual void update_vtol_state() = 0;

	/**
	 * Update transition state.
	 */
	virtual void update_transition_state() = 0;

	/**
	 * Update multicopter state.
	 */
	virtual void update_mc_state();

	/**
	 * Update fixed wing state.
	 */
	virtual void update_fw_state();

	/**
	 * Write control values to actuator output topics.
	 */
	virtual void fill_actuator_outputs() = 0;

	/**
	 * Special handling opportunity for the time right after transition to FW
	 * before TECS is running.
	 */
	virtual void waiting_on_tecs() {}

	/**
	 * Checks for fixed-wing failsafe condition and issues abort request if needed.
	 */
	void check_quadchute_condition();

	/**
	 * Returns true if we're allowed to do a mode transition on the ground.
	 */
	bool can_transition_on_ground();

	/**
	 * Pusher assist in hover (pusher/pull for standard VTOL, motor tilt for tiltrotor)
	 */
	float pusher_assist();


	mode get_mode() {return _vtol_mode;}

	bool was_in_trans_mode() {return _flag_was_in_trans_mode;}

	virtual void parameters_update() = 0;

protected:
	VtolAttitudeControl *_attc;
	mode _vtol_mode;

	static constexpr const int num_outputs_max = 8;

	struct vehicle_attitude_s		*_v_att;				//vehicle attitude
	struct vehicle_attitude_setpoint_s	*_v_att_sp;			//vehicle attitude setpoint
	struct vehicle_attitude_setpoint_s *_mc_virtual_att_sp;	// virtual mc attitude setpoint
	struct vehicle_attitude_setpoint_s *_fw_virtual_att_sp;	// virtual fw attitude setpoint
	struct vehicle_control_mode_s		*_v_control_mode;	//vehicle control mode
	struct vtol_vehicle_status_s 		*_vtol_vehicle_status;
	struct actuator_controls_s			*_actuators_out_0;			//actuator controls going to the mc mixer
	struct actuator_controls_s			*_actuators_out_1;			//actuator controls going to the fw mixer (used for elevons)
	struct actuator_controls_s			*_actuators_mc_in;			//actuator controls from mc_rate_control
	struct actuator_controls_s			*_actuators_fw_in;			//actuator controls from fw_att_control
	struct vehicle_local_position_s			*_local_pos;
	struct vehicle_local_position_setpoint_s	*_local_pos_sp;
	struct airspeed_validated_s 				*_airspeed_validated;					// airspeed
	struct tecs_status_s				*_tecs_status;
	struct vehicle_land_detected_s			*_land_detected;

	struct Params 					*_params;

	bool _flag_idle_mc = false;		//false = "idle is set for fixed wing mode"; true = "idle is set for multicopter mode"

	bool _pusher_active = false;
	float _mc_roll_weight = 1.0f;	// weight for multicopter attitude controller roll output
	float _mc_pitch_weight = 1.0f;	// weight for multicopter attitude controller pitch output
	float _mc_yaw_weight = 1.0f;	// weight for multicopter attitude controller yaw output
	float _mc_throttle_weight = 1.0f;	// weight for multicopter throttle command. Used to avoid

	// motors spinning up or cutting too fast when doing transitions.
	float _thrust_transition = 0.0f;	// thrust value applied during a front transition (tailsitter & tiltrotor only)

	float _ra_hrate = 0.0f;			// rolling average on height rate for quadchute condition
	float _ra_hrate_sp = 0.0f;		// rolling average on height rate setpoint for quadchute condition

	bool _flag_was_in_trans_mode = false;	// true if mode has just switched to transition

	hrt_abstime _trans_finished_ts = 0;

	bool _tecs_running = false;
	hrt_abstime _tecs_running_ts = 0;

	motor_state _main_motor_state = motor_state::DISABLED;
	motor_state _alternate_motor_state = motor_state::DISABLED;

	hrt_abstime _last_loop_ts = 0;
	float _transition_dt = 0;

	float _accel_to_pitch_integ = 0;

	bool _quadchute_command_treated{false};


	/**
	 * @brief      Sets MC motor min/max PWM values the correct values for
	 *             rotoray-wing flight.
	 *
	 * @return     true on success
	 */
	bool set_limits_mc();

	/**
	 * @brief      Sets MC motor min/max pwm to VT_IDLE_PWM_FW which ensures
	 *             that the motors stop spinning but remain initialized.
	 *
	 * @return     true on success
	 */
	bool set_limits_fw();

	void set_all_motor_state(motor_state target_state, int value = 0);

	void set_main_motor_state(motor_state target_state, int value = 0);

	void set_alternate_motor_state(motor_state target_state, int value = 0);

	float update_and_get_backtransition_pitch_sp();

private:


	/**
	 * @brief      Stores the max pwm values given by the system.
	 */
	struct pwm_output_values _min_mc_pwm_values {};
	struct pwm_output_values _max_mc_pwm_values {};
	struct pwm_output_values _disarmed_pwm_values {};

	struct pwm_output_values _current_max_pwm_values {};
	struct pwm_output_values _current_min_pwm_values {};

	int32_t _main_motor_channel_bitmap = 0;
	int32_t _alternate_motor_channel_bitmap = 0;

	/**
	 * @brief      Adjust minimum/maximum pwm values for the output channels.
	 *
	 * @param      pwm_output_values  Struct containing the limit values for each channel
	 * @param[in]  type               Specifies if min or max limits are adjusted.
	 *
	 * @return     True on success.
	 */
	bool apply_pwm_limits(struct pwm_output_values &pwm_values, pwm_limit_type type);

	/**
	 * @brief      Determines if channel is set in a bitmap.
	 *
	 * @param[in]  channel  The channel
	 * @param[in]  bitmap  	The bitmap to check on.
	 *
	 * @return     True if set, false otherwise.
	 */
	bool is_channel_set(const int channel, const int bitmap);

	// generates a bitmap from a number format, e.g. 1235 -> first, second, third and fifth bits should be set.
	int generate_bitmap_from_channel_numbers(const int channels);

	bool set_motor_state(const motor_state target_state, const int32_t channel_bitmap,  const int value);

	void resetAccelToPitchPitchIntegrator() { _accel_to_pitch_integ = 0.f; }

};

#endif
