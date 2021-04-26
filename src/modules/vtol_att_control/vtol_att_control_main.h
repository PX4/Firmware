/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file VTOL_att_control_main.cpp
 * Implementation of an attitude controller for VTOL airframes. This module receives data
 * from both the fixed wing- and the multicopter attitude controllers and processes it.
 * It computes the correct actuator controls depending on which mode the vehicle is in (hover,forward-
 * flight or transition). It also publishes the resulting controls on the actuator controls topics.
 *
 * @author Roman Bapst 		<bapstr@ethz.ch>
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler	<thomasgubler@gmail.com>
 * @author David Vorsin		<davidvorsin@gmail.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/vehicle_status.h>
#include "standard.h"
#include "tailsitter.h"
#include "tiltrotor.h"

using namespace time_literals;

extern "C" __EXPORT int vtol_att_control_main(int argc, char *argv[]);

class VtolAttitudeControl : public ModuleBase<VtolAttitudeControl>, public px4::WorkItem
{
public:

	VtolAttitudeControl();
	~VtolAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	bool is_fixed_wing_requested();
	void quadchute(const char *reason);

	struct actuator_controls_s 			*get_actuators_fw_in() {return &_actuators_fw_in;}
	struct actuator_controls_s 			*get_actuators_mc_in() {return &_actuators_mc_in;}
	struct actuator_controls_s 			*get_actuators_out0() {return &_actuators_out_0;}
	struct actuator_controls_s 			*get_actuators_out1() {return &_actuators_out_1;}
	struct airspeed_validated_s 				*get_airspeed() {return &_airspeed_validated;}
	struct position_setpoint_triplet_s		*get_pos_sp_triplet() {return &_pos_sp_triplet;}
	struct tecs_status_s 				*get_tecs_status() {return &_tecs_status;}
	struct vehicle_attitude_s 			*get_att() {return &_v_att;}
	struct vehicle_attitude_setpoint_s		*get_att_sp() {return &_v_att_sp;}
	struct vehicle_attitude_setpoint_s 		*get_fw_virtual_att_sp() {return &_fw_virtual_att_sp;}
	struct vehicle_attitude_setpoint_s 		*get_mc_virtual_att_sp() {return &_mc_virtual_att_sp;}
	struct vehicle_control_mode_s 			*get_control_mode() {return &_v_control_mode;}
	struct vehicle_land_detected_s			*get_land_detected() {return &_land_detected;}
	struct vehicle_local_position_s 		*get_local_pos() {return &_local_pos;}
	struct vehicle_local_position_setpoint_s	*get_local_pos_sp() {return &_local_pos_sp;}
	struct vtol_vehicle_status_s			*get_vtol_vehicle_status() {return &_vtol_vehicle_status;}

	struct Params 					*get_params() {return &_params;}

private:

	void Run() override;

	uORB::SubscriptionCallbackWorkItem _actuator_inputs_fw{this, ORB_ID(actuator_controls_virtual_fw)};
	uORB::SubscriptionCallbackWorkItem _actuator_inputs_mc{this, ORB_ID(actuator_controls_virtual_mc)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};			// airspeed subscription
	uORB::Subscription _fw_virtual_att_sp_sub{ORB_ID(fw_virtual_attitude_setpoint)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _local_pos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};			// setpoint subscription
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};			// sensor subscription
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};	//manual control setpoint subscription
	uORB::Subscription _mc_virtual_att_sp_sub{ORB_ID(mc_virtual_attitude_setpoint)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};			// local position setpoint subscription
	uORB::Subscription _tecs_status_sub{ORB_ID(tecs_status)};
	uORB::Subscription _v_att_sub{ORB_ID(vehicle_attitude)};		//vehicle attitude subscription
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};	//vehicle control mode subscription
	uORB::Subscription _vehicle_cmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<actuator_controls_s>		_actuators_0_pub{ORB_ID(actuator_controls_0)};		//input for the mixer (roll,pitch,yaw,thrust)
	uORB::Publication<actuator_controls_s>		_actuators_1_pub{ORB_ID(actuator_controls_1)};
	uORB::Publication<vehicle_attitude_setpoint_s>	_v_att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vtol_vehicle_status_s>	_vtol_vehicle_status_pub{ORB_ID(vtol_vehicle_status)};

	orb_advert_t	_mavlink_log_pub{nullptr};	// mavlink log uORB handle

	vehicle_attitude_setpoint_s		_v_att_sp{};			//vehicle attitude setpoint
	vehicle_attitude_setpoint_s 		_fw_virtual_att_sp{};	// virtual fw attitude setpoint
	vehicle_attitude_setpoint_s 		_mc_virtual_att_sp{};	// virtual mc attitude setpoint

	actuator_controls_s			_actuators_fw_in{};	//actuator controls from fw_att_control
	actuator_controls_s			_actuators_mc_in{};	//actuator controls from mc_att_control
	actuator_controls_s			_actuators_out_0{};	//actuator controls going to the mc mixer
	actuator_controls_s			_actuators_out_1{};	//actuator controls going to the fw mixer (used for elevons)

	airspeed_validated_s 				_airspeed_validated{};			// airspeed
	manual_control_switches_s		_manual_control_switches{}; //manual control setpoint
	position_setpoint_triplet_s		_pos_sp_triplet{};
	tecs_status_s				_tecs_status{};
	vehicle_attitude_s			_v_att{};				//vehicle attitude
	vehicle_control_mode_s			_v_control_mode{};	//vehicle control mode
	vehicle_land_detected_s			_land_detected{};
	vehicle_local_position_s		_local_pos{};
	vehicle_local_position_setpoint_s	_local_pos_sp{};
	vtol_vehicle_status_s 			_vtol_vehicle_status{};

	Params _params{};	// struct holding the parameters

	struct {
		param_t idle_pwm_mc;
		param_t vtol_motor_id;
		param_t vtol_fw_permanent_stab;
		param_t vtol_type;
		param_t elevons_mc_lock;
		param_t fw_min_alt;
		param_t fw_alt_err;
		param_t fw_qc_max_pitch;
		param_t fw_qc_max_roll;
		param_t front_trans_time_openloop;
		param_t front_trans_time_min;
		param_t front_trans_duration;
		param_t back_trans_duration;
		param_t transition_airspeed;
		param_t front_trans_throttle;
		param_t back_trans_throttle;
		param_t airspeed_blend;
		param_t airspeed_mode;
		param_t front_trans_timeout;
		param_t mpc_xy_cruise;
		param_t fw_motors_off;
		param_t diff_thrust;
		param_t diff_thrust_scale;
		param_t down_pitch_max;
		param_t forward_thrust_scale;
		param_t dec_to_pitch_ff;
		param_t dec_to_pitch_i;
		param_t back_trans_dec_sp;
		param_t vt_mc_on_fmu;
		param_t vt_forward_thrust_enable_mode;
		param_t mpc_land_alt1;
		param_t mpc_land_alt2;
	} _params_handles{};

	hrt_abstime _last_run_timestamp{0};

	/* for multicopters it is usual to have a non-zero idle speed of the engines
	 * for fixed wings we want to have an idle speed of zero since we do not want
	 * to waste energy when gliding. */
	int		_transition_command{vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC};

	VtolType	*_vtol_type{nullptr};	// base class for different vtol types

	bool		_initialized{false};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	void		vehicle_cmd_poll();

	int 		parameters_update();			//Update local parameter cache
};
