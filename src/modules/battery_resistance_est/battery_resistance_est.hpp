/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAfieldES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAfieldE.
 *
 ****************************************************************************/

/**
 * @file BatteryResistanceEstimator.hpp
 *
 * Estimator for the battery internal resistance parameter to run online using only the current and voltage data​.
 *
 * @author Mohamad Akkawi	<akkawi@protonmail.com>
 *
 * Implementation based on 'Online estimation of internal resistance and open-circuit voltage of lithium-ion
 * batteries in electric vehicles' by Yi-Hsien Chiang , Wu-Yang Sean, Jia-Cheng Ke
 *
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/internal_resistance.h>

#include <matrix/math.hpp>

#include <uORB/topics/vehicle_status.h>

using namespace matrix;

class InternalRes : public ModuleBase<InternalRes>, public ModuleParams, public px4::WorkItem
{
public:
	InternalRes();
	~InternalRes() {};

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	void update_internal_resistance(const float voltage_estimation_error, const Vector<float, 4> &esm_params_est);

	Vector<float, 4> extract_ecm_parameters();

	float predict_voltage(const float dt);

private:
	void Run() override;

	internal_resistance_s _inter_res;
	battery_status_s _battery_status;
	vehicle_status_s _vehicle_status;

	hrt_abstime _battery_time_prev{0};
	hrt_abstime _battery_time{0};

	hrt_abstime _last_param_update_time{0};

	Vector<float, 4> _param_est;
	Vector<float, 4> _adaptation_gain;
	Vector<float, 4> _signal;
	Vector<float, 4> _best_ecm_params_est;

	float _lambda{0.8f};
	float _voltage_estimation{0.f};

	float _current_filtered_a{0.f};
	float _voltage_filtered_v{0.f};
	float _current_filtered_a_prev{0.f};

	float _best_prediction_error{0.f};
	bool _best_prediction_error_reset{true};

//used to save estimated ecm params on disarm
	bool _armed{false};
	bool _was_armed{false};
	bool _on_standby{false};
	bool _param_bat_saved{false};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionCallbackWorkItem _battery_sub{this, ORB_ID(battery_status)};

	uORB::Publication<internal_resistance_s> _internal_res_pub{ORB_ID(internal_resistance)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT1_R_INTERNAL>) _param_bat1_r_internal,
		(ParamFloat<px4::params::BAT1_V_CHARGED>) _param_bat1_v_charged,
		(ParamInt<px4::params::BAT1_N_CELLS>) _param_bat1_n_cells,
		(ParamFloat<px4::params::BAT_VOC_INIT>) _param_v_oc_init,
		(ParamFloat<px4::params::BAT_R_S_INIT>) _param_r_s_init,
		(ParamFloat<px4::params::BAT_R_T_INIT>) _param_r_t_init,
		(ParamFloat<px4::params::BAT_C_T_INIT>) _param_c_t_init,
		(ParamFloat<px4::params::BAT_RIN_UPDATE>) _param_inter_res_update_period,
		(ParamFloat<px4::params::BAT_RIN_EST_MAX>) _param_inter_res_est_max,
		(ParamFloat<px4::params::BAT_RIN_EST_MIN>) _param_inter_res_est_min,
		(ParamFloat<px4::params::BAT_RIN_EST_INIT>) _param_inter_res_init,
		(ParamFloat<px4::params::BAT_PARAM_GAIN>) _param_param_gain,
		(ParamFloat<px4::params::BAT_V_EST_INIT>) _param_v_est_init
	)
};
