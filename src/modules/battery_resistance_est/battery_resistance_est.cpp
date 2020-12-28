/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include "battery_resistance_est.hpp"

InternalRes::InternalRes() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool InternalRes::init()
{
	if (!_battery_sub.registerCallback()) {
		PX4_ERR("battery status callback registration failed!");
		return false;
	}

	return true;
}

float InternalRes::extract_parameters() {

	float r_steady_state = param_0;
	float r_transient = (param_1/param_2) - param_0;
	float voltage_open_circuit = param_3/param_2;

	//calculate bat1_r_internal
	float internal_resistance = (battery_status.voltage_filtered_v - voltage_open_circuit) / (-battery_status.current_filtered_a);

	//logging
	inter_res.r_transient = r_transient;
	inter_res.r_steady_state = r_steady_state;
	inter_res.voltage_open_circuit = voltage_open_circuit;

	return internal_resistance;

}


void InternalRes::write_internal_resistance(float voltage_estimation_error, float internal_resistance){


	//store best esimate
	if((abs(voltage_estimation_error)) <= abs(best_prediction_error))
	{
		best_prediction = bat1_r_internal_prev;
		best_prediction_error = voltage_estimation_error;
	} else if (best_prediction_error_reset){
		best_prediction = bat1_r_internal_prev;
		best_prediction_error = voltage_estimation_error;
		best_prediction_error_reset = false;
	}

	float time_since_param_write = (hrt_absolute_time()- last_param_write_time)/ 1e6f;

	// save bat1_r_internal only periodically
	if (time_since_param_write >= _inter_res_update_period.get()){

		//BAT${i}_R_INTERNAL  increment: 0.01
		best_prediction = round(best_prediction*100)/100;

		//clamp BAT${i}_R_INTERNAL min: -1.0  max: 0.2
		if (best_prediction > 0.1f){
	    		best_prediction = 0.1f;
		}
		if (best_prediction < -1.0f){
	    		best_prediction = -1.0f;
		}

		//Set px4 Internal resistance using simplified Rint Model
		_bat1_r_internal.set(best_prediction);
		_bat1_r_internal.commit();
		last_param_write_time = hrt_absolute_time();
		best_prediction_error_reset = true;
	}

	bat1_r_internal_prev = internal_resistance;

	inter_res.bat1_r_internal = internal_resistance;

}

void InternalRes::estimate_v_dot(float voltage_estimation_error, float battery_sampling_period){

	//process signal
	signal_0 = -(battery_status.current_filtered_a - current_filtered_a_prev)/(battery_sampling_period);
	signal_1 = -battery_status.current_filtered_a;
	signal_2 = -voltage_estimation_prev;

	//update parameter vector estimate
	param_0 += adaptation_gain_0*voltage_estimation_error*signal_0*battery_sampling_period;
	param_1 += adaptation_gain_1*voltage_estimation_error*signal_1*battery_sampling_period;
	param_2 += adaptation_gain_2*voltage_estimation_error*signal_2*battery_sampling_period;
	param_3 += adaptation_gain_3*voltage_estimation_error*battery_sampling_period;

	//Apply thevenin battery model (first order system)
	float v_dot_estimate = param_0*signal_0 + param_1*signal_1 + param_2*signal_2 + param_3 + lambda*voltage_estimation_error;

	//store current values for next iteration
	v_dot_estimate_prev = v_dot_estimate;

	//logging
	inter_res.param_0 = param_0;
	inter_res.param_1 = param_1;
	inter_res.param_2 = param_2;
	inter_res.param_3 = param_3;

	inter_res.signal_0 = signal_0;
	inter_res.signal_1 = signal_1;
	inter_res.signal_2 = signal_2;

}


float InternalRes::compute_voltage_estimation_error(float battery_sampling_period){

	float voltage_estimation = v_dot_estimate_prev*battery_sampling_period + voltage_estimation_prev;

   	float voltage_estimation_error = battery_status.voltage_filtered_v - voltage_estimation;

	inter_res.voltage_estimation = voltage_estimation;
	inter_res.voltage_estimation_error = abs(voltage_estimation_error);

	voltage_estimation_prev = voltage_estimation;

	return voltage_estimation_error;
}

void InternalRes::Run()
{

	if (should_exit()) {
		_battery_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_battery_sub.update(&battery_status)) {

		if (battery_time_prev != 0 && battery_status.timestamp != battery_time_prev) {

		float battery_sampling_period = (battery_status.timestamp - battery_time_prev)/ 1e6f;

		float voltage_estimation_error = compute_voltage_estimation_error(battery_sampling_period);

		float internal_resistance = extract_parameters();

		write_internal_resistance(voltage_estimation_error, internal_resistance);

		estimate_v_dot(voltage_estimation_error,battery_sampling_period);

		//logging for debugging
		inter_res.timestamp = hrt_absolute_time();
		inter_res.voltage = battery_status.voltage_filtered_v; //for sync with replay
		_internal_res_pub.publish(inter_res);

		}

		battery_time_prev = battery_status.timestamp;
		current_filtered_a_prev = battery_status.current_filtered_a;

	}

}


int InternalRes::task_spawn(int argc, char *argv[])
{
	InternalRes *instance = new InternalRes();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int InternalRes::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int InternalRes::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int InternalRes::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
module to PX4 which uses ​only the current and voltage data​ to periodically re-estimate the
battery internal resistance. The estimate should then be written to the BAT1_R_INTERNAL
parameter. Implementation based on 'Online estimation of internal resistance and open-circuit voltage of lithium-ion
batteries in electric vehicles' by Yi-Hsien Chiang , Wu-Yang Sean, Jia-Cheng Ke
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("battery_resistance_est", "custom_module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int battery_resistance_est_main(int argc, char *argv[])
{
	return InternalRes::main(argc, argv);
}
