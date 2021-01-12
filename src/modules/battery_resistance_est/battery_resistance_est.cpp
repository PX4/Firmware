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

	_param_est(0) = r_s;
	_param_est(1) = (r_t + r_s)/(r_t*c_t);
	_param_est(2) = 1.0f/(r_t*c_t);
	_param_est(3) = v_oc/(r_t*c_t);

	_adaptation_gain(0) = 0.0001f;
	_adaptation_gain(1) = 0.0001f;
	_adaptation_gain(2) = 0.0001f;
	_adaptation_gain(3) = 0.0001f;

	inter_res.best_r_internal_est = _best_internal_resistance_est;

	return true;
}

float InternalRes::extract_parameters() {

	const float _r_steady_state = _param_est(0);
	const float _r_transient = (_param_est(1)/_param_est(2)) - _param_est(0);
	const float _voltage_open_circuit = _param_est(3)/_param_est(2);

	//calculate bat1_r_internal
	const float _internal_resistance_est = (_voltage_filtered_v - _voltage_open_circuit) / (-_current_filtered_a);

	//logging
	inter_res.r_transient = _r_transient;
	inter_res.r_steady_state = _r_steady_state;
	inter_res.voltage_open_circuit = _voltage_open_circuit;
	inter_res.r_internal_est = _internal_resistance_est;

	return _internal_resistance_est;

}


void InternalRes::update_internal_resistance(const float _voltage_estimation_error, const float _internal_resistance_est){

	//store best esimate
	if((abs(_voltage_estimation_error)) <= abs(best_prediction_error))
	{
		_best_internal_resistance_est = _internal_resistance_est;
		best_prediction_error = _voltage_estimation_error;
	} else if (best_prediction_error_reset){
		_best_internal_resistance_est = _internal_resistance_est;
		best_prediction_error = _voltage_estimation_error;
		best_prediction_error_reset = false;
	}

	const float time_since_param_update = (hrt_absolute_time()- last_param_update_time)/ 1e6f;

	// publish new bat1_r_internal only periodically
	if (time_since_param_update >= _inter_res_update_period.get()){

		//BAT${i}_R_INTERNAL  increment: 0.01
		_best_internal_resistance_est = round(_best_internal_resistance_est*100)/100;

		//clamp BAT1_R_INTERNAL
		if (_best_internal_resistance_est > 0.2f){
	    		_best_internal_resistance_est = 0.2f;
		} else if (_best_internal_resistance_est < 0.01f){
	    		_best_internal_resistance_est = 0.01f;
		}

		//Set px4 Internal resistance using simplified Rint Model
		inter_res.best_r_internal_est = _best_internal_resistance_est;
		last_param_update_time = hrt_absolute_time();
		best_prediction_error_reset = true;
	}
}

float InternalRes::predict_voltage(const float dt){

	//process signal
	signal(0) = -(battery_status.current_filtered_a - _current_filtered_a_prev)
		/ ((battery_status.timestamp - _battery_time_prev) / 1e6f); //central difference method
	signal(1) = -_current_filtered_a;
	signal(2) = -_voltage_estimation;
	signal(3) = 1.f;

	// Predict the voltage using the learned adaptive model
	_voltage_estimation += (_param_est.transpose() * signal* dt) (0,0);

	const float _voltage_estimation_error = _voltage_filtered_v - _voltage_estimation;

	_voltage_estimation += _lambda * dt * _voltage_estimation_error;

	//logging
	inter_res.voltage_estimation = _voltage_estimation;
	inter_res.voltage_estimation_error = abs(_voltage_estimation_error);

	inter_res.param_est[0] = _param_est(0);
	inter_res.param_est[1] = _param_est(1);
	inter_res.param_est[2] = _param_est(2);
	inter_res.param_est[3] = _param_est(3);

	inter_res.signal[0] = signal(0);
	inter_res.signal[1] = signal(1);
	inter_res.signal[2] = signal(2);
	inter_res.signal[3] = signal(3);

	return _voltage_estimation_error;
}

void InternalRes::Run()
{

	if (should_exit()) {
		_battery_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_battery_sub.update(&battery_status)) {

		if (_battery_time_prev != 0 && _battery_time != _battery_time_prev) {

		const float dt = (_battery_time - _battery_time_prev)/ 1e6f;

		const float _voltage_estimation_error = predict_voltage(dt);

		const float _internal_resistance = extract_parameters();

		update_internal_resistance(_voltage_estimation_error, _internal_resistance);

		//update the vector of parameters using the adaptive law
		for (int i = 0; i < 4; i++) {
			_param_est(i) += _adaptation_gain(i) * _voltage_estimation_error * signal(i) * dt;
		}

		//logging for debugging
		inter_res.timestamp = hrt_absolute_time();
		inter_res.voltage = _voltage_filtered_v; //for sync with ekfreplay
		_internal_res_pub.publish(inter_res);

		}

		//save for central difference approximation of current derivative
		_battery_time_prev = _battery_time;
		_battery_time = battery_status.timestamp;
		_current_filtered_a_prev = _current_filtered_a;
		_current_filtered_a = battery_status.current_filtered_a;
		_voltage_filtered_v = battery_status.voltage_filtered_v;
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
module uses current and voltage data​ to online estimate the
battery internal resistance. Implementation based on 'Online estimation of internal resistance and open-circuit voltage of lithium-ion
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
