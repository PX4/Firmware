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

	_param_est(0) = _param_r_s_init.get();
	_param_est(1) = (_param_r_t_init.get() + _param_r_s_init.get()) / (_param_r_t_init.get() * _param_c_t_init.get());
	_param_est(2) = 1.0f / (_param_r_t_init.get() * _param_c_t_init.get());
	_param_est(3) = (_param_v_oc_init.get() * _param_bat1_n_cells.get()) / (_param_r_t_init.get() * _param_c_t_init.get());

	_voltage_estimation = _param_v_est_init.get() * _param_bat1_n_cells.get();

	_adaptation_gain.setAll(_param_param_gain.get());

	_inter_res.best_r_internal_est = _param_inter_res_init.get();

	return true;
}

Vector<float, 4> InternalRes::extract_ecm_parameters()
{

	Vector<float, 4> ecm_params;

	ecm_params(0) = _param_est(0); //_r_steady_state
	ecm_params(1) = (_param_est(1) / _param_est(2)) - _param_est(0); //_r_transient
	ecm_params(2) = _param_est(3) / _param_est(2); //_voltage_open_circuit

	//calculate bat1_r_internal
	ecm_params(3) = (ecm_params(2) - _voltage_filtered_v) / (_current_filtered_a); //_internal_resistance_est

	//logging
	_inter_res.r_s = ecm_params(0);
	_inter_res.r_t = ecm_params(1);
	_inter_res.v_oc = ecm_params(2);
	_inter_res.r_internal_est = ecm_params(3) / _param_bat1_n_cells.get();

	return ecm_params;
}


void InternalRes::update_internal_resistance(const float voltage_estimation_error,
		const Vector<float, 4> &esm_params_est)
{

	//store best esimate
	if ((fabs(voltage_estimation_error)) <= fabs(_best_prediction_error)) {
		_best_ecm_params_est = esm_params_est;
		_best_prediction_error = voltage_estimation_error;

	} else if (_best_prediction_error_reset) {
		_best_ecm_params_est = esm_params_est;
		_best_prediction_error = voltage_estimation_error;
		_best_prediction_error_reset = false;
	}

	//clamp BAT1_R_INTERNAL
	if (_best_ecm_params_est(3) > (_param_inter_res_est_max.get() *_param_bat1_n_cells.get())) {
		_best_ecm_params_est(3) = _param_inter_res_est_max.get() * _param_bat1_n_cells.get();

	} else if (_best_ecm_params_est(3) < (_param_inter_res_est_min.get() *_param_bat1_n_cells.get())) {
		_best_ecm_params_est(3) = _param_inter_res_est_min.get() * _param_bat1_n_cells.get();
	}

	const float time_since_param_update = (hrt_absolute_time() - _last_param_update_time) / 1e6f;

	// publish new bat1_r_internal only periodically
	if (time_since_param_update >= _param_inter_res_update_period.get()) {
		_inter_res.best_r_internal_est = _best_ecm_params_est(3) / _param_bat1_n_cells.get();
		_last_param_update_time = hrt_absolute_time();
		_best_prediction_error_reset = true;
	}
}

float InternalRes::predict_voltage(const float dt)
{

	//process _signal
	_signal(0) = -(_battery_status.current_filtered_a - _current_filtered_a_prev)
		     / ((_battery_status.timestamp - _battery_time_prev) / 1e6f); //central difference method
	_signal(1) = -_current_filtered_a;
	_signal(2) = -_voltage_estimation;
	_signal(3) = 1.f;

	// Predict the voltage using the learned adaptive model
	_voltage_estimation += (_param_est.transpose() * _signal * dt)(0, 0);

	const float voltage_estimation_error = _voltage_filtered_v - _voltage_estimation;

	_voltage_estimation += _lambda * dt * voltage_estimation_error;

	//logging
	_inter_res.voltage_estimation = _voltage_estimation;
	_inter_res.voltage_estimation_error = fabs(voltage_estimation_error);

	_param_est.copyTo(_inter_res.param_est);

	_signal.copyTo(_inter_res.signal);

	return voltage_estimation_error;
}

void InternalRes::Run()
{

	if (should_exit()) {
		_battery_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_status_sub.update(&_vehicle_status)) {
		_armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		_on_standby = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY);
	}


	if (_battery_sub.update(&_battery_status)
	    && _armed) {

		if (_battery_time_prev != 0 && _battery_time != _battery_time_prev) {

			const float dt = (_battery_time - _battery_time_prev) / 1e6f;

			const float voltage_estimation_error = predict_voltage(dt);

			const Vector<float, 4> ecm_params_est = extract_ecm_parameters();

			update_internal_resistance(voltage_estimation_error, ecm_params_est);

			//update the vector of parameters using the adaptive law
			for (int i = 0; i < 4; i++) {
				_param_est(i) += _adaptation_gain(i) * voltage_estimation_error * _signal(i) * dt;
			}

			//logging for debugging
			_inter_res.timestamp = hrt_absolute_time();
			_inter_res.voltage_filtered_v = _voltage_filtered_v; //for sync with ekfreplay
			_internal_res_pub.publish(_inter_res);

			_was_armed = true;

		}

		//save for central difference approximation of current derivative
		_battery_time_prev = _battery_time;
		_battery_time = _battery_status.timestamp;
		_current_filtered_a_prev = _current_filtered_a;
		_current_filtered_a = _battery_status.current_filtered_a;
		_voltage_filtered_v = _battery_status.voltage_filtered_v;
	}

	//save ecm params on disarm after flight
	if (!_param_bat_saved && _on_standby && _was_armed) {

		_param_r_s_init.set(_best_ecm_params_est(0));
		_param_r_s_init.commit_no_notification();

		_param_r_t_init.set(_best_ecm_params_est(1));
		_param_r_t_init.commit_no_notification();

		_param_v_oc_init.set(_best_ecm_params_est(2) / _param_bat1_n_cells.get());
		_param_v_oc_init.commit_no_notification();

		_param_bat1_r_internal.set(round((_best_ecm_params_est(3) / _param_bat1_n_cells.get()) * 100) / 100);
		_param_bat1_r_internal.commit_no_notification();

		_param_inter_res_init.set(round((_best_ecm_params_est(3) / _param_bat1_n_cells.get()) * 1000) / 1000);
		_param_inter_res_init.commit_no_notification();

		_param_bat_saved = true;
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
battery internal resistance. Based on 'Online estimation of internal resistance and open-circuit voltage of lithium-ion
batteries in electric vehicles' Yi-Hsien Chiang , Wu-Yang Sean, Jia-Cheng Ke
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("battery_resistance_est", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int battery_resistance_est_main(int argc, char *argv[])
{
	return InternalRes::main(argc, argv);
}
