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
 * @file hover_thrust_estimator.cpp
 *
 * @author Mathieu Bresciani 	<brescianimathieu@gmail.com>
 */

#include "MulticopterHoverThrustEstimator.hpp"

#include <mathlib/mathlib.h>

using namespace time_literals;

MulticopterHoverThrustEstimator::MulticopterHoverThrustEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_valid_hysteresis.set_hysteresis_time_from(false, 2_s);
	updateParams();
	reset();
}

MulticopterHoverThrustEstimator::~MulticopterHoverThrustEstimator()
{
	perf_free(_cycle_perf);
}

bool MulticopterHoverThrustEstimator::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position_setpoint callback registration failed!");
		return false;
	}

	return true;
}

void MulticopterHoverThrustEstimator::reset()
{
	_hover_thrust_ekf.setHoverThrust(_param_mpc_thr_hover.get());
	_hover_thrust_ekf.setHoverThrustStdDev(_param_hte_ht_err_init.get());
	_hover_thrust_ekf.resetAccelNoise();
}

void MulticopterHoverThrustEstimator::updateParams()
{
	const float ht_err_init_prev = _param_hte_ht_err_init.get();
	ModuleParams::updateParams();

	_hover_thrust_ekf.setProcessNoiseStdDev(_param_hte_ht_noise.get());

	if (fabsf(_param_hte_ht_err_init.get() - ht_err_init_prev) > FLT_EPSILON) {
		_hover_thrust_ekf.setHoverThrustStdDev(_param_hte_ht_err_init.get());
	}

	_hover_thrust_ekf.setAccelInnovGate(_param_hte_acc_gate.get());
}

void MulticopterHoverThrustEstimator::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;

			if (_landed) {
				_in_air = false;
			}
		}
	}

	if (!_vehicle_local_position_sub.updated()) {
		return;
	}

	vehicle_local_position_s local_pos{};

	if (_vehicle_local_position_sub.copy(&local_pos)) {
		// This is only necessary because the landed
		// flag of the land detector does not guarantee that
		// the vehicle does not touch the ground anymore.
		// There is no check for the dist_bottom validity as
		// this value is always good enough after takeoff for
		// this use case.
		// TODO: improve the landed flag
		if (!_landed) {
			if (local_pos.dist_bottom > 1.f) {
				_in_air = true;
			}
		}
	}

	// new local position setpoint needed every iteration
	if (!_vehicle_local_position_setpoint_sub.updated()) {
		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	perf_begin(_cycle_perf);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	const float dt = (local_pos.timestamp - _timestamp_last) * 1e-6f;
	_timestamp_last = local_pos.timestamp;

	if (_armed && _in_air && (dt > 0.001f) && (dt < 1.f) && PX4_ISFINITE(local_pos.az)) {

		_hover_thrust_ekf.predict(dt);

		vehicle_local_position_setpoint_s local_pos_sp;

		if (_vehicle_local_position_setpoint_sub.copy(&local_pos_sp)) {
			if (PX4_ISFINITE(local_pos_sp.thrust[2])) {
				// Inform the hover thrust estimator about the measured vertical
				// acceleration (positive acceleration is up) and the current thrust (positive thrust is up)
				// Guard against fast up and down motions biasing the estimator due to large drag and prop wash effects
				if (fabsf(local_pos.vz) < 2.f) {
					_hover_thrust_ekf.fuseAccZ(-local_pos.az, -local_pos_sp.thrust[2]);
				}

				const bool valid = (_hover_thrust_ekf.getHoverThrustEstimateVar() < 0.001f)
						   && (_hover_thrust_ekf.getInnovationTestRatio() < 1.f);
				_valid_hysteresis.set_state_and_update(valid, local_pos.timestamp);
				_valid = _valid_hysteresis.get_state();

				publishStatus(local_pos.timestamp);
			}
		}

	} else {
		_valid_hysteresis.set_state_and_update(false, hrt_absolute_time());

		if (!_armed) {
			reset();
		}

		if (_valid) {
			// only publish a single message to invalidate
			publishInvalidStatus();

			_valid = false;
		}
	}

	perf_end(_cycle_perf);
}

void MulticopterHoverThrustEstimator::publishStatus(const hrt_abstime &timestamp_sample)
{
	hover_thrust_estimate_s status_msg{};

	status_msg.timestamp_sample = timestamp_sample;

	status_msg.hover_thrust = _hover_thrust_ekf.getHoverThrustEstimate();
	status_msg.hover_thrust_var = _hover_thrust_ekf.getHoverThrustEstimateVar();

	status_msg.accel_innov = _hover_thrust_ekf.getInnovation();
	status_msg.accel_innov_var = _hover_thrust_ekf.getInnovationVar();
	status_msg.accel_innov_test_ratio = _hover_thrust_ekf.getInnovationTestRatio();
	status_msg.accel_noise_var = _hover_thrust_ekf.getAccelNoiseVar();

	status_msg.valid = _valid;

	status_msg.timestamp = hrt_absolute_time();

	_hover_thrust_ekf_pub.publish(status_msg);
}

void MulticopterHoverThrustEstimator::publishInvalidStatus()
{
	hover_thrust_estimate_s status_msg{};

	status_msg.hover_thrust = NAN;
	status_msg.hover_thrust_var = NAN;
	status_msg.accel_innov = NAN;
	status_msg.accel_innov_var = NAN;
	status_msg.accel_innov_test_ratio = NAN;
	status_msg.accel_noise_var = NAN;

	status_msg.timestamp = hrt_absolute_time();

	_hover_thrust_ekf_pub.publish(status_msg);
}

int MulticopterHoverThrustEstimator::task_spawn(int argc, char *argv[])
{
	MulticopterHoverThrustEstimator *instance = new MulticopterHoverThrustEstimator();

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

int MulticopterHoverThrustEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterHoverThrustEstimator::print_status()
{
	perf_print_counter(_cycle_perf);

	return 0;
}

int MulticopterHoverThrustEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_hover_thrust_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_hover_thrust_estimator_main(int argc, char *argv[])
{
	return MulticopterHoverThrustEstimator::main(argc, argv);
}
