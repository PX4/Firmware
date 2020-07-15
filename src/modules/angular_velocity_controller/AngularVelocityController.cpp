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

#include "AngularVelocityController.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <ecl/geo/geo.h>

using namespace matrix;
using namespace time_literals;

AngularVelocityController::AngularVelocityController() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::ctrl_alloc),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

AngularVelocityController::~AngularVelocityController()
{
	perf_free(_loop_perf);
}

bool
AngularVelocityController::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void
AngularVelocityController::parameters_updated()
{
	// Control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f k_gains = Vector3f(_param_avc_x_k.get(), _param_avc_y_k.get(), _param_avc_z_k.get());

	_control.setGains(
		k_gains.emult(Vector3f(_param_avc_x_p.get(), _param_avc_y_p.get(), _param_avc_z_p.get())),
		k_gains.emult(Vector3f(_param_avc_x_i.get(), _param_avc_y_i.get(), _param_avc_z_i.get())),
		k_gains.emult(Vector3f(_param_avc_x_d.get(), _param_avc_y_d.get(), _param_avc_z_d.get())));

	_control.setIntegratorLimit(
		Vector3f(_param_avc_x_i_lim.get(), _param_avc_y_i_lim.get(), _param_avc_z_i_lim.get()));

	_control.setFeedForwardGain(
		Vector3f(_param_avc_x_ff.get(), _param_avc_y_ff.get(), _param_avc_z_ff.get()));

	// inertia matrix
	const float inertia[3][3] = {
		{_param_vm_inertia_xx.get(), _param_vm_inertia_xy.get(), _param_vm_inertia_xz.get()},
		{_param_vm_inertia_xy.get(), _param_vm_inertia_yy.get(), _param_vm_inertia_yz.get()},
		{_param_vm_inertia_xz.get(), _param_vm_inertia_yz.get(), _param_vm_inertia_zz.get()}
	};
	_control.setInertiaMatrix(matrix::Matrix3f(inertia));

	// Hover thrust
	if (!_param_mpc_use_hte.get()) {
		_hover_thrust = _param_mpc_thr_hover.get();
	}
}

void
AngularVelocityController::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s vehicle_angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity)) {
		const hrt_abstime now = hrt_absolute_time();
		_timestamp_sample = vehicle_angular_velocity.timestamp_sample;

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
		_last_run = now;

		const Vector3f angular_velocity{vehicle_angular_velocity.xyz};

		/* check for updates in other topics */
		_vehicle_status_sub.update(&_vehicle_status);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		// Check for updates of hover thrust
		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				_hover_thrust = hte.hover_thrust;
			}
		}

		// check angular acceleration topic
		vehicle_angular_acceleration_s vehicle_angular_acceleration;

		if (_vehicle_angular_acceleration_sub.update(&vehicle_angular_acceleration)) {
			_angular_acceleration = Vector3f(vehicle_angular_acceleration.xyz);
		}

		// check rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint;

		if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			_angular_velocity_sp(0) = vehicle_rates_setpoint.roll;
			_angular_velocity_sp(1) = vehicle_rates_setpoint.pitch;
			_angular_velocity_sp(2) = vehicle_rates_setpoint.yaw;
			_thrust_sp = Vector3f(vehicle_rates_setpoint.thrust_body);

			// Scale _thrust_sp in Newton, assuming _hover_thrust is equivalent to 1G
			_thrust_sp *= (_param_vm_mass.get() * CONSTANTS_ONE_G / _hover_thrust);
		}

		// run the controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {
			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_control.resetIntegral();
			}

			// update saturation status from mixer feedback
			if (_control_allocator_status_sub.updated()) {
				control_allocator_status_s control_allocator_status;

				if (_control_allocator_status_sub.copy(&control_allocator_status)) {
					Vector<bool, 3> saturation_positive;
					Vector<bool, 3> saturation_negative;

					if (not control_allocator_status.torque_setpoint_achieved) {
						for (size_t i = 0; i < 3; i++) {
							if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
								saturation_positive(i) = true;

							} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
								saturation_negative(i) = true;
							}
						}
					}

					_control.setSaturationStatus(saturation_positive, saturation_negative);
				}
			}

			// run rate controller
			_control.update(angular_velocity, _angular_velocity_sp, _angular_acceleration, dt, _maybe_landed || _landed);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			Vector3f integral = _control.getIntegral();
			rate_ctrl_status.timestamp = hrt_absolute_time();
			rate_ctrl_status.rollspeed_integ = integral(0);
			rate_ctrl_status.pitchspeed_integ = integral(1);
			rate_ctrl_status.yawspeed_integ = integral(2);
			_rate_ctrl_status_pub.publish(rate_ctrl_status);

			// publish controller output
			publish_angular_acceleration_setpoint();
			publish_torque_setpoint();
			publish_thrust_setpoint();
		}
	}

	perf_end(_loop_perf);
}

void
AngularVelocityController::publish_angular_acceleration_setpoint()
{
	Vector3f angular_accel_sp = _control.getAngularAccelerationSetpoint();

	vehicle_angular_acceleration_setpoint_s v_angular_accel_sp = {};
	v_angular_accel_sp.timestamp = hrt_absolute_time();
	v_angular_accel_sp.timestamp_sample = _timestamp_sample;
	v_angular_accel_sp.xyz[0] = (PX4_ISFINITE(angular_accel_sp(0))) ? angular_accel_sp(0) : 0.0f;
	v_angular_accel_sp.xyz[1] = (PX4_ISFINITE(angular_accel_sp(1))) ? angular_accel_sp(1) : 0.0f;
	v_angular_accel_sp.xyz[2] = (PX4_ISFINITE(angular_accel_sp(2))) ? angular_accel_sp(2) : 0.0f;

	_vehicle_angular_acceleration_setpoint_pub.publish(v_angular_accel_sp);
}

void
AngularVelocityController::publish_torque_setpoint()
{
	Vector3f torque_sp = _control.getTorqueSetpoint();

	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = _timestamp_sample;
	v_torque_sp.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
	v_torque_sp.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
	v_torque_sp.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void
AngularVelocityController::publish_thrust_setpoint()
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = _timestamp_sample;
	v_thrust_sp.xyz[0] = (PX4_ISFINITE(_thrust_sp(0))) ? (_thrust_sp(0)) : 0.0f;
	v_thrust_sp.xyz[1] = (PX4_ISFINITE(_thrust_sp(1))) ? (_thrust_sp(1)) : 0.0f;
	v_thrust_sp.xyz[2] = (PX4_ISFINITE(_thrust_sp(2))) ? (_thrust_sp(2)) : 0.0f;

	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

int AngularVelocityController::task_spawn(int argc, char *argv[])
{
	AngularVelocityController *instance = new AngularVelocityController();

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

int AngularVelocityController::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
}

int AngularVelocityController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AngularVelocityController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the angular velocity controller.
It takes angular velocity setpoints and measured angular
velocity as inputs and outputs actuator setpoints.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME(MODULE_NAME, "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Angular velocity controller app start / stop handling function
 */
extern "C" __EXPORT int angular_velocity_controller_main(int argc, char *argv[]);

int angular_velocity_controller_main(int argc, char *argv[])
{
	return AngularVelocityController::main(argc, argv);
}
