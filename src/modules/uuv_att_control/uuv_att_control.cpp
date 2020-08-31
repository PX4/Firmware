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
 *
 * This module is a modification of the fixed wing / rover module  and it is designed for unmanned underwater vehicles  (UUV).
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 * @author Philipp Hastedt <philipp.hastedt@tuhh.de>
 * @author Tim Hansen <t.hansen@tuhh.de>
 */

#include "uuv_att_control.hpp"


#define ACTUATOR_PUBLISH_PERIOD_MS 4


/**
 * UUV attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int uuv_att_control_main(int argc, char *argv[]);


UUVAttitudeControl::UUVAttitudeControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::attitude_ctrl),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

UUVAttitudeControl::~UUVAttitudeControl()
{
	perf_free(_loop_perf);
}

bool UUVAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed!");
		return false;
	}

	return true;
}

void UUVAttitudeControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void UUVAttitudeControl::constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u, float thrust_u)
{
	if (PX4_ISFINITE(roll_u)) {
		roll_u = math::constrain(roll_u, -1.0f, 1.0f);
		_actuators.control[actuator_controls_s::INDEX_ROLL] = roll_u;

	} else {
		_actuators.control[actuator_controls_s::INDEX_ROLL] = 0.0f;
	}

	if (PX4_ISFINITE(pitch_u)) {
		pitch_u = math::constrain(pitch_u, -1.0f, 1.0f);
		_actuators.control[actuator_controls_s::INDEX_PITCH] = pitch_u;

	} else {
		_actuators.control[actuator_controls_s::INDEX_PITCH] = 0.0f;
	}

	if (PX4_ISFINITE(yaw_u)) {
		yaw_u = math::constrain(yaw_u, -1.0f, 1.0f);
		_actuators.control[actuator_controls_s::INDEX_YAW] = yaw_u;

	} else {
		_actuators.control[actuator_controls_s::INDEX_YAW] = 0.0f;
	}

	if (PX4_ISFINITE(thrust_u)) {
		thrust_u = math::constrain(thrust_u, -1.0f, 1.0f);
		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = thrust_u;

	} else {
		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
	}
}

void UUVAttitudeControl::control_attitude_geo(const vehicle_attitude_s &attitude,
		const vehicle_attitude_setpoint_s &attitude_setpoint, const vehicle_angular_velocity_s &angular_velocity,
		const vehicle_rates_setpoint_s &rates_setpoint)
{
	/** Geometric Controller
	 *
	 * based on
	 * D. Mellinger, V. Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors", IEEE ICRA 2011, pp. 2520-2525.
	 * D. A. Duecker, A. Hackbarth, T. Johannink, E. Kreuzer, and E. Solowjow, “Micro Underwater Vehicle Hydrobatics: A SubmergedFuruta Pendulum,” IEEE ICRA 2018, pp. 7498–7503.
	 */
	Eulerf euler_angles(matrix::Quatf(attitude.q));

	float roll_u;
	float pitch_u;
	float yaw_u;
	float thrust_u;

	float roll_body = attitude_setpoint.roll_body;
	float pitch_body = attitude_setpoint.pitch_body;
	float yaw_body = attitude_setpoint.yaw_body;

	float roll_rate_desired = rates_setpoint.roll;
	float pitch_rate_desired = rates_setpoint.pitch;
	float yaw_rate_desired = rates_setpoint.yaw;

	/* get attitude setpoint rotational matrix */
	Dcmf rot_des = Eulerf(roll_body, pitch_body, yaw_body);

	/* get current rotation matrix from control state quaternions */
	Quatf q_att(attitude.q);
	Matrix3f rot_att =  matrix::Dcm<float>(q_att);

	Vector3f e_R_vec;
	Vector3f torques;

	/* Compute matrix: attitude error */
	Matrix3f e_R = (rot_des.transpose() * rot_att - rot_att.transpose() * rot_des) * 0.5;

	/* vee-map the error to get a vector instead of matrix e_R */
	e_R_vec(0) = e_R(2, 1);  /**< Roll  */
	e_R_vec(1) = e_R(0, 2);  /**< Pitch */
	e_R_vec(2) = e_R(1, 0);  /**< Yaw   */

	Vector3f omega{angular_velocity.xyz};
	omega(0) -= roll_rate_desired;
	omega(1) -= pitch_rate_desired;
	omega(2) -= yaw_rate_desired;

	/**< P-Control */
	torques(0) = - e_R_vec(0) * _param_roll_p.get();	/**< Roll  */
	torques(1) = - e_R_vec(1) * _param_pitch_p.get();	/**< Pitch */
	torques(2) = - e_R_vec(2) * _param_yaw_p.get();		/**< Yaw   */

	/**< PD-Control */
	torques(0) = torques(0) - omega(0) * _param_roll_d.get();  /**< Roll  */
	torques(1) = torques(1) - omega(1) * _param_pitch_d.get(); /**< Pitch */
	torques(2) = torques(2) - omega(2) * _param_yaw_d.get();   /**< Yaw   */

	roll_u = torques(0);
	pitch_u = torques(1);
	yaw_u = torques(2);

	// take thrust as
	thrust_u = attitude_setpoint.thrust_body[0];

	constrain_actuator_commands(roll_u, pitch_u, yaw_u, thrust_u);
	/* Geometric Controller END*/
}

void UUVAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	/* check vehicle control mode for changes to publication state */
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	/* update parameters from storage */
	parameters_update();

	vehicle_attitude_s attitude;

	/* only run controller if attitude changed */
	if (_vehicle_attitude_sub.update(&attitude)) {
		vehicle_angular_velocity_s angular_velocity {};
		_angular_velocity_sub.copy(&angular_velocity);

		/* Run geometric attitude controllers if NOT manual mode*/
		if (!_vcontrol_mode.flag_control_manual_enabled
		    && _vcontrol_mode.flag_control_attitude_enabled
		    && _vcontrol_mode.flag_control_rates_enabled) {

			int input_mode = _param_input_mode.get();

			_vehicle_attitude_setpoint_sub.update(&_attitude_setpoint);
			_vehicle_rates_setpoint_sub.update(&_rates_setpoint);

			if (input_mode == 1) { // process manual data
				_attitude_setpoint.roll_body = _param_direct_roll.get();
				_attitude_setpoint.pitch_body = _param_direct_pitch.get();
				_attitude_setpoint.yaw_body = _param_direct_yaw.get();
				_attitude_setpoint.thrust_body[0] = _param_direct_thrust.get();
			}

			/* Geometric Control*/
			control_attitude_geo(attitude, _attitude_setpoint, angular_velocity, _rates_setpoint);
		}
	}

	perf_end(_loop_perf);

	/* Manual Control mode (e.g. gamepad,...) - raw feedthrough no assistance */
	if (_manual_control_setpoint_sub.update(&_manual_control_setpoint)) {
		// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
		// returning immediately and this loop will eat up resources.
		if (_vcontrol_mode.flag_control_manual_enabled && !_vcontrol_mode.flag_control_rates_enabled) {
			/* manual/direct control */
			constrain_actuator_commands(_manual_control_setpoint.y, -_manual_control_setpoint.x,
						    _manual_control_setpoint.r, _manual_control_setpoint.z);
		}

	}

	_actuators.timestamp = hrt_absolute_time();

	/* Only publish if any of the proper modes are enabled */
	if (_vcontrol_mode.flag_control_manual_enabled ||
	    _vcontrol_mode.flag_control_attitude_enabled) {
		/* publish the actuator controls */
		_actuator_controls_pub.publish(_actuators);

	}

	warnx("exiting.\n");
}

int UUVAttitudeControl::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("uuv_att_ctrl",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ATTITUDE_CONTROL,
				      1700,  // maybe switch to 1500
				      (px4_main_t)&UUVAttitudeControl::run_trampoline,
				      nullptr);

	if (_task_id < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

UUVAttitudeControl *UUVAttitudeControl::instantiate(int argc, char *argv[])
{

	if (argc > 0) {
		PX4_WARN("Command 'start' takes no arguments.");
		return nullptr;
	}

	UUVAttitudeControl *instance = new UUVAttitudeControl();

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate UUVAttitudeControl object");
	}

	return instance;
}

int UUVAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int UUVAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an unmanned underwater vehicle (UUV).

Publishes `actuator_controls_0` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
 * Auto mission: The uuv runs missions

### Examples
CLI usage example:
$ uuv_att_control start
$ uuv_att_control status
$ uuv_att_control stop

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uuv_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int uuv_att_control_main(int argc, char *argv[])
{
	return UUVAttitudeControl::main(argc, argv);
}
