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
 * This module is a modification of the hippocampus control module and is designed for the
 * BlueROV2.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Tim Hansen <t.hansen@jacobs-university.de>
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 */

#include "uuv_pos_control.hpp"



/**
 * UUV pos_controller app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int uuv_pos_control_main(int argc, char *argv[]);


UUVPOSControl::UUVPOSControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

UUVPOSControl::~UUVPOSControl()
{
	perf_free(_loop_perf);
}

bool UUVPOSControl::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_pos callback registration failed!");
		return false;
	}

	return true;
}

void UUVPOSControl::parameters_update(bool force)
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

void UUVPOSControl::publish_attitude_setpoint(const float thrust_x, const float thrust_y, const float thrust_z,
		const float roll_des, const float pitch_des, const float yaw_des)
{
	//watch if inputs are not to high
	vehicle_attitude_setpoint_s vehicle_attitude_setpoint = {};
	vehicle_attitude_setpoint.timestamp = hrt_absolute_time();

	vehicle_attitude_setpoint.roll_body = roll_des;
	vehicle_attitude_setpoint.pitch_body = pitch_des;
	vehicle_attitude_setpoint.yaw_body = yaw_des;

	vehicle_attitude_setpoint.thrust_body[0] = thrust_x;
	vehicle_attitude_setpoint.thrust_body[1] = thrust_y;
	vehicle_attitude_setpoint.thrust_body[2] = thrust_z;


	_att_sp_pub.publish(vehicle_attitude_setpoint);
}

void UUVPOSControl::pose_controller_6dof(const float x_pos_des, const float y_pos_des, const float z_pos_des,
		const float roll_des, const float pitch_des, const float yaw_des,
		vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos)
{
	//get current rotation of vehicle
	Quatf q_att(vehicle_attitude.q);
	Vector3f pos_des = Vector3f(x_pos_des, y_pos_des, z_pos_des);

	Vector3f p_control_output = Vector3f(_param_pose_gain_x.get() * (pos_des(0) - vlocal_pos.x) - _param_pose_gain_d_x.get()
					     * vlocal_pos.vx,
					     _param_pose_gain_y.get() * (pos_des(1) - vlocal_pos.y) - _param_pose_gain_d_y.get() * vlocal_pos.vy,
					     _param_pose_gain_z.get() * (pos_des(2) - vlocal_pos.z) - _param_pose_gain_d_z.get() * vlocal_pos.vz);

	Vector3f rotated_input = q_att.conjugate_inversed(p_control_output);//rotate the coord.sys (from global to body)

	publish_attitude_setpoint(rotated_input(0),
				  rotated_input(1),
				  rotated_input(2),
				  roll_des, pitch_des, yaw_des);

}

void UUVPOSControl::stabilization_controller_6dof(const float x_pos_des, const float y_pos_des, const float z_pos_des,
		const float roll_des, const float pitch_des, const float yaw_des,
		vehicle_attitude_s &vehicle_attitude, vehicle_local_position_s &vlocal_pos)
{
	//get current rotation of vehicle
	Quatf q_att(vehicle_attitude.q);
	Vector3f pos_des = Vector3f(0, 0, z_pos_des);

	Vector3f p_control_output = Vector3f(0,
					     0,
					     _param_pose_gain_z.get() * (pos_des(2) - vlocal_pos.z));
	//potential d controller missing
	Vector3f rotated_input = q_att.conjugate_inversed(p_control_output);//rotate the coord.sys (from global to body)

	publish_attitude_setpoint(rotated_input(0) + x_pos_des, rotated_input(1) + y_pos_des, rotated_input(2),
				  roll_des, pitch_des, yaw_des);

}

void UUVPOSControl::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	_vcontrol_mode_sub.update(&_vcontrol_mode);


	/* update parameters from storage */
	parameters_update();

	//vehicle_attitude_s attitude;
	vehicle_local_position_s vlocal_pos;

	/* only run controller if local_pos changed */
	if (_vehicle_local_position_sub.update(&vlocal_pos)) {

		/* Run geometric attitude controllers if NOT manual mode*/
		if (!_vcontrol_mode.flag_control_manual_enabled
		    && _vcontrol_mode.flag_control_attitude_enabled
		    && _vcontrol_mode.flag_control_rates_enabled) {

			_vehicle_attitude_sub.update(&_vehicle_attitude);//get current vehicle attitude
			_trajectory_setpoint_sub.update(&_trajectory_setpoint);

			float roll_des = 0;
			float pitch_des = 0;
			float yaw_des = _trajectory_setpoint.yaw;

			float x_pos_des = _trajectory_setpoint.position[0];
			float y_pos_des = _trajectory_setpoint.position[1];
			float z_pos_des = _trajectory_setpoint.position[2];

			//stabilization controller(keep pos and hold depth + angle) vs position controller(global + yaw)
			if (_param_stabilization.get() == 0) {
				pose_controller_6dof(x_pos_des, y_pos_des, z_pos_des,
						     roll_des, pitch_des, yaw_des, _vehicle_attitude, vlocal_pos);

			} else {
				stabilization_controller_6dof(x_pos_des, y_pos_des, z_pos_des,
							      roll_des, pitch_des, yaw_des, _vehicle_attitude, vlocal_pos);
			}
		}
	}

	/* Manual Control mode (e.g. gamepad,...) - raw feedthrough no assistance */
	if (_manual_control_setpoint_sub.update(&_manual_control_setpoint)) {
		// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
		// returning immediately and this loop will eat up resources.
		if (_vcontrol_mode.flag_control_manual_enabled && !_vcontrol_mode.flag_control_rates_enabled) {
			/* manual/direct control */
		}

	}

	/* Only publish if any of the proper modes are enabled */
	if (_vcontrol_mode.flag_control_manual_enabled ||
	    _vcontrol_mode.flag_control_attitude_enabled) {
	}

	perf_end(_loop_perf);
}

int UUVPOSControl::task_spawn(int argc, char *argv[])
{
	UUVPOSControl *instance = new UUVPOSControl();

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

int UUVPOSControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int UUVPOSControl::print_usage(const char *reason)
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
$ uuv_pos_control start
$ uuv_pos_control status
$ uuv_pos_control stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("uuv_pos_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start")
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int uuv_pos_control_main(int argc, char *argv[])
{
    return UUVPOSControl::main(argc, argv);
}
