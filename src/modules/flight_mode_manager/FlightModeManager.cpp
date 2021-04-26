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

#include "FlightModeManager.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace time_literals;

FlightModeManager::FlightModeManager() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();

	// initialize all flight-tasks
	// currently this is required to get all parameters read
	for (int i = 0; i < static_cast<int>(FlightTaskIndex::Count); i++) {
		_initTask(static_cast<FlightTaskIndex>(i));
	}

	// disable all tasks
	_initTask(FlightTaskIndex::None);
}

FlightModeManager::~FlightModeManager()
{
	if (_current_task.task) {
		_current_task.task->~FlightTask();
	}

	delete _wv_controller;
	perf_free(_loop_perf);
}

bool FlightModeManager::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}

	// limit to every other vehicle_local_position update (50 Hz)
	_vehicle_local_position_sub.set_interval_us(20_ms);
	_time_stamp_last_loop = hrt_absolute_time();
	return true;
}

void FlightModeManager::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
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
	}

	// generate setpoints on local position changes
	vehicle_local_position_s vehicle_local_position;

	if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
		const hrt_abstime time_stamp_now = hrt_absolute_time();
		// Guard against too small (< 0.2ms) and too large (> 100ms) dt's.
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) / 1e6f), 0.0002f, 0.1f);
		_time_stamp_last_loop = time_stamp_now;

		_home_position_sub.update();
		_vehicle_control_mode_sub.update();
		_vehicle_land_detected_sub.update();

		if (_vehicle_status_sub.update()) {
			if (_vehicle_status_sub.get().is_vtol && (_wv_controller == nullptr)) {
				// if vehicle is a VTOL we want to enable weathervane capabilities
				_wv_controller = new WeatherVane();
			}
		}

		// activate the weathervane controller if required. If activated a flighttask can use it to implement a yaw-rate control strategy
		// that turns the nose of the vehicle into the wind
		if (_wv_controller != nullptr) {

			// in manual mode we just want to use weathervane if position is controlled as well
			// in mission, enabling wv is done in flight task
			if (_vehicle_control_mode_sub.get().flag_control_manual_enabled) {
				if (_vehicle_control_mode_sub.get().flag_control_position_enabled && _wv_controller->weathervane_enabled()) {
					_wv_controller->activate();

				} else {
					_wv_controller->deactivate();
				}
			}

			vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
			_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint);
			_wv_controller->update(matrix::Quatf(vehicle_attitude_setpoint.q_d).dcm_z(), vehicle_local_position.heading);
		}

		start_flight_task();

		if (_vehicle_command_sub.updated()) {
			handleCommand();
		}

		if (isAnyTaskActive()) {
			generateTrajectorySetpoint(dt, vehicle_local_position);
		}

	}

	perf_end(_loop_perf);
}

void FlightModeManager::updateParams()
{
	ModuleParams::updateParams();

	if (isAnyTaskActive()) {
		_current_task.task->handleParameterUpdate();
	}

	if (_wv_controller != nullptr) {
		_wv_controller->update_parameters();
	}
}

void FlightModeManager::start_flight_task()
{
	bool task_failure = false;
	bool should_disable_task = true;
	int prev_failure_count = _task_failure_count;

	// Do not run any flight task for VTOLs in fixed-wing mode
	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		switchTask(FlightTaskIndex::None);
		return;
	}

	// Only run transition flight task if altitude control is enabled (e.g. in Altitdue, Position, Auto flight mode)
	if (_vehicle_status_sub.get().in_transition_mode && _vehicle_control_mode_sub.get().flag_control_altitude_enabled) {

		should_disable_task = false;
		FlightTaskError error = switchTask(FlightTaskIndex::Transition);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Transition activation failed with error: %s", errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

		return;
	}

	// Auto-follow me
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) {
		should_disable_task = false;
		FlightTaskError error = switchTask(FlightTaskIndex::AutoFollowMe);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Follow-Me activation failed with error: %s", errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_vehicle_control_mode_sub.get().flag_control_auto_enabled) {
		// Auto related maneuvers
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error = switchTask(FlightTaskIndex::AutoLineSmoothVel);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Auto activation failed with error: %s", errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {

		// Emergency descend
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error = switchTask(FlightTaskIndex::Descend);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Descend activation failed with error: %s", errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	}

	// manual position control
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 0:
			error = switchTask(FlightTaskIndex::ManualPosition);
			break;

		case 3:
			error = switchTask(FlightTaskIndex::ManualPositionSmoothVel);
			break;

		case 4:
		default:
			if (_param_mpc_pos_mode.get() != 4) {
				PX4_ERR("MPC_POS_MODE %d invalid, resetting", _param_mpc_pos_mode.get());
				_param_mpc_pos_mode.set(4);
				_param_mpc_pos_mode.commit();
			}

			error = switchTask(FlightTaskIndex::ManualAcceleration);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Position-Ctrl activation failed with error: %s", errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_POSCTL);
			task_failure = false;
		}
	}

	// manual altitude control
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 0:
			error = switchTask(FlightTaskIndex::ManualAltitude);
			break;

		case 3:
		default:
			error = switchTask(FlightTaskIndex::ManualAltitudeSmoothVel);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Altitude-Ctrl activation failed with error: %s", errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_ALTCTL);
			task_failure = false;
		}
	}

	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT) {
		should_disable_task = false;
	}

	// check task failure
	if (task_failure) {

		// for some reason no flighttask was able to start.
		// go into failsafe flighttask
		FlightTaskError error = switchTask(FlightTaskIndex::Failsafe);

		if (error != FlightTaskError::NoError) {
			// No task was activated.
			switchTask(FlightTaskIndex::None);
		}

	} else if (should_disable_task) {
		switchTask(FlightTaskIndex::None);
	}

	_last_vehicle_nav_state = _vehicle_status_sub.get().nav_state;
}

void FlightModeManager::check_failure(bool task_failure, uint8_t nav_state)
{
	if (!task_failure) {
		// we want to be in this mode, reset the failure count
		_task_failure_count = 0;

	} else if (_task_failure_count > NUM_FAILURE_TRIES) {
		// tell commander to switch mode
		PX4_WARN("Previous flight task failed, switching to mode %d", nav_state);
		send_vehicle_cmd_do(nav_state);
		_task_failure_count = 0; // avoid immediate resending of a vehicle command in the next iteration
	}
}

void FlightModeManager::send_vehicle_cmd_do(uint8_t nav_state)
{
	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;

	// set the main mode
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_STAB:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
		command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	default: //vehicle_status_s::NAVIGATION_STATE_POSCTL
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;
	}

	// publish the vehicle command
	command.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(command);
}

void FlightModeManager::handleCommand()
{
	// get command
	vehicle_command_s command;

	while (_vehicle_command_sub.update(&command)) {
		// check what command it is
		FlightTaskIndex desired_task = switchVehicleCommand(command.command);

		// ignore all unkown commands
		if (desired_task != FlightTaskIndex::None) {
			// switch to the commanded task
			FlightTaskError switch_result = switchTask(desired_task);
			uint8_t cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;

			// if we are in/switched to the desired task
			if (switch_result >= FlightTaskError::NoError) {
				cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

				// if the task is running apply parameters to it and see if it rejects
				if (isAnyTaskActive() && !_current_task.task->applyCommandParameters(command)) {
					cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_DENIED;

					// if we just switched and parameters are not accepted, go to failsafe
					if (switch_result >= FlightTaskError::NoError) {
						switchTask(FlightTaskIndex::ManualPosition);
						cmd_result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;
					}
				}
			}

			// send back acknowledgment
			vehicle_command_ack_s command_ack{};
			command_ack.command = command.command;
			command_ack.result = cmd_result;
			command_ack.result_param1 = static_cast<int>(switch_result);
			command_ack.target_system = command.source_system;
			command_ack.target_component = command.source_component;
			command_ack.timestamp = hrt_absolute_time();
			_vehicle_command_ack_pub.publish(command_ack);
		}
	}
}

void FlightModeManager::generateTrajectorySetpoint(const float dt,
		const vehicle_local_position_s &vehicle_local_position)
{
	_current_task.task->setYawHandler(_wv_controller);

	// Inform FlightTask about the input and output of the velocity controller
	// This is used to properly initialize the velocity setpoint when onpening the position loop (position unlock)
	if (_vehicle_local_position_setpoint_sub.updated()) {
		vehicle_local_position_setpoint_s vehicle_local_position_setpoint;

		if (_vehicle_local_position_setpoint_sub.copy(&vehicle_local_position_setpoint)) {
			const Vector3f vel_sp{vehicle_local_position_setpoint.vx, vehicle_local_position_setpoint.vy, vehicle_local_position_setpoint.vz};
			const Vector3f acc_sp{vehicle_local_position_setpoint.acceleration};
			_current_task.task->updateVelocityControllerFeedback(vel_sp, acc_sp);
		}
	}

	// If the task fails sned out empty NAN setpoints and the controller will emergency failsafe
	vehicle_local_position_setpoint_s setpoint = FlightTask::empty_setpoint;
	vehicle_constraints_s constraints = FlightTask::empty_constraints;

	if (_current_task.task->updateInitialize() && _current_task.task->update() && _current_task.task->updateFinalize()) {
		// setpoints and constraints for the position controller from flighttask
		setpoint = _current_task.task->getPositionSetpoint();
		constraints = _current_task.task->getConstraints();
	}

	// limit altitude according to land detector
	limitAltitude(setpoint, vehicle_local_position);

	if (_takeoff_status_sub.updated()) {
		takeoff_status_s takeoff_status;

		if (_takeoff_status_sub.copy(&takeoff_status)) {
			_takeoff_state = takeoff_status.takeoff_state;
		}
	}

	if (_takeoff_state < takeoff_status_s::TAKEOFF_STATE_RAMPUP) {
		// reactivate the task which will reset the setpoint to current state
		_current_task.task->reActivate();
	}


	setpoint.timestamp = hrt_absolute_time();
	_trajectory_setpoint_pub.publish(setpoint);

	constraints.timestamp = hrt_absolute_time();
	_vehicle_constraints_pub.publish(constraints);

	// if there's any change in landing gear setpoint publish it
	landing_gear_s landing_gear = _current_task.task->getGear();

	if (landing_gear.landing_gear != _old_landing_gear_position
	    && landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {

		landing_gear.timestamp = hrt_absolute_time();
		_landing_gear_pub.publish(landing_gear);
	}

	_old_landing_gear_position = landing_gear.landing_gear;
}

void FlightModeManager::limitAltitude(vehicle_local_position_setpoint_s &setpoint,
				      const vehicle_local_position_s &vehicle_local_position)
{
	if (_vehicle_land_detected_sub.get().alt_max < 0.0f || !_home_position_sub.get().valid_alt
	    || !vehicle_local_position.z_valid || !vehicle_local_position.v_z_valid) {
		// there is no altitude limitation present or the required information not available
		return;
	}

	// maximum altitude == minimal z-value (NED)
	const float min_z = _home_position_sub.get().z + (-_vehicle_land_detected_sub.get().alt_max);

	if (vehicle_local_position.z < min_z) {
		// above maximum altitude, only allow downwards flight == positive vz-setpoints (NED)
		setpoint.z = min_z;
		setpoint.vz = math::max(setpoint.vz, 0.f);
	}
}

FlightTaskError FlightModeManager::switchTask(FlightTaskIndex new_task_index)
{
	// switch to the running task, nothing to do
	if (new_task_index == _current_task.index) {
		return FlightTaskError::NoError;
	}

	// Save current setpoints for the next FlightTask
	vehicle_local_position_setpoint_s last_setpoint = FlightTask::empty_setpoint;
	ekf_reset_counters_s last_reset_counters{};

	if (isAnyTaskActive()) {
		last_setpoint = _current_task.task->getPositionSetpoint();
		last_reset_counters = _current_task.task->getResetCounters();
	}

	if (_initTask(new_task_index)) {
		// invalid task
		return FlightTaskError::InvalidTask;
	}

	if (!isAnyTaskActive()) {
		// no task running
		return FlightTaskError::NoError;
	}

	// activation failed
	if (!_current_task.task->updateInitialize() || !_current_task.task->activate(last_setpoint)) {
		_current_task.task->~FlightTask();
		_current_task.task = nullptr;
		_current_task.index = FlightTaskIndex::None;
		return FlightTaskError::ActivationFailed;
	}

	_current_task.task->setResetCounters(last_reset_counters);

	return FlightTaskError::NoError;
}

FlightTaskError FlightModeManager::switchTask(int new_task_index)
{
	// make sure we are in range of the enumeration before casting
	if (static_cast<int>(FlightTaskIndex::None) <= new_task_index &&
	    static_cast<int>(FlightTaskIndex::Count) > new_task_index) {
		return switchTask(FlightTaskIndex(new_task_index));
	}

	switchTask(FlightTaskIndex::None);
	return FlightTaskError::InvalidTask;
}

const char *FlightModeManager::errorToString(const FlightTaskError error)
{
	switch (error) {
	case FlightTaskError::NoError: return "No Error";

	case FlightTaskError::InvalidTask: return "Invalid Task ";

	case FlightTaskError::ActivationFailed: return "Activation Failed";
	}

	return "This error is not mapped to a string or is unknown.";
}

int FlightModeManager::task_spawn(int argc, char *argv[])
{
	FlightModeManager *instance = new FlightModeManager();

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

int FlightModeManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlightModeManager::print_status()
{
	if (isAnyTaskActive()) {
		PX4_INFO("Running, active flight task: %i", static_cast<uint32_t>(_current_task.index));

	} else {
		PX4_INFO("Running, no flight task active");
	}

	perf_print_counter(_loop_perf);
	return 0;
}

int FlightModeManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the setpoint generation for all modes. It takes the current mode state of the vehicle as input
and outputs setpoints for controllers.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flight_mode_manager", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flight_mode_manager_main(int argc, char *argv[])
{
	return FlightModeManager::main(argc, argv);
}
