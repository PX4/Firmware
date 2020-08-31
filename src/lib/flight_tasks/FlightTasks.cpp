#include "FlightTasks.hpp"

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

FlightTasks::FlightTasks()
{
	// initialize all flight-tasks
	// currently this is required to get all parameters read
	for (int i = 0; i < static_cast<int>(FlightTaskIndex::Count); i++) {
		_initTask(static_cast<FlightTaskIndex>(i));
	}

	// disable all tasks
	_initTask(FlightTaskIndex::None);
}

bool FlightTasks::update()
{
	_updateCommand();

	if (isAnyTaskActive()) {
		return _current_task.task->updateInitialize() && _current_task.task->update() && _current_task.task->updateFinalize();
	}

	return false;
}

const vehicle_local_position_setpoint_s FlightTasks::getPositionSetpoint()
{
	if (isAnyTaskActive()) {
		return _current_task.task->getPositionSetpoint();

	} else {
		return FlightTask::empty_setpoint;
	}
}

const ekf_reset_counters_s FlightTasks::getResetCounters()
{
	if (isAnyTaskActive()) {
		return _current_task.task->getResetCounters();

	} else {
		return FlightTask::zero_reset_counters;
	}
}

const vehicle_constraints_s FlightTasks::getConstraints()
{
	if (isAnyTaskActive()) {
		return _current_task.task->getConstraints();

	} else {
		return FlightTask::empty_constraints;
	}
}

const landing_gear_s FlightTasks::getGear()
{
	if (isAnyTaskActive()) {
		return _current_task.task->getGear();

	} else {
		return FlightTask::empty_landing_gear_default_keep;
	}
}

FlightTaskError FlightTasks::switchTask(FlightTaskIndex new_task_index)
{
	// switch to the running task, nothing to do
	if (new_task_index == _current_task.index) {
		return FlightTaskError::NoError;
	}

	// Save current setpoints for the next FlightTask
	const vehicle_local_position_setpoint_s last_setpoint = getPositionSetpoint();
	const ekf_reset_counters_s last_reset_counters = getResetCounters();

	if (_initTask(new_task_index)) {
		// invalid task
		return FlightTaskError::InvalidTask;
	}

	if (!_current_task.task) {
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

FlightTaskError FlightTasks::switchTask(int new_task_index)
{
	// make sure we are in range of the enumeration before casting
	if (static_cast<int>(FlightTaskIndex::None) <= new_task_index &&
	    static_cast<int>(FlightTaskIndex::Count) > new_task_index) {
		return switchTask(FlightTaskIndex(new_task_index));
	}

	switchTask(FlightTaskIndex::None);
	return FlightTaskError::InvalidTask;
}

void FlightTasks::handleParameterUpdate()
{
	if (isAnyTaskActive()) {
		_current_task.task->handleParameterUpdate();
	}
}

const char *FlightTasks::errorToString(const FlightTaskError error)
{
	for (auto e : _taskError) {
		if (static_cast<FlightTaskError>(e.error) == error) {
			return e.msg;
		}
	}

	return "This error is not mapped to a string or is unknown.";
}

void FlightTasks::reActivate()
{
	if (_current_task.task) {
		_current_task.task->reActivate();
	}
}

void FlightTasks::_updateCommand()
{
	// check if there's any new command
	bool updated = _sub_vehicle_command.updated();

	if (!updated) {
		return;
	}

	// get command
	vehicle_command_s command{};
	_sub_vehicle_command.copy(&command);

	// check what command it is
	FlightTaskIndex desired_task = switchVehicleCommand(command.command);

	if (desired_task == FlightTaskIndex::None) {
		// ignore all unkown commands
		return;
	}

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
	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = command.command;
	command_ack.result = cmd_result;
	command_ack.result_param1 = static_cast<int>(switch_result);
	command_ack.target_system = command.source_system;
	command_ack.target_component = command.source_component;

	_pub_vehicle_command_ack.publish(command_ack);
}
