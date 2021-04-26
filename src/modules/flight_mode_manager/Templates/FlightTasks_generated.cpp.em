/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @@file FlightTasks_generated.cpp
 *
 * Generated file to switch between all required flight tasks
 *
 * @@author Christoph Tobler <christoph@@px4.io>
 */

#include "FlightModeManager.hpp"
#include "FlightTasks_generated.hpp"

int FlightModeManager::_initTask(FlightTaskIndex task_index)
{

	// disable the old task if there is any
	if (_current_task.task) {
		_current_task.task->~FlightTask();
		_current_task.task = nullptr;
		_current_task.index = FlightTaskIndex::None;
	}

	switch (task_index) {
	case FlightTaskIndex::None:
		// already disabled task
		break;

@# loop through all requested tasks
@[if tasks]@
@[for task in tasks]@
@{
firstLowercase = lambda s: s[:1].lower() + s[1:] if s else ''
}@
	case FlightTaskIndex::@(task):
		_current_task.task = new (&_task_union.@(firstLowercase(task))) FlightTask@(task)();
		break;

@[end for]@
@[end if]@
	default:
		// invalid task
		return 1;
	}

	// task construction succeeded
	_current_task.index = task_index;
	return 0;
}

FlightTaskIndex FlightModeManager::switchVehicleCommand(const int command)
{
    switch (command) {
@# loop through all additional tasks
@[if tasks_add]@
@[for task in tasks_add]@
@{
upperCase = lambda s: s[:].upper() if s else ''
}@
	case vehicle_command_s::VEHICLE_CMD_DO_@(upperCase(task)) :
		return FlightTaskIndex::@(task);
		break;

@[end for]@
@[end if]@
	// ignore all unkown commands
	default : return FlightTaskIndex::None;
	}
}
