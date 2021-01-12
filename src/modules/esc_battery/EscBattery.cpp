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

#include "EscBattery.hpp"

using namespace time_literals;

EscBattery::EscBattery() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_battery(1, this, ESC_BATTERY_INTERVAL_US)
{
}

bool
EscBattery::init()
{
	if (!_esc_status_sub.registerCallback()) {
		PX4_ERR("esc_status callback registration failed!");
		return false;
	}

	_esc_status_sub.set_interval_us(ESC_BATTERY_INTERVAL_US);

	return true;
}

void
EscBattery::parameters_updated()
{
	ModuleParams::updateParams();
}

void
EscBattery::Run()
{
	if (should_exit()) {
		_esc_status_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		// Clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		parameters_updated();
	}

	esc_status_s esc_status;

	if (_esc_status_sub.copy(&esc_status)) {

		int online_bitmask = (1 << esc_status.esc_count) - 1;

		if (online_bitmask != esc_status.esc_online_flags || esc_status.esc_count == 0 ||
		    esc_status.esc_count > esc_status_s::CONNECTED_ESC_MAX) {
			return;
		}

		float average_voltage_v = 0.0f;
		float total_current_a = 0.0f;

		for (unsigned i = 0; i < esc_status.esc_count; ++i) {
			average_voltage_v += esc_status.esc[i].esc_voltage;
			total_current_a += esc_status.esc[i].esc_current;
		}

		average_voltage_v /= esc_status.esc_count;

		const bool connected = true;
		const int priority = 0;

		actuator_controls_s ctrl;
		_actuator_ctrl_0_sub.copy(&ctrl);

		_battery.updateBatteryStatus(
			esc_status.timestamp,
			average_voltage_v,
			total_current_a,
			connected,
			battery_status_s::BATTERY_SOURCE_ESCS,
			priority,
			ctrl.control[actuator_controls_s::INDEX_THROTTLE]);
	}
}

int EscBattery::task_spawn(int argc, char *argv[])
{
	EscBattery *instance = new EscBattery();

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

int EscBattery::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EscBattery::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements using information from the ESC status and publish it as battery status.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("esc_battery", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int esc_battery_main(int argc, char *argv[])
{
	return EscBattery::main(argc, argv);
}
