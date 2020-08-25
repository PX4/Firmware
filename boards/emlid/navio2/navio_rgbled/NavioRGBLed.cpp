/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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

#include "NavioRGBLed.hpp"

NavioRGBLed::NavioRGBLed() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
};

NavioRGBLed::~NavioRGBLed()
{
	ScheduleClear();

	_ledR.off();
	_ledG.off();
	_ledB.off();
}

int NavioRGBLed::init()
{
	_ledR.off();
	_ledG.off();
	_ledB.off();

	// kick off work queue
	ScheduleNow();

	return PX4_OK;
}

void NavioRGBLed::Run()
{
	LedControlData led_control_data{};

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_ledR.on();
			_ledG.off();
			_ledB.off();
			break;

		case led_control_s::COLOR_GREEN:
			_ledR.off();
			_ledG.on();
			_ledB.off();
			break;

		case led_control_s::COLOR_BLUE:
			_ledR.off();
			_ledG.off();
			_ledB.on();
			break;

		case led_control_s::COLOR_AMBER: // make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_ledR.on();
			_ledG.on();
			_ledB.off();
			break;

		case led_control_s::COLOR_PURPLE:
			_ledR.on();
			_ledG.off();
			_ledB.on();
			break;

		case led_control_s::COLOR_CYAN:
			_ledR.off();
			_ledG.on();
			_ledB.on();
			break;

		case led_control_s::COLOR_WHITE:
			_ledR.on();
			_ledG.on();
			_ledB.on();
			break;

		default: // led_control_s::COLOR_OFF
			_ledR.off();
			_ledG.off();
			_ledB.off();
			break;
		}
	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

int NavioRGBLed::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NavioRGBLed::task_spawn(int argc, char *argv[])
{
	NavioRGBLed *instance = new NavioRGBLed();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

int NavioRGBLed::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Emlid Navio2 RGB LED driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("navio_rgbled", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int navio_rgbled_main(int argc, char *argv[])
{
	return NavioRGBLed::main(argc, argv);
}
