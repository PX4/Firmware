/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "send_event.h"

#include <math.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

namespace events
{

// Run it at 30 Hz.
static constexpr uint32_t SEND_EVENT_INTERVAL_US{1_s / 30};

int SendEvent::task_spawn(int argc, char *argv[])
{
	SendEvent *send_event = new SendEvent();

	if (!send_event) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(send_event);
	_task_id = task_id_is_work_queue;

	send_event->start();

	return 0;
}

SendEvent::SendEvent() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	if (_param_ev_tsk_stat_dis.get()) {
		_status_display = new status::StatusDisplay();
	}

	if (_param_ev_tsk_rc_loss.get()) {
		_rc_loss_alarm = new rc_loss::RC_Loss_Alarm();
	}
}

SendEvent::~SendEvent()
{
	ScheduleClear();

	delete _status_display;
	delete _rc_loss_alarm;
}

int SendEvent::start()
{
	ScheduleOnInterval(SEND_EVENT_INTERVAL_US, 10000);

	return PX4_OK;
}

void SendEvent::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	process_commands();

	if (_status_display != nullptr) {
		_status_display->process();
	}

	if (_rc_loss_alarm != nullptr) {
		_rc_loss_alarm->process();
	}
}

void SendEvent::process_commands()
{
	// TODO: do something with vehicle commands
	// TODO: what is this modules purpose?
}

void SendEvent::answer_command(const vehicle_command_s &cmd, unsigned result)
{
	/* publish ACK */
	vehicle_command_ack_s command_ack{};
	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.result = (uint8_t)result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;

	uORB::Publication<vehicle_command_ack_s>	command_ack_pub{ORB_ID(vehicle_command_ack)};
	command_ack_pub.publish(command_ack);
}

int SendEvent::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to perform housekeeping tasks.
It is currently only responsible for tone alarm on RC Loss.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("send_event", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int SendEvent::custom_command(int argc, char *argv[])
{
	// TODO: what is my purpose?
	print_usage("unrecognized command");

	return 0;
}

int send_event_main(int argc, char *argv[])
{
	return SendEvent::main(argc, argv);
}

} /* namespace events */
