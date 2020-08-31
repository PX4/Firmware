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

#pragma once

#include "status_display.h"
#include "rc_loss_alarm.h"

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

namespace events
{

extern "C" __EXPORT int send_event_main(int argc, char *argv[]);

/** @class SendEvent The SendEvent class manages the RC loss audible alarm, LED status display, and thermal calibration. */
class SendEvent : public ModuleBase<SendEvent>, public ModuleParams, public px4::ScheduledWorkItem
{
public:

	SendEvent();

	~SendEvent();

	/**
	 * @see ModuleBase
	 * @brief Recognizes custom startup commands, called from the main() function entry.
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 iff successful, otherwise < 0 on error.
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase
	 * @brief Prints usage options to the console.
	 * @param reason The requested usage reason for printing to console.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief Spawns and initializes the class in the same context as the
	 *        work queue and starts the background listener.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int task_spawn(int argc, char *argv[]);

private:

	/**
	 * @brief Returns an ACK to a vehicle_command.
	 * @param cmd The vehicle command struct being referenced.
	 * @param result The command acknowledgement result.
	 */
	void answer_command(const vehicle_command_s &cmd, unsigned result);

	/**
	 * @brief Calls process_commands() and schedules the next cycle.
	 */
	void Run() override;

	/**
	 * @brief Checks for new commands and processes them.
	 */
	void process_commands();

	/**
	 * @brief Starts background task listening for commands.
	 * @return Returns 0 iff successful, otherwise < 0 on error.
	 */
	int start();

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

	/** @var _status_display Pointer to the status display object. */
	status::StatusDisplay *_status_display = nullptr;

	/** @var _rc_loss_alarm Pointer to the RC loss alarm object. */
	rc_loss::RC_Loss_Alarm *_rc_loss_alarm = nullptr;

	/** @note Declare local parameters using defined parameters. */
	DEFINE_PARAMETERS(
		/** @var _param_status_display Parameter to enable/disable the LED status display. */
		(ParamBool<px4::params::EV_TSK_STAT_DIS>) _param_ev_tsk_stat_dis,

		/** @var _param_rc_loss The RC comms loss status flag. */
		(ParamBool<px4::params::EV_TSK_RC_LOSS>) _param_ev_tsk_rc_loss
	)
};

} // namespace events
