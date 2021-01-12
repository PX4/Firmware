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

#pragma once

#include <drivers/drv_hrt.h>

#ifdef __PX4_NUTTX
#include <sched.h>
#include <px4_platform/cpuload.h>
#endif /* __PX4_NUTTX */

namespace px4
{
namespace logger
{

struct watchdog_data_t {
#ifdef __PX4_NUTTX
	int logger_main_task_index = -1;
	int logger_writer_task_index = -1;
	hrt_abstime ready_to_run_timestamp = hrt_absolute_time();
	uint8_t last_state = TSTATE_TASK_INVALID;
#endif /* __PX4_NUTTX */
};


/**
 * Initialize the watchdog, fill in watchdog_data.
 */
void watchdog_initialize(const pid_t pid_logger_main, const pthread_t writer_thread, watchdog_data_t &watchdog_data);

/**
 * Update the watchdog and trigger it if necessary. It is triggered when the log writer task is in
 * ready state for a certain period of time, but did not get scheduled. It means that most likely
 * some other higher-prio task runs busy.
 * When the watchdog triggers, it boosts the priority of the logger's main & writer tasks to maximum, so
 * that they get scheduled again.
 *
 * Expected to be called from IRQ context.
 *
 * @param watchdog_data
 * @return true if watchdog is triggered, false otherwise
 */
bool watchdog_update(watchdog_data_t &watchdog_data);

} //namespace logger
} //namespace px4
