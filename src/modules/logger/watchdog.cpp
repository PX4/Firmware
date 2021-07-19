/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "watchdog.h"

#include <px4_platform_common/log.h>

#if defined(__PX4_NUTTX) && !defined(CONFIG_SCHED_INSTRUMENTATION)
#  error watchdog support requires CONFIG_SCHED_INSTRUMENTATION
#endif

using namespace time_literals;

namespace px4
{
namespace logger
{

bool watchdog_update(watchdog_data_t &watchdog_data)
{

#ifdef __PX4_NUTTX

	if (system_load.initialized && watchdog_data.logger_main_task_index >= 0
	    && watchdog_data.logger_writer_task_index >= 0) {
		const hrt_abstime now = hrt_absolute_time();
		const system_load_taskinfo_s &log_writer_task = system_load.tasks[watchdog_data.logger_writer_task_index];

		if (log_writer_task.valid) {
			// Trigger the watchdog if the log writer task has been ready to run for a
			// minimum duration and it has not been scheduled during that time.
			// When the writer is waiting for an SD transfer, it is not in ready state, thus a long dropout
			// will not trigger it. The longest period in ready state I measured was around 70ms,
			// after a param change.
			// We only check the log writer because it runs at lower priority than the main thread.
			// No need to lock the tcb access, since we are in IRQ context

			// update the timestamp if it has been scheduled recently
			if (log_writer_task.curr_start_time > watchdog_data.ready_to_run_timestamp) {
				watchdog_data.ready_to_run_timestamp = log_writer_task.curr_start_time;
			}

			// update the timestamp if not ready to run or if transitioned into ready to run
			uint8_t current_state = log_writer_task.tcb->task_state;

			if (current_state != TSTATE_TASK_READYTORUN
			    || (watchdog_data.last_state != TSTATE_TASK_READYTORUN && current_state == TSTATE_TASK_READYTORUN)) {
				watchdog_data.ready_to_run_timestamp = now;
			}

			watchdog_data.last_state = current_state;

#if 0 // for debugging
			// test code that prints the maximum time in ready state.
			// Note: we are in IRQ context, and thus are strictly speaking not allowed to use PX4_ERR -
			// we do it anyway since it's only used for debugging.
			static uint64_t max_time = 0;

			if (now - watchdog_data.ready_to_run_timestamp > max_time) {
				max_time = now - watchdog_data.ready_to_run_timestamp;
			}

			static int counter = 0;

			if (++counter > 300) {
				PX4_ERR("max time in ready: %i ms", (int)max_time / 1000);
				counter = 0;
				max_time = 0;
			}

#endif

			if (now - watchdog_data.ready_to_run_timestamp > 1_s) {
				// boost the priority to make sure the logger continues to write to the log.
				// Note that we never restore the priority, to keep the logic simple and because it is
				// an event that must not occur under normal circumstances (if it does, there's a bug
				// somewhere)
				sched_param param{};
				param.sched_priority = SCHED_PRIORITY_MAX;

				if (system_load.tasks[watchdog_data.logger_main_task_index].valid) {
					sched_setparam(system_load.tasks[watchdog_data.logger_main_task_index].tcb->pid, &param);
				}

				sched_setparam(log_writer_task.tcb->pid, &param);

				// make sure we won't trigger again
				watchdog_data.logger_main_task_index = -1;
				return true;
			}

		} else {
			// should never happen
			watchdog_data.logger_main_task_index = -1;
		}
	}

#endif /* __PX4_NUTTX */

	return false;

}

void watchdog_initialize(const pid_t pid_logger_main, const pthread_t writer_thread, watchdog_data_t &watchdog_data)
{
#ifdef __PX4_NUTTX

	// The pthread_t ID is equal to the PID on NuttX
	const pthread_t pid_logger_writer = writer_thread;

	sched_lock(); // need to lock the tcb access

	for (int i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
		if (system_load.tasks[i].valid) {
			if (system_load.tasks[i].tcb->pid == pid_logger_writer) {
				watchdog_data.logger_writer_task_index = i;
			}

			if (system_load.tasks[i].tcb->pid == pid_logger_main) {
				watchdog_data.logger_main_task_index = i;
			}
		}
	}

	sched_unlock();

	if (watchdog_data.logger_writer_task_index == -1 ||
	    watchdog_data.logger_main_task_index == -1) {
		// If we land here it means the NuttX implementation changed
		// and one of our assumptions is not valid anymore
		PX4_ERR("watchdog init failed");
	}

#endif /* __PX4_NUTTX */
}


} // namespace logger
} // namespace px4
