/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file cpuload.c
 *
 * Measurement of CPU load of each individual task.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 */
#include <px4_platform_common/px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

//#include <arch/arch.h>

//#include <debug.h>

#include <sys/time.h>


#include <drivers/drv_hrt.h>

#include "cpuload.h"

#ifdef CONFIG_SCHED_INSTRUMENTATION

#ifdef __PX4_NUTTX

# include <nuttx/sched_note.h>

void sched_note_suspend(FAR struct tcb_s *tcb);
void sched_note_resume(FAR struct tcb_s *tcb);

__EXPORT void sched_note_switch(FAR struct tcb_s *pFromTcb, FAR struct tcb_s *pToTcb);

__EXPORT struct system_load_s system_load;

extern FAR struct tcb_s *sched_gettcb(pid_t pid);

void cpuload_initialize_once()
{
	system_load.start_time = hrt_absolute_time();
	int i;

	for (i = 0; i < CONFIG_MAX_TASKS; i++) {
		system_load.tasks[i].valid = false;
	}

	int static_tasks_count = 2;	// there are at least 2 threads that should be initialized statically - "idle" and "init"

#ifdef CONFIG_PAGING
	static_tasks_count++;	// include paging thread in initialization
#endif /* CONFIG_PAGING */
#if CONFIG_SCHED_WORKQUEUE
	static_tasks_count++;	// include high priority work0 thread in initialization
#endif /* CONFIG_SCHED_WORKQUEUE */
#if CONFIG_SCHED_LPWORK
	static_tasks_count++;	// include low priority work1 thread in initialization
#endif /* CONFIG_SCHED_WORKQUEUE */

	// perform static initialization of "system" threads
	for (system_load.total_count = 0; system_load.total_count < static_tasks_count; system_load.total_count++) {
		system_load.tasks[system_load.total_count].total_runtime = 0;
		system_load.tasks[system_load.total_count].curr_start_time = 0;
		system_load.tasks[system_load.total_count].tcb = sched_gettcb(
					system_load.total_count);	// it is assumed that these static threads have consecutive PIDs
		system_load.tasks[system_load.total_count].valid = true;
	}

	system_load.initialized = true;
}

void sched_note_start(FAR struct tcb_s *tcb)
{
	/* search first free slot */
	int i;

	if (system_load.initialized) {
		for (i = 1; i < CONFIG_MAX_TASKS; i++) {
			if (!system_load.tasks[i].valid) {
				/* slot is available */
				system_load.tasks[i].total_runtime = 0;
				system_load.tasks[i].curr_start_time = 0;
				system_load.tasks[i].tcb = tcb;
				system_load.tasks[i].valid = true;
				system_load.total_count++;
				break;
			}
		}
	}
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
	int i;

	if (system_load.initialized) {
		for (i = 1; i < CONFIG_MAX_TASKS; i++) {
			if (system_load.tasks[i].tcb != 0 && system_load.tasks[i].tcb->pid == tcb->pid) {
				/* mark slot as fee */
				system_load.tasks[i].valid = false;
				system_load.tasks[i].total_runtime = 0;
				system_load.tasks[i].curr_start_time = 0;
				system_load.tasks[i].tcb = NULL;
				system_load.total_count--;
				break;
			}
		}
	}
}

void sched_note_suspend(FAR struct tcb_s *tcb)
{

	if (system_load.initialized) {
		uint64_t new_time = hrt_absolute_time();

		for (int i = 0; i < CONFIG_MAX_TASKS; i++) {
			/* Task ending its current scheduling run */
			if (system_load.tasks[i].valid && system_load.tasks[i].tcb != 0 && system_load.tasks[i].tcb->pid == tcb->pid) {
				system_load.tasks[i].total_runtime += new_time - system_load.tasks[i].curr_start_time;
				break;
			}
		}
	}
}

void sched_note_resume(FAR struct tcb_s *tcb)
{

	if (system_load.initialized) {
		uint64_t new_time = hrt_absolute_time();

		for (int i = 0; i < CONFIG_MAX_TASKS; i++) {
			if (system_load.tasks[i].valid && system_load.tasks[i].tcb->pid == tcb->pid) {
				// curr_start_time is accessed from an IRQ handler (in logger), so we need
				// to make the update atomic
				irqstate_t irq_state = px4_enter_critical_section();
				system_load.tasks[i].curr_start_time = new_time;
				px4_leave_critical_section(irq_state);
				break;
			}
		}
	}

}

#else
__EXPORT struct system_load_s system_load;
#endif
#endif /* CONFIG_SCHED_INSTRUMENTATION */
