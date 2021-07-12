/****************************************************************************
 *
 *   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
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
 * @file print_load.cpp
 *
 * Print the current system load.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <string.h>
#include <stdio.h>

#include <px4_platform/cpuload.h>
#include <px4_platform_common/printload.h>
#include <drivers/drv_hrt.h>

#if defined(BOARD_DMA_ALLOC_POOL_SIZE)
#include <px4_platform/board_dma_alloc.h>
#endif /* BOARD_DMA_ALLOC_POOL_SIZE */

#if defined(CONFIG_SCHED_INSTRUMENTATION)

#if !defined(CONFIG_TASK_NAME_SIZE)
#error print_load_nuttx requires CONFIG_TASK_NAME_SIZE
#endif

#if !defined(CONFIG_STACK_COLORATION)
#error print_load_nuttx requires CONFIG_STACK_COLORATION
#endif

extern struct system_load_s system_load;

#define CL "\033[K" // clear line

void init_print_load(struct print_load_s *s)
{
	cpuload_monitor_start();

	s->total_user_time = 0;

	s->running_count = 0;
	s->blocked_count = 0;

	s->new_time = system_load.start_time;
	s->interval_start_time = system_load.start_time;

	sched_lock();
	// special case for IDLE thread
	s->last_times[0] = system_load.tasks[0].total_runtime;
	sched_unlock();

	for (int i = 1; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
		s->last_times[i] = 0;
	}

	s->interval_time_us = 0.f;
}

static constexpr const char *tstate_name(const tstate_t s)
{
	switch (s) {
	case TSTATE_TASK_INVALID:
		return "init";

	case TSTATE_TASK_PENDING:
		return "PEND";

	case TSTATE_TASK_READYTORUN:
		return "READY";

	case TSTATE_TASK_RUNNING:
		return "RUN";

	case TSTATE_TASK_INACTIVE:
		return "inact";

	case TSTATE_WAIT_SEM:
		return "w:sem";
#ifndef CONFIG_DISABLE_SIGNALS

	case TSTATE_WAIT_SIG:
		return "w:sig";
#endif
#ifndef CONFIG_DISABLE_MQUEUE

	case TSTATE_WAIT_MQNOTEMPTY:
		return "w:mqe";

	case TSTATE_WAIT_MQNOTFULL:
		return "w:mqf";
#endif
#ifdef CONFIG_PAGING

	case TSTATE_WAIT_PAGEFILL:
		return "w:pgf";
#endif

	default:
		return "ERROR";
	}
}

void print_load_buffer(char *buffer, int buffer_length, print_load_callback_f cb, void *user,
		       struct print_load_s *print_state)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat" // NuttX uses a different printf format
#pragma GCC diagnostic ignored "-Wformat-extra-args"

	float idle_load = 0.f;

	// create a copy of the runtimes because this could be updated during the print output
	uint64_t total_runtime[CONFIG_FS_PROCFS_MAX_TASKS] {};
	sched_lock();

	print_state->new_time = hrt_absolute_time();

	for (int i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
		if (system_load.tasks[i].valid) {
			total_runtime[i] = system_load.tasks[i].total_runtime;
		}
	}

	sched_unlock();

	if (print_state->new_time > print_state->interval_start_time) {
		print_state->interval_time_us = print_state->new_time - print_state->interval_start_time;

		/* header for task list */
		snprintf(buffer, buffer_length, "%4s %-*s %8s %6s %11s %10s %-5s %2s",
			 "PID",
			 CONFIG_TASK_NAME_SIZE, "COMMAND",
			 "CPU(ms)",
			 "CPU(%)",
			 "USED/STACK",
			 "PRIO(BASE)",
#if CONFIG_RR_INTERVAL > 0
			 "TSLICE",
#else
			 "STATE",
#endif
			 "FD"
			);

		cb(user);
	}

	print_state->running_count = 0;
	print_state->blocked_count = 0;
	print_state->total_user_time = 0;


	for (int i = 0; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {

		sched_lock(); // need to lock the tcb access (but make it as short as possible)

		if (!system_load.tasks[i].valid) {
			sched_unlock();
			continue;
		}

		unsigned tcb_pid = system_load.tasks[i].tcb->pid;
		size_t stack_size = system_load.tasks[i].tcb->adj_stack_size;
		ssize_t stack_free = 0;
		char tcb_name[CONFIG_TASK_NAME_SIZE + 1];
		strncpy(tcb_name, system_load.tasks[i].tcb->name, CONFIG_TASK_NAME_SIZE + 1);

#if CONFIG_ARCH_INTERRUPTSTACK > 3

		if (system_load.tasks[i].tcb->pid == 0) {
			stack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
			stack_free = up_check_intstack_remain();

		} else {
			stack_free = up_check_tcbstack_remain(system_load.tasks[i].tcb);
		}

#else
		stack_free = up_check_tcbstack_remain(system_load.tasks[i].tcb);
#endif

#if CONFIG_ARCH_BOARD_SIM || !defined(CONFIG_PRIORITY_INHERITANCE)
#else
		unsigned tcb_base_priority = system_load.tasks[i].tcb->base_priority;
#endif
#if CONFIG_RR_INTERVAL > 0
		unsigned tcb_timeslice = system_load.tasks[i].tcb->timeslice;
#endif
		tstate_t tcb_task_state = (tstate_t)system_load.tasks[i].tcb->task_state;
		uint8_t tcb_sched_priority = system_load.tasks[i].tcb->sched_priority;

		unsigned int tcb_num_used_fds = 0; // number of used file descriptors
		struct filelist *filelist = &system_load.tasks[i].tcb->group->tg_filelist;

		for (int fdr = 0; fdr < filelist->fl_rows; fdr++) {
			for (int fdc = 0; fdc < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; fdc++) {
				if (filelist->fl_files[fdr][fdc].f_inode) {
					++tcb_num_used_fds;
				}
			}
		}

		sched_unlock();

		switch (tcb_task_state) {
		case TSTATE_TASK_PENDING:
		case TSTATE_TASK_READYTORUN:
		case TSTATE_TASK_RUNNING:
			print_state->running_count++;
			break;

#ifndef CONFIG_DISABLE_SIGNALS

		case TSTATE_WAIT_SIG:
#endif
#ifndef CONFIG_DISABLE_MQUEUE
		case TSTATE_WAIT_MQNOTEMPTY:
		case TSTATE_WAIT_MQNOTFULL:
#endif
#ifdef CONFIG_PAGING
		case TSTATE_WAIT_PAGEFILL:
#endif
		case TSTATE_TASK_INVALID:
		case TSTATE_TASK_INACTIVE:
		case TSTATE_WAIT_SEM:
			print_state->blocked_count++;
			break;

		case TSTATE_TASK_STOPPED:
			// DO NOTHING
			break;

		case NUM_TASK_STATES:
			// DO NOTHING
			break;
		}

		float current_load = 0.f;

		if (total_runtime[i] > print_state->last_times[i]) {
			const uint64_t interval_runtime = total_runtime[i] - print_state->last_times[i];

			current_load = interval_runtime / print_state->interval_time_us;

			if (tcb_pid == 0) {
				idle_load = current_load;

			} else {
				print_state->total_user_time += interval_runtime;
			}
		}

		print_state->last_times[i] = total_runtime[i];

		if (print_state->new_time <= print_state->interval_start_time) {
			continue; // not enough data yet
		}

		// print output
		int print_len = snprintf(buffer, buffer_length, "%4d %-*s %8d %6.3f %5u/%5u %3u (%3u) ",
					 tcb_pid,
					 CONFIG_TASK_NAME_SIZE, tcb_name,
					 total_runtime[i] / 1000, // us -> ms
					 (double)(current_load * 100.f),
					 stack_size - stack_free,
					 stack_size,
					 tcb_sched_priority,
#if CONFIG_ARCH_BOARD_SIM || !defined(CONFIG_PRIORITY_INHERITANCE)
					 0);
#else
					 tcb_base_priority);
#endif
#if CONFIG_RR_INTERVAL > 0
		/* print scheduling info with RR time slice */
		snprintf(buffer + print_len, buffer_length - print_len, " %5d %2d", tcb_timeslice, tcb_num_used_fds);
		(void)tstate_name(TSTATE_TASK_INVALID); // Stop not used warning
#else
		// print task state instead
		snprintf(buffer + print_len, buffer_length - print_len, " %-5s %2d", tstate_name(tcb_task_state), tcb_num_used_fds);
#endif
		cb(user);
	}

	if (print_state->new_time <= print_state->interval_start_time) {
		// first run, not enough data yet
		return;
	}

	// Print footer
	buffer[0] = 0;
	cb(user);

	float task_load = (float)(print_state->total_user_time) / print_state->interval_time_us;

	/* this can happen if one tasks total runtime was not computed
	   correctly by the scheduler instrumentation TODO */
	if (task_load > (1.f - idle_load)) {
		task_load = (1.f - idle_load);
	}

	const float sched_load = 1.f - idle_load - task_load;

	snprintf(buffer, buffer_length, "Processes: %d total, %d running, %d sleeping",
		 system_load.total_count,
		 print_state->running_count,
		 print_state->blocked_count);
	cb(user);
	snprintf(buffer, buffer_length, "CPU usage: %.2f%% tasks, %.2f%% sched, %.2f%% idle",
		 (double)(task_load * 100.f),
		 (double)(sched_load * 100.f),
		 (double)(idle_load * 100.f));
	cb(user);
#if defined(BOARD_DMA_ALLOC_POOL_SIZE)
	uint16_t dma_total;
	uint16_t dma_used;
	uint16_t dma_peak_used;

	if (board_get_dma_usage(&dma_total, &dma_used, &dma_peak_used) >= 0) {
		snprintf(buffer, buffer_length, "DMA Memory: %d total, %d used %d peak",
			 dma_total,
			 dma_used,
			 dma_peak_used);
		cb(user);
	}

#endif
	snprintf(buffer, buffer_length, "Uptime: %.3fs total, %.3fs idle",
		 (double)print_state->new_time / 1e6, (double)total_runtime[0] / 1e6);

	cb(user);

	print_state->interval_start_time = print_state->new_time;

#pragma GCC diagnostic pop
}


struct print_load_callback_data_s {
	int fd;
	char buffer[140];
};

static void print_load_callback(void *user)
{
	char clear_line[] {CL};
	struct print_load_callback_data_s *data = (struct print_load_callback_data_s *)user;

	if (data->fd != STDOUT_FILENO) {
		clear_line[0] = '\0';
	}

	dprintf(data->fd, "%s%s\n", clear_line, data->buffer);
}

void print_load(int fd, struct print_load_s *print_state)
{
	// print system information
	if (fd == STDOUT_FILENO) {
		// move cursor home and clear screen
		dprintf(fd, "\033[H");
	}

	print_load_callback_data_s data{};
	data.fd = fd;

	print_load_buffer(data.buffer, sizeof(data.buffer), print_load_callback, &data, print_state);
}

#endif // if CONFIG_SCHED_INSTRUMENTATION
