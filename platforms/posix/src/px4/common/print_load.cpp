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

#include <px4_platform_common/posix.h>

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <px4_platform_common/printload.h>
#include <drivers/drv_hrt.h>

#ifdef __PX4_DARWIN
#include <mach/mach.h>
#endif

#ifdef __PX4_QURT
// dprintf is not available on QURT. Use the usual output to mini-dm.
#define dprintf(_fd, _text, ...) ((_fd) == 1 ? PX4_INFO((_text), ##__VA_ARGS__) : (void)(_fd))
#endif

extern struct system_load_s system_load;

#define CL "\033[K" // clear line

void init_print_load(struct print_load_s *s)
{
	s->total_user_time = 0;

	s->running_count = 0;
	s->blocked_count = 0;

	s->new_time = hrt_absolute_time();
	s->interval_start_time = s->new_time;

	for (int i = 0; i < CONFIG_MAX_TASKS; i++) {
		s->last_times[i] = 0;
	}

	s->interval_time_us = 0.f;
}

void print_load(int fd, struct print_load_s *print_state)
{
#if defined(__PX4_LINUX) || defined(__PX4_CYGWIN) || defined(__PX4_QURT)
	char clear_line[] = CL;
	dprintf(fd, "%sTOP NOT IMPLEMENTED ON LINUX, QURT, WINDOWS (ONLY ON NUTTX, APPLE)\n", clear_line);
#elif defined(__PX4_DARWIN)
	char clear_line[] = CL;
	pid_t pid = getpid();   //-- this is the process id you need info for
	task_t task_handle;
	task_for_pid(mach_task_self(), pid, &task_handle);

	task_info_data_t tinfo;
	mach_msg_type_number_t th_info_cnt;

	th_info_cnt = TASK_INFO_MAX;
	kern_return_t kr = task_info(task_handle, TASK_BASIC_INFO, (task_info_t)tinfo, &th_info_cnt);

	if (kr != KERN_SUCCESS) {
		return;
	}

	thread_array_t thread_list;
	mach_msg_type_number_t th_cnt;

	thread_info_data_t th_info_data;
	mach_msg_type_number_t thread_info_count;

	thread_basic_info_t basic_info_th;
	uint32_t stat_thread = 0;

	/* print system information */
	if (fd == 1) {
		dprintf(fd, "\033[H"); /* move cursor home and clear screen */

	} else {
		memset(clear_line, 0, sizeof(clear_line));
	}

	// get all threads of the PX4 main task
	kr = task_threads(task_handle, &thread_list, &th_cnt);

	if (kr != KERN_SUCCESS) {
		PX4_WARN("ERROR getting thread list");
		return;
	}

	if (th_cnt > 0) {
		stat_thread += th_cnt;
	}

	long tot_sec = 0;
	long tot_usec = 0;
	long tot_cpu = 0;

	dprintf(fd, "%sThreads: %d total\n",
		clear_line,
		th_cnt);

	for (unsigned j = 0; j < th_cnt; j++) {
		thread_info_count = THREAD_INFO_MAX;
		kr = thread_info(thread_list[j], THREAD_BASIC_INFO,
				 (thread_info_t)th_info_data, &thread_info_count);

		if (kr != KERN_SUCCESS) {
			PX4_WARN("ERROR getting thread info");
			continue;
		}

		basic_info_th = (thread_basic_info_t)th_info_data;

		if (!(basic_info_th->flags & TH_FLAGS_IDLE)) {
			tot_sec = tot_sec + basic_info_th->user_time.seconds + basic_info_th->system_time.seconds;
			tot_usec = tot_usec + basic_info_th->system_time.microseconds + basic_info_th->system_time.microseconds;
			tot_cpu = tot_cpu + basic_info_th->cpu_usage;
		}

		// char tname[128];

		// int ret = pthread_getname_np(pthread_t *thread,
		//                      const char *name, size_t len);

		dprintf(fd, "thread %d\t\t %d\n", j, basic_info_th->cpu_usage);
	}

	kr = vm_deallocate(mach_task_self(), (vm_offset_t)thread_list,
			   th_cnt * sizeof(thread_t));

	if (kr != KERN_SUCCESS) {
		PX4_WARN("ERROR cleaning up thread info");
		return;
	}

#endif
}

void print_load_buffer(char *buffer, int buffer_length, print_load_callback_f cb, void *user,
		       struct print_load_s *print_state)
{

}
