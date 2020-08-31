/****************************************************************************
 *
 *   Copyright (C) 2015-2016 Mark Charlebois. All rights reserved.
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
 * @file px4_qurt_tasks.c
 * Implementation of existing task API for QURT.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/time.h>
#include <hrt_work.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>

#if !defined(__PX4_QURT)
#include <signal.h>
#endif

#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string>

#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>

#define MAX_CMD_LEN 100

#define PX4_MAX_TASKS 50
#define SHELL_TASK_ID (PX4_MAX_TASKS+1)

pthread_t _shell_task_id = 0;

struct task_entry {
	pthread_t pid;
	std::string name;
	bool isused;
	task_entry() : isused(false) {}
};

static task_entry taskmap[PX4_MAX_TASKS];

typedef struct {
	px4_main_t entry;
	int argc;
	char *argv[];
	// strings are allocated after the
} pthdata_t;

static void *entry_adapter(void *ptr)
{
	pthdata_t *data;
	data = (pthdata_t *) ptr;

	data->entry(data->argc, data->argv);
	//PX4_WARN("Before waiting infinte busy loop");
	//for( ;; )
	//{
	//   volatile int x = 0;
	//   ++x;
	// }
	free(ptr);
	px4_task_exit(0);

	return NULL;
}

px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack_size, px4_main_t entry,
			      char *const argv[])
{
	struct sched_param param;
	pthread_attr_t attr;
	pthread_t task;
	int rv;
	int argc = 0;
	int i;
	unsigned int len = 0;
	unsigned long offset;
	unsigned long structsize;
	char *p = (char *)argv;

	PX4_DEBUG("Creating %s\n", name);
	PX4_DEBUG("attr address: 0x%X, param address: 0x%X", &attr, &param);

	// Calculate argc
	while (p != (char *)0) {
		p = argv[argc];

		if (p == (char *)0) {
			break;
		}

		++argc;
		len += strlen(p) + 1;
	}

	structsize = sizeof(pthdata_t) + (argc + 1) * sizeof(char *);
	pthdata_t *taskdata = nullptr;

	// not safe to pass stack data to the thread creation
	taskdata = (pthdata_t *)malloc(structsize + len);

	if (taskdata == nullptr) {
		return -ENOMEM;
	}

	offset = ((unsigned long)taskdata) + structsize;

	taskdata->entry = entry;
	taskdata->argc = argc;

	for (i = 0; i < argc; i++) {
		PX4_DEBUG("arg %d %s\n", i, argv[i]);
		taskdata->argv[i] = (char *)offset;
		strcpy((char *)offset, argv[i]);
		offset += strlen(argv[i]) + 1;
	}

	// Must add NULL at end of argv
	taskdata->argv[argc] = (char *)0;

	rv = pthread_attr_init(&attr);

	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to init thread attrs");
		return (rv < 0) ? rv : -rv;
	}

	PX4_DEBUG("stack address after pthread_attr_init: 0x%X", attr.stackaddr);
	PX4_DEBUG("attr address: 0x%X, param address: 0x%X", &attr, &param);
	rv = pthread_attr_getschedparam(&attr, &param);
	PX4_DEBUG("stack address after pthread_attr_getschedparam: 0x%X", attr.stackaddr);

	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to get thread sched param");
		return (rv < 0) ? rv : -rv;
	}

#if 0
	rv = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to set inherit sched");
		return (rv < 0) ? rv : -rv;
	}

	rv = pthread_attr_setschedpolicy(&attr, scheduler);

	if (rv != 0) {
		PX4_WARN("px4_task_spawn_cmd: failed to set sched policy");
		return (rv < 0) ? rv : -rv;
	}

#endif
	size_t fixed_stacksize = -1;
	pthread_attr_getstacksize(&attr, &fixed_stacksize);
	PX4_DEBUG("stack size: %d passed stacksize(%d)", fixed_stacksize, stack_size);
	fixed_stacksize = 8 * 1024;
	fixed_stacksize = (fixed_stacksize < (size_t)stack_size) ? (size_t)stack_size : fixed_stacksize;

	PX4_DEBUG("setting the thread[%s] stack size to[%d]", name, fixed_stacksize);
	pthread_attr_setstacksize(&attr, PX4_STACK_ADJUSTED(fixed_stacksize));

	PX4_DEBUG("stack address after pthread_attr_setstacksize: 0x%X", attr.stackaddr);
	param.sched_priority = priority;

	rv = pthread_attr_setschedparam(&attr, &param);

	if (rv != 0) {
		PX4_ERR("px4_task_spawn_cmd: failed to set sched param");
		return (rv < 0) ? rv : -rv;
	}

	rv = pthread_create(&task, &attr, &entry_adapter, (void *) taskdata);

	if (rv != 0) {
		PX4_ERR("px4_task_spawn_cmd: pthread_create failed, error: %d", rv);
		return (rv < 0) ? rv : -rv;
	}

	for (i = 0; i < PX4_MAX_TASKS; ++i) {
		if (taskmap[i].isused == false) {
			taskmap[i].pid = task;
			taskmap[i].name = name;
			taskmap[i].isused = true;
			break;
		}
	}

	if (i >= PX4_MAX_TASKS) {
		return -ENOSPC;
	}

	return i;
}

int px4_task_delete(px4_task_t id)
{
	int rv = 0;
	pthread_t pid;
	PX4_WARN("Called px4_task_delete");

	if (id < PX4_MAX_TASKS && taskmap[id].isused) {
		pid = taskmap[id].pid;

	} else {
		return -EINVAL;
	}

	// If current thread then exit, otherwise cancel
	if (pthread_self() == pid) {
		taskmap[id].isused = false;
		pthread_exit(0);

	} else {
		rv = pthread_cancel(pid);
	}

	taskmap[id].isused = false;

	return rv;
}

void px4_task_exit(int ret)
{
	int i;
	pthread_t pid = pthread_self();

	// Get pthread ID from the opaque ID
	for (i = 0; i < PX4_MAX_TASKS; ++i) {
		if (taskmap[i].pid == pid) {
			taskmap[i].isused = false;
			break;
		}
	}

	if (i >= PX4_MAX_TASKS)  {
		PX4_ERR("px4_task_exit: self task not found!");

	} else {
		PX4_DEBUG("px4_task_exit: %s", taskmap[i].name.c_str());
	}

	//pthread_exit((void *)(unsigned long)ret);
}

int px4_task_kill(px4_task_t id, int sig)
{
	int rv = 0;
	pthread_t pid;
	PX4_DEBUG("Called px4_task_kill %d, taskname %s", sig, taskmap[id].name.c_str());

	if (id < PX4_MAX_TASKS && taskmap[id].pid != 0) {
		pid = taskmap[id].pid;

	} else {
		return -EINVAL;
	}

	// If current thread then exit, otherwise cancel
	rv = pthread_kill(pid, sig);

	return rv;
}

void px4_show_tasks()
{
	int idx;
	int count = 0;

	PX4_INFO("Active Tasks:");

	for (idx = 0; idx < PX4_MAX_TASKS; idx++) {
		if (taskmap[idx].isused) {
			PX4_INFO("   %-10s %d", taskmap[idx].name.c_str(), taskmap[idx].pid);
			count++;
		}
	}

	if (count == 0) {
		PX4_INFO("   No running tasks");
	}

}

px4_task_t px4_getpid()
{
	pthread_t pid = pthread_self();
//
//	if (pid == _shell_task_id)
//		return SHELL_TASK_ID;

	// Get pthread ID from the opaque ID
	for (int i = 0; i < PX4_MAX_TASKS; ++i) {
		if (taskmap[i].isused && taskmap[i].pid == pid) {
			return i;
		}
	}

	return ~0;
}


const char *px4_get_taskname()
{
	pthread_t pid = pthread_self();

	for (int i = 0; i < PX4_MAX_TASKS; i++) {
		if (taskmap[i].isused && taskmap[i].pid == pid) {
			return taskmap[i].name.c_str();
		}
	}

	return "Unknown App";
}

static void timer_cb(void *data)
{
	px4_sem_t *sem = reinterpret_cast<px4_sem_t *>(data);

	sem_post(sem);
}

int px4_sem_timedwait(px4_sem_t *sem, const struct timespec *ts)
{
	work_s _hpwork = {};

	// Get the current time.
	struct timespec ts_now;
	px4_clock_gettime(CLOCK_MONOTONIC, &ts_now);

	// We get an absolute time but want to calculate a timeout in us.
	hrt_abstime timeout_us = ts_to_abstime((struct timespec *)ts) - ts_to_abstime(&ts_now);

	// Create a timer to unblock.
	hrt_work_queue(&_hpwork, (worker_t)&timer_cb, (void *)sem, timeout_us);
	sem_wait(sem);
	hrt_work_cancel(&_hpwork);
	return 0;
}

int px4_prctl(int option, const char *arg2, px4_task_t pid)
{
	int rv;

	switch (option) {
	case PR_SET_NAME:
		// set the threads name - Not supported
		// rv = pthread_setname_np(pthread_self(), arg2);
		rv = -1;
		break;

	default:
		rv = -1;
		PX4_WARN("FAILED SETTING TASK NAME");
		break;
	}

	return rv;
}
