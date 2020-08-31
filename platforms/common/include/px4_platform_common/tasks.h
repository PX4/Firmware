/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file px4_tasks.h
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Mark Charlebois (2015) <charlebm@gmail.com>
 *
 * Preserve existing task API call signature with OS abstraction
 */

#pragma once

#include <stdbool.h>

#if defined(__PX4_NUTTX)
typedef int px4_task_t;

#include <sys/prctl.h>
#define px4_prctl prctl

/** Default scheduler type */
#if CONFIG_RR_INTERVAL > 0
# define SCHED_DEFAULT  SCHED_RR
#else
# define SCHED_DEFAULT  SCHED_FIFO
#endif

#define px4_task_exit(x) _exit(x)

#elif defined(__PX4_POSIX) || defined(__PX4_QURT)

#include <pthread.h>
#include <sched.h>

/** Default scheduler type */
#define SCHED_DEFAULT	SCHED_FIFO

#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)

#define SCHED_PRIORITY_MAX sched_get_priority_max(SCHED_FIFO)
#define SCHED_PRIORITY_MIN sched_get_priority_min(SCHED_FIFO)
#define SCHED_PRIORITY_DEFAULT (((sched_get_priority_max(SCHED_FIFO) - sched_get_priority_min(SCHED_FIFO)) / 2) + sched_get_priority_min(SCHED_FIFO))

#elif defined(__PX4_QURT)

#define SCHED_PRIORITY_MAX 255
#define SCHED_PRIORITY_MIN 0
#define SCHED_PRIORITY_DEFAULT 20

#else
#error "No target OS defined"
#endif

#if defined (__PX4_LINUX)
#include <sys/prctl.h>
#else
#define PR_SET_NAME	1
#endif

typedef int px4_task_t;

typedef struct {
	int argc;
	char **argv;
} px4_task_args_t;

#else
#error "No target OS defined"
#endif

// PX4 work queue starting high priority
#define PX4_WQ_HP_BASE (SCHED_PRIORITY_MAX - 15)

// Fast drivers - they need to run as quickly as possible to minimize control
// latency.
#define SCHED_PRIORITY_FAST_DRIVER		(SCHED_PRIORITY_MAX - 0)

// Actuator outputs should run as soon as the rate controller publishes
// the actuator controls topic
#define SCHED_PRIORITY_ACTUATOR_OUTPUTS		(PX4_WQ_HP_BASE - 3)

// Attitude controllers typically are in a blocking wait on driver data
// they should be the first to run on an update, using the current sensor
// data and the *previous* attitude reference from the position controller
// which typically runs at a slower rate
#define SCHED_PRIORITY_ATTITUDE_CONTROL		(PX4_WQ_HP_BASE - 4)

// Estimators should run after the attitude controller but before anything
// else in the system. They wait on sensor data which is either coming
// from the sensor hub or from a driver. Keeping this class at a higher
// priority ensures that the estimator runs first if it can, but will
// wait for the sensor hub if its data is coming from it.
#define SCHED_PRIORITY_ESTIMATOR		(PX4_WQ_HP_BASE - 5)

// Position controllers typically are in a blocking wait on estimator data
// so when new sensor data is available they will run last. Keeping them
// on a high priority ensures that they are the first process to be run
// when the estimator updates.
#define SCHED_PRIORITY_POSITION_CONTROL		(PX4_WQ_HP_BASE - 7)

// The log capture (which stores log data into RAM) should run faster
// than other components, but should not run before the control pipeline
#define SCHED_PRIORITY_LOG_CAPTURE		(PX4_WQ_HP_BASE - 10)

// Slow drivers should run at a rate where they do not impact the overall
// system execution
#define SCHED_PRIORITY_SLOW_DRIVER		(PX4_WQ_HP_BASE - 35)

// The navigation system needs to execute regularly but has no realtime needs
#define SCHED_PRIORITY_NAVIGATION		(SCHED_PRIORITY_DEFAULT + 5)
//      SCHED_PRIORITY_DEFAULT
#define SCHED_PRIORITY_LOG_WRITER		(SCHED_PRIORITY_DEFAULT - 10)
#define SCHED_PRIORITY_PARAMS			(SCHED_PRIORITY_DEFAULT - 15)
//      SCHED_PRIORITY_IDLE

typedef int (*px4_main_t)(int argc, char *argv[]);

__BEGIN_DECLS

/** Starts a task and performs any specific accounting, scheduler setup, etc. */
__EXPORT px4_task_t px4_task_spawn_cmd(const char *name,
				       int scheduler,
				       int priority,
				       int stack_size,
				       px4_main_t entry,
				       char *const argv[]);

/** Deletes a task - does not do resource cleanup **/
__EXPORT int px4_task_delete(px4_task_t pid);

/** Send a signal to a task **/
__EXPORT int px4_task_kill(px4_task_t pid, int sig);

/** Exit current task with return value **/
__EXPORT void px4_task_exit(int ret);

/** Show a list of running tasks **/
__EXPORT void px4_show_tasks(void);

/** See if a task is running **/
__EXPORT bool px4_task_is_running(const char *taskname);

#ifdef __PX4_POSIX
/** set process (and thread) options */
__EXPORT int px4_prctl(int option, const char *arg2, px4_task_t pid);
#endif

/** return the name of the current task */
__EXPORT const char *px4_get_taskname(void);

__END_DECLS
