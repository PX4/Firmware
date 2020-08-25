/****************************************************************************
 *
 * Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * @file shutdown.cpp
 * Implementation of the API declared in px4_shutdown.h.
 */

#include <board_config.h>

#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>

#ifndef MODULE_NAME
#define MODULE_NAME "shutdown"
#endif

#include <px4_platform_common/log.h>

#include <stdint.h>
#include <errno.h>
#include <pthread.h>

static pthread_mutex_t shutdown_mutex =
	PTHREAD_MUTEX_INITIALIZER; // protects access to shutdown_hooks & shutdown_lock_counter
static uint8_t shutdown_lock_counter = 0;

int px4_shutdown_lock()
{
	int ret = pthread_mutex_lock(&shutdown_mutex);

	if (ret == 0) {
		++shutdown_lock_counter;
		return pthread_mutex_unlock(&shutdown_mutex);
	}

	return ret;
}

int px4_shutdown_unlock()
{
	int ret = pthread_mutex_lock(&shutdown_mutex);

	if (ret == 0) {
		if (shutdown_lock_counter > 0) {
			--shutdown_lock_counter;

		} else {
			PX4_ERR("unmatched number of px4_shutdown_unlock() calls");
		}

		return pthread_mutex_unlock(&shutdown_mutex);
	}

	return ret;
}


#if (defined(__PX4_NUTTX) && !defined(CONFIG_SCHED_WORKQUEUE)) || __PX4_QURT
// minimal NuttX/QuRT build without work queue support

int px4_register_shutdown_hook(shutdown_hook_t hook)
{
	return -EINVAL;
}

int px4_unregister_shutdown_hook(shutdown_hook_t hook)
{
	return -EINVAL;
}

int px4_shutdown_request(bool reboot, bool to_bootloader)
{
	int ret = 0;
	pthread_mutex_lock(&shutdown_mutex);

	// FIXME: if shutdown_lock_counter > 0, we should wait, but unfortunately we don't have work queues
	if (reboot) {
		px4_systemreset(to_bootloader);

	} else {
		ret = board_shutdown();
	}

	pthread_mutex_unlock(&shutdown_mutex);

	return ret;
}

#else

static struct work_s shutdown_work = {};
static uint16_t shutdown_counter = 0; ///< count how many times the shutdown worker was executed

#define SHUTDOWN_ARG_IN_PROGRESS (1<<0)
#define SHUTDOWN_ARG_REBOOT (1<<1)
#define SHUTDOWN_ARG_TO_BOOTLOADER (1<<2)
static uint8_t shutdown_args = 0;


static const int max_shutdown_hooks = 1;
static shutdown_hook_t shutdown_hooks[max_shutdown_hooks] = {};


static const int shutdown_timeout_ms = 5000; ///< force shutdown after this time if modules do not respond in time


/**
 * work queue callback method to shutdown.
 * @param arg unused
 */
static void shutdown_worker(void *arg);


int px4_register_shutdown_hook(shutdown_hook_t hook)
{
	pthread_mutex_lock(&shutdown_mutex);

	for (int i = 0; i < max_shutdown_hooks; ++i) {
		if (!shutdown_hooks[i]) {
			shutdown_hooks[i] = hook;
			pthread_mutex_unlock(&shutdown_mutex);
			return 0;
		}
	}

	pthread_mutex_unlock(&shutdown_mutex);
	return -ENOMEM;
}

int px4_unregister_shutdown_hook(shutdown_hook_t hook)
{
	pthread_mutex_lock(&shutdown_mutex);

	for (int i = 0; i < max_shutdown_hooks; ++i) {
		if (shutdown_hooks[i] == hook) {
			shutdown_hooks[i] = nullptr;
			pthread_mutex_unlock(&shutdown_mutex);
			return 0;
		}
	}

	pthread_mutex_unlock(&shutdown_mutex);
	return -EINVAL;
}



void shutdown_worker(void *arg)
{
	PX4_DEBUG("shutdown worker (%i)", shutdown_counter);
	bool done = true;

	pthread_mutex_lock(&shutdown_mutex);

	for (int i = 0; i < max_shutdown_hooks; ++i) {
		if (shutdown_hooks[i]) {
			if (!shutdown_hooks[i]()) {
				done = false;
			}
		}
	}

	if ((done && shutdown_lock_counter == 0) || ++shutdown_counter > shutdown_timeout_ms / 10) {
		if (shutdown_args & SHUTDOWN_ARG_REBOOT) {
			PX4_WARN("Reboot NOW.");
			px4_systemreset(shutdown_args & SHUTDOWN_ARG_TO_BOOTLOADER);

		} else {
			PX4_WARN("Shutdown NOW. Good Bye.");
			board_shutdown();
		}

		pthread_mutex_unlock(&shutdown_mutex); // must NEVER come here

	} else {
		pthread_mutex_unlock(&shutdown_mutex);
		work_queue(HPWORK, &shutdown_work, (worker_t)&shutdown_worker, nullptr, USEC2TICK(10000));
	}
}

int px4_shutdown_request(bool reboot, bool to_bootloader)
{
	// fail immediately if the board does not support the requested method
#if defined BOARD_HAS_NO_RESET
	if (reboot) {
		return -EINVAL;
	}

#endif
#if !defined(BOARD_HAS_POWER_CONTROL)

	if (!reboot) {
		return -EINVAL;
	}

#endif

	if (shutdown_args & SHUTDOWN_ARG_IN_PROGRESS) {
		return 0;
	}

	shutdown_args |= SHUTDOWN_ARG_IN_PROGRESS;

	if (reboot) {
		shutdown_args |= SHUTDOWN_ARG_REBOOT;
	}

	if (to_bootloader) {
		shutdown_args |= SHUTDOWN_ARG_TO_BOOTLOADER;
	}

	shutdown_worker(nullptr);
	return 0;
}


#endif /* (defined(__PX4_NUTTX) && !defined(CONFIG_SCHED_WORKQUEUE)) || __PX4_QURT */
