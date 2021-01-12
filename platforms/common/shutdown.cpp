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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>

#include <drivers/drv_hrt.h>

#ifndef MODULE_NAME
#define MODULE_NAME "shutdown"
#endif

#include <px4_platform_common/external_reset_lockout.h>
#include <px4_platform_common/log.h>

#include <stdint.h>
#include <errno.h>
#include <pthread.h>

#ifdef __PX4_NUTTX
#include <nuttx/board.h>
#endif

using namespace time_literals;

static pthread_mutex_t shutdown_mutex =
	PTHREAD_MUTEX_INITIALIZER; // protects access to shutdown_hooks & shutdown_lock_counter
static uint8_t shutdown_lock_counter = 0;

int px4_shutdown_lock()
{
	int ret = pthread_mutex_lock(&shutdown_mutex);

	if (ret == 0) {
		++shutdown_lock_counter;
		px4_indicate_external_reset_lockout(LockoutComponent::SystemShutdownLock, true);
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

			if (shutdown_lock_counter == 0) {
				px4_indicate_external_reset_lockout(LockoutComponent::SystemShutdownLock, false);
			}

		} else {
			PX4_ERR("unmatched number of px4_shutdown_unlock() calls");
		}

		return pthread_mutex_unlock(&shutdown_mutex);
	}

	return ret;
}

#if defined(CONFIG_SCHED_WORKQUEUE)

static struct work_s shutdown_work = {};
static uint16_t shutdown_counter = 0; ///< count how many times the shutdown worker was executed

#define SHUTDOWN_ARG_IN_PROGRESS (1<<0)
#define SHUTDOWN_ARG_REBOOT (1<<1)
#define SHUTDOWN_ARG_TO_BOOTLOADER (1<<2)
static uint8_t shutdown_args = 0;

static constexpr int max_shutdown_hooks = 1;
static shutdown_hook_t shutdown_hooks[max_shutdown_hooks] = {};

static hrt_abstime shutdown_time_us = 0;
static constexpr hrt_abstime shutdown_timeout_us =
	5_s; ///< force shutdown after this time if modules do not respond in time

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

/**
 * work queue callback method to shutdown.
 * @param arg unused
 */
static void shutdown_worker(void *arg)
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

	const hrt_abstime now = hrt_absolute_time();
	const bool delay_elapsed = (now > shutdown_time_us);

	if (delay_elapsed && ((done && shutdown_lock_counter == 0) || (now > (shutdown_time_us + shutdown_timeout_us)))) {
		if (shutdown_args & SHUTDOWN_ARG_REBOOT) {
#if defined(CONFIG_BOARDCTL_RESET)
			PX4_INFO_RAW("Reboot NOW.");
			board_reset((shutdown_args & SHUTDOWN_ARG_TO_BOOTLOADER) ? 1 : 0);
#else
			PX4_PANIC("board reset not available");
#endif

		} else {
#if defined(CONFIG_BOARDCTL_POWEROFF)
			PX4_INFO_RAW("Powering off NOW.");
			board_power_off(0);
#elif !defined(CONFIG_BOARDCTL_POWEROFF) && defined(__PX4_POSIX)
			// simply exit on posix if real shutdown (poweroff) not available
			PX4_INFO_RAW("Exiting NOW.");
			system_exit(0);
#else
			PX4_PANIC("board shutdown not available");
#endif
		}

		pthread_mutex_unlock(&shutdown_mutex); // must NEVER come here

	} else {
		pthread_mutex_unlock(&shutdown_mutex);
		work_queue(HPWORK, &shutdown_work, (worker_t)&shutdown_worker, nullptr, USEC2TICK(10000));
	}
}

#if defined(CONFIG_BOARDCTL_RESET)
int px4_reboot_request(bool to_bootloader, uint32_t delay_us)
{
	pthread_mutex_lock(&shutdown_mutex);

	if (shutdown_args & SHUTDOWN_ARG_IN_PROGRESS) {
		pthread_mutex_unlock(&shutdown_mutex);
		return 0;
	}

	shutdown_args |= SHUTDOWN_ARG_REBOOT;

	if (to_bootloader) {
		shutdown_args |= SHUTDOWN_ARG_TO_BOOTLOADER;
	}

	shutdown_time_us = hrt_absolute_time();

	if (delay_us > 0) {
		shutdown_time_us += delay_us;
	}

	work_queue(HPWORK, &shutdown_work, (worker_t)&shutdown_worker, nullptr, 1);
	pthread_mutex_unlock(&shutdown_mutex);
	return 0;
}
#endif // CONFIG_BOARDCTL_RESET

#if defined(CONFIG_BOARDCTL_POWEROFF) || defined(__PX4_POSIX)
int px4_shutdown_request(uint32_t delay_us)
{
	pthread_mutex_lock(&shutdown_mutex);

	if (shutdown_args & SHUTDOWN_ARG_IN_PROGRESS) {
		pthread_mutex_unlock(&shutdown_mutex);
		return 0;
	}

	shutdown_args |= SHUTDOWN_ARG_IN_PROGRESS;

	shutdown_time_us = hrt_absolute_time();

	if (delay_us > 0) {
		shutdown_time_us += delay_us;
	}

	work_queue(HPWORK, &shutdown_work, (worker_t)&shutdown_worker, nullptr, 1);
	pthread_mutex_unlock(&shutdown_mutex);
	return 0;
}
#endif // CONFIG_BOARDCTL_POWEROFF

#endif // CONFIG_SCHED_WORKQUEUE)
