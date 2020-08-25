/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file px4_posix_impl.cpp
 *
 * PX4 Middleware Wrapper Linux Implementation
 */

#include <px4_platform_common/defines.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <stdint.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <parameters/param.h>
#include "hrt_work.h"
#include <drivers/drv_hrt.h>
#include <pthread.h>
#include <px4_platform_common/init.h>

extern pthread_t _shell_task_id;

__BEGIN_DECLS

long PX4_TICKS_PER_SEC = sysconf(_SC_CLK_TCK);

__END_DECLS

namespace px4
{

void init_once();

void init_once()
{
	_shell_task_id = pthread_self();

	work_queues_init();
	hrt_work_queue_init();

	px4_platform_init();
}

void init(int argc, char *argv[], const char *app_name)
{
	printf("\n");
	printf("______  __   __    ___ \n");
	printf("| ___ \\ \\ \\ / /   /   |\n");
	printf("| |_/ /  \\ V /   / /| |\n");
	printf("|  __/   /   \\  / /_| |\n");
	printf("| |     / /^\\ \\ \\___  |\n");
	printf("\\_|     \\/   \\/     |_/\n");
	printf("\n");
	printf("%s starting.\n", app_name);
	printf("\n");

	// set the threads name
#ifdef __PX4_DARWIN
	(void)pthread_setname_np(app_name);
#else
	(void)pthread_setname_np(pthread_self(), app_name);
#endif
}

}

