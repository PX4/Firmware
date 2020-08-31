/****************************************************************************
 *
 *  Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file test_hrt.c
 * Tests the high resolution timer.
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>
#include <sys/time.h>

#include "tests_main.h"

int test_hrt(int argc, char *argv[])
{
	struct hrt_call call;
	hrt_abstime prev, now;
	int i;
	struct timeval tv1, tv2;

	printf("start-time (hrt, sec/usec), end-time (hrt, sec/usec), microseconds per 1/10 second\n");

	for (i = 0; i < 10; i++) {
		prev = hrt_absolute_time();
		gettimeofday(&tv1, nullptr);
		px4_usleep(100000);
		now = hrt_absolute_time();
		gettimeofday(&tv2, nullptr);
		printf("%lu (%lu/%lu), %lu (%lu/%lu), %lu\n",
		       (unsigned long)prev, (unsigned long)tv1.tv_sec, (unsigned long)tv1.tv_usec,
		       (unsigned long)now, (unsigned long)tv2.tv_sec, (unsigned long)tv2.tv_usec,
		       (unsigned long)(hrt_absolute_time() - prev));
	}

	px4_usleep(1000000);

	printf("one-second ticks\n");

	for (i = 0; i < 3; i++) {
		hrt_call_after(&call, 1000000, nullptr, nullptr);

		while (!hrt_called(&call)) {
			px4_usleep(1000);
		}

		printf("tick\n");
	}

	return 0;
}
