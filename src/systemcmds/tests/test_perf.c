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
 * @file test_perf.c
 * Tests related to the performance counter.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>

#include <perf/perf_counter.h>

#include "tests_main.h"

int
test_perf(int argc, char *argv[])
{
	perf_counter_t cc = perf_alloc(PC_COUNT, "test_count");
	perf_counter_t ec = perf_alloc(PC_ELAPSED, "test_elapsed");

	if ((cc == NULL) || (ec == NULL)) {
		printf("perf: counter alloc failed\n");
		return 1;
	}

	perf_begin(ec);
	perf_count(cc);
	perf_count(cc);
	perf_count(cc);
	perf_count(cc);
	printf("perf: expect count of 4\n");
	perf_print_counter(cc);
	perf_end(ec);
	printf("perf: expect count of 1\n");
	perf_print_counter(ec);

	perf_free(cc);
	perf_free(ec);

	return OK;
}
