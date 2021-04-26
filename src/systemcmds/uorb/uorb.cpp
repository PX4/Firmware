/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <string.h>

#include <uORB/uORB.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

extern "C" { __EXPORT int uorb_main(int argc, char *argv[]); }

static void usage();

int uorb_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -1;
	}

	if (!strcmp(argv[1], "start")) {
		return uorb_start();

	} else if (!strcmp(argv[1], "status")) {
		return uorb_status();

	} else if (!strcmp(argv[1], "top")) {
		return uorb_top(argv + 2, argc - 2);
	}

	usage();
	return 0;
}

void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
uORB is the internal pub-sub messaging system, used for communication between modules.

### Implementation
The implementation is asynchronous and lock-free, ie. a publisher does not wait for a subscriber and vice versa.
This is achieved by having a separate buffer between a publisher and a subscriber.

The code is optimized to minimize the memory footprint and the latency to exchange messages.

Messages are defined in the `/msg` directory. They are converted into C/C++ code at build-time.

If compiled with ORB_USE_PUBLISHER_RULES, a file with uORB publication rules can be used to configure which
modules are allowed to publish which topics. This is used for system-wide replay.

### Examples
Monitor topic publication rates. Besides `top`, this is an important command for general system inspection:
$ uorb top
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uorb", "communication");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print topic statistics");
	PRINT_MODULE_USAGE_COMMAND_DESCR("top", "Monitor topic publication rates");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "print all instead of only currently publishing topics with subscribers", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('1', "run only once, then exit", true);
	PRINT_MODULE_USAGE_ARG("<filter1> [<filter2>]", "topic(s) to match (implies -a)", true);
}
