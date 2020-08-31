/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include "uORBManager.hpp"
#include "uORB.h"
#include "uORBCommon.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

extern "C" { __EXPORT int uorb_main(int argc, char *argv[]); }

static uORB::DeviceMaster *g_dev = nullptr;
static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
uORB is the internal pub-sub messaging system, used for communication between modules.

It is typically started as one of the very first modules and most other modules depend on it.

### Implementation
No thread or work queue is needed, the module start only makes sure to initialize the shared global state.
Communication is done via shared memory.
The implementation is asynchronous and lock-free, ie. a publisher does not wait for a subscriber and vice versa.
This is achieved by having a separate buffer between a publisher and a subscriber.

The code is optimized to minimize the memory footprint and the latency to exchange messages.

The interface is based on file descriptors: internally it uses `read`, `write` and `ioctl`. Except for the
publications, which use `orb_advert_t` handles, so that they can be used from interrupts as well (on NuttX).

Messages are defined in the `/msg` directory. They are converted into C/C++ code at build-time.

If compiled with ORB_USE_PUBLISHER_RULES, a file with uORB publication rules can be used to configure which
modules are allowed to publish which topics. This is used for system-wide replay.

### Examples
Monitor topic publication rates. Besides `top`, this is an important command for general system inspection:
$ uorb top
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uorb", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print topic statistics");
	PRINT_MODULE_USAGE_COMMAND_DESCR("top", "Monitor topic publication rates");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "print all instead of only currently publishing topics with subscribers", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('1', "run only once, then exit", true);
	PRINT_MODULE_USAGE_ARG("<filter1> [<filter2>]", "topic(s) to match (implies -a)", true);
}

int
uorb_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {

		if (g_dev != nullptr) {
			PX4_WARN("already loaded");
			/* user wanted to start uorb, its already running, no error */
			return 0;
		}

		if (!uORB::Manager::initialize()) {
			PX4_ERR("uorb manager alloc failed");
			return -ENOMEM;
		}

		/* create the driver */
		g_dev = uORB::Manager::get_instance()->get_device_master();

		if (g_dev == nullptr) {
			return -errno;
		}

#if !defined(__PX4_QURT)
		/* FIXME: this fails on Snapdragon (see https://github.com/PX4/Firmware/issues/5406),
		 * so we disable logging messages to the ulog for now. This needs further investigations.
		 */
		px4_log_initialize();
#endif

		return OK;
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "status")) {
		if (g_dev != nullptr) {
			g_dev->printStatistics();

		} else {
			PX4_INFO("uorb is not running");
		}

		return OK;
	}

	if (!strcmp(argv[1], "top")) {
		if (g_dev != nullptr) {
			g_dev->showTop(argv + 2, argc - 2);

		} else {
			PX4_INFO("uorb is not running");
		}

		return OK;
	}

	usage();
	return -EINVAL;
}
