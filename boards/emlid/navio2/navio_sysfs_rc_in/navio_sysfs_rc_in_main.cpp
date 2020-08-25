/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>

#include "NavioSysRCInput.hpp"

namespace navio_sysfs_rc_in
{

static void usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PX4_INFO("usage: navio_sysfs_rc_in {start|stop|status}");
}

static NavioSysRCInput *rc_input = nullptr;

extern "C" __EXPORT int navio_sysfs_rc_in_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (rc_input != nullptr && rc_input->isRunning()) {
			PX4_WARN("already running");
			/* this is not an error */
			return 0;
		}

		rc_input = new NavioSysRCInput();

		// Check if alloc worked.
		if (rc_input == nullptr) {
			PX4_ERR("alloc failed");
			return -1;
		}

		int ret = rc_input->start();

		if (ret != 0) {
			PX4_ERR("start failed");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {

		if (rc_input == nullptr || !rc_input->isRunning()) {
			PX4_WARN("not running");
			/* this is not an error */
			return 0;
		}

		rc_input->stop();

		// Wait for task to die
		int i = 0;

		do {
			/* wait up to 3s */
			usleep(100000);

		} while (rc_input->isRunning() && ++i < 30);

		delete rc_input;
		rc_input = nullptr;

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (rc_input != nullptr) {
			rc_input->print_status();

		} else {
			PX4_INFO("not running");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;

}

}; // namespace navio_sysfs_rc_in
