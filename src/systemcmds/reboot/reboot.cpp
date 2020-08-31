/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file reboot.c
 * Tool similar to UNIX reboot command
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/shutdown.h>
#include <string.h>

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION("Reboot the system");

	PRINT_MODULE_USAGE_NAME_SIMPLE("reboot", "command");
	PRINT_MODULE_USAGE_PARAM_FLAG('b', "Reboot into bootloader", true);

	PRINT_MODULE_USAGE_ARG("lock|unlock", "Take/release the shutdown lock (for testing)", true);
}

extern "C" __EXPORT int reboot_main(int argc, char *argv[])
{
	int ch;
	bool to_bootloader = false;

	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b", &myoptind, &myoptarg)) != -1) {
		switch (ch) {
		case 'b':
			to_bootloader = true;
			break;

		default:
			print_usage();
			return 1;

		}
	}

	if (myoptind >= 0 && myoptind < argc) {
		int ret = -1;

		if (strcmp(argv[myoptind], "lock") == 0) {
			ret = px4_shutdown_lock();

			if (ret != 0) {
				PX4_ERR("lock failed (%i)", ret);
			}
		}

		if (strcmp(argv[myoptind], "unlock") == 0) {
			ret = px4_shutdown_unlock();

			if (ret != 0) {
				PX4_ERR("unlock failed (%i)", ret);
			}
		}

		return ret;
	}

	int ret = px4_reboot_request(to_bootloader);

	if (ret < 0) {
		PX4_ERR("reboot failed (%i)", ret);
		return -1;
	}

	while (1) { px4_usleep(1); } // this command should not return on success

	return 0;
}
