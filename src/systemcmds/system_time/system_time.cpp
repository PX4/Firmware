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

#include <px4_platform_common/module.h>
#include <px4_platform_common/time.h>

using namespace time_literals;

static void	usage();

extern "C" {
	__EXPORT int system_time_main(int argc, char *argv[]);
}

int
system_time_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "get")) {
			//get system time
			struct timespec ts = {};
			px4_clock_gettime(CLOCK_REALTIME, &ts);
			time_t utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);

			//convert to date time
			char buf[80];
			struct tm date_time;
			localtime_r(&utc_time_sec, &date_time);
			strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &date_time);

			//get time since boot
			hrt_abstime since_boot_sec = hrt_absolute_time() / 1_s;

			PX4_INFO("Unix epoch time: %ld", (long)utc_time_sec);
			PX4_INFO("System time: %s", buf);
			PX4_INFO("Uptime (since boot): %" PRIu64 " s", since_boot_sec);
			return 0;
		}

		if (!strcmp(argv[1], "set")) {
			if (argc == 3) {
				//set system time
				struct timespec ts = {};
				ts.tv_sec = (time_t)strtoul(argv[2], nullptr, 0);
				ts.tv_nsec = 0.0;

				int res = px4_clock_settime(CLOCK_REALTIME, &ts);

				if (res == 0) {
					PX4_INFO("Successfully set system time");
					return 0;

				} else {
					PX4_ERR("Failed to set system time (%i)", res);
					return 1;
				}

			} else {
				usage();
				return 1;
			}
		}

		//unknown command
		PX4_ERR("system_time: Unknown subcommand");
		usage();
		return 1;
	}

	//not enough arguments
	PX4_ERR("system_time: not enough arguments");
	usage();
	return 1;
}

static void
usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Command-line tool to set and get system time.

### Examples

Set the system time and read it back
$ system_time set 1600775044
$ system_time get
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("system_time", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Set the system time, provide time in unix epoch time format");
	PRINT_MODULE_USAGE_COMMAND_DESCR("get", "Get the system time");

}