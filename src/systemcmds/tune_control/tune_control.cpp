/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file tune_control.cpp
 *
 * Command-line tool to control & test the (external) tunes.
 * To use it make sure there's a driver running, which handles the tune_control uorb topic.
 */

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <px4_platform_common/module.h>

#include <lib/tunes/tunes.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/tune_control.h>

#include <drivers/drv_hrt.h>

#define MAX_NOTE_ITERATION 50

static void usage();

static void publish_tune_control(tune_control_s &tune_control)
{
	uORB::Publication<tune_control_s> tune_control_pub{ORB_ID(tune_control)};
	tune_control.timestamp = hrt_absolute_time();
	tune_control_pub.publish(tune_control);
}

extern "C" __EXPORT int tune_control_main(int argc, char *argv[])
{
	Tunes tunes;
	bool string_input = false;
	const char *tune_string  = nullptr;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	unsigned int value;
	tune_control_s tune_control{};
	tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "f:d:t:m:s:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'f':
			value = (uint16_t)(strtol(myoptarg, nullptr, 0));

			if (value > 0 && value < 22000) {
				tune_control.frequency = value;

			} else {
				usage();
				return 1;
			}

			break;

		case 'd':
			tune_control.duration = (uint32_t)(strtol(myoptarg, nullptr, 0));
			break;

		case 't':
			value = (uint8_t)(strtol(myoptarg, nullptr, 0));

			if (value > 0 && value < tunes.get_default_tunes_size()) {
				tune_control.tune_id = value;

			} else {
				usage();
				return 1;
			}

			break;

		case 'm':
			string_input = true;
			tune_string  = myoptarg;

			// check if string is a valid melody string
			if (tune_string[0] != 'M') {
				usage();
				return 1;
			}

			break;

		case 's':
			value = (uint8_t)(strtol(myoptarg, nullptr, 0));

			if (value <= tune_control_s::VOLUME_LEVEL_MAX) {
				tune_control.volume = value;

			} else {
				tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;
			}

			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	if (myoptind >= argc) {
		usage();
		return 1;
	}

	unsigned frequency, duration, silence;
	uint8_t volume;
	int exit_counter = 0;

	if (!strcmp(argv[myoptind], "play")) {
		if (argc >= 2 && !strcmp(argv[2], "error")) {
			tune_control.tune_id = tune_control_s::TUNE_ID_ERROR;
			publish_tune_control(tune_control);

		} else if (string_input) {
			PX4_INFO("Start playback...");
			tunes.set_string(tune_string, tune_control.volume);

			while (tunes.get_next_note(frequency, duration, silence, volume) == Tunes::Status::Continue) {
				tune_control.tune_id = 0;
				tune_control.frequency = (uint16_t)frequency;
				tune_control.duration = (uint32_t)duration;
				tune_control.silence = (uint32_t)silence;
				tune_control.volume = (uint8_t)volume;
				publish_tune_control(tune_control);
				px4_usleep(duration + silence);
				exit_counter++;

				// exit if the loop is doing too many iterations
				if (exit_counter > MAX_NOTE_ITERATION) {
					break;
				}
			}

			PX4_INFO("Playback finished.");

		} else {
			// tune id instead of string has been provided
			if (tune_control.tune_id == 0) {
				tune_control.tune_id = 1;
			}

			PX4_DEBUG("Publishing standard tune %d", tune_control.tune_id);
			publish_tune_control(tune_control);
		}

	} else if (!strcmp(argv[myoptind], "libtest")) {
		Tunes::ControlResult ret = tunes.set_control(tune_control);

		if (ret == Tunes::ControlResult::InvalidTune) {
			PX4_WARN("Tune ID not recognized.");
		}

		while (tunes.get_next_note(frequency, duration, silence, volume) == Tunes::Status::Continue) {
			PX4_INFO("frequency: %d, duration %d, silence %d, volume %d",
				 frequency, duration, silence, volume);

			px4_usleep(500000);
			exit_counter++;

			// exit if the loop is doing too many iterations
			if (exit_counter > MAX_NOTE_ITERATION) {
				break;
			}
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		PX4_INFO("Stopping playback...");
		tune_control.tune_id = 0;
		tune_control.frequency = 0;
		tune_control.duration = 0;
		tune_control.silence = 0;
		tune_control.tune_override = true;
		publish_tune_control(tune_control);

		// We wait the maximum update interval to ensure
		// The stop will not be overwritten
		px4_usleep(tunes.get_maximum_update_interval());

	}	else {
		usage();
		return 1;
	}

	return 0;
}

static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Command-line tool to control & test the (external) tunes.

Tunes are used to provide audible notification and warnings (e.g. when the system arms, gets position lock, etc.).
The tool requires that a driver is running that can handle the tune_control uorb topic.

Information about the tune format and predefined system tunes can be found here:
https://github.com/PX4/Firmware/blob/master/src/lib/tunes/tune_definition.desc

### Examples

Play system tune #2:
$ tune_control play -t 2
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tune_control", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("play", "Play system tune or single note.");
	PRINT_MODULE_USAGE_ARG("error", "Play error tune", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 1, 1, 21, "Play predefined system tune", true);
	PRINT_MODULE_USAGE_PARAM_INT('f', -1, 0, 22, "Frequency of note in Hz (0-22kHz)", true);
	PRINT_MODULE_USAGE_PARAM_INT('d', -1, 1, 21, "Duration of note in us", true);
	PRINT_MODULE_USAGE_PARAM_INT('s', 40, 0, 100, "Volume level (loudness) of the note (0-100)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('m', nullptr, R"(<string> - e.g. "MFT200e8a8a")", "Melody in string form", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("libtest", "Test library");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop playback (use for repeated tunes)");
}
