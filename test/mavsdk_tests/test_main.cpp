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

#define CATCH_CONFIG_RUNNER
#include "catch2/catch.hpp"

#include <mavsdk/mavsdk.h>
#include <iostream>
#include <string>
#include "autopilot_tester.h"


static void usage(const std::string &bin_name);
static void remove_argv(int &argc, char **argv, int pos);

int main(int argc, char **argv)
{
	for (int i = 0; i < argc; ++i) {
		const std::string argv_string(argv[i]);

		if (argv_string == "-h") {
			usage(argv[0]);
			return 0;
		}

		if (argv_string == "--url") {
			if (argc > i + 1) {
				connection_url = argv[i + 1];
				remove_argv(argc, argv, i);
				remove_argv(argc, argv, i);
				--i;

			} else {
				std::cerr << "No connection URL supplied" << std::endl;
				usage(argv[0]);
				return -1;
			}

		}

		if (argv_string == "--speed-factor") {
			if (argc > i + 1) {
				try {
					speed_factor = std::make_optional(std::stof(argv[i + 1]));

				} catch (const std::invalid_argument &) {
					std::cerr << "Speed factor could not be parsed" << std::endl;
					return -1;

				} catch (const std::out_of_range &) {
					std::cerr << "Speed factor is out of range" << std::endl;
					return -1;
				}

				remove_argv(argc, argv, i);
				remove_argv(argc, argv, i);
				--i;


			} else {
				std::cerr << "No speed factor supplied" << std::endl;
				usage(argv[0]);
				return -1;
			}
		}
	}

	Catch::Session session;
	const int catch_ret = session.applyCommandLine(argc, argv);

	if (catch_ret != 0) {
		return catch_ret;
	}

	return session.run();
}

void usage(const std::string &bin_name)
{
	std::cout << std::endl
		  << "Usage : " << bin_name << " [--url CONNECTION_URL] [--speed-factor SPEED_FACTOR] [catch2 arguments]\n"
		  << "\n"
		  << "  --url          Connection URL format should be :\n"
		  << "                   For TCP : tcp://[server_host][:server_port]\n"
		  << "                   For UDP : udp://[bind_host][:bind_port]\n"
		  << "                   For Serial : serial:///path/to/serial/dev[:baudrate]\n"
		  << "                 For example, to connect to the simulator use URL: udp://:14540\n"
		  << "\n"
		  << "  --speed-factor Speed factor to compare against, for information only\n"
		  << std::flush;
}

void remove_argv(int &argc, char **argv, int pos)
{
	for (int i = pos; i + 1 < argc; ++i) {
		argv[i] = argv[i + 1];
	}

	argv[--argc] = nullptr;
}
