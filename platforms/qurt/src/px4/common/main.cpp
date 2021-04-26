/****************************************************************************
 * Copyright (C) 2015-2016 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 *
 * Basic shell to execute builtin "apps" on QURT.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/defines.h>
#include <dspal_platform.h>
#include <vector>
#include <string>
#include <map>
#include <stdio.h>
#include <stdlib.h>

#include "get_commands.h"
#include "apps.h"

#define MAX_ARGS 8 // max number of whitespace separated args after app name


static px4_task_t g_dspal_task = -1;

__BEGIN_DECLS
// The commands to run are specified in a target file: commands_<target>.c
extern const char *get_commands(void);

// Enable external library hook
void qurt_external_hook(void) __attribute__((weak));
__END_DECLS

void qurt_external_hook(void)
{
}

static void run_cmd(apps_map_type &apps, const std::vector<std::string> &appargs)
{
	// command is appargs[0]
	std::string command = appargs[0];

	//replaces app.find with iterator code to avoid null pointer exception
	for (apps_map_type::iterator it = apps.begin(); it != apps.end(); ++it)
		if (it->first == command) {
			// one for command name, one for null terminator
			const char *arg[MAX_ARGS + 2];

			unsigned int i = 0;

			if (appargs.size() > MAX_ARGS + 1) {
				PX4_ERR("%d too many arguments in run_cmd", appargs.size() - (MAX_ARGS + 1));
				return;
			}

			while (i < appargs.size() && appargs[i].c_str()[0] != '\0') {
				arg[i] = (char *)appargs[i].c_str();
				PX4_DEBUG("  arg%d = '%s'\n", i, arg[i]);
				++i;
			}

			arg[i] = (char *)0;

			//PX4_DEBUG_PRINTF(i);
			if (apps[command] == NULL) {
				PX4_ERR("Null function !!\n");

			} else {
				apps[command](i, (char **)arg);
				break;
			}

		}
}

void eat_whitespace(const char *&b, int &i)
{
	// Eat whitespace
	while (b[i] == ' ' || b[i] == '\t') { ++i; }

	b = &b[i];
	i = 0;
}

static void process_commands(apps_map_type &apps, const char *cmds)
{
	std::vector<std::string> appargs;
	int i = 0;
	const char *b = cmds;
	char arg[256];

	// Eat leading whitespace
	eat_whitespace(b, i);

	for (;;) {
		// End of command line
		if (b[i] == '\n' || b[i] == '\0') {
			strncpy(arg, b, i);
			arg[i] = '\0';
			appargs.push_back(arg);

			// If we have a command to run
			if (appargs.size() > 0) {
				PX4_DEBUG("Processing command: %s", appargs[0].c_str());

				for (int ai = 1; ai < (int)appargs.size(); ai++) {
					PX4_DEBUG("   > arg: %s", appargs[ai].c_str());
				}

				run_cmd(apps, appargs);
			}

			appargs.clear();

			if (b[i] == '\n') {
				eat_whitespace(b, ++i);
				continue;

			} else {
				break;
			}
		}

		// End of arg
		else if (b[i] == ' ') {
			strncpy(arg, b, i);
			arg[i] = '\0';
			appargs.push_back(arg);
			eat_whitespace(b, ++i);
			continue;
		}

		++i;
	}
}

namespace px4
{
extern void init_once(void);
};

__BEGIN_DECLS
int dspal_main(int argc, char *argv[]);
__END_DECLS


const char *get_commands()
{
	// All that needs to be started automatically on the DSP side
	// are uorb and qshell. After that, everything else can get
	// started from the main startup script on the Linux side.
	static const char *commands = "qshell start\n";

	return commands;
}


int dspal_entry(int argc, char *argv[])
{
	PX4_INFO("In dspal_entry");
	apps_map_type apps;
	init_app_map(apps);
	px4::init_once();
	px4::init(argc, (char **)argv, "px4");
	process_commands(apps, get_commands());
	sleep(1); // give time for all commands to execute before starting external function

	if (qurt_external_hook) {
		qurt_external_hook();
	}

	for (;;) {
		sleep(1);
	}

	return 0;
}

static void usage()
{
	PX4_INFO("Usage: dspal {start |stop}");
}


extern "C" {

	int dspal_main(int argc, char *argv[])
	{
		int ret = 0;

		if (argc == 2 && strcmp(argv[1], "start") == 0) {
			g_dspal_task = px4_task_spawn_cmd("dspal",
							  SCHED_DEFAULT,
							  SCHED_PRIORITY_MAX - 5,
							  1500,
							  dspal_entry,
							  argv);

		} else if (argc == 2 && strcmp(argv[1], "stop") == 0) {
			if (g_dspal_task < 0) {
				PX4_WARN("start up thread not running");

			} else {
				px4_task_delete(g_dspal_task);
				g_dspal_task = -1;
			}

		} else {
			usage();
			ret = -1;
		}

		return ret;
	}
}
