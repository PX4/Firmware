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

/**
 * @file param.cpp
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 *
 * Parameter tool.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

#include <float.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <inttypes.h>
#include <sys/stat.h>

#include <parameters/param.h>
#include "systemlib/err.h"

__BEGIN_DECLS
__EXPORT int param_main(int argc, char *argv[]);
__END_DECLS

enum class COMPARE_OPERATOR {
	EQUAL = 0,
	GREATER = 1,
};

enum class COMPARE_ERROR_LEVEL {
	DO_ERROR = 0,
	SILENT = 1,
};


#ifdef __PX4_QURT
#define PARAM_PRINT PX4_INFO
#else
#define PARAM_PRINT PX4_INFO_RAW
#endif

static int 	do_save(const char *param_file_name);
static int	do_save_default();
static int 	do_load(const char *param_file_name);
static int	do_import(const char *param_file_name = nullptr);
static int	do_show(const char *search_string, bool only_changed);
static int	do_show_for_airframe();
static int	do_show_all();
static int	do_show_quiet(const char *param_name);
static int	do_show_index(const char *index, bool used_index);
static void	do_show_print(void *arg, param_t param);
static void	do_show_print_for_airframe(void *arg, param_t param);
static int	do_set(const char *name, const char *val, bool fail_on_not_found);
static int	do_set_custom_default(const char *name, const char *val);
static int	do_compare(const char *name, char *vals[], unsigned comparisons, enum COMPARE_OPERATOR cmd_op,
			   enum COMPARE_ERROR_LEVEL err_level);
static int 	do_reset_all(const char *excludes[], int num_excludes);
static int 	do_reset_specific(const char *resets[], int num_resets);
static int 	do_touch(const char *params[], int num_params);
static int	do_find(const char *name);

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Command to access and manipulate parameters via shell or script.

This is used for example in the startup script to set airframe-specific parameters.

Parameters are automatically saved when changed, eg. with `param set`. They are typically stored to FRAM
or to the SD card. `param select` can be used to change the storage location for subsequent saves (this will
need to be (re-)configured on every boot).

If the FLASH-based backend is enabled (which is done at compile time, e.g. for the Intel Aero or Omnibus),
`param select` has no effect and the default is always the FLASH backend. However `param save/load <file>`
can still be used to write to/read from files.

Each parameter has a 'used' flag, which is set when it's read during boot. It is used to only show relevant
parameters to a ground control station.

### Examples
Change the airframe and make sure the airframe's default parameters are loaded:
$ param set SYS_AUTOSTART 4001
$ param set SYS_AUTOCONFIG 1
$ reboot
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("param", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("load", "Load params from a file (overwrite all)");
	PRINT_MODULE_USAGE_ARG("<file>", "File name (use default if not given)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("import", "Import params from a file");
	PRINT_MODULE_USAGE_ARG("<file>", "File name (use default if not given)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Save params to a file");
	PRINT_MODULE_USAGE_ARG("<file>", "File name (use default if not given)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("select", "Select default file");
	PRINT_MODULE_USAGE_ARG("<file>", "File name (use <root>/eeprom/parameters if not given)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("show", "Show parameter values");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Show all parameters (not just used)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('c', "Show only changed params (unused too)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('q', "quiet mode, print only param value (name needs to be exact)", true);
	PRINT_MODULE_USAGE_ARG("<filter>", "Filter by param name (wildcard at end allowed, eg. sys_*)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("show-for-airframe", "Show changed params for airframe config");

	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status of parameter system");

	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Set parameter to a value");
	PRINT_MODULE_USAGE_ARG("<param_name> <value>", "Parameter name and value to set", false);
	PRINT_MODULE_USAGE_ARG("fail", "If provided, let the command fail if param is not found", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("set-default", "Set parameter default to a value");
	PRINT_MODULE_USAGE_ARG("<param_name> <value>", "Parameter name and value to set", false);
	PRINT_MODULE_USAGE_ARG("fail", "If provided, let the command fail if param is not found", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("compare", "Compare a param with a value. Command will succeed if equal");
	PRINT_MODULE_USAGE_PARAM_FLAG('s', "If provided, silent errors if parameter doesn't exists", true);
	PRINT_MODULE_USAGE_ARG("<param_name> <value>", "Parameter name and value to compare", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("greater",
					 "Compare a param with a value. Command will succeed if param is greater than the value");
	PRINT_MODULE_USAGE_PARAM_FLAG('s', "If provided, silent errors if parameter doesn't exists", true);
	PRINT_MODULE_USAGE_ARG("<param_name> <value>", "Parameter name and value to compare", false);

	PRINT_MODULE_USAGE_ARG("<param_name> <value>", "Parameter name and value to compare", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("touch", "Mark a parameter as used");
	PRINT_MODULE_USAGE_ARG("<param_name1> [<param_name2>]", "Parameter name (one or more)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset only specified params to default");
	PRINT_MODULE_USAGE_ARG("<param1> [<param2>]", "Parameter names to reset (wildcard at end allowed)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset_all", "Reset all params to default");
	PRINT_MODULE_USAGE_ARG("<exclude1> [<exclude2>]", "Do not reset matching params (wildcard at end allowed)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("index", "Show param for a given index");
	PRINT_MODULE_USAGE_ARG("<index>", "Index: an integer >= 0", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("index_used", "Show used param for a given index");
	PRINT_MODULE_USAGE_ARG("<index>", "Index: an integer >= 0", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("find", "Show index of a param");
	PRINT_MODULE_USAGE_ARG("<param>", "param name", false);
}

int
param_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "save")) {
			if (argc >= 3) {
				return do_save(argv[2]);

			} else {
				int ret = do_save_default();

				if (ret) {
					PX4_ERR("Param save failed (%i)", ret);
					return 1;

				} else {
					return 0;
				}
			}
		}

		if (!strcmp(argv[1], "load")) {
			if (argc >= 3) {
				return do_load(argv[2]);

			} else {
				return do_load(param_get_default_file());
			}
		}

		if (!strcmp(argv[1], "import")) {
			if (argc >= 3) {
				return do_import(argv[2]);

			} else {
				return do_import();
			}
		}

		if (!strcmp(argv[1], "select")) {
			if (argc >= 3) {
				param_set_default_file(argv[2]);

			} else {
				param_set_default_file(nullptr);
			}

			const char *default_file = param_get_default_file();

			if (default_file) {
				PX4_INFO("selected parameter default file %s", default_file);
			}

			return 0;
		}

		if (!strcmp(argv[1], "show")) {
			if (argc >= 3) {
				// optional argument -c to show only non-default params
				if (!strcmp(argv[2], "-c")) {
					if (argc >= 4) {
						return do_show(argv[3], true);

					} else {
						return do_show(nullptr, true);
					}

				} else if (!strcmp(argv[2], "-a")) {
					return do_show_all();

				} else if (!strcmp(argv[2], "-q")) {
					if (argc >= 4) {
						return do_show_quiet(argv[3]);
					}

				} else {
					return do_show(argv[2], false);
				}

			} else {
				return do_show(nullptr, false);
			}
		}

		if (!strcmp(argv[1], "show-for-airframe")) {
			return do_show_for_airframe();
		}

		if (!strcmp(argv[1], "status")) {
			param_print_status();
			return PX4_OK;
		}

		if (!strcmp(argv[1], "set")) {
			if (argc >= 5) {

				/* if the fail switch is provided, fails the command if not found */
				bool fail = !strcmp(argv[4], "fail");

				return do_set(argv[2], argv[3], fail);

			} else if (argc >= 4) {
				return do_set(argv[2], argv[3], false);

			} else {
				PX4_ERR("not enough arguments.\nTry 'param set %s 3 [fail]'", (argc > 2) ? argv[2] : "PARAM_NAME");
				return 1;
			}
		}

		if (!strcmp(argv[1], "set-default")) {
			if (argc == 4) {
				return do_set_custom_default(argv[2], argv[3]);

			} else {
				PX4_ERR("not enough arguments.\nTry 'param set-default %s 3'", (argc > 2) ? argv[2] : "PARAM_NAME");
				return 1;
			}
		}

		if (!strcmp(argv[1], "compare")) {
			if (argc >= 5 && !strcmp(argv[2], "-s")) {
				return do_compare(argv[3], &argv[4], argc - 4, COMPARE_OPERATOR::EQUAL, COMPARE_ERROR_LEVEL::SILENT);

			} else if (argc >= 4) {
				return do_compare(argv[2], &argv[3], argc - 3, COMPARE_OPERATOR::EQUAL, COMPARE_ERROR_LEVEL::DO_ERROR);

			} else {
				PX4_ERR("not enough arguments.\nTry 'param compare %s 3'", (argc > 2) ? argv[2] : "PARAM_NAME");
				return 1;
			}
		}

		if (!strcmp(argv[1], "greater")) {
			if (argc >= 5 && !strcmp(argv[2], "-s")) {
				return do_compare(argv[3], &argv[4], argc - 4, COMPARE_OPERATOR::GREATER, COMPARE_ERROR_LEVEL::SILENT);

			} else if (argc >= 4) {
				return do_compare(argv[2], &argv[3], argc - 3, COMPARE_OPERATOR::GREATER, COMPARE_ERROR_LEVEL::DO_ERROR);

			} else {
				PX4_ERR("not enough arguments.\nTry 'param greater %s 3'", (argc > 2) ? argv[2] : "PARAM_NAME");
				return 1;
			}
		}

		if (!strcmp(argv[1], "reset")) {
			if (argc >= 3) {
				return do_reset_specific((const char **) &argv[2], argc - 2);

			} else {
				PX4_ERR("not enough arguments (use 'param reset_all' to reset all).");
				return 1;
			}
		}

		if (!strcmp(argv[1], "reset_all")) {
			if (argc >= 3) {
				return do_reset_all((const char **) &argv[2], argc - 2);

			} else {
				return do_reset_all(nullptr, 0);
			}
		}

		if (!strcmp(argv[1], "touch")) {
			if (argc >= 3) {
				return do_touch((const char **) &argv[2], argc - 2);

			} else {
				PX4_ERR("not enough arguments.");
				return 1;
			}
		}

		if (!strcmp(argv[1], "index_used")) {
			if (argc >= 3) {
				return do_show_index(argv[2], true);

			} else {
				PX4_ERR("no index provided");
				return 1;
			}
		}

		if (!strcmp(argv[1], "index")) {
			if (argc >= 3) {
				return do_show_index(argv[2], false);

			} else {
				PX4_ERR("no index provided");
				return 1;
			}
		}

		if (!strcmp(argv[1], "find")) {
			if (argc >= 3) {
				return do_find(argv[2]);

			} else {
				PX4_ERR("not enough arguments.\nTry 'param find %s'", (argc > 2) ? argv[2] : "PARAM_NAME");
				return 1;
			}
		}
	}

	print_usage();
	return 1;
}

static int
do_save(const char *param_file_name)
{
	/* create the file */
	int fd = open(param_file_name, O_WRONLY | O_CREAT, PX4_O_MODE_666);

	if (fd < 0) {
		PX4_ERR("open '%s' failed (%i)", param_file_name, errno);
		return 1;
	}

	int result = param_export(fd, false, nullptr);
	close(fd);

	if (result < 0) {
#ifndef __PX4_QURT
		(void)unlink(param_file_name);
#endif
		PX4_ERR("exporting to '%s' failed (%i)", param_file_name, result);
		return 1;
	}

	return 0;
}

static int
do_load(const char *param_file_name)
{
	int fd = -1;

	if (param_file_name) { // passing NULL means to select the flash storage
		fd = open(param_file_name, O_RDONLY);

		if (fd < 0) {
			PX4_ERR("open '%s' failed (%i)", param_file_name, errno);
			return 1;
		}
	}

	int result = param_load(fd);

	if (fd >= 0) {
		close(fd);
	}

	if (result < 0) {
		if (param_file_name) {
			PX4_ERR("importing from '%s' failed (%i)", param_file_name, result);

		} else {
			PX4_ERR("importing failed (%i)", result);
		}

		return 1;
	}

	return 0;
}

static int
do_import(const char *param_file_name)
{
	bool mark_saved = false;

	if (param_file_name == nullptr) {
		param_file_name = param_get_default_file();
		mark_saved = true; // if imported from default storage, mark as saved
	}

	int fd = -1;

	if (param_file_name) { // passing NULL means to select the flash storage
		fd = open(param_file_name, O_RDONLY);

		if (fd < 0) {
			PX4_ERR("open '%s' failed (%i)", param_file_name, errno);
			return 1;
		}
	}

	int result = param_import(fd, mark_saved);

	if (fd >= 0) {
		close(fd);
	}

	if (result < 0) {
		if (param_file_name) {
			PX4_ERR("importing from '%s' failed (%i)", param_file_name, result);

		} else {
			PX4_ERR("importing failed (%i)", result);
		}

		return 1;
	}

	return 0;
}

static int
do_save_default()
{
	return param_save_default();
}

static int
do_show(const char *search_string, bool only_changed)
{
	PARAM_PRINT("Symbols: x = used, + = saved, * = unsaved\n");
	// also show unused params if we show non-default values only
	param_foreach(do_show_print, (char *)search_string, only_changed, !only_changed);
	PARAM_PRINT("\n %u/%u parameters used.\n", param_count_used(), param_count());

	return 0;
}

static int
do_show_for_airframe()
{
	param_foreach(do_show_print_for_airframe, nullptr, true, true);
	int32_t sys_autostart = 0;
	param_get(param_find("SYS_AUTOSTART"), &sys_autostart);
	if (sys_autostart != 0) {
		PARAM_PRINT("# Make sure to add all params from the current airframe (ID=%i) as well\n", sys_autostart);
	}
	return 0;
}

static int
do_show_all()
{
	PARAM_PRINT("Symbols: x = used, + = saved, * = unsaved\n");
	param_foreach(do_show_print, nullptr, false, false);
	PARAM_PRINT("\n %u parameters total, %u used.\n", param_count(), param_count_used());

	return 0;
}

static int
do_show_quiet(const char *param_name)
{
	param_t param = param_find_no_notification(param_name);
	int32_t ii;
	float ff;
	// Print only the param value (can be used in scripts)

	if (param == PARAM_INVALID) {
		return 1;
	}

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		if (!param_get(param, &ii)) {
			PARAM_PRINT("%ld", (long)ii);
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &ff)) {
			PARAM_PRINT("%4.4f", (double)ff);
		}

		break;

	default:
		return 1;
	}

	return 0;
}

static int
do_find(const char *name)
{
	param_t ret = param_find_no_notification(name);

	if (ret == PARAM_INVALID) {
		PX4_ERR("Parameter %s not found", name);
		return 1;
	}

	PARAM_PRINT("Found param %s at index %i\n", name, (int)ret);
	return 0;
}

static int
do_show_index(const char *index, bool used_index)
{
	char *end;
	int i = strtol(index, &end, 10);
	param_t param;
	int32_t ii;
	float ff;

	if (used_index) {
		param = param_for_used_index(i);

	} else {
		param = param_for_index(i);
	}

	if (param == PARAM_INVALID) {
		PX4_ERR("param not found for index %u", i);
		return 1;
	}

	PARAM_PRINT("index %d: %c %c %s [%d,%d] : ", i, (param_used(param) ? 'x' : ' '),
		    param_value_unsaved(param) ? '*' : (param_value_is_default(param) ? ' ' : '+'),
		    param_name(param), param_get_used_index(param), param_get_index(param));

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		if (!param_get(param, &ii)) {
			PARAM_PRINT("%ld\n", (long)ii);
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &ff)) {
			PARAM_PRINT("%4.4f\n", (double)ff);
		}

		break;

	default:
		PARAM_PRINT("<unknown type %d>\n", 0 + param_type(param));
	}

	return 0;
}

static void
do_show_print(void *arg, param_t param)
{
	int32_t i;
	float f;
	const char *search_string = (const char *)arg;
	const char *p_name = (const char *)param_name(param);

	/* print nothing if search string is invalid and not matching */
	if (arg != nullptr) {

		/* start search */
		const char *ss = search_string;
		const char *pp = p_name;

		/* XXX this comparison is only ok for trailing wildcards */
		while (*ss != '\0' && *pp != '\0') {

			// case insensitive comparison (param_name is always upper case)
			if (toupper(*ss) == *pp) {
				ss++;
				pp++;

			} else if (*ss == '*') {
				if (*(ss + 1) != '\0') {
					PX4_WARN("* symbol only allowed at end of search string");
					// FIXME - should exit
					return;
				}

				pp++;

			} else {
				/* param not found */
				return;
			}
		}

		/* the search string must have been consumed */
		if (!(*ss == '\0' || *ss == '*') || *pp != '\0') {
			return;
		}
	}

	PARAM_PRINT("%c %c %s [%d,%d] : ", (param_used(param) ? 'x' : ' '),
		    param_value_unsaved(param) ? '*' : (param_value_is_default(param) ? ' ' : '+'),
		    param_name(param), param_get_used_index(param), param_get_index(param));

	/*
	 * This case can be expanded to handle printing common structure types.
	 */

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		if (!param_get(param, &i)) {
			PARAM_PRINT("%ld\n", (long)i);
			return;
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &f)) {
			PARAM_PRINT("%4.4f\n", (double)f);
			return;
		}

		break;

	default:
		PARAM_PRINT("<unknown type %d>\n", 0 + param_type(param));
		return;
	}

	PARAM_PRINT("<error fetching parameter %lu>\n", (unsigned long)param);
}

static void
do_show_print_for_airframe(void *arg, param_t param)
{
	// exceptions
	const char *p_name = param_name(param);

	if (!p_name || param_is_volatile(param)) {
		return;
	}

	if (!strcmp(p_name, "SYS_AUTOSTART") || !strcmp(p_name, "SYS_AUTOCONFIG")) {
		return;
	}

	if (!strncmp(p_name, "RC", 2) || !strncmp(p_name, "TC_", 3) || !strncmp(p_name, "CAL_", 4) ||
	    !strncmp(p_name, "SENS_BOARD_", 11) || !strcmp(p_name, "SENS_DPRES_OFF") ||
	    !strcmp(p_name, "MAV_TYPE")) {
		return;
	}

	int32_t i;
	float f;
	PARAM_PRINT("param set-default %s ", p_name);

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		if (!param_get(param, &i)) {
			PARAM_PRINT("%ld\n", (long)i);
			return;
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &f)) {
			PARAM_PRINT("%4.4f\n", (double)f);
			return;
		}

		break;

	default:
		return;
	}

	PARAM_PRINT("<error fetching parameter %lu>\n", (unsigned long)param);
}

static int
do_set(const char *name, const char *val, bool fail_on_not_found)
{
	int32_t i;
	float f;
	param_t param = param_find(name);

	/* set nothing if parameter cannot be found */
	if (param == PARAM_INVALID) {
		/* param not found - fail silenty in scripts as it prevents booting */
		PX4_ERR("Parameter %s not found.", name);
		return (fail_on_not_found) ? 1 : 0;
	}

	/*
	 * Set parameter if type is known and conversion from string to value turns out fine
	 */

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		if (!param_get(param, &i)) {

			/* convert string */
			char *end;
			int32_t newval = strtol(val, &end, 10);

			if (i != newval) {
				PARAM_PRINT("%c %s: ",
					    param_value_unsaved(param) ? '*' : (param_value_is_default(param) ? ' ' : '+'),
					    param_name(param));
				PARAM_PRINT("curr: %ld", (long)i);
				param_set(param, &newval);
				PARAM_PRINT(" -> new: %ld\n", (long)newval);
			}
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &f)) {

			/* convert string */
			char *end;
			float newval = strtod(val, &end);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

			if (f != newval) {
#pragma GCC diagnostic pop
				PARAM_PRINT("%c %s: ",
					    param_value_unsaved(param) ? '*' : (param_value_is_default(param) ? ' ' : '+'),
					    param_name(param));
				PARAM_PRINT("curr: %4.4f", (double)f);
				param_set(param, &newval);
				PARAM_PRINT(" -> new: %4.4f\n", (double)newval);
			}

		}

		break;

	default:
		PX4_ERR("<unknown / unsupported type %d>\n", 0 + param_type(param));
		return 1;
	}

	return 0;
}

static int
do_set_custom_default(const char *name, const char *val)
{
	param_t param = param_find_no_notification(name);

	/* set nothing if parameter cannot be found */
	if (param == PARAM_INVALID) {
		/* param not found - fail silenty in scripts as it prevents booting */
		PX4_ERR("Parameter %s not found.", name);
		return PX4_ERROR;
	}

	// Set parameter if type is known and conversion from string to value turns out fine
	switch (param_type(param)) {
	case PARAM_TYPE_INT32: {
			int32_t i;

			if (param_get_default_value(param, &i) == PX4_OK) {
				/* convert string */
				char *end;
				int32_t newval = strtol(val, &end, 10);

				if ((i != newval) && (param_set_default_value(param, &newval) == PX4_OK)) {
					PX4_DEBUG(" parameter default: %s %d -> %d", param_name(param), i, newval);
				}
			}
		}

		break;

	case PARAM_TYPE_FLOAT: {
			float f;

			if (param_get_default_value(param, &f) == PX4_OK) {
				/* convert string */
				char *end;
				float newval = strtod(val, &end);

				if ((fabsf(f - newval) > FLT_EPSILON) && (param_set_default_value(param, &newval) == PX4_OK)) {
					PX4_DEBUG(" parameter default: %s %4.2f -> %4.2f", param_name(param), (double)f, (double)newval);
				}
			}
		}

		break;

	default:
		PX4_ERR("<unknown / unsupported type %d>\n", 0 + param_type(param));
		return 1;
	}

	return 0;
}

static int
do_compare(const char *name, char *vals[], unsigned comparisons, enum COMPARE_OPERATOR cmp_op,
	   enum COMPARE_ERROR_LEVEL err_level)
{
	int32_t i;
	float f;
	param_t param = param_find(name);

	/* set nothing if parameter cannot be found */
	if (param == PARAM_INVALID) {
		/* param not found */
		if (err_level == COMPARE_ERROR_LEVEL::DO_ERROR) {
			PX4_ERR("Parameter %s not found", name);
		}

		return 1;
	}

	/*
	 * Set parameter if type is known and conversion from string to value turns out fine
	 */

	int ret = 1;

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		if (!param_get(param, &i)) {

			/* convert string */
			char *end;

			for (unsigned k = 0; k < comparisons; k++) {

				int j = strtol(vals[k], &end, 10);

				if (((cmp_op == COMPARE_OPERATOR::EQUAL) && (i == j)) ||
				    ((cmp_op == COMPARE_OPERATOR::GREATER) && (i > j))) {
					PX4_DEBUG(" %ld: ", (long)i);
					ret = 0;
				}
			}
		}

		break;

	case PARAM_TYPE_FLOAT:
		if (!param_get(param, &f)) {

			/* convert string */
			char *end;

			for (unsigned k = 0; k < comparisons; k++) {

				float g = strtod(vals[k], &end);

				if (((cmp_op == COMPARE_OPERATOR::EQUAL) && (fabsf(f - g) < 1e-7f)) ||
				    ((cmp_op == COMPARE_OPERATOR::GREATER) && (f > g))) {
					PX4_DEBUG(" %4.4f: ", (double)f);
					ret = 0;
				}
			}
		}

		break;

	default:
		PX4_ERR("<unknown / unsupported type %d>", 0 + param_type(param));
		return 1;
	}

	if (ret == 0) {
		PX4_DEBUG("%c %s: match\n",
			  param_value_unsaved(param) ? '*' : (param_value_is_default(param) ? ' ' : '+'),
			  param_name(param));
	}

	return ret;
}

static int
do_reset_all(const char *excludes[], int num_excludes)
{
	if (num_excludes > 0) {
		param_reset_excludes(excludes, num_excludes);

	} else {
		param_reset_all();
	}

	return 0;
}

static int
do_reset_specific(const char *resets[], int num_resets)
{
	param_reset_specific(resets, num_resets);
	return 0;
}

static int
do_touch(const char *params[], int num_params)
{
	for (int i = 0; i < num_params; ++i) {
		if (param_find(params[i]) == PARAM_INVALID) {
			PX4_ERR("param %s not found", params[i]);
		}
	}

	return 0;
}
