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
 * @file tests_main.c
 * Tests main file, loads individual tests.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include "tests_main.h"

#include <px4_platform_common/px4_config.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>


static int test_help(int argc, char *argv[]);
static int test_runner(unsigned option);

static int test_all(int argc, char *argv[]);
static int test_jig(int argc, char *argv[]);

const struct {
	const char 	*name;
	int	(* fn)(int argc, char *argv[]);
	unsigned	options;
#define OPT_NOHELP	(1<<0)
#define OPT_NOALLTEST	(1<<1)
#define OPT_NOJIGTEST	(1<<2)
} tests[] = {
	{"help",		test_help,		OPT_NOALLTEST | OPT_NOHELP | OPT_NOJIGTEST},
	{"all",			test_all,		OPT_NOALLTEST | OPT_NOJIGTEST},
	{"jig",			test_jig,		OPT_NOJIGTEST | OPT_NOALLTEST},

#ifdef __PX4_NUTTX
	{"adc",			test_adc,		OPT_NOJIGTEST},
	{"file",		test_file,		OPT_NOJIGTEST | OPT_NOALLTEST},
	{"led",			test_led,		0},
	{"time",		test_time,		OPT_NOJIGTEST},
	{"uart_baudchange",	test_uart_baudchange,	OPT_NOJIGTEST},
	{"uart_break",		test_uart_break,	OPT_NOJIGTEST | OPT_NOALLTEST},
	{"uart_console",	test_uart_console,	OPT_NOJIGTEST | OPT_NOALLTEST},
#else
	{"rc",			rc_tests_main,		0},
#endif /* __PX4_NUTTX */

	{"atomic_bitset",	test_atomic_bitset,	0},
	{"autodeclination",	test_autodeclination,	0},
	{"bezier",		test_bezierQuad,	0},
	{"bitset",		test_bitset,		0},
	{"bson",		test_bson,		0},
	{"conv",		test_conv,		0},
	{"dataman",		test_dataman,		OPT_NOJIGTEST | OPT_NOALLTEST},
	{"file2",		test_file2,		OPT_NOJIGTEST},
	{"float",		test_float,		0},
	{"hott_telemetry",	test_hott_telemetry,	OPT_NOJIGTEST | OPT_NOALLTEST},
	{"hrt",			test_hrt,		OPT_NOJIGTEST | OPT_NOALLTEST},
	{"int",			test_int,		0},
	{"i2c_spi_cli",		test_i2c_spi_cli,		0},
	{"IntrusiveQueue",	test_IntrusiveQueue,	0},
	{"jig_voltages",	test_jig_voltages,	OPT_NOALLTEST},
	{"List",		test_List,		0},
	{"mathlib",		test_mathlib,		0},
	{"matrix",		test_matrix,		0},
	{"microbench_hrt",	test_microbench_hrt,	0},
	{"microbench_math",	test_microbench_math,	0},
	{"microbench_matrix",	test_microbench_matrix,	0},
	{"microbench_uorb",	test_microbench_uorb,	0},
	{"mixer",		test_mixer,		OPT_NOJIGTEST},
	{"mixer",		test_mixer,		OPT_NOJIGTEST},
	{"mount",		test_mount,		OPT_NOJIGTEST | OPT_NOALLTEST},
	{"param",		test_param,		0},
	{"parameters",		test_parameters,	0},
	{"perf",		test_perf,		OPT_NOJIGTEST},
	{"ppm",			test_ppm,		OPT_NOJIGTEST | OPT_NOALLTEST},
	{"ppm_loopback",	test_ppm_loopback,	OPT_NOALLTEST},
	{"rc",			test_rc,		OPT_NOJIGTEST | OPT_NOALLTEST},
	{"search_min",		test_search_min,	0},
	{"servo",		test_servo,		OPT_NOJIGTEST | OPT_NOALLTEST},
	{"sleep",		test_sleep,		OPT_NOJIGTEST},
	{"smoothz", 		test_smooth_z,		0},
	{"tone",		test_tone,		0},
	{"uart_loopback",	test_uart_loopback,	OPT_NOJIGTEST | OPT_NOALLTEST},
	{"uart_send",		test_uart_send,		OPT_NOJIGTEST | OPT_NOALLTEST},
	{"versioning",		test_versioning,	0},


	/* external tests */
	{"commander",		commander_tests_main,	0},
	{"controllib",		controllib_test_main,	0},
	{"mavlink",		mavlink_tests_main,	0},
#ifdef __PX4_NUTTX
	{"sf0x",		sf0x_tests_main,	0},
#endif
	{"uorb",		uorb_tests_main,	0},

	{NULL,			NULL, 		0}
};

#define NTESTS (sizeof(tests) / sizeof(tests[0]))

static int
test_help(int argc, char *argv[])
{
	unsigned	i;

	printf("Available tests:\n");

	for (i = 0; tests[i].name; i++) {
		printf("  %s\n", tests[i].name);
	}

	return 0;
}

static int
test_all(int argc, char *argv[])
{
	return test_runner(OPT_NOALLTEST);
}

static int
test_jig(int argc, char *argv[])
{
	return test_runner(OPT_NOJIGTEST);
}

static int
test_runner(unsigned option)
{
	size_t i;
	char *args[2] = {"all", NULL};
	unsigned int failcount = 0;
	unsigned int testcount = 0;
	unsigned int passed[NTESTS];

	printf("\nRunning all tests...\n\n");

	for (i = 0; tests[i].name; i++) {
		// Only run tests that are not excluded.
		if (!(tests[i].options & option)) {
			for (int j = 0; j < 80; j++) {
				printf("-");
			}

			printf("\n  [%s] \t\tSTARTING TEST\n", tests[i].name);
			fflush(stdout);

			/* Execute test */
			if (tests[i].fn(1, args) != 0) {
				fprintf(stderr, "  [%s] \t\tFAIL\n", tests[i].name);
				fflush(stderr);
				failcount++;
				passed[i] = 0;

			} else {
				printf("  [%s] \t\tPASS\n", tests[i].name);
				fflush(stdout);
				passed[i] = 1;
			}

			for (int j = 0; j < 80; j++) {
				printf("-");
			}

			testcount++;
			printf("\n\n");
		}
	}

	// Print summary.
	printf("\n");

	for (size_t j = 0; j < 80; j++) {
		printf("#");
	}

	printf("\n\n     T E S T    S U M M A R Y\n\n");

	if (failcount == 0) {
		printf("  ______     __         __            ______     __  __    \n");
		printf(" /\\  __ \\   /\\ \\       /\\ \\          /\\  __ \\   /\\ \\/ /    \n");
		printf(" \\ \\  __ \\  \\ \\ \\____  \\ \\ \\____     \\ \\ \\/\\ \\  \\ \\  _\"-.  \n");
		printf("  \\ \\_\\ \\_\\  \\ \\_____\\  \\ \\_____\\     \\ \\_____\\  \\ \\_\\ \\_\\ \n");
		printf("   \\/_/\\/_/   \\/_____/   \\/_____/      \\/_____/   \\/_/\\/_/ \n");
		printf("\n");
		printf(" All tests passed (%d of %d)\n\n", testcount, testcount);

	} else {
		printf("  ______   ______     __     __ \n");
		printf(" /\\  ___\\ /\\  __ \\   /\\ \\   /\\ \\    \n");
		printf(" \\ \\  __\\ \\ \\  __ \\  \\ \\ \\  \\ \\ \\__\n");
		printf("  \\ \\_\\    \\ \\_\\ \\_\\  \\ \\_\\  \\ \\_____\\ \n");
		printf("   \\/_/     \\/_/\\/_/   \\/_/   \\/_____/ \n");
		printf("\n");
		printf(" Some tests failed (%d of %d)\n\n", failcount, testcount);
	}

	// Print failed tests.
	if (failcount > 0) {
		printf(" Failed tests:\n\n");
	}

	for (size_t k = 0; k < i; k++) {
		if (!passed[k] && !(tests[k].options & option)) {
			printf(" [%s] to obtain details, please re-run with\n\t nsh> tests %s\n\n", tests[k].name, tests[k].name);
		}
	}

	fflush(stdout);
	return (failcount > 0);
}

__EXPORT int tests_main(int argc, char *argv[]);

/**
 * Executes system tests.
 */
int tests_main(int argc, char *argv[])
{
	if (argc < 2) {
		printf("tests: missing test name - 'tests help' for a list of tests\n");
		return 1;
	}

	int tone_test_index = -1;
	char *tone_test = {"tone"};
	char *tone_fail[2] = {NULL, "2"};
	char *tone_pass[2] = {NULL, "14"};

	// Identify the tone test index for later use.
	for (size_t i = 0; tests[i].name; i++) {
		if (*tone_test == *tests[i].name) {
			tone_test_index = i;
		}
	}

	for (size_t i = 0; tests[i].name; i++) {
		if (!strcmp(tests[i].name, argv[1])) {
			if (tests[i].fn(argc - 1, argv + 1) == 0) {
				if (tone_test_index != -1) {
					tests[tone_test_index].fn(2, tone_pass); // Play a notification.
				}

				printf("%s PASSED\n", tests[i].name);
				return 0;

			} else {
				if (tone_test_index != -1) {
					tests[tone_test_index].fn(2, tone_fail); // Play an error notification.
				}

				printf("%s FAILED\n", tests[i].name);
				return -1;
			}
		}
	}

	printf("tests: no test called '%s' - 'tests help' for a list of tests\n", argv[1]);
	return 1;
}
