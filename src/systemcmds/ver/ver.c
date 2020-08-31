/****************************************************************************
*
* Copyright (c) 2014 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
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
* @file ver.c
*
* Version command, unifies way of showing versions of HW, SW, Build, GCC
* In case you want to add new version just extend version_main function
*
* @author Vladimir Kulla <ufon@kullaonline.net>
*/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <version/version.h>

/* string constants for version commands */
static const char sz_ver_hw_str[] 	= "hw";
static const char sz_ver_hwcmp_str[]    = "hwcmp";
static const char sz_ver_hwtypecmp_str[]    = "hwtypecmp";
static const char sz_ver_git_str[] 	= "git";
static const char sz_ver_bdate_str[]    = "bdate";
static const char sz_ver_buri_str[]     = "uri";
static const char sz_ver_gcc_str[] 	= "gcc";
static const char sz_ver_all_str[] 	= "all";
static const char mcu_ver_str[]		= "mcu";
static const char px4_guid_str[]         = "px4guid";

static void usage(const char *reason)
{
	if (reason != NULL) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Tool to print various version information");

	PRINT_MODULE_USAGE_NAME("ver", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("hw", "Hardware architecture");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mcu", "MCU info");
	PRINT_MODULE_USAGE_COMMAND_DESCR("git", "git version information");
	PRINT_MODULE_USAGE_COMMAND_DESCR("bdate", "Build date and time");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gcc", "Compiler info");
	PRINT_MODULE_USAGE_COMMAND_DESCR("bdate", "Build date and time");
	PRINT_MODULE_USAGE_COMMAND_DESCR("px4guid", "PX4 GUID");
	PRINT_MODULE_USAGE_COMMAND_DESCR("uri", "Build URI");

	PRINT_MODULE_USAGE_COMMAND_DESCR("all", "Print all versions");
	PRINT_MODULE_USAGE_COMMAND_DESCR("hwcmp", "Compare hardware version (returns 0 on match)");
	PRINT_MODULE_USAGE_ARG("<hw> [<hw2>]",
			       "Hardware to compare against (eg. PX4_FMU_V4). An OR comparison is used if multiple are specified", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("hwtypecmp", "Compare hardware type (returns 0 on match)");
	PRINT_MODULE_USAGE_ARG("<hwtype> [<hwtype2>]",
			       "Hardware type to compare against (eg. V2). An OR comparison is used if multiple are specified", false);
}

__EXPORT int ver_main(int argc, char *argv[]);

int ver_main(int argc, char *argv[])
{
	/* defaults to an error */
	int ret = 1;

	/* first check if there are at least 2 params */
	if (argc >= 2) {
		if (argv[1] != NULL) {

			if (!strncmp(argv[1], sz_ver_hwcmp_str, sizeof(sz_ver_hwcmp_str))) {
				if (argc >= 3 && argv[2] != NULL) {
					const char *board_name = px4_board_name();

					for (int i = 2; i < argc; ++i) {
						if (strcmp(board_name, argv[i]) == 0) {
							return 0; // if one of the arguments match, return success
						}
					}

				} else {
					PX4_ERR("Not enough arguments, try 'ver hwcmp PX4_FMU_V2'");
				}

				return 1;
			}

			if (!strncmp(argv[1], sz_ver_hwtypecmp_str, sizeof(sz_ver_hwtypecmp_str))) {
				if (argc >= 3 && argv[2] != NULL) {
					const char *board_type = px4_board_sub_type();

					for (int i = 2; i < argc; ++i) {
						if (strcmp(board_type, argv[i]) == 0) {
							return 0; // if one of the arguments match, return success
						}
					}

				} else {
					PX4_ERR("Not enough arguments, try 'ver hwtypecmp {V2|V2M|V30|V31}'");
				}

				return 1;
			}

			/* check if we want to show all */
			bool show_all = !strncmp(argv[1], sz_ver_all_str, sizeof(sz_ver_all_str));

			if (show_all || !strncmp(argv[1], sz_ver_hw_str, sizeof(sz_ver_hw_str))) {
				printf("HW arch: %s\n", px4_board_name());
#if defined(BOARD_HAS_VERSIONING)
				char vb[14] = "NA";
				char rb[14] = "NA";
				int  v = px4_board_hw_version();
				int  r = px4_board_hw_revision();

				if (v >= 0) {
					snprintf(vb, sizeof(vb), "0x%08X", v);
				}

				if (r >= 0) {
					snprintf(rb, sizeof(rb), "0x%08X", r);
				}

				printf("HW type: %s\n", strlen(px4_board_sub_type()) ? px4_board_sub_type() : "NA");
				printf("HW version: %s\n", vb);
				printf("HW revision: %s\n", rb);
#endif
				ret = 0;

			}

			if (show_all || !strncmp(argv[1], sz_ver_git_str, sizeof(sz_ver_git_str))) {
				printf("FW git-hash: %s\n", px4_firmware_version_string());
				unsigned fwver = px4_firmware_version();
				unsigned major = (fwver >> (8 * 3)) & 0xFF;
				unsigned minor = (fwver >> (8 * 2)) & 0xFF;
				unsigned patch = (fwver >> (8 * 1)) & 0xFF;
				unsigned type = (fwver >> (8 * 0)) & 0xFF;

				if (type == 255) {
					printf("FW version: Release %u.%u.%u (%u)\n", major, minor, patch, fwver);

				} else {
					printf("FW version: %u.%u.%u %x (%u)\n", major, minor, patch, type, fwver);
				}

				if (show_all) {
					const char *git_branch = px4_firmware_git_branch();

					if (git_branch && git_branch[0]) {
						printf("FW git-branch: %s\n", git_branch);
					}
				}

				fwver = px4_os_version();
				major = (fwver >> (8 * 3)) & 0xFF;
				minor = (fwver >> (8 * 2)) & 0xFF;
				patch = (fwver >> (8 * 1)) & 0xFF;
				type = (fwver >> (8 * 0)) & 0xFF;
				printf("OS: %s\n", px4_os_name());

				if (type == 255) {
					printf("OS version: Release %u.%u.%u (%u)\n", major, minor, patch, fwver);

				} else {
					printf("OS version: %u.%u.%u %u (%u)\n", major, minor, patch, type, fwver);
				}

				const char *os_git_hash = px4_os_version_string();

				if (os_git_hash) {
					printf("OS git-hash: %s\n", os_git_hash);
				}

				ret = 0;

			}

			if (show_all || !strncmp(argv[1], sz_ver_bdate_str, sizeof(sz_ver_bdate_str))) {
				printf("Build datetime: %s %s\n", __DATE__, __TIME__);
				ret = 0;

			}

			if (show_all || !strncmp(argv[1], sz_ver_buri_str, sizeof(sz_ver_buri_str))) {
				printf("Build uri: %s\n", px4_build_uri());
				ret = 0;

			}


			if (show_all || !strncmp(argv[1], sz_ver_gcc_str, sizeof(sz_ver_gcc_str))) {
				printf("Toolchain: %s, %s\n", px4_toolchain_name(), px4_toolchain_version());
				ret = 0;

			}

			if (show_all || !strncmp(argv[1], px4_guid_str, sizeof(px4_guid_str))) {
				char px4guid_fmt_buffer[PX4_GUID_FORMAT_SIZE];

				board_get_px4_guid_formated(px4guid_fmt_buffer, sizeof(px4guid_fmt_buffer));
				printf("PX4GUID: %s\n", px4guid_fmt_buffer);
				ret = 0;
			}

			if (show_all || !strncmp(argv[1], mcu_ver_str, sizeof(mcu_ver_str))) {

				char rev = ' ';
				const char *revstr = NULL;
				const char *errata = NULL;

				int chip_version = board_mcu_version(&rev, &revstr, &errata);

				if (chip_version < 0) {
					printf("UNKNOWN MCU\n");

				} else {
					printf("MCU: %s, rev. %c\n", revstr, rev);

					if (errata != NULL) {
						printf("\nWARNING   WARNING   WARNING!\n"
						       "Revision %c has a silicon errata:\n"
						       "%s"
						       "\nhttps://docs.px4.io/master/en/flight_controller/silicon_errata.html\n\n", rev, errata);
					}
				}

				ret = 0;
			}

			if (ret == 1) {
				PX4_ERR("unknown command");
				return 1;
			}

		} else {
			usage("Error, input parameter NULL.");
		}

	} else {
		usage("Error, not enough parameters.");
	}

	return ret;
}
