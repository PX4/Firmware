/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "version.h"

#include "build_git_version.h"

#include <string.h>

#if !defined(CONFIG_CDCACM_PRODUCTID)
# define CONFIG_CDCACM_PRODUCTID 0
#endif

#if defined(__PX4_LINUX)
#include <sys/utsname.h>
#endif

// dev >= 0
// alpha >= 64
// beta >= 128
// release candidate >= 192
// release == 255
enum FIRMWARE_TYPE {
	FIRMWARE_TYPE_DEV = 0,
	FIRMWARE_TYPE_ALPHA = 64,
	FIRMWARE_TYPE_BETA = 128,
	FIRMWARE_TYPE_RC = 192,
	FIRMWARE_TYPE_RELEASE = 255
};

const char *px4_build_uri(void)
{
	return STRINGIFY(BUILD_URI);
}

uint32_t version_tag_to_number(const char *tag)
{
	uint32_t version_number = 0;

	int16_t buffer = -1;
	size_t buffer_counter = 0;
	size_t dash_count = 0;
	size_t point_count = 0;
	char version[3] = {0, 0, 0};
	int firmware_type = FIRMWARE_TYPE_RELEASE;

	for (size_t i = 0; i < strlen(tag); i++) {
		if (tag[i] == '-') {
			dash_count++;

		} else if (tag[i] == '.') {
			point_count++;
		}

		if (tag[i] == 'r' && i < strlen(tag) - 1 && tag[i + 1] == 'c') {
			firmware_type = FIRMWARE_TYPE_RC;

		} else if (tag[i] == 'p') {
			firmware_type = FIRMWARE_TYPE_ALPHA;

		} else if (tag[i] == 't' && i < strlen(tag) - 1 && tag[i + 1] == 'y') {
			firmware_type = FIRMWARE_TYPE_DEV;

		} else if (tag[i] == 't') {
			firmware_type = FIRMWARE_TYPE_BETA;

		} else if (tag[i] == 'v' && i > 0) {
			firmware_type = FIRMWARE_TYPE_DEV;
		}
	}

	if ((dash_count == 1 && point_count == 2 && firmware_type == FIRMWARE_TYPE_RELEASE) ||
	    (dash_count == 2 && point_count == 2) ||
	    (dash_count == 3 && point_count == 4) ||
	    (dash_count == 4 && point_count == 4)) {
		firmware_type = FIRMWARE_TYPE_DEV;
	}

	for (size_t i = 0; i < strlen(tag); i++) {
		if (buffer_counter > 2) {
			continue;
		}

		if (tag[i] >= '0' && tag[i] <= '9') {
			buffer = (buffer == -1) ? 0 : buffer;
			buffer = buffer * 10 + (tag[i] - '0');

		} else {
			if (buffer >= 0) {
				version[buffer_counter] = buffer;
				buffer_counter++;
			}

			buffer = -1;
		}
	}

	if (buffer >= 0) {
		version[buffer_counter] = buffer;
		buffer_counter++;
	}

	if (buffer_counter <= 0) {
		firmware_type = 0x00;
	}

	if (buffer_counter == 3 || buffer_counter == 6) {
		version_number = ((uint8_t)version[0] << 8 * 3) |
				 ((uint8_t)version[1] << 8 * 2) |
				 ((uint8_t)version[2] << 8 * 1) | firmware_type;

	} else {
		version_number = 0;
	}

	return version_number;
}

uint32_t px4_firmware_version(void)
{
	return version_tag_to_number(PX4_GIT_TAG_STR);
}

uint32_t version_tag_to_vendor_version_number(const char *tag)
{
	uint32_t version_number = 0;

	int16_t buffer = -1;
	size_t buffer_counter = 0;
	char version[6] = {0, 0, 0, 0, 0, 0};
	size_t dash_count = 0;
	size_t point_count = 0;
	int firmware_type = FIRMWARE_TYPE_RELEASE;

	for (size_t i = 0; i < strlen(tag); i++) {
		if (tag[i] == '-') {
			dash_count++;

		} else if (tag[i] == '.') {
			point_count++;
		}

		if (tag[i] == 'r' && i < strlen(tag) - 1 && tag[i + 1] == 'c') {
			firmware_type = FIRMWARE_TYPE_RC;

		} else if (tag[i] == 'p') {
			firmware_type = FIRMWARE_TYPE_ALPHA;

		} else if (tag[i] == 't' && i < strlen(tag) - 1 && tag[i + 1] == 'y') {
			firmware_type = FIRMWARE_TYPE_DEV;

		} else if (tag[i] == 't') {
			firmware_type = FIRMWARE_TYPE_BETA;

		} else if (tag[i] == 'v' && i > 0) {
			firmware_type = FIRMWARE_TYPE_DEV;
		}
	}

	if ((dash_count == 1 && point_count == 2 && firmware_type == FIRMWARE_TYPE_RELEASE) ||
	    (dash_count == 2 && point_count == 2) ||
	    (dash_count == 3 && point_count == 4) ||
	    (dash_count == 4 && point_count == 4)) {
		firmware_type = FIRMWARE_TYPE_DEV;
	}

	for (size_t i = 0; i < strlen(tag); i++) {
		if (buffer_counter > 5) {
			continue;
		}

		if (tag[i] >= '0' && tag[i] <= '9') {
			buffer = (buffer == -1) ? 0 : buffer;
			buffer = buffer * 10 + (tag[i] - '0');

		} else {
			if (buffer >= 0) {
				if (buffer_counter + 1 == 4 && tag[i] == '-') {
					break;
				}

				version[buffer_counter] = buffer;
				buffer_counter++;
			}

			buffer = -1;
		}
	}

	if (buffer >= 0 && (buffer_counter + 1 == 3 || buffer_counter + 1 == 6)) {
		version[buffer_counter] = buffer;
		buffer_counter++;
	}

	if (buffer_counter == 6) {
		version_number = ((uint8_t)version[3] << 8 * 3) |
				 ((uint8_t)version[4] << 8 * 2) |
				 ((uint8_t)version[5] << 8 * 1) | firmware_type;

	} else if (buffer_counter == 3) {
		version_number = firmware_type;

	} else {
		version_number = 0;
	}

	return version_number;
}

uint32_t px4_firmware_vendor_version(void)
{
	return version_tag_to_vendor_version_number(PX4_GIT_TAG_STR);
}

const char *px4_firmware_git_branch(void)
{
	return PX4_GIT_BRANCH_NAME;
}

uint32_t px4_board_version(void)
{
#if defined(__PX4_NUTTX)
	return CONFIG_CDCACM_PRODUCTID;
#else
	return 1;
#endif
}

uint32_t px4_os_version(void)
{
#if defined(__PX4_DARWIN) || defined(__PX4_CYGWIN) || defined(__PX4_QURT)
	return 0; //TODO: implement version for Darwin, Cygwin, QuRT
#elif defined(__PX4_LINUX)
	struct utsname name;

	if (uname(&name) == 0) {
		char *c = name.release;

		// cut the part after the first '-'
		while (*c && *c != '-') {
			++c;
		}

		*c = 0;
		return version_tag_to_number(name.release);

	} else {
		return 0;
	}

#elif defined(__PX4_NUTTX)
	return version_tag_to_number(NUTTX_GIT_TAG_STR);
#else
# error "px4_os_version not implemented for current OS"
#endif
}

const char *px4_os_version_string(void)
{
#if defined(__PX4_NUTTX)
	return NUTTX_GIT_VERSION_STR;
#else
	return NULL;
#endif
}

const char *px4_os_name(void)
{
#if defined(__PX4_DARWIN)
	return "Darwin";
#elif defined(__PX4_LINUX)
	return "Linux";
#elif defined(__PX4_QURT)
	return "QuRT";
#elif defined(__PX4_NUTTX)
	return "NuttX";
#elif defined(__PX4_CYGWIN)
	return "Cygwin";
#else
# error "px4_os_name not implemented for current OS"
#endif
}

const char *px4_toolchain_name(void)
{
#if defined(__clang__)
	return "Clang/LLVM";
#elif defined(__ICC) || defined(__INTEL_COMPILER)
	return "Intel ICC";
#elif defined(__GNUC__) || defined(__GNUG__)
	return "GNU GCC";
#elif defined(_MSC_VER)
	return "MS Visual Studio";
#else
	return "Unknown";
#endif
}

const char *px4_toolchain_version(void)
{
#ifdef __VERSION__
	return __VERSION__;
#else
	return "";
#endif
}

const char *px4_firmware_version_string(void)
{
	return PX4_GIT_VERSION_STR;
}

uint64_t px4_firmware_version_binary(void)
{
	return PX4_GIT_VERSION_BINARY;
}

const char *px4_ecl_lib_version_string(void)
{
#ifdef ECL_LIB_GIT_VERSION_STRING
	return ECL_LIB_GIT_VERSION_STRING;
#else
	return NULL;
#endif
}

#ifdef MAVLINK_LIB_GIT_VERSION_BINARY
uint64_t px4_mavlink_lib_version_binary(void)
{
	return MAVLINK_LIB_GIT_VERSION_BINARY;
}
#endif /* MAVLINK_LIB_GIT_VERSION_BINARY */

uint64_t px4_os_version_binary(void)
{
#ifdef NUTTX_GIT_VERSION_BINARY
	return NUTTX_GIT_VERSION_BINARY;
#else
	return 0;
#endif
}

const char *px4_firmware_oem_version_string(void)
{
	return PX4_GIT_OEM_VERSION_STR;
}

