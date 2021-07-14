/****************************************************************************
 *
 * Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * @file module.cpp
 * Implementation of the API declared in px4_module.h.
 */

#ifndef MODULE_NAME
#define MODULE_NAME "module"
#endif

#include <px4_platform_common/module.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

pthread_mutex_t px4_modules_mutex = PTHREAD_MUTEX_INITIALIZER;

#ifndef __PX4_NUTTX

void PRINT_MODULE_DESCRIPTION(const char *description)
{
	// TODO: the output could be improved by:
	// - mark titles in bold (lines starting with ##)
	// - highlight commands (lines starting with $, or `cmd`)
	PX4_INFO_RAW("%s\n\n", description);
}

#endif /* __PX4_NUTTX */

void PRINT_MODULE_USAGE_NAME(const char *executable_name, const char *category)
{
#ifndef CONSTRAINED_FLASH_NO_HELP
	PX4_INFO_RAW("Usage: %s <command> [arguments...]\n", executable_name);
	PX4_INFO_RAW(" Commands:\n");
#else
	PX4_INFO_RAW("Usage: %s See:" CONSTRAINED_FLASH_NO_HELP "\n", executable_name);
#endif
}

void PRINT_MODULE_USAGE_SUBCATEGORY(const char *subcategory)
{
	(void)subcategory;
}

void PRINT_MODULE_USAGE_NAME_SIMPLE(const char *executable_name, const char *category)
{
#ifndef CONSTRAINED_FLASH_NO_HELP
	PX4_INFO_RAW("Usage: %s [arguments...]\n", executable_name);
#endif
}

void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *name, const char *description)
{
#ifndef CONSTRAINED_FLASH_NO_HELP

	if (description) {
		PX4_INFO_RAW("\n   %-13s %s\n", name, description);

	} else {
		PX4_INFO_RAW("\n   %s\n", name);
	}

#endif
}

void PRINT_MODULE_USAGE_PARAM_COMMENT(const char *comment)
{
#ifndef CONSTRAINED_FLASH_NO_HELP
	PX4_INFO_RAW("\n %s\n", comment);
#endif
}

void PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(bool i2c_support, bool spi_support)
{
#ifndef CONSTRAINED_FLASH_NO_HELP

	// Note: this must be kept in sync with Tools/px4moduledoc/srcparser.py
	if (i2c_support) {
		PRINT_MODULE_USAGE_PARAM_FLAG('I', "Internal I2C bus(es)", true);
		PRINT_MODULE_USAGE_PARAM_FLAG('X', "External I2C bus(es)", true);
	}

	if (spi_support) {
		PRINT_MODULE_USAGE_PARAM_FLAG('s', "Internal SPI bus(es)", true);
		PRINT_MODULE_USAGE_PARAM_FLAG('S', "External SPI bus", true);
	}

	PRINT_MODULE_USAGE_PARAM_INT('b', -1, 0, 16, "board-specific bus (default=all) (external SPI: n-th bus (default=1))",
				     true);

	if (spi_support) {
		PRINT_MODULE_USAGE_PARAM_INT('c', 1, 1, 10, "chip-select index (for external SPI)", true);
		PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 3, "SPI mode", true);
	}

	PRINT_MODULE_USAGE_PARAM_INT('f', -1, 0, 100000, "bus frequency in kHz", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('q', "quiet startup (no message if no device found)", true);
#endif
}

void PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(uint8_t default_address)
{
#ifndef CONSTRAINED_FLASH_NO_HELP
	PRINT_MODULE_USAGE_PARAM_INT('a', default_address, 0, 0xff, "I2C address", true);
#endif
}

void PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG()
{
#ifndef CONSTRAINED_FLASH_NO_HELP
	PRINT_MODULE_USAGE_PARAM_FLAG('k', "if initialization (probing) fails, keep retrying periodically", true);
#endif
}

void PRINT_MODULE_USAGE_PARAM_INT(char option_char, int default_val, int min_val, int max_val,
				  const char *description, bool is_optional)
{
#ifndef CONSTRAINED_FLASH_NO_HELP

	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

		if (default_val != -1) {
			PX4_INFO_RAW("                 default: %i\n", default_val);
		}

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}

#endif
}

void PRINT_MODULE_USAGE_PARAM_FLOAT(char option_char, float default_val, float min_val, float max_val,
				    const char *description, bool is_optional)
{
#ifndef CONSTRAINED_FLASH_NO_HELP

	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

		if (PX4_ISFINITE(default_val)) {
			PX4_INFO_RAW("                 default: %.1f\n", (double)default_val);
		}

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}

#endif
}

void PRINT_MODULE_USAGE_PARAM_FLAG(char option_char, const char *description, bool is_optional)
{
#ifndef CONSTRAINED_FLASH_NO_HELP

	if (is_optional) {
		PX4_INFO_RAW("     [-%c]        %s\n", option_char, description);

	} else {
		PX4_INFO_RAW("     -%c          %s\n", option_char, description);
	}

#endif
}

void PRINT_MODULE_USAGE_PARAM_STRING(char option_char, const char *default_val, const char *values,
				     const char *description, bool is_optional)
{
#ifndef CONSTRAINED_FLASH_NO_HELP

	if (is_optional) {
		PX4_INFO_RAW("     [-%c <val>]  %s\n", option_char, description);

	} else {
		PX4_INFO_RAW("     -%c <val>    %s\n", option_char, description);
	}

	if (values) {
		if (default_val) {
			PX4_INFO_RAW("                 values: %s, default: %s\n", values, default_val);

		} else {
			PX4_INFO_RAW("                 values: %s\n", values);
		}

	} else {
		if (default_val) {
			PX4_INFO_RAW("                 default: %s\n", default_val);
		}
	}

#endif
}


void PRINT_MODULE_USAGE_ARG(const char *values, const char *description, bool is_optional)
{
#ifndef CONSTRAINED_FLASH_NO_HELP

	if (is_optional) {
		PX4_INFO_RAW("     [%-9s] %s\n", values, description);

	} else {
		PX4_INFO_RAW("     %-11s %s\n", values, description);
	}

#endif
}

