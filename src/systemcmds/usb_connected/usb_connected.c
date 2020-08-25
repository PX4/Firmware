/****************************************************************************
 *
 *   Copyright (c) 2014 Andrew Tridgell. All rights reserved.
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
 * @file usb_connected.c
 *
 * @author Andrew Tridgell
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <board_config.h>

__EXPORT int usb_connected_main(int argc, char *argv[]);

static void print_usage(void)
{

	PRINT_MODULE_DESCRIPTION("Utility to check if USB is connected. Was previously used in startup scripts.\n"
				 "A return value of 0 means USB is connected, 1 otherwise."
				);

	PRINT_MODULE_USAGE_NAME_SIMPLE("usb_connected", "command");
}

int
usb_connected_main(int argc, char *argv[])
{
	if (argc > 1) {
		print_usage();
		return 0;
	}

	return board_read_VBUS_state();
}
