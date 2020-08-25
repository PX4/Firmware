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
 * @file test_led.c
 * Tests for led functionality.
 */

#include <px4_platform_common/time.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <drivers/drv_board_led.h>

#include "tests_main.h"


int test_led(int argc, char *argv[])
{
	int		fd;
	int		ret = 0;

	fd = px4_open(LED0_DEVICE_PATH, 0);

	if (fd < 0) {
		printf("\tLED: open fail\n");
		return ERROR;
	}

	if (px4_ioctl(fd, LED_ON, LED_BLUE) ||
	    px4_ioctl(fd, LED_ON, LED_AMBER)) {

		printf("\tLED: ioctl fail\n");
		return ERROR;
	}

	/* let them blink for fun */

	int i;
	uint8_t ledon = 1;

	for (i = 0; i < 10; i++) {
		if (ledon) {
			px4_ioctl(fd, LED_ON, LED_BLUE);
			px4_ioctl(fd, LED_OFF, LED_AMBER);

		} else {
			px4_ioctl(fd, LED_OFF, LED_BLUE);
			px4_ioctl(fd, LED_ON, LED_AMBER);
		}

		ledon = !ledon;
		px4_usleep(60000);
	}

	/* Go back to default */
	px4_ioctl(fd, LED_ON, LED_BLUE);
	px4_ioctl(fd, LED_OFF, LED_AMBER);

	printf("\t LED test completed, no errors.\n");

	return ret;
}
