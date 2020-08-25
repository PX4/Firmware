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
 * @file test_uart_baudchange.c
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>

#include "tests_main.h"

#include <math.h>
#include <float.h>

int test_uart_baudchange(int argc, char *argv[])
{
	int uart2_nwrite = 0;

	/* assuming NuttShell is on UART1 (/dev/ttyS0) */
	int uart2 = open("/dev/ttyS2", O_RDWR | O_NONBLOCK | O_NOCTTY); //

	if (uart2 < 0) {
		printf("ERROR opening UART2, aborting..\n");
		return uart2;
	}

	struct termios uart2_config;

	struct termios uart2_config_original;

	int termios_state = 0;

	int ret;

	if ((termios_state = tcgetattr(uart2, &uart2_config)) < 0) {
		printf("ERROR getting termios config for UART2: %d\n", termios_state);
		ret = termios_state;
		goto cleanup;
	}

	if ((termios_state = tcgetattr(uart2, &uart2_config_original)) < 0) {
		printf("ERROR getting termios config for UART2: %d\n", termios_state);
		ret = termios_state;
		goto cleanup;
	}

	/* Set baud rate */
	if (cfsetispeed(&uart2_config, B9600) < 0 || cfsetospeed(&uart2_config, B9600) < 0) {
		printf("ERROR setting termios config for UART2: %d\n", termios_state);
		ret = ERROR;
		goto cleanup;
	}

	if ((termios_state = tcsetattr(uart2, TCSANOW, &uart2_config)) < 0) {
		printf("ERROR setting termios config for UART2\n");
		ret = termios_state;
		goto cleanup;
	}

	/* Set back to original settings */
	if ((termios_state = tcsetattr(uart2, TCSANOW, &uart2_config_original)) < 0) {
		printf("ERROR setting termios config for UART2\n");
		ret = termios_state;
		goto cleanup;
	}

	uint8_t sample_uart2[] = {'U', 'A', 'R', 'T', '2', ' ', '#', 0, '\n'};

	int i, r;

	for (i = 0; i < 100; i++) {
		/* uart2 -> */
		r = write(uart2, sample_uart2, sizeof(sample_uart2));

		if (r > 0) {
			uart2_nwrite += r;
		}
	}

	close(uart2);

	printf("uart2_nwrite %d\n", uart2_nwrite);

	return OK;
cleanup:
	close(uart2);
	return ret;

}
