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
 * @file test_uart_send.c
 * Tests the uart send functionality.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "tests_main.h"

#include <math.h>
#include <float.h>
#include <drivers/drv_hrt.h>


int test_uart_send(int argc, char *argv[])
{
	/* input handling */
	char *uart_name = "/dev/ttyS3";

	if (argc > 1) { uart_name = argv[1]; }

	/* assuming NuttShell is on UART1 (/dev/ttyS0) */
	int test_uart = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY); //

	if (test_uart < 0) {
		printf("ERROR opening UART %s, aborting..\n", uart_name);
		return test_uart;

	} else {
		printf("Writing to UART %s\n", uart_name);
	}

	char sample_test_uart[25];// = {'S', 'A', 'M', 'P', 'L', 'E', ' ', '\n'};

	int i, n;

	uint64_t start_time = hrt_absolute_time();

	for (i = 0; i < 30000; i++) {
		n = sprintf(sample_test_uart, "SAMPLE #%d\n", i);
		write(test_uart, sample_test_uart, n);
	}

	int interval = hrt_absolute_time() - start_time;

	int bytes = i * sizeof(sample_test_uart);

	printf("Wrote %d bytes in %d ms on UART %s\n", bytes, interval / 1000, uart_name);

	close(test_uart);

	return 0;
}
