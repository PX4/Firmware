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
 * @file test_uart_break.c
 * Tests the uart console.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author David Sidrane <david_s5@nscdg.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

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

static void *receive_loop(void *arg)
{
	int uart_usb = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY);

	while (1) {
		char c;
		read(uart_usb, &c, 1);
		printf("%c", c);
		fflush(stdout);
	}

	return NULL;
}

int test_uart_console(int argc, char *argv[])
{
	/* assuming NuttShell is on UART1 (/dev/ttyS0) */
	int uart_usb = open("/dev/ttyACM0", O_WRONLY | O_NOCTTY);

	if (uart_usb < 0) {
		printf("ERROR opening /dev/ttyACM0. Do you need to run sercon first? Aborting..\n");
		return uart_usb;
	}

	uint8_t sample_uart_usb[] = {'S', 'A', 'M', 'P', 'L', 'E', ' ', '\n'};

	pthread_t receive_thread;

	pthread_create(&receive_thread, NULL, receive_loop, NULL);

	//wait for threads to complete:
	pthread_join(receive_thread, NULL);

	for (int i = 0; i < 30; i++) {
		write(uart_usb, sample_uart_usb, sizeof(sample_uart_usb));
		printf(".");
		fflush(stdout);
		px4_sleep(1);
	}

//	uint64_t start_time = hrt_absolute_time();
//
////	while (true)
//	for (int i = 0; i < 1000; i++)
//	{
//		//write(uart_usb, sample_uart_usb, sizeof(sample_uart_usb));
//		int nread = 0;
//		char c;
//		do {
//			nread = read(uart_usb, &c, 1);
//			if (nread > 0)
//			{
//				printf("%c", c);
//			}
//		} while (nread > 0);
//
//		do {
//			nread = read(uart_console, &c, 1);
//			if (nread > 0)
//			{
//				if (c == 0x03)
//				{
//					close(uart_usb);
//					close(uart_console);
//					exit(OK);
//				}
//				else
//				{
//					write(uart_usb, &c, 1);
//				}
//			}
//		} while (nread > 0);
//		usleep(10000);
//	}
//
//	int interval = hrt_absolute_time() - start_time;

	close(uart_usb);
//	close(uart_console);

	return 0;
}
