/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Simon Wilks <sjwilks@gmail.com>
 *   Author: @author Kurt Kiefer <kekiefer@gmail.com>
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
 * @file test_spektrum_telemetry.c
 *
 * Tests the Spektrum SRXL telemetry support. Based on HoTT telemetry tests.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <drivers/drv_gpio.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <sys/types.h>
#include <systemlib/err.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include "tests_main.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int open_uart(const char *device)
{
	/* baud rate */
	int speed = B115200;

	/* open uart */
	int uart = open(device, O_RDWR | O_NOCTTY);

	if (uart < 0) {
		PX4_ERR("FAIL: Error opening port");
		return ERROR;
	}

	/* Try to set baud rate */
	struct termios uart_config;

	/* Fill the struct for the new configuration */
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("FAIL: Error setting baudrate / termios config for cfsetispeed, cfsetospeed");
		return ERROR;
	}

	if (tcsetattr(uart, TCSANOW, &uart_config) < 0) {
		PX4_ERR("FAIL: Error setting baudrate / termios config for tcsetattr");
		return ERROR;
	}

	return uart;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_hott_telemetry
 ****************************************************************************/

int test_spektrum_telemetry(int argc, char *argv[])
{
	PX4_INFO("Spektrum Telemetry Test Requirements:");
	PX4_INFO("- Radio on and Electric Air. Mod on (telemetry -> sensor select).");
	PX4_INFO("- Receiver telemetry port must be in telemetry mode.");
	PX4_INFO("- Connect telemetry wire to /dev/ttyS1 (USART2).");
	PX4_INFO("Testing...");

	const char device[] = "/dev/ttyS5";
	int fd = open_uart(device);

	if (fd < 0) {
		close(fd);
		return ERROR;
	}

#ifdef TIOCSSINGLEWIRE
	/* Activate single wire mode */
	ioctl(fd, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);
#endif

	char send = 'a';
	//write(fd, &send, 1);

	/* Since TX and RX are now connected we should be able to read in what we wrote */
	const int timeout = 1000;
	struct pollfd fds[] = { { .fd = fd, .events = POLLIN } };

	if (poll(fds, 1, timeout) == 0) {
		PX4_ERR("FAIL: Could not read sent data.");
		return 1;
	}

	char receive;
	read(fd, &receive, 1);
	PX4_INFO("PASS: Single wire enabled. Sent %x and received %x", send, receive);

	/* Attempt to read HoTT poll messages from the HoTT receiver */
	int received_count = 0;
	int valid_count = 0;
	const int max_polls = 5;
	uint8_t byte;

	for (; received_count < 32; received_count++) {
		if (poll(fds, 1, timeout) == 0) {
			PX4_ERR("FAIL: Could not read sent data. Is your HoTT receiver plugged in on %s?", device);
			return 1;

		} else {
			read(fd, &byte, 1);

			if (byte == 0xb2) {
				valid_count++;
			}

			/* Read the device ID being polled */
			read(fd, &byte, 1);
		}
	}

	if (received_count > 0 && valid_count > 0) {
		if (received_count == max_polls && valid_count == max_polls) {
			PX4_INFO("PASS: Received %d out of %d valid byte pairs from the HoTT receiver device.", received_count, max_polls);

		} else {
			PX4_WARN("WARN: Received %d out of %d byte pairs of which %d were valid from the HoTT receiver device.", received_count,
				 max_polls, valid_count);
		}

	} else {
		/* Let's work out what went wrong */
		if (received_count == 0) {
			PX4_ERR("FAIL: Could not read any polls from HoTT receiver device.");
			return 1;
		}

		if (valid_count == 0) {
			PX4_ERR("FAIL: Received unexpected values from the HoTT receiver device.");
			return 1;
		}
	}


	/* Attempt to send a HoTT response messages */
	uint8_t response[] = {0x7c, 0x8e, 0x00, 0xe0, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, \
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf4, 0x01, 0x00, 0x00, \
			      0x19, 0x00, 0x00, 0x00, 0x30, 0x75, 0x78, 0x00, 0x00, 0x00, \
			      0x00, 0x00, 0x00, 0x7d, 0x12
			     };

	usleep(5000);

	for (unsigned int i = 0; i < sizeof(response); i++) {
		write(fd, &response[i], 1);
		usleep(1000);
	}

	PX4_INFO("PASS: Response sent to the HoTT receiver device. Voltage should now show 2.5V.");


#ifdef TIOCSSINGLEWIRE
	/* Disable single wire */
	ioctl(fd, TIOCSSINGLEWIRE, ~SER_SINGLEWIRE_ENABLED);
#endif

	write(fd, &send, 1);

	/* We should timeout as there will be nothing to read (TX and RX no longer connected) */
	if (poll(fds, 1, timeout) == 0) {
		PX4_ERR("FAIL: timeout expected.");
		return 1;
	}

	PX4_INFO("PASS: Single wire disabled.");

	close(fd);
	return 0;
}
