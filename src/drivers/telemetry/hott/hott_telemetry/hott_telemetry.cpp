/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
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
 * @file hott_telemetry_main.c
 * @author Simon Wilks <sjwilks@gmail.com>
 *
 * Graupner HoTT Telemetry implementation.
 *
 * The HoTT receiver polls each device at a regular interval at which point
 * a data packet can be returned if necessary.
 *
 */

#include <fcntl.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <poll.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>

#include "../comms.h"
#include "../messages.h"

#define DEFAULT_UART "/dev/ttyS6";		/**< Serial4 */

static int thread_should_exit = false;		/**< Deamon exit flag */
static int thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const char daemon_name[] = "hott_telemetry";
static const char commandline_usage[] =
	"usage: hott_telemetry start|status|stop [-d <device>] [-t <timeout ms>] [-r <read delay us>] [-w <write delay us>]";

static uint8_t read_log[16];

static int timeout_ms = POLL_TIMEOUT_IN_MSECS;
static int read_delay_us = POST_READ_DELAY_IN_USECS;
static int write_delay_us = POST_WRITE_DELAY_IN_USECS;

perf_counter_t	reqs_count;
perf_counter_t	connect_count;
perf_counter_t	recon_port;
perf_counter_t	bin_reply;
perf_counter_t	txt_reply;
perf_counter_t	bad_reply;
perf_counter_t	dead_reply;

/**
 * Deamon management function.
 */
extern "C" __EXPORT int hott_telemetry_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int hott_telemetry_thread_main(int argc, char *argv[]);

static int recv_req_id(int uart, uint8_t *id);
static int send_data(int uart, uint8_t *buffer, size_t size);

int
recv_req_id(int uart, uint8_t *id)
{
	uint8_t mode;

	struct pollfd fds;
	fds.fd = uart;
	fds.events = POLLIN;

	if (poll(&fds, 1, timeout_ms) > 0) {
		/* Get the mode: binary or text  */
		read(uart, &mode, sizeof(mode));

		perf_count(reqs_count);

		// Debug log
		for (int x = 15; x > 0; x--) {
			read_log[x] = read_log[x - 1];
		}

		read_log[0] = mode;

		/* if we have a binary mode request */
		if (mode != BINARY_MODE_REQUEST_ID) {
			return PX4_ERROR;
		}

		/* Read the device ID being polled */
		read(uart, id, sizeof(*id));

	} else {
		warnx("UART timeout on TX/RX port");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
send_data(int uart, uint8_t *buffer, size_t size)
{
	usleep(read_delay_us);

	uint16_t checksum = 0;

	for (size_t i = 0; i < size; i++) {
		if (i == size - 1) {
			/* Set the checksum: the first uint8_t is taken as the checksum. */
			buffer[i] = checksum & 0xff;

		} else {
			checksum += buffer[i];
		}

		write(uart, &buffer[i], sizeof(buffer[i]));

		/* Sleep before sending the next byte. */
		usleep(write_delay_us);
	}

	/* A hack the reads out what was written so the next read from the receiver doesn't get it. */
	/* TODO: Fix this!! */
	uint8_t dummy[MAX_MESSAGE_BUFFER_SIZE];
	read(uart, &dummy, size);

	return PX4_OK;
}

int
hott_telemetry_thread_main(int argc, char *argv[])
{
	connect_count = perf_alloc(PC_COUNT,	"reconnects       ");
	recon_port = perf_alloc(PC_COUNT,		"reopen port      ");
	reqs_count = perf_alloc(PC_COUNT,		"requests        ");
	bin_reply = perf_alloc(PC_COUNT,		"bin replies     ");
	txt_reply = perf_alloc(PC_COUNT,		"text replies    ");
	bad_reply = perf_alloc(PC_COUNT,		"unknown replies ");
	dead_reply = perf_alloc(PC_COUNT,		"dead replies    ");

	thread_running = true;

	const char *device = DEFAULT_UART;

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
			if (argc > i + 1) {
				device = argv[i + 1];

			} else {
				thread_running = false;
				errx(1, "missing parameter to -d\n%s", commandline_usage);
			}
		}
	}

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	int uart = open_uart(device);

	if (uart < 0) {
		errx(1, "Failed opening HoTT UART, exiting.");
		thread_running = false;
	}

	init_sub_messages();

	uint8_t buffer[MAX_MESSAGE_BUFFER_SIZE];
	size_t size = 0;
	uint8_t id = 0;

	bool connected = true;
	int recon = 0;

	while (!thread_should_exit) {
		// Listen for and serve poll from the receiver.
		if (recv_req_id(uart, &id) == PX4_OK) {
			if (!connected) {
				connected = true;
			}

			switch (id) {
			case EAM_SENSOR_ID:
				build_eam_response(buffer, &size);
				perf_count(bin_reply);
				break;

			case GAM_SENSOR_ID:
				build_gam_response(buffer, &size);
				perf_count(bin_reply);
				break;

			case GPS_SENSOR_ID:
				build_gps_response(buffer, &size);
				perf_count(bin_reply);
				break;

			case BINARY_MODE_REQUEST_ID:
				perf_count(dead_reply);
				break;

			default:
				perf_count(bad_reply);
				continue;	// Not a module we support.
			}

			send_data(uart, buffer, size);

		} else {
			if (connected) {
				connected = false;

			} else {
				recon++;
			}

			if (recon > 100) {
				perf_count(recon_port);
				close(uart);
				uart = open_uart(device);
				perf_reset(reqs_count);
				perf_reset(bin_reply);
				perf_reset(txt_reply);
				perf_reset(dead_reply);
				perf_reset(bad_reply);
			}
		}
	}

	warnx("exiting");

	close(uart);

	thread_running = false;

	perf_free(connect_count);
	perf_free(recon_port);
	perf_free(reqs_count);
	perf_free(bin_reply);
	perf_free(txt_reply);
	perf_free(bad_reply);
	perf_free(dead_reply);

	return 0;
}

/**
 * Process command line arguments and start the daemon.
 */
int
hott_telemetry_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "missing command\n%s", commandline_usage);
	}

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--timeout") == 0) { //device set
			if (argc > i + 1) {
				timeout_ms = atoi(argv[i + 1]);

			} else {
				errx(1, "missing parameter to -t\n%s", commandline_usage);
			}
		}

		if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--read-delay") == 0) { //device set
			if (argc > i + 1) {
				read_delay_us = atoi(argv[i + 1]);

			} else {
				errx(1, "missing parameter to -r\n%s", commandline_usage);
			}
		}

		if (strcmp(argv[i], "-w") == 0 || strcmp(argv[i], "--write-delay") == 0) { //device set
			if (argc > i + 1) {
				write_delay_us = atoi(argv[i + 1]);

			} else {
				errx(1, "missing parameter to -w\n%s", commandline_usage);
			}
		}
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd(daemon_name,
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 1500,
						 hott_telemetry_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");


			for (int x = 15; x >= 0; x--) {
				printf("%2x ", read_log[x]);
			}

			printf("\npoll timeout     : %i ms\n", timeout_ms);
			printf("post write delay : %i us\n", write_delay_us);
			printf("post read delay  : %i us\n", read_delay_us);
			perf_print_counter(recon_port);
			perf_print_counter(connect_count);
			perf_print_counter(reqs_count);
			perf_print_counter(bin_reply);
			perf_print_counter(txt_reply);
			perf_print_counter(bad_reply);
			perf_print_counter(dead_reply);

		} else {
			warnx("not started");
		}

		exit(0);
	}

	errx(1, "unrecognized command\n%s", commandline_usage);
}
