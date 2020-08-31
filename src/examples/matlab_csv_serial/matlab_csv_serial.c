/****************************************************************************
 *
 *   Copyright (c) 2014-2015 PX4 Development Team. All rights reserved.
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
 * @file matlab_csv_serial_main.c
 *
 * Matlab CSV / ASCII format interface at 921600 baud, 8 data bits,
 * 1 stop bit, no parity
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <px4_platform_common/px4_config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <poll.h>

__EXPORT int matlab_csv_serial_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int matlab_csv_serial_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int matlab_csv_serial_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("matlab_csv_serial",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 matlab_csv_serial_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("running");

		} else {
			warnx("stopped");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int matlab_csv_serial_thread_main(int argc, char *argv[])
{

	if (argc < 2) {
		errx(1, "need a serial port name as argument");
	}

	const char *uart_name = argv[1];

	warnx("opening port %s", uart_name);

	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

	unsigned speed = 921600;

	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(serial_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			close(serial_fd);
			return -1;
		}

	}

	if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", uart_name);
		close(serial_fd);
		return -1;
	}

	/* subscribe to vehicle status, attitude, sensors and flow*/
	struct sensor_accel_s accel0;
	struct sensor_accel_s accel1;
	struct sensor_gyro_s gyro0;
	struct sensor_gyro_s gyro1;

	/* subscribe to parameter changes */
	int accel0_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	int accel1_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 1);
	int gyro0_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 0);
	int gyro1_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 1);

	thread_running = true;

	while (!thread_should_exit) {

		/*This runs at the rate of the sensors */
		struct pollfd fds[] = {
			{ .fd = accel0_sub, .events = POLLIN }
		};

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 500);

		if (ret < 0) {
			/* poll error, ignore */

		} else if (ret == 0) {
			/* no return value, ignore */
			warnx("no sensor data");

		} else {

			/* accel0 update available? */
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_accel), accel0_sub, &accel0);
				orb_copy(ORB_ID(sensor_accel), accel1_sub, &accel1);
				orb_copy(ORB_ID(sensor_gyro), gyro0_sub, &gyro0);
				orb_copy(ORB_ID(sensor_gyro), gyro1_sub, &gyro1);

				// write out on accel 0, but collect for all other sensors as they have updates
				dprintf(serial_fd, "%llu,%d,%d,%d,%d,%d,%d\n", accel0.timestamp, (int)accel0.x_raw, (int)accel0.y_raw,
					(int)accel0.z_raw,
					(int)accel1.x_raw, (int)accel1.y_raw, (int)accel1.z_raw);
			}

		}
	}

	warnx("exiting");
	thread_running = false;

	fflush(stdout);
	return 0;
}


