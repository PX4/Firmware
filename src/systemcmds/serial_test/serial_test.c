/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 *   This work based on the original work of Cliff Brake
 *   cbrake@bec-systems.com released under the MIT license below.
 *   See https://github.com/cbrake/linux-serial-test
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

// SPDX-License-Identifier: MIT
/*
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:

 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/*
 * To use this test
 * 1) Build the https://github.com/cbrake/linux-serial-test on
 *    a linux box.
 *
 * 2) Add serial_test to the DUT board's default.cmake
 * 3) Connect an FTDI cable from the Linux box to the DUT's Telem port.
 *
 * 4) run ./linux-serial-test -e -b 921600 -p /dev/serial/by-id/usb-FTDI_... -c
 * on the linux box (Use control C to exit)
 *
 * 5) Run  serial_test -e -b 921600 -p /dev/ttyS2 -c on DUT
 *    Use ESC or control C to exit'
 *
 */

#if defined(__PX4_NUTTX)
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#endif

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <poll.h>
#include <getopt.h>
#include <time.h>
#if defined(__PX4_NUTTX)
#include <nuttx/serial/serial.h>
#else
#include <linux/serial.h>
#endif
#include <errno.h>

/*
 * glibc for MIPS has its own bits/termios.h which does not define
 * CMSPAR, so we vampirise the value from the generic bits/termios.h
 */
#ifndef CMSPAR
#define CMSPAR 010000000000
#endif

// command line args
struct cli_args_t {
	int          _baud;
	char        *_port;
	int          _divisor;
	int          _rx_dump;
	int          _rx_dump_ascii;
	int          _tx_detailed;
	int          _stats;
	int          _stop_on_error;
	int          _single_byte;
	int          _another_byte;
	int          _rts_cts;
	int          _2_stop_bit;
	int          _parity;
	int          _odd_parity;
	int          _stick_parity;
	int          _dump_err;
	int          _no_rx;
	int          _no_tx;
	int          _rx_delay;
	int          _tx_delay;
	int          _tx_bytes;
	int          _rs485_delay;
	unsigned int _tx_time;
	unsigned int _rx_time;
	int          _ascii_range;
};

struct g_mod_t {

	unsigned char  _write_count_value;
	unsigned char  _read_count_value;
	int            _fd;
	unsigned char *_write_data;
	ssize_t        _write_size;

	// keep our own counts for cases where the driver stats don't work
	long long int _write_count;
	long long int _read_count;
	long long int _error_count;
};

static struct cli_args_t g_cl = {
	._single_byte  = -1,
	._another_byte = -1,
	._rs485_delay  = -1,
};

static struct g_mod_t g_mod = {
	._fd           = -1,
};

static void dump_data(unsigned char *b, int count)
{
	printf("%i bytes: ", count);
	int i;

	for (i = 0; i < count; i++) {
		printf("%02x ", b[i]);
	}

	printf("\n");
}

static void dump_data_ascii(unsigned char *b, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		printf("%c", b[i]);
	}
}

static void set_baud_divisor(int speed)
{
#if defined(__PX4_NUTTX)
	// default baud was not found, so try to set a custom divisor

	struct termios t;

	tcgetattr(g_mod._fd, &t);
	cfsetspeed(&t, speed);
	tcsetattr(g_mod._fd, TCSANOW, &t);

#else
	// default baud was not found, so try to set a custom divisor
	struct serial_struct ss;

	if (ioctl(g_mod._fd, TIOCGSERIAL, &ss) < 0) {
		perror("TIOCGSERIAL failed");
		exit(1);
	}

	ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
	ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
	int closest_speed = ss.baud_base / ss.custom_divisor;

	if (closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100) {
		fprintf(stderr, "Cannot set speed to %d, closest is %d\n", speed, closest_speed);
		exit(1);
	}

	printf("closest baud = %i, base = %i, divisor = %i\n", closest_speed, ss.baud_base,
	       ss.custom_divisor);

	if (ioctl(g_mod._fd, TIOCSSERIAL, &ss) < 0) {
		perror("TIOCSSERIAL failed");
		exit(1);
	}

#endif
}

// converts integer baud to Linux define
static int get_baud(int baud)
{
	switch (baud) {
	case 9600:
		return B9600;

	case 19200:
		return B19200;

	case 38400:
		return B38400;

	case 57600:
		return B57600;

	case 115200:
		return B115200;

	case 230400:
		return B230400;

	case 460800:
		return B460800;

	case 500000:
		return B500000;

	case 576000:
		return B576000;

	case 921600:
		return B921600;
#ifdef B1000000

	case 1000000:
		return B1000000;
#endif
#ifdef B1152000

	case 1152000:
		return B1152000;
#endif
#ifdef B1500000

	case 1500000:
		return B1500000;
#endif
#ifdef B2000000

	case 2000000:
		return B2000000;
#endif
#ifdef B2500000

	case 2500000:
		return B2500000;
#endif
#ifdef B3000000

	case 3000000:
		return B3000000;
#endif
#ifdef B3500000

	case 3500000:
		return B3500000;
#endif
#ifdef B4000000

	case 4000000:
		return B4000000;
#endif

	default:
		return -1;
	}
}

static void display_help(void)
{
#if defined(__PX4_NUTTX)
	printf("Usage: serial_test [OPTION]\n"
#else
	printf("Usage: linux-serial-test [OPTION]\n"
#endif
	       "\n"
	       "  -h, --help\n"
	       "  -b, --baud        Baud rate, 115200, etc (115200 is default)\n"
	       "  -p, --port        Port (/dev/ttyS0, etc) (must be specified)\n"
	       "  -d, --divisor     UART Baud rate divisor (can be used to set custom baud rates)\n"
	       "  -R, --rx_dump     Dump Rx data (ascii, raw)\n"
	       "  -T, --detailed_tx Detailed Tx data\n"
	       "  -s, --stats       Dump serial port stats every 5s\n"
	       "  -S, --stop-on-err Stop program if we encounter an error\n"
	       "  -y, --single-byte Send specified byte to the serial port\n"
	       "  -z, --second-byte Send another specified byte to the serial port\n"
	       "  -c, --rts-cts     Enable RTS/CTS flow control\n"
	       "  -B, --2-stop-bit  Use two stop bits per character\n"
	       "  -P, --parity      Use parity bit (odd, even, mark, space)\n"
	       "  -e, --dump-err    Display errors\n"
	       "  -r, --no-rx       Don't receive data (can be used to test flow control)\n"
	       "                    when serial driver buffer is full\n"
	       "  -t, --no-tx       Don't transmit data\n"
	       "  -l, --rx-delay    Delay between reading data (ms) (can be used to test flow control)\n"
	       "  -a, --tx-delay    Delay between writing data (ms)\n"
	       "  -w, --tx-bytes    Number of bytes for each write (default is to repeatedly write 1024 bytes\n"
	       "                    until no more are accepted)\n"
	       "  -q, --rs485       Enable RS485 direction control on port, and set delay\n"
	       "                    from when TX is finished and RS485 driver enable is\n"
	       "                    de-asserted. Delay is specified in bit times.\n"
	       "  -o, --tx-time     Number of seconds to transmit for (defaults to 0, meaning no limit)\n"
	       "  -i, --rx-time     Number of seconds to receive for (defaults to 0, meaning no limit)\n"
	       "  -A, --ascii       Output bytes range from 32 to 126 (default is 0 to 255)\n"
	       "example: serial_test -e -b 921600 -p /dev/ttyS2 -c -l 250\n"
	       "\n"
	      );
}

static void process_options(int argc, char *argv[])
{
#if defined(__PX4_NUTTX)
	int myoptind = 1;
	const char *myoptarg = NULL;
	int ch;

	while ((ch = px4_getopt(argc, argv, "hb:p:d:R:TsSy:z:cBertq:l:a:w:o:i:P:A", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
#else
#define myoptarg optarg

	for (;;) {
		int option_index = 0;
		static const char *short_options = "hb:p:d:R:TsSy:z:cBertq:l:a:w:o:i:P:A";
		static const struct option long_options[] = {
			{"help", no_argument, 0, 0},
			{"baud", required_argument, 0, 'b'},
			{"port", required_argument, 0, 'p'},
			{"divisor", required_argument, 0, 'd'},
			{"rx_dump", required_argument, 0, 'R'},
			{"detailed_tx", no_argument, 0, 'T'},
			{"stats", no_argument, 0, 's'},
			{"stop-on-err", no_argument, 0, 'S'},
			{"single-byte", no_argument, 0, 'y'},
			{"second-byte", no_argument, 0, 'z'},
			{"rts-cts", no_argument, 0, 'c'},
			{"2-stop-bit", no_argument, 0, 'B'},
			{"parity", required_argument, 0, 'P'},
			{"dump-err", no_argument, 0, 'e'},
			{"no-rx", no_argument, 0, 'r'},
			{"no-tx", no_argument, 0, 't'},
			{"rx-delay", required_argument, 0, 'l'},
			{"tx-delay", required_argument, 0, 'a'},
			{"tx-bytes", required_argument, 0, 'w'},
			{"rs485", required_argument, 0, 'q'},
			{"tx-time", required_argument, 0, 'o'},
			{"rx-time", required_argument, 0, 'i'},
			{"ascii", no_argument, 0, 'A'},
			{0, 0, 0, 0},
		};

		int c = getopt_long(argc, argv, short_options,
				    long_options, &option_index);

		if (c == EOF) {
			break;
		}

		switch (c) {
#endif

		case 0:
		case 'h':
			display_help();
			exit(0);
			break;

		case 'b':
			g_cl._baud = atoi(myoptarg);
			break;

		case 'p':
			g_cl._port = strdup(myoptarg);
			break;

		case 'd':
			g_cl._divisor = atoi(myoptarg);
			break;

		case 'R':
			g_cl._rx_dump = 1;
			g_cl._rx_dump_ascii = !strcmp(myoptarg, "ascii");
			break;

		case 'T':
			g_cl._tx_detailed = 1;
			break;

		case 's':
			g_cl._stats = 1;
			break;

		case 'S':
			g_cl._stop_on_error = 1;
			break;

		case 'y': {
				char *endptr;
				g_cl._single_byte = strtol(myoptarg, &endptr, 0);
				break;
			}

		case 'z': {
				char *endptr;
				g_cl._another_byte = strtol(myoptarg, &endptr, 0);
				break;
			}

		case 'c':
			g_cl._rts_cts = 1;
			break;

		case 'B':
			g_cl._2_stop_bit = 1;
			break;

		case 'P':
			g_cl._parity = 1;
			g_cl._odd_parity = (!strcmp(myoptarg, "mark") || !strcmp(myoptarg, "odd"));
			g_cl._stick_parity = (!strcmp(myoptarg, "mark") || !strcmp(myoptarg, "space"));
			break;

		case 'e':
			g_cl._dump_err = 1;
			break;

		case 'r':
			g_cl._no_rx = 1;
			break;

		case 't':
			g_cl._no_tx = 1;
			break;

		case 'l': {
				char *endptr;
				g_cl._rx_delay = strtol(myoptarg, &endptr, 0);
				break;
			}

		case 'a': {
				char *endptr;
				g_cl._tx_delay = strtol(myoptarg, &endptr, 0);
				break;
			}

		case 'w': {
				char *endptr;
				g_cl._tx_bytes = strtol(myoptarg, &endptr, 0);
				break;
			}

		case 'q': {
				char *endptr;
				g_cl._rs485_delay = strtol(myoptarg, &endptr, 0);
				break;
			}

		case 'o': {
				char *endptr;
				g_cl._tx_time = strtol(myoptarg, &endptr, 0);
				break;
			}

		case 'i': {
				char *endptr;
				g_cl._rx_time = strtol(myoptarg, &endptr, 0);
				break;
			}

		case 'A':
			g_cl._ascii_range = 1;
			break;
		}
	}
}

static void dump_serial_port_stats(void)
{
	printf("%s: count for this session: rx=%lld, tx=%lld, rx err=%lld\n", g_cl._port, g_mod._read_count, g_mod._write_count,
	       g_mod._error_count);
#if !defined(__PX4_NUTTX)
	struct serial_icounter_struct icount = { 0 };

	int ret = ioctl(g_mod._fd, TIOCGICOUNT, &icount);

	if (ret != -1) {
		printf("%s: TIOCGICOUNT: ret=%i, rx=%i, tx=%i, frame = %i, overrun = %i, parity = %i, brk = %i, buf_overrun = %i\n",
		       g_cl._port, ret, icount.rx, icount.tx, icount.frame, icount.overrun, icount.parity, icount.brk,
		       icount.buf_overrun);
	}

#endif
}

static unsigned char next_count_value(unsigned char c)
{
	c++;

	if (g_cl._ascii_range && c == 127) {
		c = 32;
	}

	return c;
}

static void process_read_data(void)
{
	unsigned char rb[1024];
	int c = read(g_mod._fd, &rb, sizeof(rb));

	if (c > 0) {
		if (g_cl._rx_dump) {
			if (g_cl._rx_dump_ascii) {
				dump_data_ascii(rb, c);

			} else {
				dump_data(rb, c);
			}
		}

		// verify read count is incrementing
		int i;

		for (i = 0; i < c; i++) {
			if (rb[i] != g_mod._read_count_value) {
				if (g_cl._dump_err) {
					printf("Error, count: %lld, expected %02x, got %02x\n",
					       g_mod._read_count + i, g_mod._read_count_value, rb[i]);
				}

				g_mod._error_count++;

				if (g_cl._stop_on_error) {
					dump_serial_port_stats();
					exit(1);
				}

				g_mod._read_count_value = rb[i];
			}

			g_mod._read_count_value = next_count_value(g_mod._read_count_value);
		}

		g_mod._read_count += c;
	}
}

static void process_write_data(void)
{
	ssize_t count = 0;
	int repeat = (g_cl._tx_bytes == 0);

	do {
		ssize_t i;

		for (i = 0; i < g_mod._write_size; i++) {
			g_mod._write_data[i] = g_mod._write_count_value;
			g_mod._write_count_value = next_count_value(g_mod._write_count_value);
		}

		ssize_t c = write(g_mod._fd, g_mod._write_data, g_mod._write_size);

		if (c < 0) {
			if (errno != EAGAIN) {
				printf("write failed - errno=%d (%s)\n", errno, strerror(errno));
			}

			c = 0;
		}

		count += c;

		if (c < g_mod._write_size) {
			g_mod._write_count_value = g_mod._write_data[c];
			repeat = 0;
		}
	} while (repeat);

	g_mod._write_count += count;

	if (g_cl._tx_detailed) {
		printf("wrote %zd bytes\n", count);
	}
}


static void setup_serial_port(int baud)
{
	struct termios newtio;
	struct serial_rs485 rs485;

	if (g_mod._fd == -1) {
		g_mod._fd = open(g_cl._port, O_RDWR | O_NONBLOCK);

		if (g_mod._fd < 0) {
			perror("Error opening serial port");
			free(g_cl._port);
			exit(1);
		}
	}

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

#if defined(__PX4_NUTTX)
	tcgetattr(g_mod._fd, &newtio);
	cfsetspeed(&newtio, baud);
	newtio.c_cflag = CS8 | CLOCAL | CREAD;
#else
	/* man termios get more info on below settings */
	newtio.c_cflag = baud | CS8 | CLOCAL | CREAD;

#endif

	if (g_cl._rts_cts) {
		newtio.c_cflag |= CRTSCTS;
	}

	if (g_cl._2_stop_bit) {
		newtio.c_cflag |= CSTOPB;
	}

	if (g_cl._parity) {
		newtio.c_cflag |= PARENB;

		if (g_cl._odd_parity) {
			newtio.c_cflag |= PARODD;
		}

		if (g_cl._stick_parity) {
			newtio.c_cflag |= CMSPAR;
		}
	}

	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;

	// block for up till 128 characters
	newtio.c_cc[VMIN] = 128;

	// 0.5 seconds read timeout
	newtio.c_cc[VTIME] = 5;

	/* now clean the modem line and activate the settings for the port */
	tcflush(g_mod._fd, TCIOFLUSH);
	tcsetattr(g_mod._fd, TCSANOW, &newtio);

	/* enable/disable rs485 direction control */
	if (ioctl(g_mod._fd, TIOCGRS485, (int) &rs485) < 0) {
		if (g_cl._rs485_delay >= 0) {
			/* error could be because hardware is missing rs485 support so only print when actually trying to activate it */
			perror("Error getting RS-485 mode");
		}

	} else if (g_cl._rs485_delay >= 0) {
		rs485.flags |= SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND | SER_RS485_RTS_AFTER_SEND;
		rs485.delay_rts_after_send = g_cl._rs485_delay;
		rs485.delay_rts_before_send = 0;

		if (ioctl(g_mod._fd, TIOCSRS485, (int) &rs485) < 0) {
			perror("Error setting RS-485 mode");
		}

	} else {
		rs485.flags &= ~(SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND | SER_RS485_RTS_AFTER_SEND);
		rs485.delay_rts_after_send = 0;
		rs485.delay_rts_before_send = 0;

		if (ioctl(g_mod._fd, TIOCSRS485, (int) &rs485) < 0) {
			perror("Error setting RS-232 mode");
		}
	}
}

static int diff_ms(const struct timespec *t1, const struct timespec *t2)
{
	struct timespec diff;

	diff.tv_sec = t1->tv_sec - t2->tv_sec;
	diff.tv_nsec = t1->tv_nsec - t2->tv_nsec;

	if (diff.tv_nsec < 0) {
		diff.tv_sec--;
		diff.tv_nsec += 1000000000;
	}

	return (diff.tv_sec * 1000 + diff.tv_nsec / 1000000);
}

#if defined(__PX4_NUTTX)
int serial_test_main(int argc, char *argv[])
{
	printf("serial test app\n");

	memset(&g_cl, 0, sizeof(g_cl));
	g_cl._single_byte = -1;
	g_cl._another_byte = -1;
	g_cl._rs485_delay = -1;
	memset(&g_mod, 0, sizeof(g_mod));
	g_mod._fd = -1;

#else
int main(int argc, char *argv[])
{
	printf("Linux serial test app\n");
#endif

	process_options(argc, argv);

	if (!g_cl._port) {
		fprintf(stderr, "ERROR: Port argument required\n");
		display_help();
		return 1;
	}

	int baud = B115200;

	if (g_cl._baud) {
		baud = get_baud(g_cl._baud);
	}

	if (baud <= 0) {
		printf("NOTE: non standard baud rate, trying custom divisor\n");
		baud = B38400;
		setup_serial_port(B38400);
		set_baud_divisor(g_cl._baud);

	} else {
		setup_serial_port(baud);
	}

	if (g_cl._single_byte >= 0) {
		unsigned char data[2];
		int bytes = 1;
		int written;
		data[0] = (unsigned char)g_cl._single_byte;

		if (g_cl._another_byte >= 0) {
			data[1] = (unsigned char)g_cl._another_byte;
			bytes++;
		}

		written = write(g_mod._fd, &data, bytes);

		if (written < 0) {
			perror("write()");
			return 1;

		} else if (written != bytes) {
			fprintf(stderr, "ERROR: write() returned %d, not %d\n", written, bytes);
			return 1;
		}

		return 0;
	}

	g_mod._write_size = (g_cl._tx_bytes == 0) ? 1024 : g_cl._tx_bytes;

	g_mod._write_data = malloc(g_mod._write_size);

	if (g_mod._write_data == NULL) {
		fprintf(stderr, "ERROR: Memory allocation failed\n");
		return 1;
	}

	if (g_cl._ascii_range) {
		g_mod._read_count_value = g_mod._write_count_value = 32;
	}

	struct pollfd serial_poll;

	memset(&serial_poll, 0, sizeof(serial_poll));

	serial_poll.fd = g_mod._fd;

	if (!g_cl._no_rx) {
		serial_poll.events |= POLLIN;

	} else {
		serial_poll.events &= ~POLLIN;
	}

	if (!g_cl._no_tx) {
		serial_poll.events |= POLLOUT;

	} else {
		serial_poll.events &= ~POLLOUT;
	}

	struct timespec start_time, last_stat, last_timeout, last_read, last_write;

	clock_gettime(CLOCK_MONOTONIC, &start_time);

	last_stat = start_time;

	last_timeout = start_time;

	last_read = start_time;

	last_write = start_time;

#if defined(__PX4_NUTTX)
	bool _exit = false;

	struct pollfd fds;

	int ret;

	fds.fd = 0; /* stdin */

	fds.events = POLLIN;

	while (!_exit && !(g_cl._no_rx && g_cl._no_tx)) {

		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			char c;
			ret = read(0, &c, 1);

			if (ret > 0) {
				switch (c) {
				case 0x03: // ctrl-c
				case 0x1b: // esc
				case 'c':
				case 'q':
					_exit = true;
					break;

				default:
					printf("\n");
					fflush(stdout);
					break;
				}
			}
		}

#else

	while (!(g_cl._no_rx && g_cl._no_tx)) {
#endif
		struct timespec current;
		int retval = poll(&serial_poll, 1, 1000);

		clock_gettime(CLOCK_MONOTONIC, &current);

		if (retval == -1) {
			perror("poll()");

		} else if (retval) {
			if (serial_poll.revents & POLLIN) {
				if (g_cl._rx_delay) {
					// only read if it has been rx-delay ms
					// since the last read
					if (diff_ms(&current, &last_read) > g_cl._rx_delay) {
						process_read_data();
						last_read = current;
					}

				} else {
					process_read_data();
					last_read = current;
				}
			}

			if (serial_poll.revents & POLLOUT) {
				if (g_cl._tx_delay) {
					// only write if it has been tx-delay ms
					// since the last write
					if (diff_ms(&current, &last_write) > g_cl._tx_delay) {
						process_write_data();
						last_write = current;
					}

				} else {
					process_write_data();
					last_write = current;
				}
			}
		}

		// Has it been at least a second since we reported a timeout?
		if (diff_ms(&current, &last_timeout) > 1000) {
			int rx_timeout, tx_timeout;

			// Has it been over two seconds since we transmitted or received data?
			rx_timeout = (!g_cl._no_rx && diff_ms(&current, &last_read) > 2000);
			tx_timeout = (!g_cl._no_tx && diff_ms(&current, &last_write) > 2000);

			// Special case - we don't want to warn about receive
			// timeouts at the end of a loopback test (where we are
			// no longer transmitting and the receive count equals
			// the transmit count).
			if (g_cl._no_tx && g_mod._write_count != 0 && g_mod._write_count == g_mod._read_count) {
				rx_timeout = 0;
			}

			if (rx_timeout || tx_timeout) {
				const char *s;

				if (rx_timeout) {
					printf("No data received for %.1fs.",
					       (double)(diff_ms(&current, &last_read) / 1000));
					s = " ";

				} else {
					s = "";
				}

				if (tx_timeout) {
					printf("%sNo data transmitted for %.1fs.",
					       s, (double)(diff_ms(&current, &last_write) / 1000));
				}

				printf("\n");
				last_timeout = current;
			}
		}

		if (g_cl._stats) {
			if (current.tv_sec - last_stat.tv_sec > 5) {
				dump_serial_port_stats();
				last_stat = current;
			}
		}

		if (g_cl._tx_time) {
			if (current.tv_sec - start_time.tv_sec >= g_cl._tx_time) {
				g_cl._tx_time = 0;
				g_cl._no_tx = 1;
				serial_poll.events &= ~POLLOUT;
				printf("Stopped transmitting.\n");
			}
		}

		if (g_cl._rx_time) {
			if (current.tv_sec - start_time.tv_sec >= g_cl._rx_time) {
				g_cl._rx_time = 0;
				g_cl._no_rx = 1;
				serial_poll.events &= ~POLLIN;
				printf("Stopped receiving.\n");
			}
		}
	}

	printf("Exiting...\n");

	/* ignore HS to allow exit */

	if (g_cl._rts_cts != 0) {
		g_cl._rts_cts = 0;
		setup_serial_port(baud);
	}

	tcdrain(g_mod._fd);
	dump_serial_port_stats();
	tcflush(g_mod._fd, TCIOFLUSH);
	free(g_cl._port);
	close(g_mod._fd);
	g_mod._fd = -1;
	long long int result = llabs(g_mod._write_count - g_mod._read_count) + g_mod._error_count;

	return (result > 125) ? 125 : (int)result;
}
