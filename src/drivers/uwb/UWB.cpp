/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "UWB.h"
#include <px4_log.h>
#include <px4_getopt.h>
#include <px4_cli.h>
#include <errno.h>
#include <fcntl.h>
#include <systemlib/err.h>

extern "C" __EXPORT int uwb_main(int argc, char *argv[]);

UWB::UWB(const char *device_name, int baudrate):
	_time_perf(perf_alloc(PC_ELAPSED, "uwb_time")),
	_read_count_perf(perf_alloc(PC_COUNT, "uwb_count")),
	_read_err_perf(perf_alloc(PC_COUNT, "uwb_err"))
{

	speed_t baud = B115200;

	switch (baudrate) {
	case 9600:
		baud = B9600;
		break;

	case 19200:
		baud = B19200;
		break;

	case 38400:
		baud = B38400;
		break;

	case 57600:
		baud = B57600;
		break;

	case 115200:
		baud = B115200;
		break;

	case 460800:
		baud = B460800;
		break;

	case 500000:
		baud = B500000;
		break;

	case 921600:
		baud = B921600;
		break;

	default:
		err(1, "%d is not a valid baud rate.", baudrate);
	}

	// start serial port
	_uart = open(device_name, O_RDWR | O_NOCTTY);

	if (_uart < 0) { err(1, "could not open %s", device_name); }

	int ret = 0;
	struct termios uart_config {};
	ret = tcgetattr(_uart, &uart_config);

	if (ret < 0) { err(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, baud);

	if (ret < 0) { err(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, baud);

	if (ret < 0) { err(1, "failed to set output speed"); }

	ret = tcsetattr(_uart, TCSANOW, &uart_config);

	if (ret < 0) { err(1, "failed to set attr"); }

}

UWB::~UWB()
{

}

void UWB::run()
{

	for (int i = 3; i > 0; i--) {
		PX4_INFO("%d...", i);
		px4_sleep(1);
	}

//	printf("UUID Reversed: [");
//	uint8_t grid_uuid_reversed[sizeof(GRID_UUID)];
//	for(size_t i = 0; i < sizeof(GRID_UUID); i++){
//		grid_uuid_reversed[i] = GRID_UUID[sizeof(GRID_UUID) - i - 1];
//		printf("0x%02X, ", grid_uuid_reversed[i]);
//	}
//	printf("]\n");

	uint8_t command[sizeof(CMD_START_RANGING) + sizeof(GRID_UUID)];
	memcpy(&command[0], CMD_PURE_RANGING, sizeof(CMD_START_RANGING));
	memcpy(&command[sizeof(CMD_START_RANGING)], GRID_UUID, sizeof(GRID_UUID));
	write(_uart, command, sizeof(command));

	px4_sleep(1);

	union {
		position_msg_t msg;
		uint8_t buffer[sizeof(position_msg_t)];
	} data;

	perf_begin(_time_perf);

	while (!should_exit()) {
		//write(_uart, command, sizeof(command));

		FD_ZERO(&_uart_set);
		FD_SET(_uart, &_uart_set);
		_uart_timeout.tv_sec = 1;
		_uart_timeout.tv_usec = 0;

		size_t buffer_location = 0;

		// Experimentally I get one message about every 37ms.
		while (buffer_location < sizeof(data.buffer) - 1
		       && select(_uart + 1, &_uart_set, nullptr, nullptr, &_uart_timeout) > 0) {
			int bytes_read = read(_uart, &data.buffer[buffer_location], sizeof(data.buffer) - buffer_location);
			buffer_location += bytes_read;

			FD_ZERO(&_uart_set);
			FD_SET(_uart, &_uart_set);
			_uart_timeout.tv_sec = 0;
			//_uart_timeout.tv_usec = 70; // In an ideal world...
			_uart_timeout.tv_usec = 10000;
		}

		perf_count(_read_count_perf);

		if (buffer_location == sizeof(position_msg_t)) {

			PX4_INFO("Total bytes read: %d", (int) buffer_location);
			PX4_INFO("Status: 0x%02X. Pos: (%.4f, %.4f, %.4f)", data.msg.status, (double) data.msg.pos_x, (double) data.msg.pos_y,
				 (double) data.msg.pos_z);

		} else {
			PX4_ERR("Read %d bytes instead of %d.", (int) buffer_location, (int) sizeof(position_msg_t));
			perf_count(_read_err_perf);
		}

		//px4_sleep(1);
	}

	perf_end(_time_perf);

	memcpy(&command[0], CMD_STOP_RANGING, sizeof(CMD_STOP_RANGING));
	write(_uart, &command[0], sizeof(command));
}

int UWB::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command.");
}

int UWB::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	//TODO: Print module usage and whatnot, which is also documentation

	return 0;
}

int UWB::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd(
			      "uwb_driver",
			      SCHED_DEFAULT,
			      SCHED_PRIORITY_DEFAULT,
			      2048,
			      &run_trampoline,
			      argv
		      );

	if (task_id < 0) {
		return -errno;

	} else {
		_task_id = task_id;
		return 0;
	}
}

UWB *UWB::instantiate(int argc, char *argv[])
{
	int ch;
	int option_index = 1;
	const char *option_arg;
	const char *device_name = nullptr;
	bool error_flag = false;
	int baudrate = 115200;

	while ((ch = px4_getopt(argc, argv, "b:d:", &option_index, &option_arg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(option_arg, baudrate) != 0) {
				PX4_ERR("Error parsing \"%s\"", option_arg);
				error_flag = true;
			}

			break;

		case 'd':
			device_name = option_arg;
			break;

		default:
			PX4_WARN("Unrecognized flag: %c", ch);
			error_flag = true;
			break;
		}
	}

	if (!error_flag && device_name == nullptr) {
		print_usage("Device name not provided.");
		error_flag = true;
	}

	if (!error_flag && baudrate == 0) {
		print_usage("Baudrate not provided.");
		error_flag = true;
	}

	if (error_flag) {
		PX4_WARN("Failed to start UWB driver.");
		return nullptr;

	} else {
		PX4_INFO("Constructing UWB. Device: %s, Baud: %d", device_name, baudrate);
		return new UWB(device_name, baudrate);
	}
}

int uwb_main(int argc, char *argv[])
{
	return UWB::main(argc, argv);
}