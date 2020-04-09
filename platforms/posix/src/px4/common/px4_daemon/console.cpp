/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file console.cpp
 *
 * @author SalimTerryLi <lhf2613@gmail.com>
 */

#ifdef __PX4_LINUX_CONSOLE

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>

#include "console.h"
#include <px4_daemon/px4_console.h>

using namespace px4_console;

int px4_console::_fake_stdin_pipe[2] = {-1, -1};
int px4_console::_mavshell_out_pipe[2] = {-1, -1};
int px4_console::_sys_stdin_backup = -1;

px4_task_t _task;
int _thread_should_exit_pipe[2] = {-1, -1};

int from_console_output_fd(int index)
{
	return px4_console::_mavshell_out_pipe[index];
}

int to_console_input_fd(int index)
{
	return px4_console::_fake_stdin_pipe[index];
}

int px4_console::prepare_fds()
{
	int ret = 0;
	ret = pipe(_fake_stdin_pipe);

	if (ret == -1) {
		return -errno;
	}

	ret = pipe(_mavshell_out_pipe);

	if (ret == -1) {
		clean_fds();
		return -errno;
	}

	_sys_stdin_backup = dup(0);	// backup stdin

	if (_sys_stdin_backup == -1) {
		clean_fds();
		return -errno;
	}

	fcntl(_sys_stdin_backup, F_SETFL, fcntl(_sys_stdin_backup, F_GETFL) | O_NONBLOCK);	// set stdin non blocking

	fcntl(_mavshell_out_pipe[1], F_SETFL, fcntl(_mavshell_out_pipe[1],
			F_GETFL) | O_NONBLOCK);	// prevent write blocking if pipe is full

	ret = dup2(_fake_stdin_pipe[0], 0);	// override stdin

	if (ret == -1) {
		clean_fds();
		return -errno;
	}

	return 0;
}

void px4_console::clean_fds()
{
	for (int i = 0; i < 2; ++i) {
		if (_fake_stdin_pipe[i] >= 0) {
			close(_fake_stdin_pipe[i]);
			_fake_stdin_pipe[i] = -1;
		}

		if (_mavshell_out_pipe[i] >= 0) {
			close(_mavshell_out_pipe[i]);
			_mavshell_out_pipe[i] = -1;
		}
	}

	if (_sys_stdin_backup >= 0) {
		fcntl(_sys_stdin_backup, F_SETFL, fcntl(_sys_stdin_backup, F_GETFL) & ~O_NONBLOCK);	// reset stdin
		dup2(_sys_stdin_backup, 0);	// reset stdin
		close(_sys_stdin_backup);
		_sys_stdin_backup = -1;
	}
}

int console_thread(int argc, char *argv[])
{
	int ret = 0;
	ret = pipe(_thread_should_exit_pipe);	// create signal pipe

	if (ret == -1) {
		// no more we can do.
		PX4_ERR("failed to create pipe: %d", errno);
		return -1;
	}

	while (true) {
		pollfd fds[2];
		fds[0].fd = _thread_should_exit_pipe[0];	// should exit
		fds[0].events = POLLIN;
		fds[1].fd = _sys_stdin_backup;	// capture stdin
		fds[1].events = POLLIN;
		ret = poll(fds, 2, -1);

		if (ret == -1) {
			if (errno == EINTR) {
				continue;

			} else {
				// undefined behavior
				PX4_ERR("poll failed");
				break;
			}
		}

		if (fds[0].revents == POLLIN) {
			break;
		}

		if (fds[1].revents == POLLIN) {
			char tmp[32];
			ret = read(fds[1].fd, tmp, 32);

			if (ret == 0) {	// EOF
				// TODO: handle this
			} else if (ret == -1) {
				if (errno == EINTR) {
					continue;

				} else {
					// this should not happen
					PX4_ERR("failed to read from stdin");
				}

			} else {
				ret = write(_fake_stdin_pipe[1], tmp, ret);

				if (ret == -1) {
					PX4_ERR("failed to write back to stdin");
				}
			}
		}
	}

	close(_thread_should_exit_pipe[0]);
	close(_thread_should_exit_pipe[1]);
	_thread_should_exit_pipe[0] = -1;
	_thread_should_exit_pipe[1] = -1;

	return 0;
}

int px4_console::launch_thread()
{
	_task = px4_task_spawn_cmd("px4_console",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_DEFAULT,
				   2048,
				   &console_thread,
				   nullptr);

	if (_task < 0) {
		PX4_ERR("failed to launch console thread");
		return -1;
	}

	return 0;
}

void px4_console::stop_thread()
{
	if (_thread_should_exit_pipe[1] >= 0) {
		char tmp = '\0';
		int ret = write(_thread_should_exit_pipe[1], &tmp, 1);

		if (ret == -1) {
			PX4_ERR("failed to send exit signal");
		}
	}
}

#endif	// #ifdef __PX4_LINUX