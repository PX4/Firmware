/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_shell.h
 * A shell to be used via MAVLink
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */


#include <stddef.h>
#include <stdint.h>
#include <px4_platform_common/tasks.h>

#pragma once

class MavlinkShell
{
public:
	MavlinkShell() = default;

	~MavlinkShell();

	/**
	 * Start the mavlink shell.
	 *
	 * @return		0 on success.
	 */
	int		start();

	/**
	 * Write to the shell
	 * @return number of written bytes
	 */
	size_t write(uint8_t *buffer, size_t len);

	/**
	 * Read from the shell. This is blocking, if 0 bytes are available, this will block.
	 * @param len buffer length
	 * @return number of bytes read.
	 */
	size_t read(uint8_t *buffer, size_t len);

	/**
	 * Get the number of bytes that can be read.
	 */
	size_t available();

private:

	int _to_shell_fd = -1; /** fd to write to the shell */
	int _from_shell_fd = -1; /** fd to read from the shell */
#ifdef __PX4_NUTTX
	int _shell_fds[2] = { -1, -1}; /** stdin & out used by the shell */
	px4_task_t _task;
#endif

	static int shell_start_thread(int argc, char *argv[]);

	/* do not allow copying this class */
	MavlinkShell(const MavlinkShell &) = delete;
	MavlinkShell operator=(const MavlinkShell &) = delete;
};
