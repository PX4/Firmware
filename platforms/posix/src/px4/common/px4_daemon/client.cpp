/****************************************************************************
 *
 *   Copyright (C) 2016, 2018 PX4 Development Team. All rights reserved.
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
 * @file client.cpp
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @author Mara Bos <m-ou.se@m-ou.se>
 */

#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <string>

#include <px4_platform_common/log.h>
#include "client.h"

namespace px4_daemon
{

Client::Client(int instance_id) :
	_fd(-1),
	_instance_id(instance_id)
{}

int
Client::process_args(const int argc, const char **argv)
{
	std::string sock_path = get_socket_path(_instance_id);

	_fd = socket(AF_UNIX, SOCK_STREAM, 0);

	if (_fd < 0) {
		PX4_ERR("error creating socket");
		return -1;
	}

	sockaddr_un addr = {};
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, sock_path.c_str(), sizeof(addr.sun_path) - 1);

	if (connect(_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("error connecting to socket: %s", strerror(errno));
		return -1;
	}

	int ret = _send_cmds(argc, argv);

	if (ret != 0) {
		PX4_ERR("Could not send commands");
		return -3;
	}

	return _listen();
}

int
Client::_send_cmds(const int argc, const char **argv)
{
	std::string cmd_buf;

	for (int i = 0; i < argc; ++i) {
		cmd_buf += argv[i];

		if (i + 1 != argc) {
			// TODO: Use '\0' as argument separator (and parse this server-side as well),
			// so (quoted) whitespace within arguments doesn't get lost.
			cmd_buf += " ";
		}
	}

	// Last byte is 'isatty'.
	cmd_buf.push_back(isatty(STDOUT_FILENO));

	size_t n = cmd_buf.size();
	const char *buf = cmd_buf.data();

	while (n > 0) {
		int n_sent = write(_fd, buf, n);

		if (n_sent < 0) {
			PX4_ERR("write() failed: %s", strerror(errno));
			return -1;
		}

		n -= n_sent;
		buf += n_sent;
	}

	return 0;
}

int
Client::_listen()
{
	char buffer[1024];
	int n_buffer_used = 0;

	// The response ends in {0, retval}. So when we detect a 0, or a 0 followed
	// by another byte, we don't output it yet, until we know whether it was
	// the end of the stream or not.
	while (true) {
		int n_read = read(_fd, buffer + n_buffer_used, sizeof buffer - n_buffer_used);

		if (n_read < 0) {
			PX4_ERR("unable to read from socket");
			return -1;

		} else if (n_read == 0) {
			if (n_buffer_used == 2) {
				return buffer[1];

			} else {
				// Missing return value at end of stream. Stream was abruptly ended.
				return -1;
			}

		} else {
			n_read += n_buffer_used;

			if (n_read >= 2 && buffer[n_read - 2] == 0) {
				// If the buffer ends in {0, retval}, keep it.
				fwrite(buffer, n_read - 2, 1, stdout);
				buffer[0] = 0;
				buffer[1] = buffer[n_read - 1];
				n_buffer_used = 2;

			} else if (n_read >= 1 && buffer[n_read - 1] == 0) {
				// If the buffer ends in a 0-byte, keep it.
				fwrite(buffer, n_read - 1, 1, stdout);
				buffer[0] = 0;
				n_buffer_used = 1;

			} else {
				fwrite(buffer, n_read, 1, stdout);
				n_buffer_used = 0;
			}
		}
	}
}

Client::~Client()
{
	if (_fd >= 0) {
		close(_fd);
	}
}

} // namespace px4_daemon
