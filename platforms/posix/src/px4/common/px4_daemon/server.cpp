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
 * @file server.cpp
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @author Mara Bos <m-ou.se@m-ou.se>
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <pthread.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <vector>

#include <px4_platform_common/log.h>

#include "pxh.h"
#include "server.h"

namespace px4_daemon
{

Server *Server::_instance = nullptr;

Server::Server(int instance_id)
	: _mutex(PTHREAD_MUTEX_INITIALIZER),
	  _instance_id(instance_id)
{
	_instance = this;
}

Server::~Server()
{
	_instance = nullptr;
}

int
Server::start()
{
	std::string sock_path = get_socket_path(_instance_id);

	// Delete socket in case it exists already.
	unlink(sock_path.c_str());

	_fd = socket(AF_UNIX, SOCK_STREAM, 0);

	if (_fd < 0) {
		PX4_ERR("error creating socket");
		return -1;
	}

	sockaddr_un addr = {};
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, sock_path.c_str(), sizeof(addr.sun_path) - 1);

	if (bind(_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("error binding socket %s, error = %s", sock_path.c_str(), strerror(errno));
		return -1;
	}

	if (listen(_fd, 10) < 0) {
		PX4_ERR("error listening to socket: %s", strerror(errno));
		return -1;
	}

	if (0 != pthread_create(&_server_main_pthread,
				nullptr,
				_server_main_trampoline,
				this)) {
		PX4_ERR("error creating client handler thread");
		return -1;
	}

	return 0;
}

void *
Server::_server_main_trampoline(void *self)
{
	((Server *)self)->_server_main();
	return nullptr;
}

void Server::_pthread_key_destructor(void *arg)
{
	delete ((CmdThreadSpecificData *)arg);
}

void
Server::_server_main()
{
	int ret = pthread_key_create(&_key, _pthread_key_destructor);

	if (ret != 0) {
		PX4_ERR("failed to create pthread key");
		return;
	}

	// The list of file descriptors to watch.
	std::vector<pollfd> poll_fds;

	// Watch the listening socket for incoming connections.
	poll_fds.push_back(pollfd {_fd, POLLIN, 0});

	// The list of FILE pointers that we'll need to fclose().
	// stdouts[i] corresponds to poll_fds[i+1].
	std::vector<FILE *> stdouts;

	while (true) {
		int n_ready = poll(poll_fds.data(), poll_fds.size(), -1);

		if (n_ready < 0) {
			PX4_ERR("poll() failed: %s", strerror(errno));
			return;
		}

		_lock();

		// Handle any new connections.
		if (poll_fds[0].revents & POLLIN) {
			--n_ready;
			int client = accept(_fd, nullptr, nullptr);

			if (client == -1) {
				PX4_ERR("failed to accept client: %s", strerror(errno));
				_unlock();
				return;
			}

			FILE *thread_stdout = fdopen(client, "w");

			if (thread_stdout == nullptr) {
				PX4_ERR("could not open stdout for new thread");
				close(client);

			} else {
				// Set stream to line buffered.
				setvbuf(thread_stdout, nullptr, _IOLBF, BUFSIZ);

				// Start a new thread to handle the client.
				pthread_t *thread = &_fd_to_thread[client];
				ret = pthread_create(thread, nullptr, Server::_handle_client, thread_stdout);

				if (ret != 0) {
					PX4_ERR("could not start pthread (%i)", ret);
					_fd_to_thread.erase(client);
					fclose(thread_stdout);

				} else {
					// We won't join the thread, so detach to automatically release resources at its end
					pthread_detach(*thread);

					// Start listening for the client hanging up.
					poll_fds.push_back(pollfd {client, POLLHUP, 0});

					// Remember the FILE *, so we can fclose() it later.
					stdouts.push_back(thread_stdout);
				}
			}
		}

		// Handle any closed connections.
		for (size_t i = 1; n_ready > 0 && i < poll_fds.size();) {
			if (poll_fds[i].revents) {
				--n_ready;
				auto thread = _fd_to_thread.find(poll_fds[i].fd);

				if (thread != _fd_to_thread.end()) {
					// Thread is still running, so we cancel it.
					// TODO: use a more graceful exit method to avoid resource leaks
					pthread_cancel(thread->second);
					_fd_to_thread.erase(thread);
				}

				fclose(stdouts[i - 1]);
				stdouts.erase(stdouts.begin() + i - 1);
				poll_fds.erase(poll_fds.begin() + i);

			} else {
				++i;
			}
		}

		_unlock();
	}

	close(_fd);
}

void
*Server::_handle_client(void *arg)
{
	FILE *out = (FILE *)arg;
	int fd = fileno(out);

	// Read until the end of the incoming stream.
	std::string cmd;

	while (true) {
		size_t n = cmd.size();
		cmd.resize(n + 1024);
		ssize_t n_read = read(fd, &cmd[n], cmd.size() - n);

		if (n_read <= 0) {
			_cleanup(fd);
			return nullptr;
		}

		cmd.resize(n + n_read);

		// Command ends in 0x00 (no tty) or 0x01 (tty).
		if (!cmd.empty() && cmd.back() < 2) {
			break;
		}
	}

	if (cmd.size() < 2) {
		_cleanup(fd);
		return nullptr;
	}

	// Last byte is 'isatty'.
	uint8_t isatty = cmd.back();
	cmd.pop_back();

	// We register thread specific data. This is used for PX4_INFO (etc.) log calls.
	CmdThreadSpecificData *thread_data_ptr;

	if ((thread_data_ptr = (CmdThreadSpecificData *)pthread_getspecific(_instance->_key)) == nullptr) {
		thread_data_ptr = new CmdThreadSpecificData;
		thread_data_ptr->thread_stdout = out;
		thread_data_ptr->is_atty = isatty;

		(void)pthread_setspecific(_instance->_key, (void *)thread_data_ptr);
	}

	// Run the actual command.
	int retval = Pxh::process_line(cmd, true);

	// Report return value.
	char buf[2] = {0, (char)retval};

	if (fwrite(buf, sizeof buf, 1, out) != 1) {
		// Don't care it went wrong, as we're cleaning up anyway.
	}

	// Flush the FILE*'s buffer before we shut down the connection.
	fflush(out);

	_cleanup(fd);
	return nullptr;
}

void
Server::_cleanup(int fd)
{
	_instance->_lock();
	_instance->_fd_to_thread.erase(fd);
	_instance->_unlock();

	// We can't close() the fd here, since the main thread is probably
	// polling for it: close()ing it causes a race condition.
	// So, we only call shutdown(), which causes the main thread to register a
	// 'POLLHUP', such that the main thread can close() it for us.
	// We already removed this thread from _fd_to_thread, so there is no risk
	// of the main thread trying to cancel this thread after it already exited.
	shutdown(fd, SHUT_RDWR);
}

} //namespace px4_daemon
