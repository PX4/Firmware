
/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file cdevtest_example.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include "cdevtest_example.h"

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <lib/cdev/CDev.hpp>
#include <unistd.h>
#include <stdio.h>

px4::AppState CDevExample::appState;

#define TESTDEV "/dev/cdevtest"

static bool g_exit = false;

static int writer_main(int argc, char *argv[])
{
	char buf[1];

	int fd = px4_open(TESTDEV, PX4_F_WRONLY);

	if (fd < 0) {
		PX4_INFO("Writer: Open failed %d", fd);
		return PX4_ERROR;
	}

	int ret = 0;
	int i = 0;

	while (!g_exit) {
		// Wait for 2 seconds
		PX4_INFO("Writer: Sleeping for 2 sec");
		ret = px4_sleep(2);

		if (ret < 0) {
			PX4_INFO("Writer: sleep failed %d %d", ret, errno);
			return ret;
		}

		buf[0] = 'A' + (char)(i % 26);
		PX4_INFO("Writer: writing char '%c'", buf[0]);
		ret = px4_write(fd, buf, 1);
		++i;
	}

	px4_close(fd);
	PX4_INFO("Writer: stopped");
	return ret;
}

class PrivData
{
public:
	PrivData() : _read_offset(0) {}
	~PrivData() = default;

	size_t _read_offset;
};

class CDevNode : public cdev::CDev
{
public:
	CDevNode() :
		CDev(TESTDEV),
		_is_open_for_write(false),
		_write_offset(0) {}

	~CDevNode() override = default;

	int open(cdev::file_t *handlep) override;
	int close(cdev::file_t *handlep) override;
	ssize_t write(cdev::file_t *handlep, const char *buffer, size_t buflen) override;
	ssize_t read(cdev::file_t *handlep, char *buffer, size_t buflen) override;
private:
	bool _is_open_for_write;
	size_t _write_offset;
	char     _buf[1000];
};

int CDevNode::open(cdev::file_t *handlep)
{
	// Only allow one writer
	if (_is_open_for_write && (handlep->f_oflags & PX4_F_WRONLY)) {
		errno = EBUSY;
		return -1;
	}

	int ret = CDev::open(handlep);

	if (ret != 0) {
		return ret;
	}

	handlep->f_priv = new PrivData;

	if (_is_open_for_write && (handlep->f_oflags & PX4_F_WRONLY)) {
		_is_open_for_write = true;
	}

	return 0;
}

int CDevNode::close(cdev::file_t *handlep)
{
	delete (PrivData *)handlep->f_priv;
	handlep->f_priv = nullptr;
	CDev::close(handlep);

	// Enable a new writer of the device is re-opened for write
	if ((handlep->f_oflags & PX4_F_WRONLY) && _is_open_for_write) {
		_is_open_for_write = false;
	}

	return 0;
}

ssize_t CDevNode::write(cdev::file_t *handlep, const char *buffer, size_t buflen)
{
	for (size_t i = 0; i < buflen && _write_offset < 1000; i++) {
		_buf[_write_offset] = buffer[i];
		_write_offset++;
	}

	// ignore what was written, but let pollers know something was written
	poll_notify(POLLIN);

	return buflen;
}

ssize_t CDevNode::read(cdev::file_t *handlep, char *buffer, size_t buflen)
{
	PrivData *p = (PrivData *)handlep->f_priv;
	ssize_t chars_read = 0;
	PX4_INFO("read %zu write %zu", p->_read_offset, _write_offset);

	for (size_t i = 0; i < buflen && (p->_read_offset < _write_offset); i++) {
		buffer[i] = _buf[p->_read_offset];
		p->_read_offset++;
		chars_read++;
	}

	return chars_read;
}

CDevExample::~CDevExample()
{
	if (_node) {
		delete _node;
		_node = nullptr;
	}
}

int CDevExample::do_poll(int fd, int timeout, int iterations, int delayms_after_poll)
{
	int pollret, readret;
	int loop_count = 0;
	char readbuf[10];
	px4_pollfd_struct_t fds[1];

	fds[0].fd = fd;
	fds[0].events = POLLIN;
	fds[0].revents = 0;

	bool mustblock = (timeout < 0);

	// Test indefinte blocking poll
	while ((!appState.exitRequested()) && (loop_count < iterations)) {
		pollret = px4_poll(fds, 1, timeout);

		if (pollret < 0) {
			PX4_ERR("Reader: px4_poll failed %d FAIL", pollret);
			goto fail;
		}

		PX4_INFO("Reader: px4_poll returned %d", pollret);

		if (pollret) {
			readret = px4_read(fd, readbuf, 10);

			if (readret != 1) {
				if (mustblock) {
					PX4_ERR("Reader:     read failed %d FAIL", readret);
					goto fail;

				} else {
					PX4_INFO("Reader:     read failed %d FAIL", readret);
				}

			} else {
				readbuf[readret] = '\0';
				PX4_INFO("Reader: px4_poll     returned %d, read '%s' PASS", pollret, readbuf);
			}
		}

		if (delayms_after_poll) {
			px4_usleep(delayms_after_poll * 1000);
		}

		loop_count++;
	}

	return 0;
fail:
	return 1;
}
int CDevExample::main()
{
	appState.setRunning(true);

	_node = new CDevNode();

	if (_node == nullptr) {
		PX4_INFO("Failed to allocate CDevNode");
		return -ENOMEM;
	}

	if (_node->init() != PX4_OK) {
		PX4_INFO("Failed to init CDevNode");
		return 1;
	}

	int fd = px4_open(TESTDEV, PX4_F_RDONLY);

	if (fd < 0) {
		PX4_INFO("Open failed %d", fd);
		return PX4_ERROR;
	}

	// Start a task that will write something in 4 seconds
	(void)px4_task_spawn_cmd("writer",
				 SCHED_DEFAULT,
				 SCHED_PRIORITY_MAX - 6,
				 2000,
				 writer_main,
				 (char *const *)nullptr);

	int ret = 0;

	PX4_INFO("TEST: BLOCKING POLL ---------------");

	if (do_poll(fd, -1, 3, 0)) {
		ret = 1;
		goto fail2;
	}

	PX4_INFO("TEST: ZERO TIMEOUT POLL -----------");

	if (do_poll(fd, 0, 3, 0)) {
		ret = 1;
		goto fail2;
		goto fail2;
	}

	PX4_INFO("TEST: ZERO TIMEOUT POLL -----------");

	if (do_poll(fd, 0, 3, 0)) {
		ret = 1;
		goto fail2;
		goto fail2;
	}

	PX4_INFO("TEST: ZERO TIMEOUT POLL -----------");

	if (do_poll(fd, 0, 3, 0)) {
		ret = 1;
		goto fail2;
	}

	PX4_INFO("TEST: 100ms TIMEOUT POLL -----------");

	if (do_poll(fd, 0, 30, 100)) {
		ret = 1;
		goto fail2;
	}

	PX4_INFO("TEST: 1 SEC TIMOUT POLL ------------");

	if (do_poll(fd, 1000, 3, 0)) {
		ret = 1;
		goto fail2;
	}

	PX4_INFO("TEST: waiting for writer to stop");
fail2:
	g_exit = true;
	px4_close(fd);
	PX4_INFO("TEST: waiting for writer to stop");
	px4_sleep(3);
	return ret;
}
