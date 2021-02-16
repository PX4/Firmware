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
 * @file protocol_splitter.cpp
 * NuttX Driver to multiplex mavlink and RTPS on a single serial port.
 * Makes sure the two protocols can be read & written simultanously by 2 processes.
 * It will create two devices:
 *    /dev/mavlink
 *    /dev/rtps
 */

#include <lib/cdev/CDev.hpp>
#include <px4_platform_common/sem.hpp>
#include <px4_platform_common/log.h>

#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>
#include <string.h>

class Mavlink2Dev;
class RtpsDev;
class ReadBuffer;

extern "C" __EXPORT int protocol_splitter_main(int argc, char *argv[]);


const char *Sp2HeaderMagic = "SP2";
const int   Sp2HeaderSize  = 8;

struct StaticData {
	Mavlink2Dev *mavlink2;
	RtpsDev *rtps;
	sem_t r_lock;
	sem_t w_lock;
	char device_name[16];
	ReadBuffer *read_buffer;
};

namespace
{
static StaticData *objects = nullptr;
}

class ReadBuffer
{
public:
	int read(int fd);
	void copy(void *dest, size_t pos, size_t n);
	void remove(size_t pos, size_t n);

	uint8_t buffer[512] = {};
	size_t buf_size = 0;

	static const size_t BUFFER_THRESHOLD = sizeof(buffer) * 0.8;
};

int ReadBuffer::read(int fd)
{
	/* Discard whole buffer if it's filled beyond a threshold,
	 * This should prevent buffer being filled by garbage that
	 * no reader (MAVLink or RTPS) can understand.
	 *
	 * TODO: a better approach would be checking if both reader
	 * start understanding messages beyond a certain buffer size,
	 * meaning that everything before is garbage.
	 */
	if (buf_size > BUFFER_THRESHOLD) {
		buf_size = 0;
	}

	int r = ::read(fd, buffer + buf_size, sizeof(buffer) - buf_size);

	if (r < 0) {
		return r;
	}

	buf_size += r;

	return r;
}

void ReadBuffer::copy(void *dest, size_t pos, size_t n)
{
	ASSERT(pos < buf_size);
	ASSERT(pos + n <= buf_size);

	if (dest) {
		memmove(dest, buffer + pos, n);        // send desired data
	}
}

void ReadBuffer::remove(size_t pos, size_t n)
{
	ASSERT(pos < buf_size);
	ASSERT(pos + n <= buf_size);

	memmove(buffer + pos, buffer + (pos + n), sizeof(buffer) - pos - n);
	buf_size -= n;
}

class DevCommon : public cdev::CDev
{
public:
	DevCommon(const char *device_path);
	virtual ~DevCommon();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int	open(file *filp);
	virtual int	close(file *filp);

	enum Operation {Read, Write};

protected:

	/*
	struct Sp2Header {
		char magic[3];
		uint8_t type;
		uint16_t payload_len;
		uint16_t reserved (align)
	}
	*/

	enum MessageType {Mavlink = 0, Rtps};

	uint8_t _header[8] = {};

	virtual pollevent_t poll_state(struct file *filp);


	void lock(enum Operation op)
	{
		sem_t *this_lock = op == Read ? &objects->r_lock : &objects->w_lock;

		while (sem_wait(this_lock) != 0) {
			/* The only case that an error should occur here is if
			 * the wait was awakened by a signal.
			 */
			ASSERT(get_errno() == EINTR);
		}
	}

	void unlock(enum Operation op)
	{
		sem_t *this_lock = op == Read ? &objects->r_lock : &objects->w_lock;
		sem_post(this_lock);
	}

	int _fd = -1;

	uint16_t _packet_len;
	enum class ParserState : uint8_t {
		Idle = 0,
		GotLength
	};
	ParserState _parser_state = ParserState::Idle;

	bool _had_data = false; ///< whether poll() returned available data

private:
};

DevCommon::DevCommon(const char *device_path)
	: CDev(device_path)
{
}

DevCommon::~DevCommon()
{
	if (_fd >= 0) {
		::close(_fd);
	}
}

int DevCommon::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	//pretend we have enough space left to write, so mavlink will not drop data and throw off
	//our parsing state
	if (cmd == FIONSPACE) {
		*(int *)arg = 1024;
		return 0;
	}

	return ::ioctl(_fd, cmd, arg);
}

int DevCommon::open(file *filp)
{
	_fd = ::open(objects->device_name, O_RDWR | O_NOCTTY);
	CDev::open(filp);
	return _fd >= 0 ? 0 : -1;
}

int DevCommon::close(file *filp)
{
	//int ret = ::close(_fd); // FIXME: calling this results in a dead-lock, because DevCommon::close()
	// is called from within another close(), and NuttX seems to hold a semaphore at this point
	_fd = -1;
	CDev::close(filp);
	return 0;
}

pollevent_t DevCommon::poll_state(struct file *filp)
{
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	/* Here we should just check the poll state (which is called before an actual poll waiting).
	 * Instead we poll on the fd with some timeout, and then pretend that there is data.
	 * This will let the calling poll return immediately (there's still no busy loop since
	 * we do actually poll here).
	 * We do this because there is no simple way with the given interface to poll on
	 * the _fd in here or by overriding some other method.
	 */

	int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
	_had_data = ret > 0 && (fds[0].revents & POLLIN);

	return POLLIN;
}

class Mavlink2Dev : public DevCommon
{
public:
	Mavlink2Dev(ReadBuffer *_read_buffer);
	virtual ~Mavlink2Dev() {}

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);

protected:
	ReadBuffer *_read_buffer;
	size_t _remaining_partial = 0;
	size_t _partial_start = 0;
	uint8_t _partial_buffer[512] = {};
};

Mavlink2Dev::Mavlink2Dev(ReadBuffer *read_buffer)
	: DevCommon("/dev/mavlink")
	, _read_buffer{read_buffer}
{
	memcpy(_header, Sp2HeaderMagic, 3);
	_header[3] = MessageType::Mavlink;
	memset(&_header[4], 0, 4);
}

ssize_t Mavlink2Dev::read(struct file *filp, char *buffer, size_t buflen)
{
	int i, ret;
	uint16_t packet_len, payload_len;

	/* last reading was partial (i.e., buffer didn't fit whole message),
	 * so now we'll just send remaining bytes */
	if (_remaining_partial > 0) {
		size_t len = _remaining_partial;

		if (buflen < len) {
			len = buflen;
		}

		memmove(buffer, _partial_buffer + _partial_start, len);
		_partial_start += len;
		_remaining_partial -= len;

		if (_remaining_partial == 0) {
			_partial_start = 0;
		}

		return len;
	}

	if (!_had_data) {
		return 0;
	}

	lock(Read);
	ret = _read_buffer->read(_fd);

	if (ret < 0) {
		goto end;
	}

	ret = 0;

	if (_read_buffer->buf_size < Sp2HeaderSize) {
		goto end;
	}

	// Search for a mavlink packet on buffer to send it
	i = 0;

	while ((unsigned)i < (_read_buffer->buf_size - Sp2HeaderSize) &&
	       (_read_buffer->buffer[i] != 'S'
		|| _read_buffer->buffer[i + 1] != 'P'
		|| _read_buffer->buffer[i + 2] != '2'
		|| _read_buffer->buffer[i + 3] != (uint8_t) MessageType::Mavlink)) {
		i++;
	}

	// We need at least the first six bytes to get packet len
	if ((unsigned)i >= _read_buffer->buf_size - Sp2HeaderSize) {
		goto end;
	}

	payload_len = ((uint16_t)_read_buffer->buffer[i + 4] << 8) | _read_buffer->buffer[i + 5];
	packet_len = payload_len + Sp2HeaderSize;

	// packet is bigger than what we've read, better luck next time
	if ((unsigned)i + packet_len > _read_buffer->buf_size) {
		goto end;
	}

	/* if buffer doesn't fit message, send what's possible and copy remaining
	 * data into a temporary buffer on this class */
	if (payload_len > buflen) {
		_read_buffer->copy(buffer, i + Sp2HeaderSize, buflen);
		_read_buffer->copy(_partial_buffer, i + Sp2HeaderSize + buflen, payload_len - buflen);
		_read_buffer->remove(i, packet_len);
		_remaining_partial = payload_len - buflen;
		ret = buflen;
		goto end;
	}

	_read_buffer->copy(buffer, i + Sp2HeaderSize, payload_len);
	_read_buffer->remove(i, packet_len);

	ret = payload_len;

end:
	unlock(Read);
	return ret;
}

ssize_t Mavlink2Dev::write(struct file *filp, const char *buffer, size_t buflen)
{
	/*
	 * we need to look into the data to make sure the output is locked for the duration
	 * of a whole packet.
	 * assumptions:
	 * - packet header is written all at once (or at least it contains the payload length)
	 * - a single write call does not contain multiple (or parts of multiple) packets
	 */
	ssize_t ret = 0;

	switch (_parser_state) {
	case ParserState::Idle:
		ASSERT(buflen >= 3);

		if ((unsigned char)buffer[0] == 253) {
			uint8_t payload_len = buffer[1];
			uint8_t incompat_flags = buffer[2];
			_packet_len = payload_len + 12;

			if (incompat_flags & 0x1) { //signing
				_packet_len += 13;
			}

			_parser_state = ParserState::GotLength;
			lock(Write);

		} else if ((unsigned char)buffer[0] == 254) { // mavlink 1
			uint8_t payload_len = buffer[1];
			_packet_len = payload_len + 8;

			_parser_state = ParserState::GotLength;
			lock(Write);

		} else {
			PX4_ERR("parser error");
			return 0;
		}

	/* FALLTHROUGH */

	case ParserState::GotLength: {
			_packet_len -= buflen;
			int buf_free;
			::ioctl(_fd, FIONSPACE, (unsigned long)&buf_free);

			if (buf_free < (int)buflen) {
				//let write fail, to let mavlink know the buffer would overflow
				//(this is because in the ioctl we pretend there is always enough space)
				ret = -1;

			} else {
				_header[4] = (uint8_t)((buflen >> 8) & 0xff);
				_header[5] = (uint8_t)(buflen & 0xff);
				::write(_fd, _header, 8);
				ret = ::write(_fd, buffer, buflen);
			}

			if (_packet_len == 0) {
				unlock(Write);
				_parser_state = ParserState::Idle;
			}
		}

		break;
	}

	return ret;
}

class RtpsDev : public DevCommon
{
public:
	RtpsDev(ReadBuffer *_read_buffer);
	virtual ~RtpsDev() {}

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);

protected:
	ReadBuffer *_read_buffer;

	static const uint8_t HEADER_SIZE = 9;
};

RtpsDev::RtpsDev(ReadBuffer *read_buffer)
	: DevCommon("/dev/rtps")
	, _read_buffer{read_buffer}
{
	memcpy(_header, Sp2HeaderMagic, 3);
	_header[3] = MessageType::Rtps;
	memset(&_header[4], 0, 4);

}

ssize_t RtpsDev::read(struct file *filp, char *buffer, size_t buflen)
{
	int i, ret;
	uint16_t packet_len, payload_len;

	if (!_had_data) {
		return 0;
	}

	lock(Read);
	ret = _read_buffer->read(_fd);

	if (ret < 0) {
		goto end;
	}

	ret = 0;

	if (_read_buffer->buf_size < Sp2HeaderSize) {
		goto end;
	}

	// Search for a rtps packet on buffer to send it
	i = 0;

	while ((unsigned)i < (_read_buffer->buf_size - Sp2HeaderSize) &&
	       (_read_buffer->buffer[i] != 'S'
		|| _read_buffer->buffer[i + 1] != 'P'
		|| _read_buffer->buffer[i + 2] != '2'
		|| _read_buffer->buffer[i + 3] != (uint8_t) MessageType::Rtps)) {
		i++;
	}

	// We need at least the first six bytes to get packet len
	if ((unsigned)i >= _read_buffer->buf_size - Sp2HeaderSize) {
		goto end;
	}

	payload_len = ((uint16_t)_read_buffer->buffer[i + 4] << 8) | _read_buffer->buffer[i + 5];
	packet_len = payload_len + Sp2HeaderSize;

	// packet is bigger than what we've read, better luck next time
	if ((unsigned)i + packet_len > _read_buffer->buf_size) {
		goto end;
	}

	// buffer should be big enough to hold a rtps packet
	if (packet_len > buflen) {
		ret = -EMSGSIZE;
		goto end;
	}

	_read_buffer->copy(buffer, i + Sp2HeaderSize, payload_len);
	_read_buffer->remove(i, packet_len);
	ret = payload_len;

end:
	unlock(Read);
	return ret;
}

ssize_t RtpsDev::write(struct file *filp, const char *buffer, size_t buflen)
{
	/*
	 * we need to look into the data to make sure the output is locked for the duration
	 * of a whole packet.
	 * assumptions:
	 * - packet header is written all at once (or at least it contains the payload length)
	 * - a single write call does not contain multiple (or parts of multiple) packets
	 */
	ssize_t ret = 0;
	uint16_t payload_len;

	switch (_parser_state) {
	case ParserState::Idle:
		ASSERT(buflen >= HEADER_SIZE);

		if (memcmp(buffer, ">>>", 3) != 0) {
			PX4_ERR("parser error");
			return 0;
		}

		payload_len = ((uint16_t)buffer[5] << 8) | buffer[6];
		_packet_len = payload_len + HEADER_SIZE;
		_parser_state = ParserState::GotLength;
		lock(Write);

	/* FALLTHROUGH */

	case ParserState::GotLength: {
			_packet_len -= buflen;
			int buf_free;
			::ioctl(_fd, FIONSPACE, (unsigned long)&buf_free);

			// TODO should I care about this for rtps?
			if ((unsigned)buf_free < buflen) {
				//let write fail, to let rtps know the buffer would overflow
				//(this is because in the ioctl we pretend there is always enough space)
				ret = -1;

			} else {
				_header[4] = (uint8_t)((buflen >> 8) & 0xff);
				_header[5] = (uint8_t)(buflen & 0xff);
				::write(_fd, _header, 8);
				ret = ::write(_fd, buffer, buflen);
			}

			if (_packet_len == 0) {
				unlock(Write);
				_parser_state = ParserState::Idle;
			}
		}

		break;
	}

	return ret;
}

int protocol_splitter_main(int argc, char *argv[])
{
	if (argc < 2) {
		goto out;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (objects) {
			PX4_ERR("already running");
			return 1;
		}

		if (argc != 3) {
			goto out;
		}

		objects = new StaticData();

		if (!objects) {
			PX4_ERR("alloc failed");
			return -1;
		}

		strncpy(objects->device_name, argv[2], sizeof(objects->device_name));
		sem_init(&objects->r_lock, 1, 1);
		sem_init(&objects->w_lock, 1, 1);
		objects->read_buffer = new ReadBuffer();
		objects->mavlink2 = new Mavlink2Dev(objects->read_buffer);
		objects->rtps = new RtpsDev(objects->read_buffer);

		if (!objects->mavlink2 || !objects->rtps) {
			delete objects->mavlink2;
			delete objects->rtps;
			delete objects->read_buffer;
			sem_destroy(&objects->r_lock);
			sem_destroy(&objects->w_lock);
			delete objects;
			objects = nullptr;
			PX4_ERR("alloc failed");
			return -1;

		} else {
			objects->mavlink2->init();
			objects->rtps->init();
		}
	}

	if (!strcmp(argv[1], "stop")) {
		if (objects) {
			delete objects->mavlink2;
			delete objects->rtps;
			delete objects->read_buffer;
			sem_destroy(&objects->r_lock);
			sem_destroy(&objects->w_lock);
			delete objects;
			objects = nullptr;
		}
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		if (objects) {
			PX4_INFO("running");

		} else {
			PX4_INFO("not running");
		}
	}

	return 0;

out:
	PX4_ERR("unrecognized command, try 'start <device>', 'stop', 'status'");
	return 1;
}
