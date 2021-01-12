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

#pragma once

#include <stdint.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/ulog_stream.h>
#include <uORB/topics/ulog_stream_ack.h>

namespace px4
{
namespace logger
{

/**
 * @class LogWriterMavlink
 * Writes logging data to uORB, and then sent via mavlink
 */
class LogWriterMavlink
{
public:
	LogWriterMavlink();
	~LogWriterMavlink();

	bool init();

	void start_log();

	void stop_log();

	bool is_started() const { return _is_started; }

	/** @see LogWriter::write_message() */
	int write_message(void *ptr, size_t size);

	void set_need_reliable_transfer(bool need_reliable);

	bool need_reliable_transfer() const
	{
		return _need_reliable_transfer;
	}

private:

	/** publish message, wait for ack if needed & reset message */
	int publish_message();

	ulog_stream_s _ulog_stream_data{};
	uORB::Publication<ulog_stream_s> _ulog_stream_pub{ORB_ID(ulog_stream)};
	int _ulog_stream_ack_sub{-1};
	bool _need_reliable_transfer{false};
	bool _is_started{false};
};

}
}
