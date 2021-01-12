/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_stream.h
 * Mavlink messages stream definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef MAVLINK_STREAM_H_
#define MAVLINK_STREAM_H_

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <containers/List.hpp>

class Mavlink;

class MavlinkStream : public ListNode<MavlinkStream *>
{

public:

	MavlinkStream(Mavlink *mavlink);
	virtual ~MavlinkStream() = default;

	// no copy, assignment, move, move assignment
	MavlinkStream(const MavlinkStream &) = delete;
	MavlinkStream &operator=(const MavlinkStream &) = delete;
	MavlinkStream(MavlinkStream &&) = delete;
	MavlinkStream &operator=(MavlinkStream &&) = delete;

	/**
	 * Get the interval
	 *
	 * @param interval the interval in microseconds (us) between messages
	 */
	void set_interval(const int interval) { _interval = interval; }

	/**
	 * Get the interval
	 *
	 * @return the inveral in microseconds (us) between messages
	 */
	int get_interval() { return _interval; }

	/**
	 * @return 0 if updated / sent, -1 if unchanged
	 */
	int update(const hrt_abstime &t);
	virtual const char *get_name() const = 0;
	virtual uint16_t get_id() = 0;

	/**
	 * @return true if steam rate shouldn't be adjusted
	 */
	virtual bool const_rate() { return false; }

	/**
	 * Get maximal total messages size on update
	 */
	virtual unsigned get_size() = 0;

	/**
	 * This function is called in response to a MAV_CMD_REQUEST_MESSAGE command.
	 */
	virtual bool request_message(float param2 = 0.0, float param3 = 0.0, float param4 = 0.0,
				     float param5 = 0.0, float param6 = 0.0, float param7 = 0.0)
	{
		return send();
	}

	/**
	 * Get the average message size
	 *
	 * For a normal stream this equals the message size,
	 * for something like a parameter or mission message
	 * this equals usually zero, as no bandwidth
	 * needs to be reserved
	 */
	virtual unsigned get_size_avg() { return get_size(); }

	/**
	 * @return true if the first message of this stream has been sent
	 */
	bool first_message_sent() const { return _first_message_sent; }

	/**
	 * Reset the time of last sent to 0. Can be used if a message over this
	 * stream needs to be sent immediately.
	 */
	void reset_last_sent() { _last_sent = 0; }

protected:
	Mavlink      *const _mavlink;
	int _interval{1000000};		///< if set to negative value = unlimited rate

	virtual bool send() = 0;

	/**
	 * Function to collect/update data for the streams at a high rate independant of
	 * actual stream rate.
	 *
	 * This function is called at every iteration of the mavlink module.
	 */
	virtual void update_data() { }

private:
	hrt_abstime _last_sent{0};
	bool _first_message_sent{false};
};


#endif /* MAVLINK_STREAM_H_ */
