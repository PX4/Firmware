/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file mavlink_messages.h
 * MAVLink 1.0 message formatters definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef MAVLINK_MESSAGES_H_
#define MAVLINK_MESSAGES_H_

#include "mavlink_stream.h"

#include <commander/px4_custom_mode.h>

class StreamListItem
{

public:
	MavlinkStream *(*new_instance)(Mavlink *mavlink);
	const char *name;
	uint16_t id;

	StreamListItem(MavlinkStream * (*inst)(Mavlink *mavlink), const char *_name, uint16_t _id) :
		new_instance(inst),
		name(_name),
		id(_id) {}

	const char *get_name() const { return name; }
	uint16_t get_id() const { return id; }
};

template <class T>
static StreamListItem create_stream_list_item()
{
	return StreamListItem(&T::new_instance, T::get_name_static(), T::get_id_static());
}

const char *get_stream_name(const uint16_t msg_id);

MavlinkStream *create_mavlink_stream(const char *stream_name, Mavlink *mavlink);

MavlinkStream *create_mavlink_stream(const uint16_t msg_id, Mavlink *mavlink);

union px4_custom_mode get_px4_custom_mode(uint8_t nav_state);

#endif /* MAVLINK_MESSAGES_H_ */
