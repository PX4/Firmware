/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef STATUSTEXT_HPP
#define STATUSTEXT_HPP

#include <uORB/topics/mavlink_log.h>

class MavlinkStreamStatustext : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamStatustext(mavlink); }

	static constexpr const char *get_name_static() { return "STATUSTEXT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_STATUSTEXT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _mavlink_log_sub.updated() ? (MAVLINK_MSG_ID_STATUSTEXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamStatustext(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _mavlink_log_sub{ORB_ID(mavlink_log)};
	uint16_t _id{0};
	uint16_t _missed_msg_count{0};

	bool send() override
	{
		if (_mavlink->is_connected() && (_mavlink->get_free_tx_buf() >= get_size())) {

			const unsigned last_generation = _mavlink_log_sub.get_last_generation();

			mavlink_log_s mavlink_log;

			if (_mavlink_log_sub.update(&mavlink_log)) {
				if (_mavlink_log_sub.get_last_generation() != (last_generation + 1)) {
					_missed_msg_count++;
					PX4_ERR("channel %d has missed %d mavlink log messages", _mavlink->get_channel(), _missed_msg_count);
				}

				mavlink_statustext_t msg{};
				const char *text = mavlink_log.text;
				constexpr unsigned max_chunk_size = sizeof(msg.text);
				msg.severity = mavlink_log.severity;
				msg.chunk_seq = 0;
				msg.id = _id++;
				unsigned text_size;

				while ((text_size = strlen(text)) > 0) {
					unsigned chunk_size = math::min(text_size, max_chunk_size);

					if (chunk_size < max_chunk_size) {
						memcpy(&msg.text[0], &text[0], chunk_size);
						// pad with zeros
						memset(&msg.text[0] + chunk_size, 0, max_chunk_size - chunk_size);

					} else {
						memcpy(&msg.text[0], &text[0], chunk_size);
					}

					mavlink_msg_statustext_send_struct(_mavlink->get_channel(), &msg);

					if (text_size <= max_chunk_size) {
						break;

					} else {
						text += max_chunk_size;
					}

					msg.chunk_seq += 1;
				}

				return true;
			}
		}

		return false;
	}
};

#endif // STATUSTEXT_HPP
