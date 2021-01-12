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

#ifndef ESC_STATUS_HPP
#define ESC_STATUS_HPP

#include <uORB/topics/esc_status.h>

class MavlinkStreamESCStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamESCStatus(mavlink); }

	static constexpr const char *get_name_static() { return "ESC_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ESC_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_batch = MAVLINK_MSG_ID_ESC_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return _esc_status_sub.advertised() ? size_per_batch * _number_of_batches : 0;
	}

private:
	explicit MavlinkStreamESCStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uint8_t _number_of_batches{0};

	bool send() override
	{
		static constexpr uint8_t batch_size = MAVLINK_MSG_ESC_STATUS_FIELD_RPM_LEN;
		esc_status_s esc_status;

		if (_esc_status_sub.update(&esc_status)) {
			mavlink_esc_status_t msg{};

			msg.time_usec = esc_status.timestamp;

			// Ceil value of integer division. For 1-4 esc => 1 batch, 5-8 esc => 2 batches etc
			_number_of_batches = ceilf((float)esc_status.esc_count / batch_size);

			for (int batch_number = 0; batch_number < _number_of_batches; batch_number++) {
				msg.index = batch_number * batch_size;

				for (int esc_index = 0; esc_index < batch_size ; esc_index++) {
					msg.rpm[esc_index] = esc_status.esc[esc_index].esc_rpm;
					msg.voltage[esc_index] = esc_status.esc[esc_index].esc_voltage;
					msg.current[esc_index] = esc_status.esc[esc_index].esc_current;
				}

				mavlink_msg_esc_status_send_struct(_mavlink->get_channel(), &msg);
			}

			return true;
		}

		return false;
	}
};

#endif // ESC_STATUS_HPP
