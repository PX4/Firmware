/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef COMMAND_LONG_HPP
#define COMMAND_LONG_HPP

#include <uORB/topics/vehicle_command.h>

class MavlinkStreamCommandLong : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCommandLong(mavlink); }

	static constexpr const char *get_name_static() { return "COMMAND_LONG"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_COMMAND_LONG; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return 0; // commands stream is not regular and not predictable
	}

private:
	explicit MavlinkStreamCommandLong(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

	bool send() override
	{
		bool sent = false;

		static constexpr size_t COMMAND_LONG_SIZE = MAVLINK_MSG_ID_COMMAND_LONG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

		while ((_mavlink->get_free_tx_buf() >= COMMAND_LONG_SIZE) && _vehicle_command_sub.updated()) {

			const unsigned last_generation = _vehicle_command_sub.get_last_generation();
			vehicle_command_s cmd;

			if (_vehicle_command_sub.update(&cmd)) {
				if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("COMMAND_LONG vehicle_command lost, generation %d -> %d", last_generation,
						_vehicle_command_sub.get_last_generation());
				}

				if (!cmd.from_external && cmd.command < vehicle_command_s::VEHICLE_CMD_PX4_INTERNAL_START) {
					PX4_DEBUG("sending command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);

					MavlinkCommandSender::instance().handle_vehicle_command(cmd, _mavlink->get_channel());
					sent = true;

				} else {
					PX4_DEBUG("not forwarding command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);
				}
			}
		}

		MavlinkCommandSender::instance().check_timeout(_mavlink->get_channel());

		return sent;
	}
};

#endif // COMMAND_LONG_HPP
