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

#ifndef NAV_CONTROLLER_OUTPUT_HPP
#define NAV_CONTROLLER_OUTPUT_HPP

#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/tecs_status.h>

class MavlinkStreamNavControllerOutput : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamNavControllerOutput(mavlink); }

	static constexpr const char *get_name_static() { return "NAV_CONTROLLER_OUTPUT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _position_controller_status_sub.advertised() ? MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN +
		       MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamNavControllerOutput(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _position_controller_status_sub{ORB_ID(position_controller_status)};
	uORB::Subscription _tecs_status_sub{ORB_ID(tecs_status)};

	bool send() override
	{
		position_controller_status_s pos_ctrl_status;

		if (_position_controller_status_sub.update(&pos_ctrl_status)) {

			tecs_status_s tecs_status{};
			_tecs_status_sub.copy(&tecs_status);

			mavlink_nav_controller_output_t msg{};

			msg.nav_roll = math::degrees(pos_ctrl_status.nav_roll);
			msg.nav_pitch = math::degrees(pos_ctrl_status.nav_pitch);
			msg.nav_bearing = roundf(math::degrees(pos_ctrl_status.nav_bearing));
			msg.target_bearing = roundf(math::degrees(pos_ctrl_status.target_bearing));
			msg.wp_dist = math::constrain(roundf(pos_ctrl_status.wp_dist), 0.f, (float)UINT16_MAX);
			msg.xtrack_error = pos_ctrl_status.xtrack_error;
			msg.alt_error = tecs_status.altitude_filtered - tecs_status.altitude_sp;
			msg.aspd_error = tecs_status.true_airspeed_filtered - tecs_status.true_airspeed_sp;

			mavlink_msg_nav_controller_output_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // NAV_CONTROLLER_OUTPUT_HPP
