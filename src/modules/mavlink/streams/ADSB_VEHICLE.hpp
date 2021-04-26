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

#ifndef ADSB_VEHICLE_HPP
#define ADSB_VEHICLE_HPP

#include <uORB/topics/transponder_report.h>

class MavlinkStreamADSBVehicle : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamADSBVehicle(mavlink); }

	static constexpr const char *get_name_static() { return "ADSB_VEHICLE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ADSB_VEHICLE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return _transponder_report_sub.advertised() ? MAVLINK_MSG_ID_ADSB_VEHICLE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamADSBVehicle(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _transponder_report_sub{ORB_ID(transponder_report)};

	bool send() override
	{
		bool sent = false;

		transponder_report_s pos;

		while ((_mavlink->get_free_tx_buf() >= get_size()) && _transponder_report_sub.update(&pos)) {

			if (!(pos.flags & transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE)) {
				continue;
			}

			mavlink_adsb_vehicle_t msg{};
			msg.ICAO_address = pos.icao_address;
			msg.lat = pos.lat * 1e7;
			msg.lon = pos.lon * 1e7;
			msg.altitude_type = pos.altitude_type;
			msg.altitude = pos.altitude * 1e3f;
			msg.heading = (pos.heading + M_PI_F) / M_PI_F * 180.0f * 100.0f;
			msg.hor_velocity = pos.hor_velocity * 100.0f;
			msg.ver_velocity = pos.ver_velocity * 100.0f;
			memcpy(&msg.callsign[0], &pos.callsign[0], sizeof(msg.callsign));
			msg.emitter_type = pos.emitter_type;
			msg.tslc = pos.tslc;
			msg.squawk = pos.squawk;

			msg.flags = 0;

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS) { msg.flags |= ADSB_FLAGS_VALID_COORDS; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE) { msg.flags |= ADSB_FLAGS_VALID_ALTITUDE; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING) { msg.flags |= ADSB_FLAGS_VALID_HEADING; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY) { msg.flags |= ADSB_FLAGS_VALID_VELOCITY; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) { msg.flags |= ADSB_FLAGS_VALID_CALLSIGN; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK) { msg.flags |= ADSB_FLAGS_VALID_SQUAWK; }

			mavlink_msg_adsb_vehicle_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

#endif // ADSB_VEHICLE_HPP
