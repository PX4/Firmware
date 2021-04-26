/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#ifndef GPS_GLOBAL_ORIGIN_HPP
#define GPS_GLOBAL_ORIGIN_HPP

#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamGpsGlobalOrigin : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGpsGlobalOrigin(mavlink); }

	static constexpr const char *get_name_static() { return "GPS_GLOBAL_ORIGIN"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_local_position_sub.advertised()) {
			return MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

	bool request_message(float param2, float param3, float param4, float param5, float param6, float param7) override
	{
		if (_valid) {
			_force_next_send = true;
			return true;
		}

		return false;
	}

private:
	explicit MavlinkStreamGpsGlobalOrigin(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	uint64_t _ref_timestamp{0};
	double _ref_lat{static_cast<double>(NAN)};
	double _ref_lon{static_cast<double>(NAN)};
	float _ref_alt{NAN};

	bool _valid{false};
	bool _force_next_send{true};

	bool send() override
	{
		vehicle_local_position_s vehicle_local_position;

		if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
			if (vehicle_local_position.xy_global && vehicle_local_position.z_global) {

				static constexpr double LLA_MIN_DIFF = 0.0000001; // ~11.132 mm at the equator

				if (_force_next_send
				    || (_ref_timestamp != vehicle_local_position.ref_timestamp)
				    || (fabs(_ref_lat - vehicle_local_position.ref_lat) > LLA_MIN_DIFF)
				    || (fabs(_ref_lon - vehicle_local_position.ref_lon) > LLA_MIN_DIFF)
				    || (fabsf(_ref_alt - vehicle_local_position.ref_alt) > 0.001f)) {

					mavlink_gps_global_origin_t msg{};
					msg.latitude = round(vehicle_local_position.ref_lat * 1e7); // double degree -> int32 degreeE7
					msg.longitude = round(vehicle_local_position.ref_lon * 1e7); // double degree -> int32 degreeE7
					msg.altitude = roundf(vehicle_local_position.ref_alt * 1e3f); // float m -> int32 mm
					msg.time_usec = vehicle_local_position.ref_timestamp; // int64 time since system boot
					mavlink_msg_gps_global_origin_send_struct(_mavlink->get_channel(), &msg);

					_ref_timestamp = vehicle_local_position.ref_timestamp;
					_ref_lat       = vehicle_local_position.ref_lat;
					_ref_lon       = vehicle_local_position.ref_lon;
					_ref_alt       = vehicle_local_position.ref_alt;

					_force_next_send = false;
					_valid = true;

					return true;
				}
			}
		}

		return false;
	}
};

#endif // GPS_GLOBAL_ORIGIN_HPP
