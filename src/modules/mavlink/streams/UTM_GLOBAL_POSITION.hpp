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

#ifndef UTM_GLOBAL_POSITION_HPP
#define UTM_GLOBAL_POSITION_HPP

#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>

class MavlinkStreamUTMGlobalPosition : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamUTMGlobalPosition(mavlink); }

	static constexpr const char *get_name_static() { return "UTM_GLOBAL_POSITION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_UTM_GLOBAL_POSITION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return _global_pos_sub.advertised() ? MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamUTMGlobalPosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};

	bool send() override
	{
		vehicle_global_position_s global_pos;

		if (_global_pos_sub.update(&global_pos)) {
			mavlink_utm_global_position_t msg{};

			// Compute Unix epoch and set time field
			timespec tv;
			px4_clock_gettime(CLOCK_REALTIME, &tv);
			uint64_t unix_epoch = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

			// If the time is before 2001-01-01, it's probably the default 2000
			if (unix_epoch > 978307200000000) {
				msg.time = unix_epoch;
				msg.flags |= UTM_DATA_AVAIL_FLAGS_TIME_VALID;
			}

#ifndef BOARD_HAS_NO_UUID
			px4_guid_t px4_guid;
			board_get_px4_guid(px4_guid);
			static_assert(sizeof(px4_guid_t) == sizeof(msg.uas_id), "GUID byte length mismatch");
			memcpy(&msg.uas_id, &px4_guid, sizeof(msg.uas_id));
			msg.flags |= UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE;
#else
			// TODO Fill ID with something reasonable
			memset(&msg.uas_id[0], 0, sizeof(msg.uas_id));
#endif /* BOARD_HAS_NO_UUID */

			// Handle global position
			msg.lat = global_pos.lat * 1e7;
			msg.lon = global_pos.lon * 1e7;
			msg.alt = global_pos.alt_ellipsoid * 1000.f;

			msg.h_acc = global_pos.eph * 1000.f;
			msg.v_acc = global_pos.epv * 1000.f;

			msg.flags |= UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE;
			msg.flags |= UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE;

			// Handle local position
			vehicle_local_position_s local_pos;

			if (_local_pos_sub.copy(&local_pos)) {
				float evh = 0.f;
				float evv = 0.f;

				if (local_pos.v_xy_valid) {
					msg.vx = local_pos.vx * 100.f;
					msg.vy = local_pos.vy * 100.f;
					evh = local_pos.evh;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE;
				}

				if (local_pos.v_z_valid) {
					msg.vz = local_pos.vz * 100.f;
					evv = local_pos.evv;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE;
				}

				msg.vel_acc = sqrtf(evh * evh + evv * evv) * 100.f;

				if (local_pos.dist_bottom_valid) {
					msg.relative_alt = local_pos.dist_bottom * 1000.f;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE;
				}
			}

			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			bool vehicle_in_auto_mode = (vehicle_status.timestamp > 0)
						    && (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
							|| vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

			// Handle next waypoint if it is valid
			position_setpoint_triplet_s position_setpoint_triplet;

			if (vehicle_in_auto_mode && _position_setpoint_triplet_sub.copy(&position_setpoint_triplet)) {
				if (position_setpoint_triplet.current.valid) {
					msg.next_lat = position_setpoint_triplet.current.lat * 1e7;
					msg.next_lon = position_setpoint_triplet.current.lon * 1e7;
					// HACK We assume that the offset between AMSL and WGS84 is constant between the current
					// vehicle position and the the target waypoint.
					msg.next_alt = (position_setpoint_triplet.current.alt + (global_pos.alt_ellipsoid - global_pos.alt)) * 1000.f;
					msg.flags |= UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE;
				}
			}

			// Handle flight state
			vehicle_land_detected_s land_detected{};
			_land_detected_sub.copy(&land_detected);

			if (vehicle_status.timestamp > 0 && land_detected.timestamp > 0
			    && vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				if (land_detected.landed) {
					msg.flight_state |= UTM_FLIGHT_STATE_GROUND;

				} else {
					msg.flight_state |= UTM_FLIGHT_STATE_AIRBORNE;
				}

			} else {
				msg.flight_state |= UTM_FLIGHT_STATE_UNKNOWN;
			}

			msg.update_rate = 0; // Data driven mode

			mavlink_msg_utm_global_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // UTM_GLOBAL_POSITION_HPP
