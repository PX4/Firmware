/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_mission.cpp
 * MAVLink mission manager implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#include "mavlink_mission.h"
#include "mavlink_main.h"

#include <lib/ecl/geo/geo.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <navigator/navigation.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

using matrix::wrap_2pi;

dm_item_t MavlinkMissionManager::_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
bool MavlinkMissionManager::_dataman_init = false;
uint16_t MavlinkMissionManager::_count[3] = { 0, 0, 0 };
int32_t MavlinkMissionManager::_current_seq = 0;
bool MavlinkMissionManager::_transfer_in_progress = false;
constexpr uint16_t MavlinkMissionManager::MAX_COUNT[];
uint16_t MavlinkMissionManager::_geofence_update_counter = 0;
uint16_t MavlinkMissionManager::_safepoint_update_counter = 0;


#define CHECK_SYSID_COMPID_MISSION(_msg)		(_msg.target_system == mavlink_system.sysid && \
		((_msg.target_component == mavlink_system.compid) || \
		 (_msg.target_component == MAV_COMP_ID_MISSIONPLANNER) || \
		 (_msg.target_component == MAV_COMP_ID_ALL)))

MavlinkMissionManager::MavlinkMissionManager(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	init_offboard_mission();
}

void
MavlinkMissionManager::init_offboard_mission()
{
	if (!_dataman_init) {
		_dataman_init = true;

		/* lock MISSION_STATE item */
		int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

		if (dm_lock_ret != 0) {
			PX4_ERR("DM_KEY_MISSION_STATE lock failed");
		}

		mission_s mission_state;
		int ret = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

		/* unlock MISSION_STATE item */
		if (dm_lock_ret == 0) {
			dm_unlock(DM_KEY_MISSION_STATE);
		}

		if (ret > 0) {
			_dataman_id = (dm_item_t)mission_state.dataman_id;
			_count[MAV_MISSION_TYPE_MISSION] = mission_state.count;
			_current_seq = mission_state.current_seq;

		} else if (ret < 0) {
			PX4_ERR("offboard mission init failed (%i)", ret);
		}

		load_geofence_stats();

		load_safepoint_stats();
	}

	_my_dataman_id = _dataman_id;
}

int
MavlinkMissionManager::load_geofence_stats()
{
	mission_stats_entry_s stats;
	// initialize fence points count
	int ret = dm_read(DM_KEY_FENCE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (ret == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_FENCE] = stats.num_items;
		_geofence_update_counter = stats.update_counter;
	}

	return ret;
}

int
MavlinkMissionManager::load_safepoint_stats()
{
	mission_stats_entry_s stats;
	// initialize safe points count
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (ret == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_RALLY] = stats.num_items;
	}

	return ret;
}

/**
 * Publish mission topic to notify navigator about changes.
 */
int
MavlinkMissionManager::update_active_mission(dm_item_t dataman_id, uint16_t count, int32_t seq)
{
	// We want to make sure the whole struct is initialized including padding before getting written by dataman.
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.dataman_id = dataman_id;
	mission.count = count;
	mission.current_seq = seq;

	/* update mission state in dataman */

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM_KEY_MISSION_STATE lock failed");
	}

	int res = dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));

	/* unlock MISSION_STATE item */
	if (dm_lock_ret == 0) {
		dm_unlock(DM_KEY_MISSION_STATE);
	}

	if (res == sizeof(mission_s)) {
		/* update active mission state */
		_dataman_id = dataman_id;
		_count[MAV_MISSION_TYPE_MISSION] = count;
		_current_seq = seq;
		_my_dataman_id = _dataman_id;

		/* mission state saved successfully, publish offboard_mission topic */
		_offboard_mission_pub.publish(mission);

		return PX4_OK;

	} else {
		PX4_ERR("WPM: can't save mission state");

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
		}

		return PX4_ERROR;
	}
}
int
MavlinkMissionManager::update_geofence_count(unsigned count)
{
	mission_stats_entry_s stats;
	stats.num_items = count;
	stats.update_counter = ++_geofence_update_counter; // this makes sure navigator will reload the fence data

	/* update stats in dataman */
	int res = dm_write(DM_KEY_FENCE_POINTS, 0, DM_PERSIST_POWER_ON_RESET, &stats, sizeof(mission_stats_entry_s));

	if (res == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_FENCE] = count;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
		}

		return PX4_ERROR;
	}

	return PX4_OK;
}

int
MavlinkMissionManager::update_safepoint_count(unsigned count)
{
	mission_stats_entry_s stats;
	stats.num_items = count;
	stats.update_counter = ++_safepoint_update_counter;

	/* update stats in dataman */
	int res = dm_write(DM_KEY_SAFE_POINTS, 0, DM_PERSIST_POWER_ON_RESET, &stats, sizeof(mission_stats_entry_s));

	if (res == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_RALLY] = count;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
		}

		return PX4_ERROR;
	}

	return PX4_OK;
}

void
MavlinkMissionManager::send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_mission_ack_t wpa{};

	wpa.target_system = sysid;
	wpa.target_component = compid;
	wpa.type = type;
	wpa.mission_type = _mission_type;

	mavlink_msg_mission_ack_send_struct(_mavlink->get_channel(), &wpa);

	PX4_DEBUG("WPM: Send MISSION_ACK type %u to ID %u", wpa.type, wpa.target_system);
}

void
MavlinkMissionManager::send_mission_current(int32_t seq)
{
	int32_t item_count = _count[MAV_MISSION_TYPE_MISSION];

	if (seq < item_count) {
		mavlink_mission_current_t wpc{};
		wpc.seq = seq;
		mavlink_msg_mission_current_send_struct(_mavlink->get_channel(), &wpc);

	} else if (seq <= 0 && item_count == 0) {
		/* don't broadcast if no WPs */

	} else {
		PX4_DEBUG("WPM: Send MISSION_CURRENT ERROR: seq %d out of bounds", seq);

		_mavlink->send_statustext_critical("ERROR: wp index out of bounds");
	}
}

void
MavlinkMissionManager::send_mission_count(uint8_t sysid, uint8_t compid, uint16_t count, MAV_MISSION_TYPE mission_type)
{
	_time_last_sent = hrt_absolute_time();

	mavlink_mission_count_t wpc{};

	wpc.target_system = sysid;
	wpc.target_component = compid;
	wpc.count = count;
	wpc.mission_type = mission_type;

	mavlink_msg_mission_count_send_struct(_mavlink->get_channel(), &wpc);

	PX4_DEBUG("WPM: Send MISSION_COUNT %u to ID %u, mission type=%i", wpc.count, wpc.target_system, mission_type);
}

void
MavlinkMissionManager::send_mission_item(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	mission_item_s mission_item{};
	int read_result = 0;

	switch (_mission_type) {

	case MAV_MISSION_TYPE_MISSION: {
			read_result = dm_read(_dataman_id, seq, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s);
		}
		break;

	case MAV_MISSION_TYPE_FENCE: { // Read a geofence point
			mission_fence_point_s mission_fence_point;
			read_result = dm_read(DM_KEY_FENCE_POINTS, seq + 1, &mission_fence_point, sizeof(mission_fence_point_s)) ==
				      sizeof(mission_fence_point_s);

			mission_item.nav_cmd = mission_fence_point.nav_cmd;
			mission_item.frame = mission_fence_point.frame;
			mission_item.lat = mission_fence_point.lat;
			mission_item.lon = mission_fence_point.lon;
			mission_item.altitude = mission_fence_point.alt;

			if (mission_fence_point.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
			    mission_fence_point.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
				mission_item.vertex_count = mission_fence_point.vertex_count;

			} else {
				mission_item.circle_radius = mission_fence_point.circle_radius;
			}
		}
		break;

	case MAV_MISSION_TYPE_RALLY: { // Read a safe point / rally point
			mission_safe_point_s mission_safe_point;
			read_result = dm_read(DM_KEY_SAFE_POINTS, seq + 1, &mission_safe_point, sizeof(mission_safe_point_s)) ==
				      sizeof(mission_safe_point_s);

			mission_item.nav_cmd = MAV_CMD_NAV_RALLY_POINT;
			mission_item.frame = mission_safe_point.frame;
			mission_item.lat = mission_safe_point.lat;
			mission_item.lon = mission_safe_point.lon;
			mission_item.altitude = mission_safe_point.alt;
		}
		break;

	default:
		_mavlink->send_statustext_critical("Received unknown mission type, abort.");
		break;
	}

	if (read_result > 0) {
		_time_last_sent = hrt_absolute_time();

		if (_int_mode) {
			mavlink_mission_item_int_t wp{};
			format_mavlink_mission_item(&mission_item, reinterpret_cast<mavlink_mission_item_t *>(&wp));

			wp.target_system = sysid;
			wp.target_component = compid;
			wp.seq = seq;
			wp.current = (_current_seq == seq) ? 1 : 0;

			mavlink_msg_mission_item_int_send_struct(_mavlink->get_channel(), &wp);

			PX4_DEBUG("WPM: Send MISSION_ITEM_INT seq %u to ID %u", wp.seq, wp.target_system);

		} else {
			mavlink_mission_item_t wp{};
			format_mavlink_mission_item(&mission_item, &wp);

			wp.target_system = sysid;
			wp.target_component = compid;
			wp.seq = seq;
			wp.current = (_current_seq == seq) ? 1 : 0;

			mavlink_msg_mission_item_send_struct(_mavlink->get_channel(), &wp);

			PX4_DEBUG("WPM: Send MISSION_ITEM seq %u to ID %u", wp.seq, wp.target_system);
		}

	} else {
		send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to read from microSD");
		}

		PX4_DEBUG("WPM: Send MISSION_ITEM ERROR: could not read seq %u from dataman ID %i", seq, _dataman_id);
	}
}

uint16_t
MavlinkMissionManager::current_max_item_count()
{
	if (_mission_type >= sizeof(MAX_COUNT) / sizeof(MAX_COUNT[0])) {
		PX4_ERR("WPM: MAX_COUNT out of bounds (%u)", _mission_type);
		return 0;
	}

	return MAX_COUNT[_mission_type];
}

uint16_t
MavlinkMissionManager::current_item_count()
{
	if (_mission_type >= sizeof(_count) / sizeof(_count[0])) {
		PX4_ERR("WPM: _count out of bounds (%u)", _mission_type);
		return 0;
	}

	return _count[_mission_type];
}

void
MavlinkMissionManager::send_mission_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < current_max_item_count()) {
		_time_last_sent = hrt_absolute_time();

		if (_int_mode) {
			mavlink_mission_request_int_t wpr{};
			wpr.target_system = sysid;
			wpr.target_component = compid;
			wpr.seq = seq;
			wpr.mission_type = _mission_type;
			mavlink_msg_mission_request_int_send_struct(_mavlink->get_channel(), &wpr);

			PX4_DEBUG("WPM: Send MISSION_REQUEST_INT seq %u to ID %u", wpr.seq, wpr.target_system);

		} else {

			mavlink_mission_request_t wpr{};
			wpr.target_system = sysid;
			wpr.target_component = compid;
			wpr.seq = seq;
			wpr.mission_type = _mission_type;

			mavlink_msg_mission_request_send_struct(_mavlink->get_channel(), &wpr);

			PX4_DEBUG("WPM: Send MISSION_REQUEST seq %u to ID %u", wpr.seq, wpr.target_system);
		}

	} else {
		_mavlink->send_statustext_critical("ERROR: Waypoint index exceeds list capacity");

		PX4_DEBUG("WPM: Send MISSION_REQUEST ERROR: seq %u exceeds list capacity", seq);
	}
}

void
MavlinkMissionManager::send_mission_item_reached(uint16_t seq)
{
	mavlink_mission_item_reached_t wp_reached{};

	wp_reached.seq = seq;

	mavlink_msg_mission_item_reached_send_struct(_mavlink->get_channel(), &wp_reached);

	PX4_DEBUG("WPM: Send MISSION_ITEM_REACHED reached_seq %u", wp_reached.seq);
}

void
MavlinkMissionManager::send()
{
	// do not send anything over high latency communication
	if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM) {
		return;
	}

	mission_result_s mission_result{};

	if (_mission_result_sub.update(&mission_result)) {

		if (_current_seq != mission_result.seq_current) {
			_current_seq = mission_result.seq_current;

			PX4_DEBUG("WPM: got mission result, new current_seq: %u", _current_seq);
		}

		if (_last_reached != mission_result.seq_reached) {
			_last_reached = mission_result.seq_reached;
			_reached_sent_count = 0;

			if (_last_reached >= 0) {
				send_mission_item_reached((uint16_t)mission_result.seq_reached);
			}

			PX4_DEBUG("WPM: got mission result, new seq_reached: %d", _last_reached);
		}

		send_mission_current(_current_seq);

		if (mission_result.item_do_jump_changed) {
			/* Send a mission item again if the remaining DO_JUMPs has changed, but don't interfere
			 * if there are ongoing transfers happening already. */
			if (_state == MAVLINK_WPM_STATE_IDLE) {
				_mission_type = MAV_MISSION_TYPE_MISSION;
				send_mission_item(_transfer_partner_sysid, _transfer_partner_compid,
						  (uint16_t)mission_result.item_changed_index);
			}
		}

	} else {
		if (_slow_rate_limiter.check(hrt_absolute_time())) {
			send_mission_current(_current_seq);

			// send the reached message another 10 times
			if (_last_reached >= 0 && (_reached_sent_count < 10)) {
				send_mission_item_reached((uint16_t)_last_reached);
				_reached_sent_count++;
			}
		}
	}

	/* check for timed-out operations */
	if (_state == MAVLINK_WPM_STATE_GETLIST && (_time_last_sent > 0)
	    && hrt_elapsed_time(&_time_last_sent) > MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT) {

		// try to request item again after timeout
		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);

	} else if (_state != MAVLINK_WPM_STATE_IDLE && (_time_last_recv > 0)
		   && hrt_elapsed_time(&_time_last_recv) > MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT) {

		_mavlink->send_statustext_critical("Operation timeout");

		PX4_DEBUG("WPM: Last operation (state=%u) timed out, changing state to MAVLINK_WPM_STATE_IDLE", _state);

		switch_to_idle_state();

		// since we are giving up, reset this state also, so another request can be started.
		_transfer_in_progress = false;

	} else if (_state == MAVLINK_WPM_STATE_IDLE) {
		// reset flags
		_time_last_sent = 0;
		_time_last_recv = 0;
	}
}


void
MavlinkMissionManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_MISSION_ACK:
		handle_mission_ack(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		handle_mission_set_current(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		handle_mission_request_list(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST:
		handle_mission_request(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
		handle_mission_request_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_COUNT:
		handle_mission_count(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM:
		handle_mission_item(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM_INT:
		handle_mission_item_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
		handle_mission_clear_all(msg);
		break;

	default:
		break;
	}
}

void
MavlinkMissionManager::handle_mission_ack(const mavlink_message_t *msg)
{
	mavlink_mission_ack_t wpa{};
	mavlink_msg_mission_ack_decode(msg, &wpa);

	if (CHECK_SYSID_COMPID_MISSION(wpa)) {
		if ((msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid)) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST && _mission_type == wpa.mission_type) {

				_time_last_recv = hrt_absolute_time();

				if (wpa.type == MAV_MISSION_ACCEPTED && _transfer_seq == current_item_count()) {
					PX4_DEBUG("WPM: MISSION_ACK OK all items sent, switch to state IDLE");

				} else if (wpa.type == MAV_MISSION_OPERATION_CANCELLED) {
					PX4_DEBUG("WPM: MISSION_ACK CANCELLED, switch to state IDLE");

				} else {
					PX4_DEBUG("WPM: MISSION_ACK ERROR: not all items sent, switch to state IDLE anyway");
				}

				switch_to_idle_state();

			} else if (_state == MAVLINK_WPM_STATE_GETLIST) {

				// INT or float mode is not supported
				if (wpa.type == MAV_MISSION_UNSUPPORTED) {

					if (_int_mode) {
						_int_mode = false;
						send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);

					} else {
						_int_mode = true;
						send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
					}

				} else if (wpa.type == MAV_MISSION_OPERATION_CANCELLED) {
					PX4_DEBUG("WPM: MISSION_ACK CANCELLED, switch to state IDLE");
					switch_to_idle_state();
					_transfer_in_progress = false;

				} else if (wpa.type != MAV_MISSION_ACCEPTED) {
					PX4_WARN("Mission ack result was %d", wpa.type);
				}
			}

		} else {
			_mavlink->send_statustext_critical("REJ. WP CMD: partner id mismatch");

			PX4_DEBUG("WPM: MISSION_ACK ERR: ID mismatch");
		}
	}
}

void
MavlinkMissionManager::handle_mission_set_current(const mavlink_message_t *msg)
{
	mavlink_mission_set_current_t wpc;
	mavlink_msg_mission_set_current_decode(msg, &wpc);

	if (CHECK_SYSID_COMPID_MISSION(wpc)) {
		if (_state == MAVLINK_WPM_STATE_IDLE) {
			_time_last_recv = hrt_absolute_time();

			if (wpc.seq < _count[MAV_MISSION_TYPE_MISSION]) {
				if (update_active_mission(_dataman_id, _count[MAV_MISSION_TYPE_MISSION], wpc.seq) == PX4_OK) {
					PX4_DEBUG("WPM: MISSION_SET_CURRENT seq=%d OK", wpc.seq);

				} else {
					PX4_DEBUG("WPM: MISSION_SET_CURRENT seq=%d ERROR", wpc.seq);

					_mavlink->send_statustext_critical("WPM: WP CURR CMD: Error setting ID");
				}

			} else {
				PX4_ERR("WPM: MISSION_SET_CURRENT seq=%d ERROR: not in list", wpc.seq);

				_mavlink->send_statustext_critical("WPM: WP CURR CMD: Not in list");
			}

		} else {
			PX4_DEBUG("WPM: MISSION_SET_CURRENT ERROR: busy");

			_mavlink->send_statustext_critical("WPM: IGN WP CURR CMD: Busy");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request_list(const mavlink_message_t *msg)
{
	mavlink_mission_request_list_t wprl;
	mavlink_msg_mission_request_list_decode(msg, &wprl);

	if (CHECK_SYSID_COMPID_MISSION(wprl)) {
		const bool maybe_completed = (_transfer_seq == current_item_count());

		// If all mission items have been sent and a new mission request list comes in, we can proceed even if  MISSION_ACK was
		// never received. This could happen on a quick reconnect that doesn't trigger MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT
		if (maybe_completed) {
			switch_to_idle_state();
		}

		if (_state == MAVLINK_WPM_STATE_IDLE || (_state == MAVLINK_WPM_STATE_SENDLIST
				&& (uint8_t)_mission_type == wprl.mission_type)) {
			_time_last_recv = hrt_absolute_time();

			_state = MAVLINK_WPM_STATE_SENDLIST;
			_mission_type = (MAV_MISSION_TYPE)wprl.mission_type;

			// make sure our item counts are up-to-date
			switch (_mission_type) {
			case MAV_MISSION_TYPE_FENCE:
				load_geofence_stats();
				break;

			case MAV_MISSION_TYPE_RALLY:
				load_safepoint_stats();
				break;

			default:
				break;
			}

			_transfer_seq = 0;
			_transfer_count = current_item_count();
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;

			if (_transfer_count > 0) {
				PX4_DEBUG("WPM: MISSION_REQUEST_LIST OK, %u mission items to send, mission type=%i", _transfer_count, _mission_type);

			} else {
				PX4_DEBUG("WPM: MISSION_REQUEST_LIST OK nothing to send, mission is empty, mission type=%i", _mission_type);
			}

			send_mission_count(msg->sysid, msg->compid, _transfer_count, _mission_type);

		} else {
			PX4_DEBUG("WPM: MISSION_REQUEST_LIST ERROR: busy");

			_mavlink->send_statustext_info("Mission download request ignored, already active");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request(const mavlink_message_t *msg)
{
	// The request comes in the old float mode, so we switch to it.
	if (_int_mode) {
		_int_mode = false;
	}

	handle_mission_request_both(msg);
}

void
MavlinkMissionManager::handle_mission_request_int(const mavlink_message_t *msg)
{
	// The request comes in the new int mode, so we switch to it.
	if (!_int_mode) {
		_int_mode = true;
	}

	handle_mission_request_both(msg);
}

void
MavlinkMissionManager::handle_mission_request_both(const mavlink_message_t *msg)
{
	/* The mavlink_message_t could also be a mavlink_mission_request_int_t, however the structs
	 * are basically the same, so we can ignore it. */
	mavlink_mission_request_t wpr;
	mavlink_msg_mission_request_decode(msg, &wpr);

	if (CHECK_SYSID_COMPID_MISSION(wpr)) {
		if (msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST) {

				if (_mission_type != wpr.mission_type) {
					PX4_WARN("WPM: Unexpected mission type (%u %u)", wpr.mission_type, _mission_type);
					return;
				}

				_time_last_recv = hrt_absolute_time();

				/* _transfer_seq contains sequence of expected request */
				if (wpr.seq == _transfer_seq && _transfer_seq < _transfer_count) {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) seq %u from ID %u", wpr.seq, msg->sysid);

					_transfer_seq++;

				} else if (wpr.seq == _transfer_seq - 1) {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) seq %u from ID %u (again)", wpr.seq, msg->sysid);

				} else {
					if (_transfer_seq > 0 && _transfer_seq < _transfer_count) {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i or %i", wpr.seq, msg->sysid,
							  _transfer_seq - 1, _transfer_seq);

					} else if (_transfer_seq <= 0) {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid,
							  _transfer_seq);

					} else {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid,
							  _transfer_seq - 1);
					}

					switch_to_idle_state();

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected");
					return;
				}

				/* double check bounds in case of items count changed */
				if (wpr.seq < current_item_count()) {
					send_mission_item(_transfer_partner_sysid, _transfer_partner_compid, wpr.seq);

				} else {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u out of bound [%u, %u]", wpr.seq, wpr.seq,
						  current_item_count() - 1);

					switch_to_idle_state();

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected");
				}

			} else if (_state == MAVLINK_WPM_STATE_IDLE) {
				PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: no transfer");

				// Silently ignore this as some OSDs have buggy mission protocol implementations
				//_mavlink->send_statustext_critical("IGN MISSION_ITEM_REQUEST(_INT): No active transfer");

			} else {
				PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: busy (state %d).", _state);

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy");
			}

		} else {
			_mavlink->send_statustext_critical("WPM: REJ. CMD: partner id mismatch");

			PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: rejected, partner ID mismatch");
		}
	}
}


void
MavlinkMissionManager::handle_mission_count(const mavlink_message_t *msg)
{
	mavlink_mission_count_t wpc;
	mavlink_msg_mission_count_decode(msg, &wpc);

	if (CHECK_SYSID_COMPID_MISSION(wpc)) {
		if (_state == MAVLINK_WPM_STATE_IDLE) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_in_progress) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

			_transfer_in_progress = true;
			_mission_type = (MAV_MISSION_TYPE)wpc.mission_type;

			if (wpc.count > current_max_item_count()) {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: too many waypoints (%d), supported: %d", wpc.count, current_max_item_count());

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_NO_SPACE);
				_transfer_in_progress = false;
				return;
			}

			if (wpc.count == 0) {
				PX4_DEBUG("WPM: MISSION_COUNT 0, clearing waypoints list and staying in state MAVLINK_WPM_STATE_IDLE");

				switch (_mission_type) {
				case MAV_MISSION_TYPE_MISSION:

					/* alternate dataman ID anyway to let navigator know about changes */

					if (_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0) {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_1, 0, 0);

					} else {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
					}

					break;

				case MAV_MISSION_TYPE_FENCE:
					update_geofence_count(0);
					break;

				case MAV_MISSION_TYPE_RALLY:
					update_safepoint_count(0);
					break;

				default:
					PX4_ERR("mission type %u not handled", _mission_type);
					break;
				}

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);
				_transfer_in_progress = false;
				return;
			}

			PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u, changing state to MAVLINK_WPM_STATE_GETLIST", wpc.count, msg->sysid);

			_state = MAVLINK_WPM_STATE_GETLIST;
			_transfer_seq = 0;
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;
			_transfer_count = wpc.count;
			_transfer_dataman_id = (_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
						DM_KEY_WAYPOINTS_OFFBOARD_0);	// use inactive storage for transmission
			_transfer_current_seq = -1;

			if (_mission_type == MAV_MISSION_TYPE_FENCE) {
				// We're about to write new geofence items, so take the lock. It will be released when
				// switching back to idle
				PX4_DEBUG("locking fence dataman items");

				int ret = dm_lock(DM_KEY_FENCE_POINTS);

				if (ret == 0) {
					_geofence_locked = true;

				} else {
					PX4_ERR("locking failed (%i)", errno);
				}
			}

		} else if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_seq == 0) {
				/* looks like our MISSION_REQUEST was lost, try again */
				PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u (again)", wpc.count, msg->sysid);

			} else {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, already receiving seq %u", _transfer_seq);

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy");

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

		} else {
			PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, state %i", _state);

			_mavlink->send_statustext_critical("WPM: IGN MISSION_COUNT: Busy");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
	}
}

void
MavlinkMissionManager::switch_to_idle_state()
{
	// when switching to idle, we *always* check if the lock was held and release it.
	// This is to ensure we don't end up in a state where we forget to release it.
	if (_geofence_locked) {
		dm_unlock(DM_KEY_FENCE_POINTS);
		_geofence_locked = false;

		PX4_DEBUG("unlocking geofence");
	}

	_state = MAVLINK_WPM_STATE_IDLE;
}


void
MavlinkMissionManager::handle_mission_item(const mavlink_message_t *msg)
{
	if (_int_mode) {
		// It seems that we should be using the float mode, let's switch out of int mode.
		_int_mode = false;
	}

	handle_mission_item_both(msg);
}

void
MavlinkMissionManager::handle_mission_item_int(const mavlink_message_t *msg)
{
	if (!_int_mode) {
		// It seems that we should be using the int mode, let's switch to it.
		_int_mode = true;
	}

	handle_mission_item_both(msg);
}

void
MavlinkMissionManager::handle_mission_item_both(const mavlink_message_t *msg)
{

	// The mavlink_message could also contain a mavlink_mission_item_int_t. We ignore that here
	// and take care of it later in parse_mavlink_mission_item depending on _int_mode.

	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

	if (CHECK_SYSID_COMPID_MISSION(wp)) {

		if (wp.mission_type != _mission_type) {
			PX4_WARN("WPM: Unexpected mission type (%u %u)", (int)wp.mission_type, (int)_mission_type);
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (wp.seq != _transfer_seq) {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u was not the expected %u", wp.seq, _transfer_seq);

				/* request next item again */
				send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
				return;
			}

		} else if (_state == MAVLINK_WPM_STATE_IDLE) {
			if (_transfer_seq == wp.seq + 1) {
				// Assume this is a duplicate, where we already successfully got all mission items,
				// but the GCS did not receive the last ack and sent the same item again
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: no transfer");

				_mavlink->send_statustext_critical("IGN MISSION_ITEM: No transfer");
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			return;

		} else {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: busy, state %i", _state);

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		struct mission_item_s mission_item = {};

		int ret = parse_mavlink_mission_item(&wp, &mission_item);

		if (ret != PX4_OK) {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u invalid item", wp.seq);

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy");

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, ret);
			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		bool write_failed = false;
		bool check_failed = false;

		switch (_mission_type) {

		case MAV_MISSION_TYPE_MISSION: {
				// check that we don't get a wrong item (hardening against wrong client implementations, the list here
				// does not need to be complete)
				if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_RALLY_POINT) {
					check_failed = true;

				} else {
					dm_item_t dm_item = _transfer_dataman_id;

					write_failed = dm_write(dm_item, wp.seq, DM_PERSIST_POWER_ON_RESET, &mission_item,
								sizeof(struct mission_item_s)) != sizeof(struct mission_item_s);

					if (!write_failed) {
						/* waypoint marked as current */
						if (wp.current) {
							_transfer_current_seq = wp.seq;
						}
					}
				}
			}
			break;

		case MAV_MISSION_TYPE_FENCE: { // Write a geofence point
				mission_fence_point_s mission_fence_point;
				mission_fence_point.nav_cmd = mission_item.nav_cmd;
				mission_fence_point.lat = mission_item.lat;
				mission_fence_point.lon = mission_item.lon;
				mission_fence_point.alt = mission_item.altitude;

				if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
					mission_fence_point.vertex_count = mission_item.vertex_count;

					if (mission_item.vertex_count < 3) { // feasibility check
						PX4_ERR("Fence: too few vertices");
						check_failed = true;
						update_geofence_count(0);
					}

				} else {
					mission_fence_point.circle_radius = mission_item.circle_radius;
				}

				mission_fence_point.frame = mission_item.frame;

				if (!check_failed) {
					write_failed = dm_write(DM_KEY_FENCE_POINTS, wp.seq + 1, DM_PERSIST_POWER_ON_RESET, &mission_fence_point,
								sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s);
				}

			}
			break;

		case MAV_MISSION_TYPE_RALLY: { // Write a safe point / rally point
				mission_safe_point_s mission_safe_point;
				mission_safe_point.lat = mission_item.lat;
				mission_safe_point.lon = mission_item.lon;
				mission_safe_point.alt = mission_item.altitude;
				mission_safe_point.frame = mission_item.frame;
				write_failed = dm_write(DM_KEY_SAFE_POINTS, wp.seq + 1, DM_PERSIST_POWER_ON_RESET, &mission_safe_point,
							sizeof(mission_safe_point_s)) != sizeof(mission_safe_point_s);
			}
			break;

		default:
			_mavlink->send_statustext_critical("Received unknown mission type, abort.");
			break;
		}

		if (write_failed || check_failed) {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: error writing seq %u to dataman ID %i", wp.seq, _transfer_dataman_id);

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);

			if (write_failed) {
				_mavlink->send_statustext_critical("Unable to write on micro SD");
			}

			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		/* waypoint marked as current */
		if (wp.current) {
			_transfer_current_seq = wp.seq;
		}

		PX4_DEBUG("WPM: MISSION_ITEM seq %u received", wp.seq);

		_transfer_seq = wp.seq + 1;

		if (_transfer_seq == _transfer_count) {
			/* got all new mission items successfully */
			PX4_DEBUG("WPM: MISSION_ITEM got all %u items, current_seq=%u, changing state to MAVLINK_WPM_STATE_IDLE",
				  _transfer_count, _transfer_current_seq);

			ret = 0;

			switch (_mission_type) {
			case MAV_MISSION_TYPE_MISSION:
				ret = update_active_mission(_transfer_dataman_id, _transfer_count, _transfer_current_seq);
				break;

			case MAV_MISSION_TYPE_FENCE:
				ret = update_geofence_count(_transfer_count);
				break;

			case MAV_MISSION_TYPE_RALLY:
				ret = update_safepoint_count(_transfer_count);
				break;

			default:
				PX4_ERR("mission type %u not handled", _mission_type);
				break;
			}

			// Note: the switch to idle needs to happen after update_geofence_count is called, for proper unlocking order
			switch_to_idle_state();


			if (ret == PX4_OK) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			_transfer_in_progress = false;

		} else {
			/* request next item */
			send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
		}
	}
}


void
MavlinkMissionManager::handle_mission_clear_all(const mavlink_message_t *msg)
{
	mavlink_mission_clear_all_t wpca;
	mavlink_msg_mission_clear_all_decode(msg, &wpca);

	if (CHECK_SYSID_COMPID_MISSION(wpca)) {

		if (_state == MAVLINK_WPM_STATE_IDLE) {
			/* don't touch mission items storage itself, but only items count in mission state */
			_time_last_recv = hrt_absolute_time();

			_mission_type = (MAV_MISSION_TYPE)wpca.mission_type; // this is needed for the returned ack
			int ret = 0;

			switch (wpca.mission_type) {
			case MAV_MISSION_TYPE_MISSION:
				ret = update_active_mission(_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
							    DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
				break;

			case MAV_MISSION_TYPE_FENCE:
				ret = update_geofence_count(0);
				break;

			case MAV_MISSION_TYPE_RALLY:
				ret = update_safepoint_count(0);
				break;

			case MAV_MISSION_TYPE_ALL:
				ret = update_active_mission(_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
							    DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
				ret = update_geofence_count(0) || ret;
				ret = update_safepoint_count(0) || ret;
				break;

			default:
				PX4_ERR("mission type %u not handled", _mission_type);
				break;
			}

			if (ret == PX4_OK) {
				PX4_DEBUG("WPM: CLEAR_ALL OK (mission_type=%i)", _mission_type);

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

		} else {
			_mavlink->send_statustext_critical("WPM: IGN CLEAR CMD: Busy");

			PX4_DEBUG("WPM: CLEAR_ALL IGNORED: busy");
		}
	}
}

int
MavlinkMissionManager::parse_mavlink_mission_item(const mavlink_mission_item_t *mavlink_mission_item,
		struct mission_item_s *mission_item)
{
	if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
	    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
	    (_int_mode && (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT))) {

		// Switch to int mode if that is what we are receiving
		if ((mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
		     mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)) {
			_int_mode = true;
		}

		if (_int_mode) {
			/* The argument is actually a mavlink_mission_item_int_t in int_mode.
			 * mavlink_mission_item_t and mavlink_mission_item_int_t have the same
			 * alignment, so we can just swap float for int32_t. */
			const mavlink_mission_item_int_t *item_int
				= reinterpret_cast<const mavlink_mission_item_int_t *>(mavlink_mission_item);
			mission_item->lat = ((double)item_int->x) * 1e-7;
			mission_item->lon = ((double)item_int->y) * 1e-7;

		} else {
			mission_item->lat = (double)mavlink_mission_item->x;
			mission_item->lon = (double)mavlink_mission_item->y;
		}

		mission_item->altitude = mavlink_mission_item->z;

		if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
		    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT) {
			mission_item->altitude_is_relative = false;

		} else if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
			mission_item->altitude_is_relative = true;
		}

		switch (mavlink_mission_item->command) {
		case MAV_CMD_NAV_WAYPOINT:
			mission_item->nav_cmd = NAV_CMD_WAYPOINT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->acceptance_radius = mavlink_mission_item->param2;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:
			mission_item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			mission_item->nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->force_heading = (mavlink_mission_item->param2 > 0);
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			// Yaw is only valid for multicopter but we set it always because
			// it's just ignored for fixedwing.
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LAND:
			mission_item->nav_cmd = NAV_CMD_LAND;
			// TODO: abort alt param1
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			mission_item->land_precision = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_TAKEOFF:
			mission_item->nav_cmd = NAV_CMD_TAKEOFF;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));

			break;

		case MAV_CMD_NAV_LOITER_TO_ALT:
			mission_item->nav_cmd = NAV_CMD_LOITER_TO_ALT;
			mission_item->force_heading = (mavlink_mission_item->param1 > 0);
			mission_item->loiter_radius = mavlink_mission_item->param2;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI:
			if ((int)mavlink_mission_item->param1 == MAV_ROI_LOCATION) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_LOCATION;

				mission_item->params[6] = mavlink_mission_item->z;

			} else if ((int)mavlink_mission_item->param1 == MAV_ROI_NONE) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_NONE;

			} else {
				return MAV_MISSION_INVALID_PARAM1;
			}

			break;

		case MAV_CMD_DO_SET_ROI_LOCATION:
			mission_item->nav_cmd = NAV_CMD_DO_SET_ROI_LOCATION;
			mission_item->params[6] = mavlink_mission_item->z;
			break;

		case MAV_CMD_NAV_VTOL_TAKEOFF:
		case MAV_CMD_NAV_VTOL_LAND:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_CONDITION_GATE:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->vertex_count = (uint16_t)(mavlink_mission_item->param1 + 0.5f);
			break;

		case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
		case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->circle_radius = mavlink_mission_item->param1;
			break;

		case MAV_CMD_NAV_RALLY_POINT:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;

			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = mavlink_mission_item->frame;

	} else if (mavlink_mission_item->frame == MAV_FRAME_MISSION) {

		// this is a mission item with no coordinates

		mission_item->params[0] = mavlink_mission_item->param1;
		mission_item->params[1] = mavlink_mission_item->param2;
		mission_item->params[2] = mavlink_mission_item->param3;
		mission_item->params[3] = mavlink_mission_item->param4;

		if (_int_mode) {
			/* The argument is actually a mavlink_mission_item_int_t in int_mode.
			 * mavlink_mission_item_t and mavlink_mission_item_int_t have the same
			 * alignment, so we can just swap float for int32_t. */
			const mavlink_mission_item_int_t *item_int
				= reinterpret_cast<const mavlink_mission_item_int_t *>(mavlink_mission_item);
			mission_item->params[4] = ((double)item_int->x);
			mission_item->params[5] = ((double)item_int->y);

		} else {
			mission_item->params[4] = (double)mavlink_mission_item->x;
			mission_item->params[5] = (double)mavlink_mission_item->y;
		}

		mission_item->params[6] = mavlink_mission_item->z;

		switch (mavlink_mission_item->command) {
		case MAV_CMD_DO_JUMP:
			mission_item->nav_cmd = NAV_CMD_DO_JUMP;
			mission_item->do_jump_mission_index = mavlink_mission_item->param1;
			mission_item->do_jump_current_count = 0;
			mission_item->do_jump_repeat_count = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI: {
				const int roi_mode = mavlink_mission_item->param1;

				if (roi_mode == MAV_ROI_NONE || roi_mode == MAV_ROI_WPNEXT || roi_mode == MAV_ROI_WPINDEX) {
					mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;

				} else {
					return MAV_MISSION_INVALID_PARAM1;
				}
			}
			break;

		case MAV_CMD_DO_CHANGE_SPEED:
		case MAV_CMD_DO_SET_HOME:
		case MAV_CMD_DO_SET_SERVO:
		case MAV_CMD_DO_LAND_START:
		case MAV_CMD_DO_TRIGGER_CONTROL:
		case MAV_CMD_DO_DIGICAM_CONTROL:
		case MAV_CMD_DO_MOUNT_CONFIGURE:
		case MAV_CMD_DO_MOUNT_CONTROL:
		case NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
		case NAV_CMD_SET_CAMERA_FOCUS:
		case NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
		case MAV_CMD_IMAGE_START_CAPTURE:
		case MAV_CMD_IMAGE_STOP_CAPTURE:
		case MAV_CMD_VIDEO_START_CAPTURE:
		case MAV_CMD_VIDEO_STOP_CAPTURE:
		case MAV_CMD_DO_CONTROL_VIDEO:
		case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
		case MAV_CMD_OBLIQUE_SURVEY:
		case MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
		case MAV_CMD_SET_CAMERA_MODE:
		case MAV_CMD_SET_CAMERA_ZOOM:
		case MAV_CMD_DO_VTOL_TRANSITION:
		case MAV_CMD_NAV_DELAY:
		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		case MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
		case MAV_CMD_DO_SET_ROI_NONE:
		case MAV_CMD_CONDITION_DELAY:
		case MAV_CMD_CONDITION_DISTANCE:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;

			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = MAV_FRAME_MISSION;

	} else {
		PX4_DEBUG("Unsupported frame %d", mavlink_mission_item->frame);

		return MAV_MISSION_UNSUPPORTED_FRAME;
	}

	mission_item->autocontinue = mavlink_mission_item->autocontinue;
	// mission_item->index = mavlink_mission_item->seq;

	mission_item->origin = ORIGIN_MAVLINK;

	return MAV_MISSION_ACCEPTED;
}


int
MavlinkMissionManager::format_mavlink_mission_item(const struct mission_item_s *mission_item,
		mavlink_mission_item_t *mavlink_mission_item)
{
	mavlink_mission_item->frame = mission_item->frame;
	mavlink_mission_item->command = mission_item->nav_cmd;
	mavlink_mission_item->autocontinue = mission_item->autocontinue;

	/* default mappings for generic commands */
	if (mission_item->frame == MAV_FRAME_MISSION) {
		mavlink_mission_item->param1 = mission_item->params[0];
		mavlink_mission_item->param2 = mission_item->params[1];
		mavlink_mission_item->param3 = mission_item->params[2];
		mavlink_mission_item->param4 = mission_item->params[3];

		mavlink_mission_item->x = mission_item->params[4];
		mavlink_mission_item->y = mission_item->params[5];

		if (_int_mode) {
			// This function actually receives a mavlink_mission_item_int_t in _int_mode
			// which has the same alignment as mavlink_mission_item_t and the only
			// difference is int32_t vs. float for x and y.
			mavlink_mission_item_int_t *item_int =
				reinterpret_cast<mavlink_mission_item_int_t *>(mavlink_mission_item);

			item_int->x = round(mission_item->params[4]);
			item_int->y = round(mission_item->params[5]);

		} else {
			mavlink_mission_item->x = (float)mission_item->params[4];
			mavlink_mission_item->y = (float)mission_item->params[5];
		}

		mavlink_mission_item->z = mission_item->params[6];

		switch (mavlink_mission_item->command) {
		case NAV_CMD_DO_JUMP:
			mavlink_mission_item->param1 = mission_item->do_jump_mission_index;
			mavlink_mission_item->param2 = mission_item->do_jump_repeat_count;
			break;

		case NAV_CMD_DO_CHANGE_SPEED:
		case NAV_CMD_DO_SET_HOME:
		case NAV_CMD_DO_SET_SERVO:
		case NAV_CMD_DO_LAND_START:
		case NAV_CMD_DO_TRIGGER_CONTROL:
		case NAV_CMD_DO_DIGICAM_CONTROL:
		case NAV_CMD_IMAGE_START_CAPTURE:
		case NAV_CMD_IMAGE_STOP_CAPTURE:
		case NAV_CMD_VIDEO_START_CAPTURE:
		case NAV_CMD_VIDEO_STOP_CAPTURE:
		case NAV_CMD_DO_CONTROL_VIDEO:
		case NAV_CMD_DO_MOUNT_CONFIGURE:
		case NAV_CMD_DO_MOUNT_CONTROL:
		case NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
		case NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
		case NAV_CMD_DO_SET_ROI:
		case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
		case NAV_CMD_OBLIQUE_SURVEY:
		case NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
		case NAV_CMD_SET_CAMERA_MODE:
		case NAV_CMD_SET_CAMERA_ZOOM:
		case NAV_CMD_SET_CAMERA_FOCUS:
		case NAV_CMD_DO_VTOL_TRANSITION:
			break;

		default:
			return PX4_ERROR;
		}

	} else {
		mavlink_mission_item->param1 = 0.0f;
		mavlink_mission_item->param2 = 0.0f;
		mavlink_mission_item->param3 = 0.0f;
		mavlink_mission_item->param4 = 0.0f;

		if (_int_mode) {
			// This function actually receives a mavlink_mission_item_int_t in _int_mode
			// which has the same alignment as mavlink_mission_item_t and the only
			// difference is int32_t vs. float for x and y.
			mavlink_mission_item_int_t *item_int =
				reinterpret_cast<mavlink_mission_item_int_t *>(mavlink_mission_item);

			item_int->x = round(mission_item->lat * 1e7);
			item_int->y = round(mission_item->lon * 1e7);

		} else {
			mavlink_mission_item->x = (float)mission_item->lat;
			mavlink_mission_item->y = (float)mission_item->lon;
		}

		mavlink_mission_item->z = mission_item->altitude;

		if (mission_item->altitude_is_relative) {
			if (_int_mode) {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;

			} else {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
			}

		} else {
			if (_int_mode) {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_INT;

			} else {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL;
			}
		}

		switch (mission_item->nav_cmd) {
		case NAV_CMD_WAYPOINT:
			mavlink_mission_item->param1 = mission_item->time_inside;
			mavlink_mission_item->param2 = mission_item->acceptance_radius;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_UNLIMITED:
			mavlink_mission_item->param3 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_TIME_LIMIT:
			mavlink_mission_item->param1 = mission_item->time_inside;
			mavlink_mission_item->param2 = mission_item->force_heading;
			mavlink_mission_item->param3 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = mission_item->loiter_exit_xtrack;
			break;

		case NAV_CMD_LAND:
			// TODO: param1 abort alt
			mavlink_mission_item->param2 = mission_item->land_precision;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_TAKEOFF:
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_TO_ALT:
			mavlink_mission_item->param1 = mission_item->force_heading;
			mavlink_mission_item->param2 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = mission_item->loiter_exit_xtrack;
			break;

		case MAV_CMD_NAV_VTOL_TAKEOFF:
		case MAV_CMD_NAV_VTOL_LAND:
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
			mavlink_mission_item->param1 = (float)mission_item->vertex_count;
			break;

		case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
		case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
			mavlink_mission_item->param1 = mission_item->circle_radius;
			break;

		case MAV_CMD_NAV_RALLY_POINT:
			break;


		default:
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}


void MavlinkMissionManager::check_active_mission()
{
	// do not send anything over high latency communication
	if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM) {
		return;
	}

	if (!(_my_dataman_id == _dataman_id)) {
		PX4_DEBUG("WPM: New mission detected (possibly over different Mavlink instance) Updating");

		_my_dataman_id = _dataman_id;
		send_mission_count(_transfer_partner_sysid, _transfer_partner_compid, _count[MAV_MISSION_TYPE_MISSION],
				   MAV_MISSION_TYPE_MISSION);
	}
}
