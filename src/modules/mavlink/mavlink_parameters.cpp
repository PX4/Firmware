/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_parameters.cpp
 * Mavlink parameters manager implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Beat Kueng <beat@px4.io>
 */

#include <stdio.h>

#include "mavlink_parameters.h"
#include "mavlink_main.h"
#include <lib/systemlib/mavlink_log.h>

MavlinkParametersManager::MavlinkParametersManager(Mavlink *mavlink) :
	_mavlink(mavlink)
{
}

unsigned
MavlinkParametersManager::get_size()
{
	return MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

void
MavlinkParametersManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			/* request all parameters */
			mavlink_param_request_list_t req_list;
			mavlink_msg_param_request_list_decode(msg, &req_list);

			if (req_list.target_system == mavlink_system.sysid &&
			    (req_list.target_component == mavlink_system.compid || req_list.target_component == MAV_COMP_ID_ALL)) {
				if (_send_all_index < 0) {
					_send_all_index = PARAM_HASH;

				} else {
					/* a restart should skip the hash check on the ground */
					_send_all_index = 0;
				}
			}

			if (req_list.target_system == mavlink_system.sysid && req_list.target_component < 127 &&
			    (req_list.target_component != mavlink_system.compid || req_list.target_component == MAV_COMP_ID_ALL)) {
				// publish list request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req{};
				req.message_type = msg->msgid;
				req.node_id = req_list.target_component;
				req.param_index = 0;
				req.timestamp = hrt_absolute_time();
				_uavcan_parameter_request_pub.publish(req);
			}

			break;
		}

	case MAVLINK_MSG_ID_PARAM_SET: {
			/* set parameter */
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			if (set.target_system == mavlink_system.sysid &&
			    (set.target_component == mavlink_system.compid || set.target_component == MAV_COMP_ID_ALL)) {

				/* local name buffer to enforce null-terminated string */
				char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
				strncpy(name, set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				/* enforce null termination */
				name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				/* Whatever the value is, we're being told to stop sending */
				if (strncmp(name, "_HASH_CHECK", sizeof(name)) == 0) {

					if (_mavlink->hash_check_enabled()) {
						_send_all_index = -1;
					}

					/* No other action taken, return */
					return;
				}

				/* attempt to find parameter, set and send it */
				param_t param = param_find_no_notification(name);

				if (param == PARAM_INVALID) {
					char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					sprintf(buf, "[pm] unknown param: %s", name);
					_mavlink->send_statustext_info(buf);

				} else if (!((param_type(param) == PARAM_TYPE_INT32 && set.param_type == MAV_PARAM_TYPE_INT32) ||
					     (param_type(param) == PARAM_TYPE_FLOAT && set.param_type == MAV_PARAM_TYPE_REAL32))) {
					char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
					sprintf(buf, "[pm] param types mismatch param: %s", name);
					_mavlink->send_statustext_info(buf);

				} else {
					// According to the mavlink spec we should always acknowledge a write operation.
					param_set(param, &(set.param_value));
					send_param(param);
				}
			}

			if (set.target_system == mavlink_system.sysid && set.target_component < 127 &&
			    (set.target_component != mavlink_system.compid || set.target_component == MAV_COMP_ID_ALL)) {
				// publish set request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req{};
				req.message_type = msg->msgid;
				req.node_id = set.target_component;
				req.param_index = -1;
				strncpy(req.param_id, set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
				req.param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				if (set.param_type == MAV_PARAM_TYPE_REAL32) {
					req.param_type = MAV_PARAM_TYPE_REAL32;
					req.real_value = set.param_value;

				} else {
					int32_t val;
					memcpy(&val, &set.param_value, sizeof(int32_t));
					req.param_type = MAV_PARAM_TYPE_INT64;
					req.int_value = val;
				}

				req.timestamp = hrt_absolute_time();
				_uavcan_parameter_request_pub.publish(req);
			}

			break;
		}

	case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			/* request one parameter */
			mavlink_param_request_read_t req_read;
			mavlink_msg_param_request_read_decode(msg, &req_read);

			if (req_read.target_system == mavlink_system.sysid &&
			    (req_read.target_component == mavlink_system.compid || req_read.target_component == MAV_COMP_ID_ALL)) {

				/* when no index is given, loop through string ids and compare them */
				if (req_read.param_index < 0) {
					/* XXX: I left this in so older versions of QGC wouldn't break */
					if (strncmp(req_read.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
						/* return hash check for cached params */
						uint32_t hash = param_hash_check();

						/* build the one-off response message */
						mavlink_param_value_t param_value;
						param_value.param_count = param_count_used();
						param_value.param_index = -1;
						strncpy(param_value.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
						param_value.param_type = MAV_PARAM_TYPE_UINT32;
						memcpy(&param_value.param_value, &hash, sizeof(hash));
						mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &param_value);

					} else {
						/* local name buffer to enforce null-terminated string */
						char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
						strncpy(name, req_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
						/* enforce null termination */
						name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
						/* attempt to find parameter and send it */
						send_param(param_find_no_notification(name));
					}

				} else {
					/* when index is >= 0, send this parameter again */
					int ret = send_param(param_for_used_index(req_read.param_index));

					if (ret == 1) {
						char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						sprintf(buf, "[pm] unknown param ID: %u", req_read.param_index);
						_mavlink->send_statustext_info(buf);

					} else if (ret == 2) {
						char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						sprintf(buf, "[pm] failed loading param from storage ID: %u", req_read.param_index);
						_mavlink->send_statustext_info(buf);
					}
				}
			}

			if (req_read.target_system == mavlink_system.sysid && req_read.target_component < 127 &&
			    (req_read.target_component != mavlink_system.compid || req_read.target_component == MAV_COMP_ID_ALL)) {
				// publish set request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req{};
				req.timestamp = hrt_absolute_time();
				req.message_type = msg->msgid;
				req.node_id = req_read.target_component;
				req.param_index = req_read.param_index;
				strncpy(req.param_id, req_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
				req.param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				// Enque the request and forward the first to the uavcan node
				enque_uavcan_request(&req);
				request_next_uavcan_parameter();
			}

			break;
		}

	case MAVLINK_MSG_ID_PARAM_MAP_RC: {
			/* map a rc channel to a parameter */
			mavlink_param_map_rc_t map_rc;
			mavlink_msg_param_map_rc_decode(msg, &map_rc);

			if (map_rc.target_system == mavlink_system.sysid &&
			    (map_rc.target_component == mavlink_system.compid ||
			     map_rc.target_component == MAV_COMP_ID_ALL)) {

				/* Copy values from msg to uorb using the parameter_rc_channel_index as index */
				size_t i = map_rc.parameter_rc_channel_index;

				if (i >= sizeof(_rc_param_map.param_index) / sizeof(_rc_param_map.param_index[0])) {
					mavlink_log_warning(_mavlink->get_mavlink_log_pub(), "parameter_rc_channel_index out of bounds");
					break;
				}

				_rc_param_map.param_index[i] = map_rc.param_index;
				strncpy(&(_rc_param_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]), map_rc.param_id,
					MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				/* enforce null termination */
				_rc_param_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1) + rc_parameter_map_s::PARAM_ID_LEN] = '\0';
				_rc_param_map.scale[i] = map_rc.scale;
				_rc_param_map.value0[i] = map_rc.param_value0;
				_rc_param_map.value_min[i] = map_rc.param_value_min;
				_rc_param_map.value_max[i] = map_rc.param_value_max;

				if (map_rc.param_index == -2) { // -2 means unset map
					_rc_param_map.valid[i] = false;

				} else {
					_rc_param_map.valid[i] = true;
				}

				_rc_param_map.timestamp = hrt_absolute_time();
				_rc_param_map_pub.publish(_rc_param_map);
			}

			break;
		}

	default:
		break;
	}
}

void
MavlinkParametersManager::send()
{
	if (!_first_send) {
		// parameters QGC can't tolerate not finding (2020-11-11)
		param_find("BAT_A_PER_V");
		param_find("BAT_CRIT_THR");
		param_find("BAT_EMERGEN_THR");
		param_find("BAT_LOW_THR");
		param_find("BAT_N_CELLS");
		param_find("BAT_V_CHARGED");
		param_find("BAT_V_DIV");
		param_find("BAT_V_EMPTY");
		param_find("CAL_ACC0_ID");
		param_find("CAL_GYRO0_ID");
		param_find("CAL_MAG0_ID");
		param_find("CAL_MAG0_ROT");
		param_find("CAL_MAG1_ID");
		param_find("CAL_MAG1_ROT");
		param_find("CAL_MAG2_ID");
		param_find("CAL_MAG2_ROT");
		param_find("CAL_MAG3_ID");
		param_find("CAL_MAG3_ROT");
		param_find("SENS_BOARD_ROT");
		param_find("SENS_BOARD_X_OFF");
		param_find("SENS_BOARD_Y_OFF");
		param_find("SENS_BOARD_Z_OFF");
		param_find("SENS_DPRES_OFF");
		param_find("TRIG_MODE");
		param_find("UAVCAN_ENABLE");

		_first_send = true;
	}

	int max_num_to_send;

	if (_mavlink->get_protocol() == Protocol::SERIAL && !_mavlink->is_usb_uart()) {
		max_num_to_send = 3;

	} else {
		// speed up parameter loading via UDP or USB: try to send 20 at once
		max_num_to_send = 20;
	}

	int i = 0;

	// Send while burst is not exceeded, we still have buffer space and still something to send
	while ((i++ < max_num_to_send) && (_mavlink->get_free_tx_buf() >= get_size()) && !_mavlink->radio_status_critical()
	       && send_params()) {}
}

bool
MavlinkParametersManager::send_params()
{
	if (send_uavcan()) {
		return true;

	} else if (send_one()) {
		return true;

	} else if (send_untransmitted()) {
		return true;

	} else {
		return false;
	}
}

bool
MavlinkParametersManager::send_untransmitted()
{
	bool sent_one = false;

	if (_parameter_update_sub.updated()) {
		// clear the update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// Schedule an update if not already the case
		if (_param_update_time == 0) {
			_param_update_time = pupdate.timestamp;
			_param_update_index = 0;
		}
	}

	if ((_param_update_time != 0) && ((_param_update_time + 5 * 1000) < hrt_absolute_time())) {

		param_t param = 0;

		// send out all changed values
		do {
			// skip over all parameters which are not invalid and not used
			do {
				param = param_for_index(_param_update_index);
				++_param_update_index;
			} while (param != PARAM_INVALID && !param_used(param));

			// send parameters which are untransmitted while there is
			// space in the TX buffer
			if ((param != PARAM_INVALID) && param_value_unsaved(param)) {
				int ret = send_param(param);
				char buf[100];
				strncpy(&buf[0], param_name(param), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				sent_one = true;

				if (ret != PX4_OK) {
					break;
				}
			}
		} while ((_mavlink->get_free_tx_buf() >= get_size()) && !_mavlink->radio_status_critical()
			 && (_param_update_index < (int) param_count()));

		// Flag work as done once all params have been sent
		if (_param_update_index >= (int) param_count()) {
			_param_update_time = 0;
		}
	}

	return sent_one;
}

bool
MavlinkParametersManager::send_uavcan()
{
	/* Send parameter values received from the UAVCAN topic */
	uavcan_parameter_value_s value{};

	if (_uavcan_parameter_value_sub.update(&value)) {

		// Check if we received a matching parameter, drop it from the list and request the next
		if ((_uavcan_open_request_list != nullptr)
		    && (value.param_index == _uavcan_open_request_list->req.param_index)
		    && (value.node_id == _uavcan_open_request_list->req.node_id)) {

			dequeue_uavcan_request();
			request_next_uavcan_parameter();
		}

		mavlink_param_value_t msg{};
		msg.param_count = value.param_count;
		msg.param_index = value.param_index;
#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif
		/*
		 * coverity[buffer_size_warning : FALSE]
		 *
		 * The MAVLink spec does not require the string to be NUL-terminated if it
		 * has length 16. In this case the receiving end needs to terminate it
		 * when copying it.
		 */
		strncpy(msg.param_id, value.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif

		if (value.param_type == MAV_PARAM_TYPE_REAL32) {
			msg.param_type = MAVLINK_TYPE_FLOAT;
			msg.param_value = value.real_value;

		} else {
			int32_t val = (int32_t)value.int_value;
			memcpy(&msg.param_value, &val, sizeof(int32_t));
			msg.param_type = MAVLINK_TYPE_INT32_T;
		}

		// Re-pack the message with the UAVCAN node ID
		mavlink_message_t mavlink_packet{};
		mavlink_msg_param_value_encode_chan(mavlink_system.sysid, value.node_id, _mavlink->get_channel(), &mavlink_packet,
						    &msg);
		_mavlink_resend_uart(_mavlink->get_channel(), &mavlink_packet);

		return true;
	}

	return false;
}

bool
MavlinkParametersManager::send_one()
{
	if (_send_all_index >= 0) {
		/* send all parameters if requested, but only after the system has booted */

		/* The first thing we send is a hash of all values for the ground
		 * station to try and quickly load a cached copy of our params
		 */
		if (_send_all_index == PARAM_HASH) {
			/* return hash check for cached params */
			uint32_t hash = param_hash_check();

			/* build the one-off response message */
			mavlink_param_value_t msg;
			msg.param_count = param_count_used();
			msg.param_index = -1;
			strncpy(msg.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
			msg.param_type = MAV_PARAM_TYPE_UINT32;
			memcpy(&msg.param_value, &hash, sizeof(hash));
			mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &msg);

			/* after this we should start sending all params */
			_send_all_index = 0;

			/* No further action, return now */
			return true;
		}

		/* look for the first parameter which is used */
		param_t p;

		do {
			/* walk through all parameters, including unused ones */
			p = param_for_index(_send_all_index);
			_send_all_index++;
		} while (p != PARAM_INVALID && !param_used(p));

		if (p != PARAM_INVALID) {
			send_param(p);
		}

		if ((p == PARAM_INVALID) || (_send_all_index >= (int) param_count())) {
			_send_all_index = -1;
			return false;

		} else {
			return true;
		}
	}

	return false;
}

int
MavlinkParametersManager::send_param(param_t param, int component_id)
{
	if (param == PARAM_INVALID) {
		return 1;
	}

	/* no free TX buf to send this param */
	if (_mavlink->get_free_tx_buf() < MAVLINK_MSG_ID_PARAM_VALUE_LEN) {
		return 1;
	}

	mavlink_param_value_t msg;

	/*
	 * get param value, since MAVLink encodes float and int params in the same
	 * space during transmission, copy param onto float val_buf
	 */
	float param_value{};

	if (param_get(param, &param_value) != OK) {
		return 2;
	}

	msg.param_value = param_value;

	msg.param_count = param_count_used();
	msg.param_index = param_get_used_index(param);

#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif
	/*
	 * coverity[buffer_size_warning : FALSE]
	 *
	 * The MAVLink spec does not require the string to be NUL-terminated if it
	 * has length 16. In this case the receiving end needs to terminate it
	 * when copying it.
	 */
	strncpy(msg.param_id, param_name(param), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif

	/* query parameter type */
	param_type_t type = param_type(param);

	/*
	 * Map onboard parameter type to MAVLink type,
	 * endianess matches (both little endian)
	 */
	if (type == PARAM_TYPE_INT32) {
		msg.param_type = MAVLINK_TYPE_INT32_T;

	} else if (type == PARAM_TYPE_FLOAT) {
		msg.param_type = MAVLINK_TYPE_FLOAT;

	} else {
		msg.param_type = MAVLINK_TYPE_FLOAT;
	}

	/* default component ID */
	if (component_id < 0) {
		mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &msg);

	} else {
		// Re-pack the message with a different component ID
		mavlink_message_t mavlink_packet;
		mavlink_msg_param_value_encode_chan(mavlink_system.sysid, component_id, _mavlink->get_channel(), &mavlink_packet, &msg);
		_mavlink_resend_uart(_mavlink->get_channel(), &mavlink_packet);
	}

	return 0;
}

void MavlinkParametersManager::request_next_uavcan_parameter()
{
	// Request a parameter if we are not already waiting on a response and if the list is not empty
	if (!_uavcan_waiting_for_request_response && _uavcan_open_request_list != nullptr) {
		uavcan_parameter_request_s req = _uavcan_open_request_list->req;

		_uavcan_parameter_request_pub.publish(req);

		_uavcan_waiting_for_request_response = true;
	}
}

void MavlinkParametersManager::enque_uavcan_request(uavcan_parameter_request_s *req)
{
	// We store at max 10 requests to keep memory consumption low.
	// Dropped requests will be repeated by the ground station
	if (_uavcan_queued_request_items >= 10) {
		return;
	}

	_uavcan_open_request_list_item *new_reqest = new _uavcan_open_request_list_item;
	new_reqest->req = *req;
	new_reqest->next = nullptr;

	_uavcan_open_request_list_item *item = _uavcan_open_request_list;
	++_uavcan_queued_request_items;

	if (item == nullptr) {
		// Add the first item to the list
		_uavcan_open_request_list = new_reqest;

	} else {
		// Find the last item and add the new request at the end
		while (item->next != nullptr) {
			item = item->next;
		}

		item->next = new_reqest;
	}
}

void MavlinkParametersManager::dequeue_uavcan_request()
{
	if (_uavcan_open_request_list != nullptr) {
		// Drop the first item in the list and free the used memory
		_uavcan_open_request_list_item *first = _uavcan_open_request_list;
		_uavcan_open_request_list = first->next;
		--_uavcan_queued_request_items;
		delete first;
		_uavcan_waiting_for_request_response = false;
	}
}
