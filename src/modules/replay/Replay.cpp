/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file Replay.cpp
 * This module reads messages from an ULog file and publishes them.
 * It sets the parameters from the log file and handles user-defined
 * parameter overrides.
 *
 * @author Beat Kueng
*/

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/shutdown.h>
#include <lib/parameters/param.h>

#include <cstring>
#include <float.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <logger/messages.h>

#include "Replay.hpp"
#include "ReplayEkf2.hpp"

#define PARAMS_OVERRIDE_FILE PX4_ROOTFSDIR "/replay_params.txt"

using namespace std;
using namespace time_literals;

namespace px4
{

char *Replay::_replay_file = nullptr;

Replay::CompatSensorCombinedDtType::CompatSensorCombinedDtType(int gyro_integral_dt_offset_log,
		int gyro_integral_dt_offset_intern, int accelerometer_integral_dt_offset_log,
		int accelerometer_integral_dt_offset_intern)
	: _gyro_integral_dt_offset_log(gyro_integral_dt_offset_log),
	  _gyro_integral_dt_offset_intern(gyro_integral_dt_offset_intern),
	  _accelerometer_integral_dt_offset_log(accelerometer_integral_dt_offset_log),
	  _accelerometer_integral_dt_offset_intern(accelerometer_integral_dt_offset_intern)
{
}

Replay::~Replay()
{
	for (size_t i = 0; i < _subscriptions.size(); ++i) {
		delete (_subscriptions[i]);
	}

	_subscriptions.clear();
}

void *
Replay::CompatSensorCombinedDtType::apply(void *data)
{
	// the types have the same size so we can do the conversion in-place
	uint8_t *ptr = (uint8_t *)data;

	float gyro_integral_dt;
	memcpy(&gyro_integral_dt, ptr + _gyro_integral_dt_offset_log, sizeof(float));

	float accel_integral_dt;
	memcpy(&accel_integral_dt, ptr + _accelerometer_integral_dt_offset_log, sizeof(float));

	uint32_t igyro_integral_dt = (uint32_t)(gyro_integral_dt * 1e6f);
	memcpy(ptr + _gyro_integral_dt_offset_intern, &igyro_integral_dt, sizeof(float));

	uint32_t iaccel_integral_dt = (uint32_t)(accel_integral_dt * 1e6f);
	memcpy(ptr + _accelerometer_integral_dt_offset_intern, &iaccel_integral_dt, sizeof(float));

	return data;
}

void
Replay::setupReplayFile(const char *file_name)
{
	if (_replay_file) {
		free(_replay_file);
	}

	_replay_file = strdup(file_name);
}

void
Replay::setUserParams(const char *filename)
{
	string line, param_name, value_string;
	ifstream myfile(filename);

	if (!myfile.is_open()) {
		return;
	}

	PX4_INFO("Applying override params from %s...", filename);

	while (!myfile.eof()) {
		getline(myfile, line);

		if (line.empty() || line[0] == '#') {
			continue;
		}

		istringstream mystrstream(line);
		mystrstream >> param_name;
		mystrstream >> value_string;

		double param_value_double = stod(value_string);

		param_t handle = param_find(param_name.c_str());
		param_type_t param_format = param_type(handle);
		_overridden_params.insert(param_name);

		if (param_format == PARAM_TYPE_INT32) {
			int32_t value = 0;
			value = (int32_t)param_value_double;
			param_set(handle, (const void *)&value);

		} else if (param_format == PARAM_TYPE_FLOAT) {
			float value = 0;
			value = (float)param_value_double;
			param_set(handle, (const void *)&value);
		}
	}
}

bool
Replay::readFileHeader(std::ifstream &file)
{
	file.seekg(0);
	ulog_file_header_s msg_header;
	file.read((char *)&msg_header, sizeof(msg_header));

	if (!file) {
		return false;
	}

	_file_start_time = msg_header.timestamp;
	//verify it's an ULog file
	char magic[8];
	magic[0] = 'U';
	magic[1] = 'L';
	magic[2] = 'o';
	magic[3] = 'g';
	magic[4] = 0x01;
	magic[5] = 0x12;
	magic[6] = 0x35;
	return memcmp(magic, msg_header.magic, 7) == 0;
}

bool
Replay::readFileDefinitions(std::ifstream &file)
{
	PX4_INFO("Applying params from ULog file...");

	ulog_message_header_s message_header;
	file.seekg(sizeof(ulog_file_header_s));

	while (true) {
		file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

		if (!file) {
			return false;
		}

		switch (message_header.msg_type) {
		case (int)ULogMessageType::FLAG_BITS:
			if (!readFlagBits(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::FORMAT:
			if (!readFormat(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::PARAMETER:
			if (!readAndApplyParameter(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::ADD_LOGGED_MSG:
			_data_section_start = file.tellg() - (streamoff)ULOG_MSG_HEADER_LEN;
			return true;

		case (int)ULogMessageType::INFO: //skip
		case (int)ULogMessageType::INFO_MULTIPLE: //skip
			file.seekg(message_header.msg_size, ios::cur);
			break;

		default:
			PX4_ERR("unknown log definition type %i, size %i (offset %i)",
				(int)message_header.msg_type, (int)message_header.msg_size, (int)file.tellg());
			file.seekg(message_header.msg_size, ios::cur);
			break;
		}
	}

	return true;
}

bool
Replay::readFlagBits(std::ifstream &file, uint16_t msg_size)
{
	if (msg_size != 40) {
		PX4_ERR("unsupported message length for FLAG_BITS message (%i)", msg_size);
		return false;
	}

	_read_buffer.reserve(msg_size);
	uint8_t *message = (uint8_t *)_read_buffer.data();
	file.read((char *)message, msg_size);
	//uint8_t *compat_flags = message;
	uint8_t *incompat_flags = message + 8;

	// handle & validate the flags
	bool contains_appended_data = incompat_flags[0] & ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK;
	bool has_unknown_incompat_bits = false;

	if (incompat_flags[0] & ~0x1) {
		has_unknown_incompat_bits = true;
	}

	for (int i = 1; i < 8; ++i) {
		if (incompat_flags[i]) {
			has_unknown_incompat_bits = true;
		}
	}

	if (has_unknown_incompat_bits) {
		PX4_ERR("Log contains unknown incompat bits set. Refusing to parse");
		return false;
	}

	if (contains_appended_data) {
		uint64_t appended_offsets[3];
		memcpy(appended_offsets, message + 16, sizeof(appended_offsets));

		if (appended_offsets[0] > 0) {
			// the appended data is currently only used for hardfault dumps, so it's safe to ignore it.
			PX4_INFO("Log contains appended data. Replay will ignore this data");
			_read_until_file_position = appended_offsets[0];
		}
	}

	return true;
}

bool
Replay::readFormat(std::ifstream &file, uint16_t msg_size)
{
	_read_buffer.reserve(msg_size + 1);
	char *format = (char *)_read_buffer.data();
	file.read(format, msg_size);
	format[msg_size] = 0;

	if (!file) {
		return false;
	}

	string str_format(format);
	size_t pos = str_format.find(':');

	if (pos == string::npos) {
		return false;
	}

	string name = str_format.substr(0, pos);
	string fields = str_format.substr(pos + 1);
	_file_formats[name] = fields;

	return true;
}

bool
Replay::readAndAddSubscription(std::ifstream &file, uint16_t msg_size)
{
	_read_buffer.reserve(msg_size + 1);
	char *message = (char *)_read_buffer.data();
	streampos this_message_pos = file.tellg() - (streamoff)ULOG_MSG_HEADER_LEN;
	file.read(message, msg_size);
	message[msg_size] = 0;

	if (!file) {
		return false;
	}

	if (file.tellg() <= _subscription_file_pos) { //already read this subscription
		return true;
	}

	_subscription_file_pos = file.tellg();

	uint8_t multi_id = *(uint8_t *)message;
	uint16_t msg_id = ((uint16_t) message[1]) | (((uint16_t) message[2]) << 8);
	string topic_name(message + 3);
	const orb_metadata *orb_meta = findTopic(topic_name);

	if (!orb_meta) {
		PX4_WARN("Topic %s not found internally. Will ignore it", topic_name.c_str());
		return true;
	}

	CompatBase *compat = nullptr;

	// check the format: the field definitions must match
	// FIXME: this should check recursively, all used nested types
	string file_format = _file_formats[topic_name];

	if (file_format != orb_meta->o_fields) {
		// check if we have a compatibility conversion available
		if (topic_name == "sensor_combined") {
			if (string(orb_meta->o_fields) == "uint64_t timestamp;float[3] gyro_rad;uint32_t gyro_integral_dt;"
			    "int32_t accelerometer_timestamp_relative;float[3] accelerometer_m_s2;"
			    "uint32_t accelerometer_integral_dt" &&
			    file_format == "uint64_t timestamp;float[3] gyro_rad;float gyro_integral_dt;"
			    "int32_t accelerometer_timestamp_relative;float[3] accelerometer_m_s2;"
			    "float accelerometer_integral_dt;") {

				int gyro_integral_dt_offset_log;
				int gyro_integral_dt_offset_intern;
				int accelerometer_integral_dt_offset_log;
				int accelerometer_integral_dt_offset_intern;
				int unused;

				if (findFieldOffset(file_format, "gyro_integral_dt", gyro_integral_dt_offset_log, unused) &&
				    findFieldOffset(orb_meta->o_fields, "gyro_integral_dt", gyro_integral_dt_offset_intern, unused) &&
				    findFieldOffset(file_format, "accelerometer_integral_dt", accelerometer_integral_dt_offset_log, unused) &&
				    findFieldOffset(orb_meta->o_fields, "accelerometer_integral_dt", accelerometer_integral_dt_offset_intern, unused)) {

					compat = new CompatSensorCombinedDtType(gyro_integral_dt_offset_log, gyro_integral_dt_offset_intern,
										accelerometer_integral_dt_offset_log, accelerometer_integral_dt_offset_intern);
				}
			}
		}

		if (!compat) {
			PX4_WARN("Formats for %s don't match. Will ignore it.", topic_name.c_str());
			PX4_WARN(" Internal format: %s", orb_meta->o_fields);
			PX4_WARN(" File format    : %s", file_format.c_str());
			return true; // not a fatal error
		}
	}

	Subscription *subscription = new Subscription();
	subscription->orb_meta = orb_meta;
	subscription->multi_id = multi_id;
	subscription->compat = compat;

	//find the timestamp offset
	int field_size;
	bool timestamp_found = findFieldOffset(orb_meta->o_fields, "timestamp", subscription->timestamp_offset, field_size);

	if (!timestamp_found) {
		return true;
	}

	if (field_size != 8) {
		PX4_ERR("Unsupported timestamp with size %i, ignoring the topic %s", field_size, orb_meta->o_name);
		return true;
	}

	//find first data message (and the timestamp)
	streampos cur_pos = file.tellg();
	subscription->next_read_pos = this_message_pos; //this will be skipped

	if (!nextDataMessage(file, *subscription, msg_id)) {
		return false;
	}

	file.seekg(cur_pos);

	if (!subscription->orb_meta) {
		//no message found. This is not a fatal error
		return true;
	}

	PX4_DEBUG("adding subscription for %s (msg_id %i)", subscription->orb_meta->o_name, msg_id);

	//add subscription
	if (_subscriptions.size() <= msg_id) {
		_subscriptions.resize(msg_id + 1);
	}

	_subscriptions[msg_id] = subscription;

	onSubscriptionAdded(*_subscriptions[msg_id], msg_id);

	return true;
}

bool
Replay::findFieldOffset(const string &format, const string &field_name, int &offset, int &field_size)
{
	size_t prev_field_end = 0;
	size_t field_end = format.find(';');
	offset = 0;
	field_size = 0;

	while (field_end != string::npos) {
		size_t space_pos = format.find(' ', prev_field_end);

		if (space_pos != string::npos) {
			string type_name_full = format.substr(prev_field_end, space_pos - prev_field_end);
			string cur_field_name = format.substr(space_pos + 1, field_end - space_pos - 1);

			if (cur_field_name == field_name) {
				field_size = sizeOfFullType(type_name_full);
				return true;

			} else {
				offset += sizeOfFullType(type_name_full);
			}
		}

		prev_field_end = field_end + 1;
		field_end = format.find(';', prev_field_end);
	}

	return false;
}

bool
Replay::readAndHandleAdditionalMessages(std::ifstream &file, std::streampos end_position)
{
	ulog_message_header_s message_header;

	while (file.tellg() < end_position) {
		file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

		if (!file) {
			return false;
		}

		switch (message_header.msg_type) {
		case (int)ULogMessageType::PARAMETER:
			if (!readAndApplyParameter(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::DROPOUT:
			readDropout(file, message_header.msg_size);
			break;

		default: //skip all others
			file.seekg(message_header.msg_size, ios::cur);
			break;
		}
	}

	return true;
}

bool
Replay::readAndApplyParameter(std::ifstream &file, uint16_t msg_size)
{
	_read_buffer.reserve(msg_size);
	uint8_t *message = (uint8_t *)_read_buffer.data();
	file.read((char *)message, msg_size);

	if (!file) {
		return false;
	}

	uint8_t key_len = message[0];
	string key((char *)message + 1, key_len);

	size_t pos = key.find(' ');

	if (pos == string::npos) {
		return false;
	}

	string type = key.substr(0, pos);
	string param_name = key.substr(pos + 1);

	if (_overridden_params.find(param_name) != _overridden_params.end()) {
		//this parameter is overridden, so don't apply it
		return true;
	}

	if (type != "int32_t" && type != "float") {
		PX4_WARN("unknown parameter type %s, name %s (ignoring it)", type.c_str(), param_name.c_str());
		return true;
	}

	param_t handle = param_find(param_name.c_str());

	if (handle != PARAM_INVALID) {
		param_set(handle, (const void *)(message + 1 + key_len));
	}

	return true;
}

bool
Replay::readDropout(std::ifstream &file, uint16_t msg_size)
{
	uint16_t duration;
	file.read((char *)&duration, sizeof(duration));

	PX4_INFO("Dropout in replayed log, %i ms", (int)duration);
	return file.good();
}

bool
Replay::nextDataMessage(std::ifstream &file, Subscription &subscription, int msg_id)
{
	ulog_message_header_s message_header;
	file.seekg(subscription.next_read_pos);
	//ignore the first message (it's data we already read)
	file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

	if (file) {
		file.seekg(message_header.msg_size, ios::cur);
	}

	uint16_t file_msg_id;
	bool done = false;

	while (file && !done) {
		streampos cur_pos = file.tellg();
		file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

		if (!file) {
			break;
		}

		if (((streamoff)cur_pos) + ULOG_MSG_HEADER_LEN + message_header.msg_size > _read_until_file_position) {
			file.setstate(std::ios::eofbit);
			break;
		}

		switch (message_header.msg_type) {
		case (int)ULogMessageType::ADD_LOGGED_MSG:
			readAndAddSubscription(file, message_header.msg_size);
			break;

		case (int)ULogMessageType::DATA:
			file.read((char *)&file_msg_id, sizeof(file_msg_id));

			if (file) {
				if (msg_id == file_msg_id) {
					if (message_header.msg_size == subscription.orb_meta->o_size_no_padding + 2) {
						subscription.next_read_pos = cur_pos;
						file.seekg(subscription.timestamp_offset, ios::cur);
						file.read((char *)&subscription.next_timestamp, sizeof(subscription.next_timestamp));
						done = true;

					} else { //sanity check failed!
						PX4_ERR("data message %s has wrong size %i (expected %i). Skipping",
							subscription.orb_meta->o_name, message_header.msg_size,
							subscription.orb_meta->o_size_no_padding + 2);
						file.seekg(message_header.msg_size - sizeof(file_msg_id), ios::cur);
					}

				} else { //not the one we are looking for
					file.seekg(message_header.msg_size - sizeof(file_msg_id), ios::cur);
				}
			}

			break;

		case (int)ULogMessageType::REMOVE_LOGGED_MSG: //skip these
		case (int)ULogMessageType::PARAMETER:
		case (int)ULogMessageType::DROPOUT:
		case (int)ULogMessageType::INFO:
		case (int)ULogMessageType::INFO_MULTIPLE:
		case (int)ULogMessageType::SYNC:
		case (int)ULogMessageType::LOGGING:
			file.seekg(message_header.msg_size, ios::cur);
			break;

		default:
			//this really should not happen
			PX4_ERR("unknown log message type %i, size %i (offset %i)",
				(int)message_header.msg_type, (int)message_header.msg_size, (int)file.tellg());
			file.seekg(message_header.msg_size, ios::cur);
			break;
		}
	}

	if (file.eof()) { //no more data messages for this subscription
		subscription.orb_meta = nullptr;
		file.clear();
	}

	return file.good();
}

const orb_metadata *
Replay::findTopic(const std::string &name)
{
	const orb_metadata *const *topics = orb_get_topics();

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (name == topics[i]->o_name) {
			return topics[i];
		}
	}

	return nullptr;
}

std::string
Replay::extractArraySize(const std::string &type_name_full, int &array_size)
{
	size_t start_pos = type_name_full.find('[');
	size_t end_pos = type_name_full.find(']');

	if (start_pos == string::npos || end_pos == string::npos) {
		array_size = 1;
		return type_name_full;
	}

	array_size = atoi(type_name_full.substr(start_pos + 1, end_pos - start_pos - 1).c_str());
	return type_name_full.substr(0, start_pos);
}

size_t
Replay::sizeOfType(const std::string &type_name)
{
	if (type_name == "int8_t" || type_name == "uint8_t") {
		return 1;

	} else if (type_name == "int16_t" || type_name == "uint16_t") {
		return 2;

	} else if (type_name == "int32_t" || type_name == "uint32_t") {
		return 4;

	} else if (type_name == "int64_t" || type_name == "uint64_t") {
		return 8;

	} else if (type_name == "float") {
		return 4;

	} else if (type_name == "double") {
		return 8;

	} else if (type_name == "char" || type_name == "bool") {
		return 1;
	}

	const orb_metadata *orb_meta = findTopic(type_name);

	if (orb_meta) {
		return orb_meta->o_size;
	}

	PX4_ERR("unknown type: %s", type_name.c_str());
	return 0;
}

size_t
Replay::sizeOfFullType(const std::string &type_name_full)
{
	int array_size;
	string type_name = extractArraySize(type_name_full, array_size);
	return sizeOfType(type_name) * array_size;
}

bool
Replay::readDefinitionsAndApplyParams(std::ifstream &file)
{
	// log reader currently assumes little endian
	int num = 1;

	if (*(char *)&num != 1) {
		PX4_ERR("Replay only works on little endian!");
		return false;
	}

	if (!file.is_open()) {
		PX4_ERR("Failed to open replay file");
		return false;
	}

	if (!readFileHeader(file)) {
		PX4_ERR("Failed to read file header. Not a valid ULog file");
		return false;
	}

	//initialize the formats and apply the parameters from the log file
	if (!readFileDefinitions(file)) {
		PX4_ERR("Failed to read ULog definitions section. Broken file?");
		return false;
	}

	setUserParams(PARAMS_OVERRIDE_FILE);
	return true;
}

void
Replay::run()
{
	ifstream replay_file(_replay_file, ios::in | ios::binary);

	if (!readDefinitionsAndApplyParams(replay_file)) {
		return;
	}

	_speed_factor = 1.f;
	const char *speedup = getenv("PX4_SIM_SPEED_FACTOR");

	if (speedup) {
		_speed_factor = atof(speedup);
	}

	onEnterMainLoop();

	_replay_start_time = hrt_absolute_time();

	PX4_INFO("Replay in progress...");

	ulog_message_header_s message_header;
	replay_file.seekg(_data_section_start);

	//we know the next message must be an ADD_LOGGED_MSG
	replay_file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

	if (!readAndAddSubscription(replay_file, message_header.msg_size)) {
		PX4_ERR("Failed to read subscription");
		return;
	}

	const uint64_t timestamp_offset = getTimestampOffset();
	uint32_t nr_published_messages = 0;
	streampos last_additional_message_pos = _data_section_start;

	while (!should_exit() && replay_file) {

		//Find the next message to publish. Messages from different subscriptions don't need
		//to be in chronological order, so we need to check all subscriptions
		uint64_t next_file_time = 0;
		int next_msg_id = -1;
		bool first_time = true;

		for (size_t i = 0; i < _subscriptions.size(); ++i) {
			const Subscription *subscription = _subscriptions[i];

			if (!subscription) {
				continue;
			}

			if (subscription->orb_meta && !subscription->ignored) {
				if (first_time || subscription->next_timestamp < next_file_time) {
					first_time = false;
					next_msg_id = (int)i;
					next_file_time = subscription->next_timestamp;
				}
			}
		}

		if (next_msg_id == -1) {
			break; //no active subscription anymore. We're done.
		}

		Subscription &sub = *_subscriptions[next_msg_id];

		if (next_file_time == 0) {
			//someone didn't set the timestamp properly. Consider the message invalid
			nextDataMessage(replay_file, sub, next_msg_id);
			continue;
		}

		//handle additional messages between last and next published data
		replay_file.seekg(last_additional_message_pos);
		streampos next_additional_message_pos = sub.next_read_pos;
		readAndHandleAdditionalMessages(replay_file, next_additional_message_pos);
		last_additional_message_pos = next_additional_message_pos;

		const uint64_t publish_timestamp = handleTopicDelay(next_file_time, timestamp_offset);

		// It's time to publish
		readTopicDataToBuffer(sub, replay_file);
		memcpy(_read_buffer.data() + sub.timestamp_offset, &publish_timestamp, sizeof(uint64_t)); //adjust the timestamp

		if (handleTopicUpdate(sub, _read_buffer.data(), replay_file)) {
			++nr_published_messages;
		}

		nextDataMessage(replay_file, sub, next_msg_id);

		// TODO: output status (eg. every sec), including total duration...
	}

	for (auto &subscription : _subscriptions) {
		if (!subscription) {
			continue;
		}

		if (subscription->compat) {
			delete subscription->compat;
			subscription->compat = nullptr;
		}

		if (subscription->orb_advert) {
			orb_unadvertise(subscription->orb_advert);
			subscription->orb_advert = nullptr;
		}
	}

	if (!should_exit()) {
		PX4_INFO("Replay done (published %u msgs, %.3lf s)", nr_published_messages,
			 (double)hrt_elapsed_time(&_replay_start_time) / 1.e6);
	}

	onExitMainLoop();

	if (!should_exit()) {
		replay_file.close();
		px4_shutdown_request();
		// we need to ensure the shutdown logic gets updated and eventually triggers shutdown
		hrt_abstime t = hrt_absolute_time();

		for (int i = 0; i < 1000; ++i) {
			struct timespec ts;
			abstime_to_ts(&ts, t);
			px4_clock_settime(CLOCK_MONOTONIC, &ts);
			t += 10_ms;
		}
	}
}

void
Replay::readTopicDataToBuffer(const Subscription &sub, std::ifstream &replay_file)
{
	const size_t msg_read_size = sub.orb_meta->o_size_no_padding;
	const size_t msg_write_size = sub.orb_meta->o_size;
	_read_buffer.reserve(msg_write_size);
	replay_file.seekg(sub.next_read_pos + (streamoff)(ULOG_MSG_HEADER_LEN + 2)); //skip header & msg id
	replay_file.read((char *)_read_buffer.data(), msg_read_size);
}

bool
Replay::handleTopicUpdate(Subscription &sub, void *data, std::ifstream &replay_file)
{
	return publishTopic(sub, data);
}

uint64_t
Replay::handleTopicDelay(uint64_t next_file_time, uint64_t timestamp_offset)
{
	const uint64_t publish_timestamp = next_file_time + timestamp_offset;

	// wait if necessary
	uint64_t cur_time = hrt_absolute_time();

	// if some topics have a timestamp smaller than the log file start, publish them immediately
	if (cur_time < publish_timestamp && next_file_time > _file_start_time) {
		if (_speed_factor > FLT_EPSILON) {
			// avoid many small usleep calls
			_accumulated_delay += (publish_timestamp - cur_time) / _speed_factor;

			if (_accumulated_delay > 3000) {
				system_usleep(_accumulated_delay);
				_accumulated_delay = 0.f;
			}
		}

		// adjust the lockstep time to the publication time
		struct timespec ts;
		abstime_to_ts(&ts, publish_timestamp);
		px4_clock_settime(CLOCK_MONOTONIC, &ts);
	}

	return publish_timestamp;
}

bool
Replay::publishTopic(Subscription &sub, void *data)
{
	bool published = false;

	if (sub.compat) {
		data = sub.compat->apply(data);
	}

	if (sub.orb_advert) {
		orb_publish(sub.orb_meta, sub.orb_advert, data);
		published = true;

	} else {
		if (sub.multi_id == 0) {
			sub.orb_advert = orb_advertise(sub.orb_meta, data);
			published = true;

		} else {
			// make sure the other instances are advertised already so that we get the correct instance
			bool advertised = false;

			for (const auto &subscription : _subscriptions) {
				if (!subscription) {
					continue;
				}

				if (subscription->orb_meta) {
					if (strcmp(sub.orb_meta->o_name, subscription->orb_meta->o_name) == 0 &&
					    subscription->orb_advert && subscription->multi_id == sub.multi_id - 1) {
						advertised = true;
					}
				}
			}

			if (advertised) {
				int instance;
				sub.orb_advert = orb_advertise_multi(sub.orb_meta, data, &instance);
				published = true;
			}
		}
	}

	if (published) {
		++sub.publication_counter;
	}

	return published;
}

int
Replay::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "tryapplyparams")) {
		return Replay::applyParams(true);
	}

	if (!strcmp(argv[0], "trystart")) {
		return Replay::task_spawn(argc, argv);
	}

	return print_usage("unknown command");
}

int
Replay::task_spawn(int argc, char *argv[])
{
	// check if a log file was found
	if (!isSetup()) {
		if (argc > 0 && strncmp(argv[0], "try", 3) == 0) {
			return 0;
		}

		PX4_ERR("no log file given (via env variable %s)", replay::ENV_FILENAME);
		return -1;
	}

	_task_id = px4_task_spawn_cmd("replay",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX - 5,
				      4000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int
Replay::applyParams(bool quiet)
{
	if (!isSetup()) {
		if (quiet) {
			return -1;
		}

		PX4_ERR("no log file given (via env variable %s)", replay::ENV_FILENAME);
		return -1;
	}

	int ret = 0;
	Replay *r = new Replay();

	if (r == nullptr) {
		PX4_ERR("alloc failed");
		return -ENOMEM;
	}

	ifstream replay_file(_replay_file, ios::in | ios::binary);

	if (!r->readDefinitionsAndApplyParams(replay_file)) {
		ret = -1;
	}

	delete r;

	return ret;
}

Replay *
Replay::instantiate(int argc, char *argv[])
{
	// check the replay mode
	const char *replay_mode = getenv(replay::ENV_MODE);

	Replay *instance = nullptr;

	if (replay_mode && strcmp(replay_mode, "ekf2") == 0) {
		PX4_INFO("Ekf2 replay mode");
		instance = new ReplayEkf2();

	} else {
		instance = new Replay();
	}

	return instance;
}

int
Replay::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is used to replay ULog files.

There are 2 environment variables used for configuration: `replay`, which must be set to an ULog file name - it's
the log file to be replayed. The second is the mode, specified via `replay_mode`:
- `replay_mode=ekf2`: specific EKF2 replay mode. It can only be used with the ekf2 module, but allows the replay
  to run as fast as possible.
- Generic otherwise: this can be used to replay any module(s), but the replay will be done with the same speed as the
  log was recorded.

The module is typically used together with uORB publisher rules, to specify which messages should be replayed.
The replay module will just publish all messages that are found in the log. It also applies the parameters from
the log.

The replay procedure is documented on the [System-wide Replay](https://dev.px4.io/master/en/debug/system_wide_replay.html)
page.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("replay", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start replay, using log file from ENV variable 'replay'");
	PRINT_MODULE_USAGE_COMMAND_DESCR("trystart", "Same as 'start', but silently exit if no log file given");
	PRINT_MODULE_USAGE_COMMAND_DESCR("tryapplyparams", "Try to apply the parameters from the log file");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} //namespace px4
