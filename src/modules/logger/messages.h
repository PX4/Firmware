/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cstdint>

enum class ULogMessageType : uint8_t {
	FORMAT = 'F',
	DATA = 'D',
	INFO = 'I',
	INFO_MULTIPLE = 'M',
	PARAMETER = 'P',
	PARAMETER_DEFAULT = 'Q',
	ADD_LOGGED_MSG = 'A',
	REMOVE_LOGGED_MSG = 'R',
	SYNC = 'S',
	DROPOUT = 'O',
	LOGGING = 'L',
	LOGGING_TAGGED = 'C',
	FLAG_BITS = 'B',
};


/* declare message data structs with byte alignment (no padding) */
#pragma pack(push, 1)

/** first bytes of the file */
struct ulog_file_header_s {
	uint8_t magic[8];
	uint64_t timestamp;
};

#define ULOG_MSG_HEADER_LEN 3 //accounts for msg_size and msg_type
struct ulog_message_header_s {
	uint16_t msg_size;
	uint8_t msg_type;
};

struct ulog_message_format_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::FORMAT);

	char format[1500];
};

struct ulog_message_add_logged_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::ADD_LOGGED_MSG);

	uint8_t multi_id;
	uint16_t msg_id;
	char message_name[255];
};

struct ulog_message_remove_logged_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::REMOVE_LOGGED_MSG);

	uint16_t msg_id;
};

struct ulog_message_sync_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::SYNC);

	uint8_t sync_magic[8];
};

struct ulog_message_dropout_s {
	uint16_t msg_size = sizeof(uint16_t); //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::DROPOUT);

	uint16_t duration; //in ms
};

struct ulog_message_data_header_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::DATA);

	uint16_t msg_id;
};

struct ulog_message_info_header_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	uint8_t key_len;
	char key[255];
};

struct ulog_message_info_multiple_header_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::INFO_MULTIPLE);

	uint8_t is_continued; ///< can be used for arrays: set to 1, if this message is part of the previous with the same key
	uint8_t key_len;
	char key[255];
};

struct ulog_message_logging_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::LOGGING);

	uint8_t log_level; //same levels as in the linux kernel
	uint64_t timestamp;
	char message[128]; //defines the maximum length of a logged message string
};

struct ulog_message_logging_tagged_s {
	uint16_t msg_size; //size of message - ULOG_MSG_HEADER_LEN
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::LOGGING_TAGGED);

	uint8_t log_level; //same levels as in the linux kernel
	uint16_t tag;
	uint64_t timestamp;
	char message[128]; //defines the maximum length of a logged message string
};

struct ulog_message_parameter_header_s {
	uint16_t msg_size;
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);

	uint8_t key_len;
	char key[255];
};

enum class ulog_parameter_default_type_t : uint8_t {
	system = (1 << 0),
	current_setup = (1 << 1) //airframe default
};

inline ulog_parameter_default_type_t operator|(ulog_parameter_default_type_t a, ulog_parameter_default_type_t b)
{
	return static_cast<ulog_parameter_default_type_t>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

struct ulog_message_parameter_default_header_s {
	uint16_t msg_size;
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER_DEFAULT);

	ulog_parameter_default_type_t default_types;
	uint8_t key_len;
	char key[255];
};


#define ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK (1<<0)

#define ULOG_COMPAT_FLAG0_DEFAULT_PARAMETERS_MASK (1<<0)

struct ulog_message_flag_bits_s {
	uint16_t msg_size;
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::FLAG_BITS);

	uint8_t compat_flags[8];
	uint8_t incompat_flags[8]; ///< @see ULOG_INCOMPAT_FLAG_*
	uint64_t appended_offsets[3]; ///< file offset(s) for appended data if ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK is set
};

#pragma pack(pop)
