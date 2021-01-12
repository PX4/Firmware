/****************************************************************************
 *
 *   Copyright (C) 2014-2020 PX4 Development Team. All rights reserved.
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

/// @file mavlink_ftp_test.cpp
///	@author Don Gagne <don@thegagnes.com>

#include <sys/stat.h>
#include <crc32.h>
#include <stdio.h>
#include <fcntl.h>

#include "mavlink_ftp_test.h"
#include "../mavlink_ftp.h"

#ifdef __PX4_NUTTX
#define PX4_MAVLINK_TEST_DATA_DIR "/fs/microsd/ftp_unit_test_data"
#else
#define PX4_MAVLINK_TEST_DATA_DIR "ftp_unit_test_data"
#endif

static const char *_test_files[] = {
	PX4_MAVLINK_TEST_DATA_DIR  "/" "test_238.data",
	PX4_MAVLINK_TEST_DATA_DIR  "/" "test_239.data",
	PX4_MAVLINK_TEST_DATA_DIR  "/" "test_240.data"
};

const MavlinkFtpTest::DownloadTestCase MavlinkFtpTest::_rgDownloadTestCases[] = {
	{ _test_files[0], MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::PayloadHeader) - 1,	true, false },	// Read takes less than single packet
	{ _test_files[1], MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::PayloadHeader),	true, true },	// Read completely fills single packet
	{ _test_files[2], MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::PayloadHeader) + 1,	false, false },	// Read take two packets
};

const char MavlinkFtpTest::_unittest_microsd_dir[] = PX4_STORAGEDIR "/ftp_unit_test_dir";
const char MavlinkFtpTest::_unittest_microsd_file[] = PX4_STORAGEDIR "/ftp_unit_test_dir/file";

MavlinkFtpTest::MavlinkFtpTest() :
	_ftp_server(nullptr),
	_expected_seq_number(0),
	_reply_msg{}
{
}

/// @brief Called before every test to initialize the FTP Server.
void MavlinkFtpTest::_init()
{
	_expected_seq_number = 0;
	_ftp_server = new MavlinkFTP(nullptr);
	_ftp_server->set_unittest_worker(MavlinkFtpTest::receive_message_handler_generic, this);

	_create_test_files();

	_cleanup_microsd();
}

bool MavlinkFtpTest::_create_test_files()
{
	int ret = ::mkdir(PX4_MAVLINK_TEST_DATA_DIR, S_IRWXU | S_IRWXG | S_IRWXO);
	ut_assert("mkdir failed", ret == 0 || errno == EEXIST);

	ret = ::mkdir(PX4_MAVLINK_TEST_DATA_DIR "/empty_dir", S_IRWXU | S_IRWXG | S_IRWXO);
	ut_assert("mkdir failed", ret == 0 || errno == EEXIST);

	bool failed = false;

	for (int i = 0; i < 3; ++i) {
		int fd = ::open(_test_files[i], O_CREAT | O_EXCL | O_WRONLY, S_IRWXU | S_IRWXG | S_IRWXO);

		if (fd < 0) {
			printf("fd: %d, error: %s\n", fd, strerror(errno));
			ut_assert("Open failed", fd != -1);
		}

		// We create 3 files, with bytes counting from 0 to 238, 239, and 240.
		uint8_t len = 238 + i;

		for (uint8_t c = 0; c < len; ++c) {
			ret = ::write(fd, &c, 1);

			if (ret != 1) {
				failed = true;
			}
		}

		close(fd);
	}

	ut_assert("Could not write test file", !failed);

	return !failed;
}

/// @brief Called after every test to take down the FTP Server.
void MavlinkFtpTest::_cleanup()
{
	delete _ftp_server;

	_cleanup_microsd();
	_remove_test_files();
}

bool MavlinkFtpTest::_remove_test_files()
{
	for (int i = 0; i < 3; ++i) {
		::unlink(_test_files[i]);
	}

	::rmdir(PX4_MAVLINK_TEST_DATA_DIR "/empty_dir");
	::rmdir(PX4_MAVLINK_TEST_DATA_DIR);

	return true;
}


/// @brief Tests for correct behavior of an Ack response.
bool MavlinkFtpTest::_ack_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;

	payload.opcode = MavlinkFTP::kCmdNone;

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 0,		// size in bytes of data
					 nullptr,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
	ut_compare("Incorrect payload size", reply->size, 0);

	return true;
}

/// @brief Tests for correct response to an invalid opcode.
bool MavlinkFtpTest::_bad_opcode_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;

	payload.opcode = 0xFF;	// bogus opcode

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 0,		// size in bytes of data
					 nullptr,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrUnknownCommand);

	return true;
}

/// @brief Tests for correct reponse to a payload which an invalid data size field.
bool MavlinkFtpTest::_bad_datasize_test()
{
	mavlink_message_t			msg;
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;

	payload.opcode = MavlinkFTP::kCmdListDirectory;

	_setup_ftp_msg(&payload, 0, nullptr, &msg);

	// Set the data size to be one larger than is legal
	((MavlinkFTP::PayloadHeader *)((mavlink_file_transfer_protocol_t *)msg.payload64)->payload)->size =
		MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN + 1;

	_ftp_server->handle_message(&msg);

	if (!_decode_message(&_reply_msg, &reply)) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrInvalidDataSize);

	return true;
}

bool MavlinkFtpTest::_list_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;

	struct _testCase {
		const char	*dir;		///< Directory to run List command on
		const char	*response;	///< Expected response entries from List command
		int		response_count;	///< Number of directories that should be returned
		bool		success;	///< true: List command should succeed, false: List command should fail
	};
	struct _testCase rgTestCases[] = {
		{ "/bogus",			nullptr,		0,	false },
#ifdef __PX4_NUTTX
		{ PX4_MAVLINK_TEST_DATA_DIR,	"Dempty_dir|Ftest_238.data\t238|Ftest_239.data\t239|Ftest_240.data\t240", 	4,	true },
#else
		{ PX4_MAVLINK_TEST_DATA_DIR,	"Dempty_dir|Ftest_238.data\t238|Ftest_239.data\t239|Ftest_240.data\t240|S|S",	6,	true },   // readdir on Linux adds . and ..
#endif
	};

	for (size_t i = 0; i < sizeof(rgTestCases) / sizeof(rgTestCases[0]); i++) {
		const struct _testCase *test = &rgTestCases[i];

		payload.opcode = MavlinkFTP::kCmdListDirectory;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->dir) + 1,	// size in bytes of data
						 (uint8_t *)test->dir,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		if (test->success) {
			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Incorrect payload size", reply->size, strlen(test->response) + 1);

			// The return order of directories from the List command is not repeatable. So we can't do a direct comparison
			// to a hardcoded return result string.

			// Convert null terminators to seperator char so we can use strok to parse returned data
			char list_entry[256];

			for (uint8_t j = 0; j < reply->size - 1; j++) {
				if (reply->data[j] == 0) {
					list_entry[j] = '|';

				} else {
					list_entry[j] = reply->data[j];
				}
			}

			list_entry[reply->size - 1] = 0;

			// Loop over returned directory entries trying to find then in the response list
			char *dir;
			int response_count = 0;
			dir = strtok(list_entry, "|");

			while (dir != nullptr) {
				ut_assert("Returned directory not found in expected response", strstr(test->response, dir));
				response_count++;
				dir = strtok(nullptr, "|");
			}

			ut_compare("Incorrect number of directory entires returned", test->response_count, response_count);

		} else {
			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
			ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrFileNotFound);
			ut_compare("Incorrect payload size", reply->size, 1);
		}
	}

	return true;
}

/// @brief Tests for correct response to a List command on a valid directory, but with an offset that
/// is beyond the last directory entry.
bool MavlinkFtpTest::_list_eof_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;
	const char				*dir = PX4_MAVLINK_TEST_DATA_DIR;

	payload.opcode = MavlinkFTP::kCmdListDirectory;
#ifdef __PX4_NUTTX
	payload.offset = 4;	// (3 test files, 1 test folder)
#else
	payload.offset = 6;	// (3 test files, 1 test folder, two skipped ./..)
#endif

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 strlen(dir) + 1,	// size in bytes of data
					 (uint8_t *)dir,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrEOF);

	return true;
}

/// @brief Tests for correct response to an Open command on a file which does not exist.
bool MavlinkFtpTest::_open_badfile_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;
	const char				*dir = "/foo";	// non-existent file

	payload.opcode = MavlinkFTP::kCmdOpenFileRO;
	payload.offset = 0;

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 strlen(dir) + 1,	// size in bytes of data
					 (uint8_t *)dir,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrFileNotFound);

	return true;
}

/// @brief Tests for correct reponse to an Open command on a file, followed by Terminate
bool MavlinkFtpTest::_open_terminate_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;

	for (size_t i = 0; i < sizeof(_rgDownloadTestCases) / sizeof(_rgDownloadTestCases[0]); i++) {
		struct stat st;
		const DownloadTestCase *test = &_rgDownloadTestCases[i];

		payload.opcode = MavlinkFTP::kCmdOpenFileRO;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->file) + 1,	// size in bytes of data
						 (uint8_t *)test->file,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		ut_compare("stat failed", stat(test->file, &st), 0);

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Incorrect payload size", reply->size, sizeof(uint32_t));
		ut_compare("File size incorrect", *((uint32_t *)&reply->data[0]), (uint32_t)st.st_size);

		payload.opcode = MavlinkFTP::kCmdTerminateSession;
		payload.session = reply->session;
		payload.size = 0;

		success = _send_receive_msg(&payload,	// FTP payload header
					    0,		// size in bytes of data
					    nullptr,	// Data to start into FTP message payload
					    &reply);	// Payload inside FTP message response

		if (!success) {
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Incorrect payload size", reply->size, 0);
	}

	return true;
}

/// @brief Tests for correct reponse to a Terminate command on an invalid session.
bool MavlinkFtpTest::_terminate_badsession_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;
	const char				*file = _rgDownloadTestCases[0].file;

	payload.opcode = MavlinkFTP::kCmdOpenFileRO;
	payload.offset = 0;

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 strlen(file) + 1,	// size in bytes of data
					 (uint8_t *)file,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);

	payload.opcode = MavlinkFTP::kCmdTerminateSession;
	payload.session = reply->session + 1;
	payload.size = 0;

	success = _send_receive_msg(&payload,	// FTP payload header
				    0,		// size in bytes of data
				    nullptr,	// Data to start into FTP message payload
				    &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrInvalidSession);

	return true;
}

/// @brief Tests for correct reponse to a Read command on an open session.
bool MavlinkFtpTest::_read_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;

	for (size_t i = 0; i < sizeof(_rgDownloadTestCases) / sizeof(_rgDownloadTestCases[0]); i++) {
		struct stat st;
		const DownloadTestCase *test = &_rgDownloadTestCases[i];

		// Read in the file so we can compare it to what we get back
		ut_compare("stat failed", stat(test->file, &st), 0);
		uint8_t *bytes = new uint8_t[st.st_size];
		ut_assert("new failed", bytes != nullptr);
		int fd = ::open(test->file, O_RDONLY);
		ut_assert("open failed", fd != -1);
		int bytes_read = ::read(fd, bytes, st.st_size);
		ut_compare("read failed", bytes_read, st.st_size);
		::close(fd);

		// Test case data files are created for specific boundary conditions
		ut_compare("Test case data files are out of date", test->length, st.st_size);

		payload.opcode = MavlinkFTP::kCmdOpenFileRO;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->file) + 1,	// size in bytes of data
						 (uint8_t *)test->file,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			delete[] bytes;
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);

		payload.opcode = MavlinkFTP::kCmdReadFile;
		payload.session = reply->session;
		payload.offset = 0;

		success = _send_receive_msg(&payload,	// FTP payload header
					    0,		// size in bytes of data
					    nullptr,	// Data to start into FTP message payload
					    &reply);	// Payload inside FTP message response

		if (!success) {
			delete[] bytes;
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Offset incorrect", reply->offset, 0);

		uint32_t full_packet_bytes = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::PayloadHeader);
		uint32_t expected_bytes = test->singlePacketRead ? (uint32_t)st.st_size : full_packet_bytes;
		ut_compare("Payload size incorrect", reply->size, expected_bytes);
		ut_compare("File contents differ", memcmp(reply->data, bytes, expected_bytes), 0);

		payload.offset += expected_bytes;

		if (test->singlePacketRead) {
			// Try going past EOF
			success = _send_receive_msg(&payload,	// FTP payload header
						    0,		// size in bytes of data
						    nullptr,	// Data to start into FTP message payload
						    &reply);	// Payload inside FTP message response

			if (!success) {
				delete[] bytes;
				return false;
			}

			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);

		} else {
			success = _send_receive_msg(&payload,	// FTP payload header
						    0,		// size in bytes of data
						    nullptr,	// Data to start into FTP message payload
						    &reply);	// Payload inside FTP message response

			if (!success) {
				delete[] bytes;
				return false;
			}

			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Offset incorrect", reply->offset, payload.offset);

			expected_bytes = (uint32_t)st.st_size - full_packet_bytes;
			ut_compare("Payload size incorrect", reply->size, expected_bytes);
			ut_compare("File contents differ", memcmp(reply->data, &bytes[payload.offset], expected_bytes), 0);
		}

		payload.opcode = MavlinkFTP::kCmdTerminateSession;
		payload.session = reply->session;
		payload.size = 0;

		success = _send_receive_msg(&payload,	// FTP payload header
					    0,		// size in bytes of data
					    nullptr,	// Data to start into FTP message payload
					    &reply);	// Payload inside FTP message response

		if (!success) {
			delete[] bytes;
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Incorrect payload size", reply->size, 0);

		delete[] bytes;
		bytes = nullptr;
	}

	return true;
}

/// @brief Tests for correct reponse to a Read command on an open session.
bool MavlinkFtpTest::_burst_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;
	BurstInfo				burst_info;



	for (size_t i = 0; i < sizeof(_rgDownloadTestCases) / sizeof(_rgDownloadTestCases[0]); i++) {
		struct stat st;
		const DownloadTestCase *test = &_rgDownloadTestCases[i];

		// Read in the file so we can compare it to what we get back
		ut_compare("stat failed", stat(test->file, &st), 0);
		uint8_t *bytes = new uint8_t[st.st_size];
		ut_assert("new failed", bytes != nullptr);
		int fd = ::open(test->file, O_RDONLY);
		ut_assert("open failed", fd != -1);
		int bytes_read = ::read(fd, bytes, st.st_size);
		ut_compare("read failed", bytes_read, st.st_size);
		::close(fd);

		// Test case data files are created for specific boundary conditions
		ut_compare("Test case data files are out of date", test->length, st.st_size);

		payload.opcode = MavlinkFTP::kCmdOpenFileRO;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->file) + 1,	// size in bytes of data
						 (uint8_t *)test->file,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			delete[] bytes;
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);

		// Setup for burst response handler
		burst_info.burst_state = burst_state_first_ack;
		burst_info.single_packet_file = test->singlePacketRead;
		burst_info.file_size = st.st_size;
		burst_info.file_bytes = bytes;
		burst_info.ftp_test_class = this;
		_ftp_server->set_unittest_worker(MavlinkFtpTest::receive_message_handler_burst, &burst_info);

		// Send the burst command, message response will be handled by _receive_message_handler_stream
		payload.opcode = MavlinkFTP::kCmdBurstReadFile;
		payload.session = reply->session;
		payload.offset = 0;

		mavlink_message_t msg;
		_setup_ftp_msg(&payload, 0, nullptr, &msg);
		_ftp_server->handle_message(&msg);

		// First packet is sent using stream mechanism, so we need to force it out ourselves
		_ftp_server->send();

		ut_compare("Incorrect sequence of messages", burst_info.burst_state, burst_state_complete);

		// Put back generic message handler
		_ftp_server->set_unittest_worker(MavlinkFtpTest::receive_message_handler_generic, this);

		// Terminate session
		payload.opcode = MavlinkFTP::kCmdTerminateSession;
		payload.session = reply->session;
		payload.size = 0;

		success = _send_receive_msg(&payload,	// FTP payload header
					    0,		// size in bytes of data
					    nullptr,	// Data to start into FTP message payload
					    &reply);	// Payload inside FTP message response

		if (!success) {
			delete[] bytes;
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Incorrect payload size", reply->size, 0);

		delete[] bytes;
		bytes = nullptr;
	}

	return true;
}

/// @brief Tests for correct reponse to a Read command on an invalid session.
bool MavlinkFtpTest::_read_badsession_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;
	const char				*file = _rgDownloadTestCases[0].file;

	payload.opcode = MavlinkFTP::kCmdOpenFileRO;
	payload.offset = 0;

	bool success = _send_receive_msg(&payload,		// FTP payload header
					 strlen(file) + 1,	// size in bytes of data
					 (uint8_t *)file,	// Data to start into FTP message payload
					 &reply);		// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);

	payload.opcode = MavlinkFTP::kCmdReadFile;
	payload.session = reply->session + 1;	// Invalid session
	payload.offset = 0;

	success = _send_receive_msg(&payload,	// FTP payload header
				    0,		// size in bytes of data
				    nullptr,	// Data to start into FTP message payload
				    &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrInvalidSession);

	return true;
}

bool MavlinkFtpTest::_removedirectory_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;
	int					fd;

	struct _testCase {
		const char	*dir;
		bool		success;
		bool		deleteFile;
		uint8_t		reply_size;
		uint8_t		error_code;
	};
	static const struct _testCase rgTestCases[] = {
		{ "/bogus",						false,	false, 1, MavlinkFTP::kErrFileNotFound },
		{ _unittest_microsd_dir,				false,	false, 2, MavlinkFTP::kErrFailErrno },
		{ _unittest_microsd_file,				false,	false, 2, MavlinkFTP::kErrFailErrno },
		{ _unittest_microsd_dir,				true,	true, 0, MavlinkFTP::kErrNone },
	};

	ut_compare("mkdir failed", ::mkdir(_unittest_microsd_dir, S_IRWXU | S_IRWXG | S_IRWXO), 0);
	ut_assert("open failed", (fd = ::open(_unittest_microsd_file, O_CREAT | O_EXCL, S_IRWXU | S_IRWXG | S_IRWXO)) != -1);
	::close(fd);

	for (size_t i = 0; i < sizeof(rgTestCases) / sizeof(rgTestCases[0]); i++) {
		const struct _testCase *test = &rgTestCases[i];

		if (test->deleteFile) {
			ut_compare("unlink failed", ::unlink(_unittest_microsd_file), 0);
		}

		payload.opcode = MavlinkFTP::kCmdRemoveDirectory;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->dir) + 1,	// size in bytes of data
						 (uint8_t *)test->dir,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		if (test->success) {
			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Incorrect payload size", reply->size, test->reply_size);

		} else {
			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
			ut_compare("Incorrect payload size", reply->size, test->reply_size);
			ut_compare("Incorrect error code", reply->data[0], test->error_code);
		}
	}

	return true;
}

bool MavlinkFtpTest::_createdirectory_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;

	struct _testCase {
		const char	*dir;
		bool		success;
		uint8_t		reply_size;
		uint8_t		error_code;
	};
	static const struct _testCase rgTestCases[] = {
		{ _unittest_microsd_dir,	true,	0, MavlinkFTP::kErrNone},
		{ _unittest_microsd_dir,	false,	1, MavlinkFTP::kErrFailFileExists},
#ifdef __PX4_NUTTX
		{ PX4_MAVLINK_TEST_DATA_DIR "/bogus/bogus",	false,	2, MavlinkFTP::kErrFailErrno} // on NuttX missing folders is EIO
#else
		{ PX4_MAVLINK_TEST_DATA_DIR "/bogus/bogus",	false,	1, MavlinkFTP::kErrFileNotFound} // on Linux it is ENOENT
#endif
	};

	for (size_t i = 0; i < sizeof(rgTestCases) / sizeof(rgTestCases[0]); i++) {
		const struct _testCase *test = &rgTestCases[i];

		payload.opcode = MavlinkFTP::kCmdCreateDirectory;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->dir) + 1,	// size in bytes of data
						 (uint8_t *)test->dir,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		if (test->success) {
			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Incorrect payload size", reply->size, test->reply_size);

		} else {
			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
			ut_compare("Incorrect error code", reply->data[0], test->error_code);
			ut_compare("Incorrect payload size", reply->size, test->reply_size);
		}
	}

	return true;
}

bool MavlinkFtpTest::_removefile_test()
{
	MavlinkFTP::PayloadHeader		payload;
	const MavlinkFTP::PayloadHeader		*reply;
	int					fd;

	struct _testCase {
		const char	*file;
		bool		success;
		uint8_t		reply_size;
		uint8_t		error_code;
	};
	static const struct _testCase rgTestCases[] = {
		{ "/bogus",			false, 1, MavlinkFTP::kErrFileNotFound },
		{ _unittest_microsd_dir,	false, 2, MavlinkFTP::kErrFailErrno },
		{ _unittest_microsd_file,	true,  0, MavlinkFTP::kErrNone },
		{ _unittest_microsd_file,	false, 1, MavlinkFTP::kErrFileNotFound },
	};

	ut_compare("mkdir failed", ::mkdir(_unittest_microsd_dir, S_IRWXU | S_IRWXG | S_IRWXO), 0);
	ut_assert("open failed", (fd = ::open(_unittest_microsd_file, O_CREAT | O_EXCL, S_IRWXU | S_IRWXG | S_IRWXO)) != -1);
	::close(fd);

	for (size_t i = 0; i < sizeof(rgTestCases) / sizeof(rgTestCases[0]); i++) {
		const struct _testCase *test = &rgTestCases[i];

		payload.opcode = MavlinkFTP::kCmdRemoveFile;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->file) + 1,	// size in bytes of data
						 (uint8_t *)test->file,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		if (test->success) {
			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Incorrect payload size", reply->size, test->reply_size);

		} else {
			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
			ut_compare("Incorrect payload size", reply->size, test->reply_size);
			ut_compare("Incorrect error code", reply->data[0], test->error_code);
		}
	}

	return true;
}

/// Static method used as callback from MavlinkFTP for generic use. This method will be called by MavlinkFTP when
/// it needs to send a message out on Mavlink.
void MavlinkFtpTest::receive_message_handler_generic(const mavlink_file_transfer_protocol_t *ftp_req, void *worker_data)
{
	((MavlinkFtpTest *)worker_data)->_receive_message_handler_generic(ftp_req);
}

void MavlinkFtpTest::_receive_message_handler_generic(const mavlink_file_transfer_protocol_t *ftp_req)
{
	// Move the message into our own member variable
	memcpy(&_reply_msg, ftp_req, sizeof(mavlink_file_transfer_protocol_t));
}

/// Static method used as callback from MavlinkFTP for stream download testing. This method will be called by MavlinkFTP when
/// it needs to send a message out on Mavlink.
void MavlinkFtpTest::receive_message_handler_burst(const mavlink_file_transfer_protocol_t *ftp_req, void *worker_data)
{
	BurstInfo *burst_info = (BurstInfo *)worker_data;
	burst_info->ftp_test_class->_receive_message_handler_burst(ftp_req, burst_info);
}

bool MavlinkFtpTest::_receive_message_handler_burst(const mavlink_file_transfer_protocol_t *ftp_msg,
		BurstInfo *burst_info)
{
	const MavlinkFTP::PayloadHeader *reply{nullptr};
	uint32_t full_packet_bytes = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::PayloadHeader);
	uint32_t expected_bytes;

	_decode_message(ftp_msg, &reply);

	switch (burst_info->burst_state) {
	case burst_state_first_ack:
		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Offset incorrect", reply->offset, 0);

		expected_bytes = burst_info->single_packet_file ? burst_info->file_size : full_packet_bytes;
		ut_compare("Payload size incorrect", reply->size, expected_bytes);
		ut_compare("burst_complete incorrect", reply->burst_complete, 0);
		ut_compare("File contents differ", memcmp(reply->data, burst_info->file_bytes, expected_bytes), 0);

		// Setup for next expected message
		burst_info->burst_state = burst_info->single_packet_file ? burst_state_nak_eof : burst_state_last_ack;

		ut_assert("Remaining stream packets missing", _ftp_server->get_size());
		_ftp_server->send();
		break;

	case burst_state_last_ack:
		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Offset incorrect", reply->offset, full_packet_bytes);

		expected_bytes = burst_info->file_size - full_packet_bytes;
		ut_compare("Payload size incorrect", reply->size, expected_bytes);
		ut_compare("burst_complete incorrect", reply->burst_complete, 0);
		ut_compare("File contents differ", memcmp(reply->data, &burst_info->file_bytes[full_packet_bytes], expected_bytes), 0);

		// Setup for next expected message
		burst_info->burst_state = burst_state_nak_eof;

		ut_assert("Remaining stream packets missing", _ftp_server->get_size());
		_ftp_server->send();
		break;

	case burst_state_nak_eof:
		// Signal complete
		burst_info->burst_state = burst_state_complete;
		ut_compare("All packets should have been seent", _ftp_server->get_size(), 0);
		break;

	}

	return true;
}

/// @brief Decode and validate the incoming message
bool MavlinkFtpTest::_decode_message(const mavlink_file_transfer_protocol_t	*ftp_msg,	///< Incoming FTP message
				     const MavlinkFTP::PayloadHeader		**payload)	///< Payload inside FTP message response
{
	//warnx("_decode_message");

	// Make sure the targets are correct
	ut_compare("Target network non-zero", ftp_msg->target_network, 0);
	ut_compare("Target system id mismatch", ftp_msg->target_system, clientSystemId);
	ut_compare("Target component id mismatch", ftp_msg->target_component, clientComponentId);

	*payload = reinterpret_cast<const MavlinkFTP::PayloadHeader *>(ftp_msg->payload);

	// Make sure we have a good sequence number
	ut_compare("Sequence number mismatch", (*payload)->seq_number, _expected_seq_number);
	_expected_seq_number++;

	return true;
}

/// @brief Initializes an FTP message into a mavlink message
void MavlinkFtpTest::_setup_ftp_msg(const MavlinkFTP::PayloadHeader	*payload_header,	///< FTP payload header
				    uint8_t				size,			///< size in bytes of data
				    const uint8_t			*data,			///< Data to start into FTP message payload
				    mavlink_message_t			*msg)			///< Returned mavlink message
{
	uint8_t payload_bytes[MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN];
	MavlinkFTP::PayloadHeader *payload = reinterpret_cast<MavlinkFTP::PayloadHeader *>(payload_bytes);

	memcpy(payload, payload_header, sizeof(MavlinkFTP::PayloadHeader));

	payload->seq_number = _expected_seq_number++;
	payload->size = size;

	if (size != 0) {
		memcpy(payload->data, data, size);
	}

	payload->burst_complete = 0;
	payload->padding = 0;

	msg->checksum = 0;
	mavlink_msg_file_transfer_protocol_pack(clientSystemId,		// Sender system id
						clientComponentId,	// Sender component id
						msg,			// Message to pack payload into
						0,			// Target network
						serverSystemId,		// Target system id
						serverComponentId,	// Target component id
						payload_bytes);		// Payload to pack into message
}

/// @brief Sends the specified FTP message to the server and returns response
bool MavlinkFtpTest::_send_receive_msg(MavlinkFTP::PayloadHeader	*payload_header,	///< FTP payload header
				       uint8_t				size,			///< size in bytes of data
				       const uint8_t			*data,			///< Data to start into FTP message payload
				       const MavlinkFTP::PayloadHeader	**payload_reply)	///< Payload inside FTP message response
{
	mavlink_message_t msg;

	_setup_ftp_msg(payload_header, size, data, &msg);
	_ftp_server->handle_message(&msg);
	return _decode_message(&_reply_msg, payload_reply);
}

/// @brief Cleans up an files created on microsd during testing
void MavlinkFtpTest::_cleanup_microsd()
{
	::unlink(_unittest_microsd_file);
	::rmdir(_unittest_microsd_dir);
}

/// @brief Runs all the unit tests
bool MavlinkFtpTest::run_tests()
{
	ut_run_test(_ack_test);
	ut_run_test(_bad_opcode_test);
	ut_run_test(_bad_datasize_test);
	ut_run_test(_list_test);
	ut_run_test(_list_eof_test);
	ut_run_test(_open_badfile_test);
	ut_run_test(_open_terminate_test);
	ut_run_test(_terminate_badsession_test);
	ut_run_test(_read_test);
	ut_run_test(_read_badsession_test);
	ut_run_test(_burst_test);
	ut_run_test(_removedirectory_test);
	ut_run_test(_createdirectory_test);
	ut_run_test(_removefile_test);

	return (_tests_failed == 0);

}

ut_declare_test(mavlink_ftp_test, MavlinkFtpTest)
