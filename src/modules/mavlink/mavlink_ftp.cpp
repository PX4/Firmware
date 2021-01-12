/****************************************************************************
 *
 *   Copyright (c) 2014-2020 PX4 Development Team. All rights reserved.
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

/// @file mavlink_ftp.cpp
///	@author px4dev, Don Gagne <don@thegagnes.com>

#include <crc32.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>

#include "mavlink_ftp.h"
#include "mavlink_tests/mavlink_ftp_test.h"

#ifndef MAVLINK_FTP_UNIT_TEST
#include "mavlink_main.h"
#else
#include <v2.0/standard/mavlink.h>
#endif

using namespace time_literals;

constexpr const char MavlinkFTP::_root_dir[];

MavlinkFTP::MavlinkFTP(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	// initialize session
	_session_info.fd = -1;
}

MavlinkFTP::~MavlinkFTP()
{
	delete[] _work_buffer1;
	delete[] _work_buffer2;
}

unsigned
MavlinkFTP::get_size()
{
	if (_session_info.stream_download) {
		return MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	} else {
		return 0;
	}
}

#ifdef MAVLINK_FTP_UNIT_TEST
void
MavlinkFTP::set_unittest_worker(ReceiveMessageFunc_t rcvMsgFunc, void *worker_data)
{
	_utRcvMsgFunc = rcvMsgFunc;
	_worker_data = worker_data;
}
#endif

uint8_t
MavlinkFTP::_getServerSystemId()
{
#ifdef MAVLINK_FTP_UNIT_TEST
	// We use fake ids when unit testing
	return MavlinkFtpTest::serverSystemId;
#else
	// Not unit testing, use the real thing
	return _mavlink->get_system_id();
#endif
}

uint8_t
MavlinkFTP::_getServerComponentId()
{
#ifdef MAVLINK_FTP_UNIT_TEST
	// We use fake ids when unit testing
	return MavlinkFtpTest::serverComponentId;
#else
	// Not unit testing, use the real thing
	return _mavlink->get_component_id();
#endif
}

uint8_t
MavlinkFTP::_getServerChannel()
{
#ifdef MAVLINK_FTP_UNIT_TEST
	// We use fake ids when unit testing
	return MavlinkFtpTest::serverChannel;
#else
	// Not unit testing, use the real thing
	return _mavlink->get_channel();
#endif
}

void
MavlinkFTP::handle_message(const mavlink_message_t *msg)
{
	//warnx("MavlinkFTP::handle_message %d %d", buf_size_1, buf_size_2);

	if (msg->msgid == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
		mavlink_file_transfer_protocol_t ftp_request;
		mavlink_msg_file_transfer_protocol_decode(msg, &ftp_request);

#ifdef MAVLINK_FTP_DEBUG
		PX4_INFO("FTP: received ftp protocol message target_system: %d target_component: %d",
			 ftp_request.target_system, ftp_request.target_component);
#endif

		if ((ftp_request.target_system == _getServerSystemId() || ftp_request.target_system == 0) &&
		    (ftp_request.target_component == _getServerComponentId() || ftp_request.target_component == 0)) {
			_process_request(&ftp_request, msg->sysid, msg->compid);
		}
	}
}

/// @brief Processes an FTP message
void
MavlinkFTP::_process_request(
	mavlink_file_transfer_protocol_t *ftp_req,
	uint8_t target_system_id,
	uint8_t target_comp_id)
{
	bool stream_send = false;
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

	ErrorCode errorCode = kErrNone;

	if (!_ensure_buffers_exist()) {
		PX4_ERR("Failed to allocate buffers");
		errorCode = kErrFailErrno;
		errno = ENOMEM;
		goto out;
	}

	// basic sanity checks; must validate length before use
	if (payload->size > kMaxDataLength) {
		errorCode = kErrInvalidDataSize;
		goto out;
	}

	// check the sequence number: if this is a resent request, resend the last response
	if (_last_reply_valid) {
		mavlink_file_transfer_protocol_t *last_reply = reinterpret_cast<mavlink_file_transfer_protocol_t *>(_last_reply);
		PayloadHeader *last_payload = reinterpret_cast<PayloadHeader *>(&last_reply->payload[0]);

		if (payload->seq_number + 1 == last_payload->seq_number) {
			// this is the same request as the one we replied to last. It means the (n)ack got lost, and the GCS
			// resent the request
#ifdef MAVLINK_FTP_UNIT_TEST
			_utRcvMsgFunc(last_reply, _worker_data);
#else
			mavlink_msg_file_transfer_protocol_send_struct(_mavlink->get_channel(), last_reply);
#endif
			return;
		}
	}



#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("ftp: channel %u opc %u size %u offset %u", _getServerChannel(), payload->opcode, payload->size,
		 payload->offset);
#endif

	switch (payload->opcode) {
	case kCmdNone:
		break;

	case kCmdTerminateSession:
		errorCode = _workTerminate(payload);
		break;

	case kCmdResetSessions:
		errorCode = _workReset(payload);
		break;

	case kCmdListDirectory:
		errorCode = _workList(payload);
		break;

	case kCmdOpenFileRO:
		errorCode = _workOpen(payload, O_RDONLY);
		break;

	case kCmdCreateFile:
		errorCode = _workOpen(payload, O_CREAT | O_TRUNC | O_WRONLY);
		break;

	case kCmdOpenFileWO:
		errorCode = _workOpen(payload, O_CREAT | O_WRONLY);
		break;

	case kCmdReadFile:
		errorCode = _workRead(payload);
		break;

	case kCmdBurstReadFile:
		errorCode = _workBurst(payload, target_system_id, target_comp_id);
		stream_send = true;
		break;

	case kCmdWriteFile:
		errorCode = _workWrite(payload);
		break;

	case kCmdRemoveFile:
		errorCode = _workRemoveFile(payload);
		break;

	case kCmdRename:
		errorCode = _workRename(payload);
		break;

	case kCmdTruncateFile:
		errorCode = _workTruncateFile(payload);
		break;

	case kCmdCreateDirectory:
		errorCode = _workCreateDirectory(payload);
		break;

	case kCmdRemoveDirectory:
		errorCode = _workRemoveDirectory(payload);
		break;

	case kCmdCalcFileCRC32:
		errorCode = _workCalcFileCRC32(payload);
		break;

	default:
		errorCode = kErrUnknownCommand;
		break;
	}

out:
	payload->seq_number++;

	// handle success vs. error
	if (errorCode == kErrNone) {
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspAck;

	} else {
		int r_errno = errno;
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspNak;
		payload->size = 1;

		if (r_errno == EEXIST) {
			errorCode = kErrFailFileExists;

		} else if (r_errno == ENOENT && errorCode == kErrFailErrno) {
			errorCode = kErrFileNotFound;
		}

		payload->data[0] = errorCode;

		if (errorCode == kErrFailErrno) {
			payload->size = 2;
			payload->data[1] = r_errno;
		}
	}

	_last_reply_valid = false;

	// Stream download replies are sent through mavlink stream mechanism. Unless we need to Nack.
	if (!stream_send || errorCode != kErrNone) {
		// respond to the request
		ftp_req->target_system = target_system_id;
		ftp_req->target_network = 0;
		ftp_req->target_component = target_comp_id;
		_reply(ftp_req);
	}
}

bool MavlinkFTP::_ensure_buffers_exist()
{
	_last_work_buffer_access = hrt_absolute_time();

	if (!_work_buffer1) {
		_work_buffer1 = new char[_work_buffer1_len];
	}

	if (!_work_buffer2) {
		_work_buffer2 = new char[_work_buffer2_len];
	}

	return _work_buffer1 && _work_buffer2;
}

/// @brief Sends the specified FTP response message out through mavlink
void
MavlinkFTP::_reply(mavlink_file_transfer_protocol_t *ftp_req)
{
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

	// keep a copy of the last sent response ((n)ack), so that if it gets lost and the GCS resends the request,
	// we can simply resend the response.
	// we only keep small responses to reduce RAM usage and avoid large memcpy's. The larger responses are all data
	// retrievals without side-effects, meaning it's ok to reexecute them if a response gets lost
	if (payload->size <= sizeof(uint32_t)) {
		_last_reply_valid = true;
		memcpy(_last_reply, ftp_req, sizeof(_last_reply));
	}

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: %s seq_number: %d", payload->opcode == kRspAck ? "Ack" : "Nak", payload->seq_number);
#endif

#ifdef MAVLINK_FTP_UNIT_TEST
	// Unit test hook is set, call that instead
	_utRcvMsgFunc(ftp_req, _worker_data);
#else
	mavlink_msg_file_transfer_protocol_send_struct(_mavlink->get_channel(), ftp_req);
#endif

}

/// @brief Responds to a List command
MavlinkFTP::ErrorCode
MavlinkFTP::_workList(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	ErrorCode errorCode = kErrNone;
	unsigned offset = 0;

	DIR *dp = opendir(_work_buffer1);

	if (dp == nullptr) {
		PX4_WARN("File open failed %s", _work_buffer1);
		return kErrFileNotFound;
	}

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: list %s offset %d", _work_buffer1, payload->offset);
#endif

	struct dirent *result = nullptr;

	// move to the requested offset
	int requested_offset = payload->offset;

	while (requested_offset-- > 0 && readdir(dp)) {}

	for (;;) {
		errno = 0;
		result = readdir(dp);

		// read the directory entry
		if (result == nullptr) {
			if (errno) {
				PX4_WARN("readdir failed");
				payload->data[offset++] = kDirentSkip;
				*((char *)&payload->data[offset]) = '\0';
				offset++;
				payload->size = offset;
				closedir(dp);

				return errorCode;
			}

			// no more entries?
			if (payload->offset != 0 && offset == 0) {
				// User is requesting subsequent dir entries but there were none. This means the user asked
				// to seek past EOF.
				errorCode = kErrEOF;
			}

			// Otherwise we are just at the last directory entry, so we leave the errorCode at kErrorNone to signal that
			break;
		}

		uint32_t fileSize = 0;
		char direntType;

		// Determine the directory entry type
		switch (result->d_type) {
#ifdef __PX4_NUTTX

		case DTYPE_FILE: {
#else

		case DT_REG: {
#endif
				// For files we get the file size as well
				direntType = kDirentFile;
				int ret = snprintf(_work_buffer2, _work_buffer2_len, "%s/%s", _work_buffer1, result->d_name);
				bool buf_is_ok = ((ret > 0) && (ret < _work_buffer2_len));

				if (buf_is_ok) {
					struct stat st;

					if (stat(_work_buffer2, &st) == 0) {
						fileSize = st.st_size;
					}
				}

				break;
			}

#ifdef __PX4_NUTTX

		case DTYPE_DIRECTORY:
#else
		case DT_DIR:
#endif
			if (strcmp(result->d_name, ".") == 0 || strcmp(result->d_name, "..") == 0) {
				// Don't bother sending these back
				direntType = kDirentSkip;

			} else {
				direntType = kDirentDir;
			}

			break;

		default:
			// We only send back file and diretory entries, skip everything else
			direntType = kDirentSkip;
		}

		if (direntType == kDirentSkip) {
			// Skip send only dirent identifier
			_work_buffer2[0] = '\0';

		} else if (direntType == kDirentFile) {
			// Files send filename and file length
			int ret = snprintf(_work_buffer2, _work_buffer2_len, "%s\t%d", result->d_name, fileSize);
			bool buf_is_ok = ((ret > 0) && (ret < _work_buffer2_len));

			if (!buf_is_ok) {
				_work_buffer2[_work_buffer2_len - 1] = '\0';
			}

		} else {
			// Everything else just sends name
			strncpy(_work_buffer2, result->d_name, _work_buffer2_len);
			_work_buffer2[_work_buffer2_len - 1] = '\0';
		}

		size_t nameLen = strlen(_work_buffer2);

		// Do we have room for the name, the one char directory identifier and the null terminator?
		if ((offset + nameLen + 2) > kMaxDataLength) {
			break;
		}

		// Move the data into the buffer
		payload->data[offset++] = direntType;
		strcpy((char *)&payload->data[offset], _work_buffer2);
#ifdef MAVLINK_FTP_DEBUG
		PX4_INFO("FTP: list %s %s", _work_buffer1, (char *)&payload->data[offset - 1]);
#endif
		offset += nameLen + 1;
	}

	closedir(dp);
	payload->size = offset;

	return errorCode;
}

/// @brief Responds to an Open command
MavlinkFTP::ErrorCode
MavlinkFTP::_workOpen(PayloadHeader *payload, int oflag)
{
	if (_session_info.fd >= 0) {
		PX4_ERR("FTP: Open failed - out of sessions\n");
		return kErrNoSessionsAvailable;
	}

	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: open '%s'", _work_buffer1);
#endif

	uint32_t fileSize = 0;
	struct stat st;

	if (stat(_work_buffer1, &st) != 0) {
		// fail only if requested open for read
		if (oflag & O_RDONLY) {
			return kErrFailErrno;

		} else {
			st.st_size = 0;
		}
	}

	fileSize = st.st_size;

	// Set mode to 666 incase oflag has O_CREAT
	int fd = ::open(_work_buffer1, oflag, PX4_O_MODE_666);

	if (fd < 0) {
		return kErrFailErrno;
	}

	_session_info.fd = fd;
	_session_info.file_size = fileSize;
	_session_info.stream_download = false;

	payload->session = 0;
	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &fileSize, payload->size);

	return kErrNone;
}

/// @brief Responds to a Read command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRead(PayloadHeader *payload)
{
	if (payload->session != 0 || _session_info.fd < 0) {
		return kErrInvalidSession;
	}

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: read offset:%d", payload->offset);
#endif

	// We have to test seek past EOF ourselves, lseek will allow seek past EOF
	if (payload->offset >= _session_info.file_size) {
		PX4_ERR("request past EOF");
		return kErrEOF;
	}

	if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
		PX4_ERR("seek fail");
		return kErrFailErrno;
	}

	int bytes_read = ::read(_session_info.fd, &payload->data[0], kMaxDataLength);

	if (bytes_read < 0) {
		// Negative return indicates error other than eof
		PX4_ERR("read fail %d", bytes_read);
		return kErrFailErrno;
	}

	payload->size = bytes_read;

	return kErrNone;
}

/// @brief Responds to a Stream command
MavlinkFTP::ErrorCode
MavlinkFTP::_workBurst(PayloadHeader *payload, uint8_t target_system_id, uint8_t target_component_id)
{
	if (payload->session != 0 && _session_info.fd < 0) {
		return kErrInvalidSession;
	}

#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("FTP: burst offset:%d", payload->offset);
#endif
	// Setup for streaming sends
	_session_info.stream_download = true;
	_session_info.stream_offset = payload->offset;
	_session_info.stream_chunk_transmitted = 0;
	_session_info.stream_seq_number = payload->seq_number + 1;
	_session_info.stream_target_system_id = target_system_id;
	_session_info.stream_target_component_id = target_component_id;

	return kErrNone;
}

/// @brief Responds to a Write command
MavlinkFTP::ErrorCode
MavlinkFTP::_workWrite(PayloadHeader *payload)
{
	if (payload->session != 0 && _session_info.fd < 0) {
		return kErrInvalidSession;
	}

	if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
		// Unable to see to the specified location
		PX4_ERR("seek fail");
		return kErrFailErrno;
	}

	int bytes_written = ::write(_session_info.fd, &payload->data[0], payload->size);

	if (bytes_written < 0) {
		// Negative return indicates error other than eof
		PX4_ERR("write fail %d", bytes_written);
		return kErrFailErrno;
	}

	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &bytes_written, payload->size);

	return kErrNone;
}

/// @brief Responds to a RemoveFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveFile(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	if (unlink(_work_buffer1) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a TruncateFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTruncateFile(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';
	payload->size = 0;

#ifdef __PX4_NUTTX

	// emulate truncate(_work_buffer1, payload->offset) by
	// copying to temp and overwrite with O_TRUNC flag (NuttX does not support truncate()).
	const char temp_file[] = PX4_STORAGEDIR"/.trunc.tmp";

	struct stat st;

	if (stat(_work_buffer1, &st) != 0) {
		return kErrFailErrno;
	}

	if (!S_ISREG(st.st_mode)) {
		errno = EISDIR;
		return kErrFailErrno;
	}

	// check perms allow us to write (not romfs)
	if (!(st.st_mode & (S_IWUSR | S_IWGRP | S_IWOTH))) {
		errno = EROFS;
		return kErrFailErrno;
	}

	if (payload->offset == (unsigned)st.st_size) {
		// nothing to do
		return kErrNone;

	} else if (payload->offset == 0) {
		// 1: truncate all data
		int fd = ::open(_work_buffer1, O_TRUNC | O_WRONLY);

		if (fd < 0) {
			return kErrFailErrno;
		}

		::close(fd);
		return kErrNone;

	} else if (payload->offset > (unsigned)st.st_size) {
		// 2: extend file
		int fd = ::open(_work_buffer1, O_WRONLY);

		if (fd < 0) {
			return kErrFailErrno;
		}

		if (lseek(fd, payload->offset - 1, SEEK_SET) < 0) {
			::close(fd);
			return kErrFailErrno;
		}

		bool ok = 1 == ::write(fd, "", 1);
		::close(fd);

		return (ok) ? kErrNone : kErrFailErrno;

	} else {
		// 3: truncate
		if (_copy_file(_work_buffer1, temp_file, payload->offset) != 0) {
			return kErrFailErrno;
		}

		if (_copy_file(temp_file, _work_buffer1, payload->offset) != 0) {
			return kErrFailErrno;
		}

		if (::unlink(temp_file) != 0) {
			return kErrFailErrno;
		}

		return kErrNone;
	}

#else
	int ret = truncate(_work_buffer1, payload->offset);

	if (ret == 0) {
		return kErrNone;
	}

	return kErrFailErrno;
#endif /* __PX4_NUTTX */
}

/// @brief Responds to a Terminate command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTerminate(PayloadHeader *payload)
{
	if (payload->session != 0 || _session_info.fd < 0) {
		return kErrInvalidSession;
	}

	::close(_session_info.fd);
	_session_info.fd = -1;
	_session_info.stream_download = false;

	payload->size = 0;

	return kErrNone;
}

/// @brief Responds to a Reset command
MavlinkFTP::ErrorCode
MavlinkFTP::_workReset(PayloadHeader *payload)
{
	if (_session_info.fd != -1) {
		::close(_session_info.fd);
		_session_info.fd = -1;
		_session_info.stream_download = false;
	}

	payload->size = 0;

	return kErrNone;
}

/// @brief Responds to a Rename command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRename(PayloadHeader *payload)
{
	char *ptr = _data_as_cstring(payload);
	size_t oldpath_sz = strlen(ptr);

	if (oldpath_sz == payload->size) {
		// no newpath
		errno = EINVAL;
		return kErrFailErrno;
	}

	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, ptr, _work_buffer1_len - _root_dir_len);
	_work_buffer1[_work_buffer1_len - 1] = '\0'; // ensure termination

	strncpy(_work_buffer2, _root_dir, _work_buffer2_len);
	strncpy(_work_buffer2 + _root_dir_len, ptr + oldpath_sz + 1, _work_buffer2_len - _root_dir_len);
	_work_buffer2[_work_buffer2_len - 1] = '\0'; // ensure termination

	if (rename(_work_buffer1, _work_buffer2) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a RemoveDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveDirectory(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	if (rmdir(_work_buffer1) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a CreateDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCreateDirectory(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	if (mkdir(_work_buffer1, S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a CalcFileCRC32 command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCalcFileCRC32(PayloadHeader *payload)
{
	uint32_t checksum = 0;
	ssize_t bytes_read;
	strncpy(_work_buffer2, _root_dir, _work_buffer2_len);
	strncpy(_work_buffer2 + _root_dir_len, _data_as_cstring(payload), _work_buffer2_len - _root_dir_len);
	// ensure termination
	_work_buffer2[_work_buffer2_len - 1] = '\0';

	int fd = ::open(_work_buffer2, O_RDONLY);

	if (fd < 0) {
		return kErrFailErrno;
	}

	do {
		bytes_read = ::read(fd, _work_buffer2, _work_buffer2_len);

		if (bytes_read < 0) {
			int r_errno = errno;
			::close(fd);
			errno = r_errno;
			return kErrFailErrno;
		}

		checksum = crc32part((uint8_t *)_work_buffer2, bytes_read, checksum);
	} while (bytes_read == _work_buffer2_len);

	::close(fd);

	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &checksum, payload->size);
	return kErrNone;
}

/// @brief Guarantees that the payload data is null terminated.
///     @return Returns a pointer to the payload data as a char *
char *
MavlinkFTP::_data_as_cstring(PayloadHeader *payload)
{
	// guarantee nul termination
	if (payload->size < kMaxDataLength) {
		payload->data[payload->size] = '\0';

	} else {
		payload->data[kMaxDataLength - 1] = '\0';
	}

	// and return data
	return (char *) & (payload->data[0]);
}

/// @brief Copy file (with limited space)
int
MavlinkFTP::_copy_file(const char *src_path, const char *dst_path, size_t length)
{
	int src_fd = -1, dst_fd = -1;
	int op_errno = 0;

	src_fd = ::open(src_path, O_RDONLY);

	if (src_fd < 0) {
		return -1;
	}

	dst_fd = ::open(dst_path, O_CREAT | O_TRUNC | O_WRONLY
// POSIX requires the permissions to be supplied if O_CREAT passed
#ifdef __PX4_POSIX
			, 0666
#endif
		       );

	if (dst_fd < 0) {
		op_errno = errno;
		::close(src_fd);
		errno = op_errno;
		return -1;
	}

	while (length > 0) {
		ssize_t bytes_read, bytes_written;
		size_t blen = (length > _work_buffer2_len) ? _work_buffer2_len : length;

		bytes_read = ::read(src_fd, _work_buffer2, blen);

		if (bytes_read == 0) {
			// EOF
			break;

		} else if (bytes_read < 0) {
			PX4_ERR("cp: read");
			op_errno = errno;
			break;
		}

		bytes_written = ::write(dst_fd, _work_buffer2, bytes_read);

		if (bytes_written != bytes_read) {
			PX4_ERR("cp: short write");
			op_errno = errno;
			break;
		}

		length -= bytes_written;
	}

	::close(src_fd);
	::close(dst_fd);

	errno = op_errno;
	return (length > 0) ? -1 : 0;
}

void MavlinkFTP::send()
{

	if (_work_buffer1 || _work_buffer2) {
		// free the work buffers if they are not used for a while
		if (hrt_elapsed_time(&_last_work_buffer_access) > 2_s) {
			if (_work_buffer1) {
				delete[] _work_buffer1;
				_work_buffer1 = nullptr;
			}

			if (_work_buffer2) {
				delete[] _work_buffer2;
				_work_buffer2 = nullptr;
			}
		}

	} else if (_session_info.fd != -1) {
		// close session without activity
		if (hrt_elapsed_time(&_last_work_buffer_access) > 10_s) {
			::close(_session_info.fd);
			_session_info.fd = -1;
			_session_info.stream_download = false;
			_last_reply_valid = false;
			PX4_WARN("Session was closed without activity");
		}
	}

	// Anything to stream?
	if (!_session_info.stream_download) {
		return;
	}

#ifndef MAVLINK_FTP_UNIT_TEST
	// Skip send if not enough room
	unsigned max_bytes_to_send = _mavlink->get_free_tx_buf();
#ifdef MAVLINK_FTP_DEBUG
	PX4_INFO("MavlinkFTP::send max_bytes_to_send(%d) get_free_tx_buf(%d)", max_bytes_to_send, _mavlink->get_free_tx_buf());
#endif

	if (max_bytes_to_send < get_size()) {
		return;
	}

#endif

	// Send stream packets until buffer is full

	bool more_data;

	do {
		more_data = false;

		ErrorCode error_code = kErrNone;

		mavlink_file_transfer_protocol_t ftp_msg;
		PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_msg.payload[0]);

		payload->seq_number = _session_info.stream_seq_number;
		payload->session = 0;
		payload->opcode = kRspAck;
		payload->req_opcode = kCmdBurstReadFile;
		payload->offset = _session_info.stream_offset;
		_session_info.stream_seq_number++;

#ifdef MAVLINK_FTP_DEBUG
		PX4_INFO("stream send: offset %d", _session_info.stream_offset);
#endif

		// We have to test seek past EOF ourselves, lseek will allow seek past EOF
		if (_session_info.stream_offset >= _session_info.file_size) {
			error_code = kErrEOF;
#ifdef MAVLINK_FTP_DEBUG
			PX4_INFO("stream download: sending Nak EOF");
#endif
		}

		if (error_code == kErrNone) {
			if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
				error_code = kErrFailErrno;
#ifdef MAVLINK_FTP_DEBUG
				PX4_WARN("stream download: seek fail");
#endif
			}
		}

		if (error_code == kErrNone) {
			int bytes_read = ::read(_session_info.fd, &payload->data[0], kMaxDataLength);

			if (bytes_read < 0) {
				// Negative return indicates error other than eof
				error_code = kErrFailErrno;
#ifdef MAVLINK_FTP_DEBUG
				PX4_WARN("stream download: read fail");
#endif

			} else {
				payload->size = bytes_read;
				_session_info.stream_offset += bytes_read;
				_session_info.stream_chunk_transmitted += bytes_read;
			}
		}

		if (error_code != kErrNone) {
			payload->opcode = kRspNak;
			payload->size = 1;
			uint8_t *pData = &payload->data[0];
			*pData = error_code; // Straight reference to data[0] is causing bogus gcc array subscript error

			if (error_code == kErrFailErrno) {
				int r_errno = errno;
				payload->size = 2;
				payload->data[1] = r_errno;
			}

			_session_info.stream_download = false;

		} else {
#ifndef MAVLINK_FTP_UNIT_TEST

			if (max_bytes_to_send < (get_size() * 2)) {
				more_data = false;

				/* perform transfers in 35K chunks - this is determined empirical */
				if (_session_info.stream_chunk_transmitted > 35000) {
					payload->burst_complete = true;
					_session_info.stream_download = false;
					_session_info.stream_chunk_transmitted = 0;
				}

			} else {
#endif
				more_data = true;
				payload->burst_complete = false;
#ifndef MAVLINK_FTP_UNIT_TEST
				max_bytes_to_send -= get_size();
			}

#endif
		}

		ftp_msg.target_system = _session_info.stream_target_system_id;
		ftp_msg.target_network = 0;
		ftp_msg.target_component = _session_info.stream_target_component_id;
		_reply(&ftp_msg);
	} while (more_data);
}
