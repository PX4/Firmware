/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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

/// @file mavlink_ftp.h
///     @author px4dev, Don Gagne <don@thegagnes.com>

#include <dirent.h>
#include <queue.h>

#include <px4_platform_common/defines.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#ifndef MAVLINK_FTP_UNIT_TEST
#include "mavlink_bridge_header.h"
#else
#include <v2.0/standard/mavlink.h>
#endif

class MavlinkFtpTest;
class Mavlink;

/// MAVLink remote file server. Support FTP like commands using MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL message.
class MavlinkFTP
{
public:
	MavlinkFTP(Mavlink *mavlink);
	~MavlinkFTP();

	/**
	 * Handle sending of messages. Call this regularly at a fixed frequency.
	 * @param t current time
	 */
	void send();

	/// Handle possible FTP message
	void handle_message(const mavlink_message_t *msg);

	typedef void (*ReceiveMessageFunc_t)(const mavlink_file_transfer_protocol_t *ftp_req, void *worker_data);

	/// @brief Sets up the server to run in unit test mode.
	///	@param rcvmsgFunc Function which will be called to handle outgoing mavlink messages.
	///	@param worker_data Data to pass to worker
	void set_unittest_worker(ReceiveMessageFunc_t rcvMsgFunc, void *worker_data);

	/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
	/// This needs to be packed, because it's typecasted from mavlink_file_transfer_protocol_t.payload, which starts
	/// at a 3 byte offset, causing an unaligned access to seq_number and offset
	struct __attribute__((__packed__)) PayloadHeader {
		uint16_t	seq_number;	///< sequence number for message
		uint8_t		session;	///< Session id for read and write commands
		uint8_t		opcode;		///< Command opcode
		uint8_t		size;		///< Size of data
		uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
		uint8_t		burst_complete; ///< Only used if req_opcode=kCmdBurstReadFile - 1: set of burst packets complete, 0: More burst packets coming.
		uint8_t		padding;        ///< 32 bit aligment padding
		uint32_t	offset;		///< Offsets for List and Read commands
		uint8_t		data[];		///< command data, varies by Opcode
	};

	/// @brief Command opcodes
	enum Opcode : uint8_t {
		kCmdNone,		///< ignored, always acked
		kCmdTerminateSession,	///< Terminates open Read session
		kCmdResetSessions,	///< Terminates all open Read sessions
		kCmdListDirectory,	///< List files in <path> from <offset>
		kCmdOpenFileRO,		///< Opens file at <path> for reading, returns <session>
		kCmdReadFile,		///< Reads <size> bytes from <offset> in <session>
		kCmdCreateFile,		///< Creates file at <path> for writing, returns <session>
		kCmdWriteFile,		///< Writes <size> bytes to <offset> in <session>
		kCmdRemoveFile,		///< Remove file at <path>
		kCmdCreateDirectory,	///< Creates directory at <path>
		kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty
		kCmdOpenFileWO,		///< Opens file at <path> for writing, returns <session>
		kCmdTruncateFile,	///< Truncate file at <path> to <offset> length
		kCmdRename,		///< Rename <path1> to <path2>
		kCmdCalcFileCRC32,	///< Calculate CRC32 for file at <path>
		kCmdBurstReadFile,	///< Burst download session file

		kRspAck = 128,		///< Ack response
		kRspNak			///< Nak response
	};

	/// @brief Error codes returned in Nak response PayloadHeader.data[0].
	enum ErrorCode : uint8_t {
		kErrNone,
		kErrFail,			///< Unknown failure
		kErrFailErrno,			///< Command failed, errno sent back in PayloadHeader.data[1]
		kErrInvalidDataSize,		///< PayloadHeader.size is invalid
		kErrInvalidSession,		///< Session is not currently open
		kErrNoSessionsAvailable,	///< All available Sessions in use
		kErrEOF,			///< Offset past end of file for List and Read commands
		kErrUnknownCommand,		///< Unknown command opcode
		kErrFailFileExists,		///< File/directory exists already
		kErrFailFileProtected,		///< File/directory is write protected
		kErrFileNotFound                ///< File/directory not found
	};

	unsigned get_size();

private:
	char		*_data_as_cstring(PayloadHeader *payload);

	void		_process_request(mavlink_file_transfer_protocol_t *ftp_req, uint8_t target_system_id, uint8_t target_comp_id);
	void		_reply(mavlink_file_transfer_protocol_t *ftp_req);
	int		_copy_file(const char *src_path, const char *dst_path, size_t length);

	ErrorCode	_workList(PayloadHeader *payload);
	ErrorCode	_workOpen(PayloadHeader *payload, int oflag);
	ErrorCode	_workRead(PayloadHeader *payload);
	ErrorCode	_workBurst(PayloadHeader *payload, uint8_t target_system_id, uint8_t target_component_id);
	ErrorCode	_workWrite(PayloadHeader *payload);
	ErrorCode	_workTerminate(PayloadHeader *payload);
	ErrorCode	_workReset(PayloadHeader *payload);
	ErrorCode	_workRemoveDirectory(PayloadHeader *payload);
	ErrorCode	_workCreateDirectory(PayloadHeader *payload);
	ErrorCode	_workRemoveFile(PayloadHeader *payload);
	ErrorCode	_workTruncateFile(PayloadHeader *payload);
	ErrorCode	_workRename(PayloadHeader *payload);
	ErrorCode	_workCalcFileCRC32(PayloadHeader *payload);

	uint8_t _getServerSystemId(void);
	uint8_t _getServerComponentId(void);
	uint8_t _getServerChannel(void);

	/**
	 * make sure that the working buffers _work_buffer* are allocated
	 * @return true if buffers exist, false if allocation failed
	 */
	bool _ensure_buffers_exist();

	static const char	kDirentFile = 'F';	///< Identifies File returned from List command
	static const char	kDirentDir = 'D';	///< Identifies Directory returned from List command
	static const char	kDirentSkip = 'S';	///< Identifies Skipped entry from List command

	/// @brief Maximum data size in RequestHeader::data
	static const uint8_t	kMaxDataLength = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(PayloadHeader);

	struct SessionInfo {
		int		fd;
		uint32_t	file_size;
		bool		stream_download;
		uint32_t	stream_offset;
		uint16_t	stream_seq_number;
		uint8_t		stream_target_system_id;
		uint8_t         stream_target_component_id;
		unsigned	stream_chunk_transmitted;
	};
	struct SessionInfo _session_info {};	///< Session info, fd=-1 for no active session

	ReceiveMessageFunc_t	_utRcvMsgFunc{};	///< Unit test override for mavlink message sending
	void			*_worker_data{nullptr};	///< Additional parameter to _utRcvMsgFunc;

	Mavlink *_mavlink;

	/* do not allow copying this class */
	MavlinkFTP(const MavlinkFTP &);
	MavlinkFTP operator=(const MavlinkFTP &);

	/* work buffers: they're allocated as soon as we get the first request (lazy, since FTP is rarely used) */
	char *_work_buffer1{nullptr};
	static constexpr int _work_buffer1_len = kMaxDataLength;
	char *_work_buffer2{nullptr};
	static constexpr int _work_buffer2_len = 256;
	hrt_abstime _last_work_buffer_access{0}; ///< timestamp when the buffers were last accessed

	// prepend a root directory to each file/dir access to avoid enumerating the full FS tree (e.g. on Linux).
	// Note that requests can still fall outside of the root dir by using ../..
#ifdef MAVLINK_FTP_UNIT_TEST
	static constexpr const char _root_dir[] = "";
#else
	static constexpr const char _root_dir[] = PX4_ROOTFSDIR;
#endif
	static constexpr const int _root_dir_len = sizeof(_root_dir) - 1;

	bool _last_reply_valid = false;
	uint8_t _last_reply[MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN - MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN
								      + sizeof(PayloadHeader) + sizeof(uint32_t)];

	// Mavlink test needs to be able to call send
	friend class MavlinkFtpTest;
};
