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

/// @file mavlink_log_handler.h
/// @author px4dev, Gus Grubba <mavlink@grubba.com>

#include "mavlink_log_handler.h"
#include "mavlink_main.h"
#include <sys/stat.h>
#include <time.h>
#include <systemlib/err.h>

#define MOUNTPOINT PX4_STORAGEDIR

static const char *kLogRoot    = MOUNTPOINT "/log";
static const char *kLogData    = MOUNTPOINT "/logdata.txt";
static const char *kTmpData    = MOUNTPOINT "/$log$.txt";

#ifdef __PX4_NUTTX
#define PX4LOG_REGULAR_FILE DTYPE_FILE
#define PX4LOG_DIRECTORY    DTYPE_DIRECTORY
#else
#define PX4LOG_REGULAR_FILE DT_REG
#define PX4LOG_DIRECTORY    DT_DIR
#endif

//#define MAVLINK_LOG_HANDLER_VERBOSE

#ifdef MAVLINK_LOG_HANDLER_VERBOSE
#define PX4LOG_WARN(fmt, ...) warnx(fmt, ##__VA_ARGS__)
#else
#define PX4LOG_WARN(fmt, ...)
#endif

//-------------------------------------------------------------------
static bool
stat_file(const char *file, time_t *date = nullptr, uint32_t *size = nullptr)
{
	struct stat st;

	if (stat(file, &st) == 0) {
		if (date) { *date = st.st_mtime; }

		if (size) { *size = st.st_size; }

		return true;
	}

	return false;
}

//-------------------------------------------------------------------
MavlinkLogHandler::MavlinkLogHandler(Mavlink *mavlink)
	: _mavlink(mavlink)
{

}
MavlinkLogHandler::~MavlinkLogHandler()
{
	_close_and_unlink_files();
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
		_log_request_list(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
		_log_request_data(msg);
		break;

	case MAVLINK_MSG_ID_LOG_ERASE:
		_log_request_erase(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_END:
		_log_request_end(msg);
		break;
	}
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::send()
{
	//-- An arbitrary count of max bytes in one go (one of the two below but never both)
#define MAX_BYTES_SEND 256 * 1024
	size_t count = 0;

	//-- Log Entries
	while (_current_status == LogHandlerState::Listing
	       && _mavlink->get_free_tx_buf() > MAVLINK_MSG_ID_LOG_ENTRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES
	       && count < MAX_BYTES_SEND) {
		count += _log_send_listing();
	}

	//-- Log Data
	while (_current_status == LogHandlerState::SendingData
	       && _mavlink->get_free_tx_buf() > MAVLINK_MSG_ID_LOG_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES
	       && count < MAX_BYTES_SEND) {
		count += _log_send_data();
	}
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_list(const mavlink_message_t *msg)
{
	mavlink_log_request_list_t request;
	mavlink_msg_log_request_list_decode(msg, &request);

	//-- Check for re-requests (data loss) or new request
	if (_current_status != LogHandlerState::Inactive) {
		//-- Is this a new request?
		if ((request.end - request.start) > _log_count) {
			_current_status = LogHandlerState::Inactive;
			_close_and_unlink_files();

		} else {
			_current_status = LogHandlerState::Idle;

		}
	}

	if (_current_status == LogHandlerState::Inactive) {
		//-- Prepare new request

		_reset_list_helper();
		_init_list_helper();
		_current_status = LogHandlerState::Idle;
	}

	if (_log_count) {
		//-- Define (and clamp) range
		_next_entry = request.start < _log_count ? request.start :
			      _log_count - 1;
		_last_entry = request.end   < _log_count ? request.end :
			      _log_count - 1;
	}

	PX4LOG_WARN("\nMavlinkLogHandler::_log_request_list: start: %u last: %u count: %u",
		    _next_entry,
		    _last_entry,
		    _log_count);
	//-- Enable streaming
	_current_status = LogHandlerState::Listing;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_data(const mavlink_message_t *msg)
{
	//-- If we haven't listed, we can't do much
	if (_current_status == LogHandlerState::Inactive) {
		PX4LOG_WARN("MavlinkLogHandler::_log_request_data Log request with no list requested.");
		return;
	}

	mavlink_log_request_data_t request;
	mavlink_msg_log_request_data_decode(msg, &request);

	//-- Does the requested log exist?
	if (request.id >= _log_count) {
		PX4LOG_WARN("MavlinkLogHandler::_log_request_data Requested log %u but we only have %u.", request.id,
			    _log_count);
		return;
	}

	//-- If we were sending log entries, stop it
	_current_status = LogHandlerState::Idle;

	if (_current_log_index != request.id) {
		//-- Init send log dataset
		_current_log_filename[0] = 0;
		_current_log_index = request.id;
		uint32_t time_utc = 0;

		if (!_get_entry(_current_log_index, _current_log_size, time_utc,
				_current_log_filename, sizeof(_current_log_filename))) {
			PX4LOG_WARN("LogListHelper::get_entry failed.");
			return;
		}

		_open_for_transmit();
	}

	_current_log_data_offset = request.ofs;

	if (_current_log_data_offset >= _current_log_size) {
		_current_log_data_remaining = 0;

	} else {
		_current_log_data_remaining = _current_log_size - request.ofs;
	}

	if (_current_log_data_remaining > request.count) {
		_current_log_data_remaining = request.count;
	}

	//-- Enable streaming
	_current_status = LogHandlerState::SendingData;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_erase(const mavlink_message_t * /*msg*/)
{
	/*
	mavlink_log_erase_t request;
	mavlink_msg_log_erase_decode(msg, &request);
	*/
	_current_status = LogHandlerState::Inactive;
	_close_and_unlink_files();

	//-- Delete all logs
	_delete_all(kLogRoot);
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_end(const mavlink_message_t * /*msg*/)
{
	PX4LOG_WARN("MavlinkLogHandler::_log_request_end");

	_current_status = LogHandlerState::Inactive;
	_close_and_unlink_files();
}

//-------------------------------------------------------------------
size_t
MavlinkLogHandler::_log_send_listing()
{
	mavlink_log_entry_t response;
	uint32_t size, date;
	_get_entry(_next_entry, size, date);
	response.size         = size;
	response.time_utc     = date;
	response.id           = _next_entry;
	response.num_logs     = _log_count;
	response.last_log_num = _last_entry;
	mavlink_msg_log_entry_send_struct(_mavlink->get_channel(), &response);

	//-- If we're done listing, flag it.
	if (_next_entry == _last_entry) {
		_current_status = LogHandlerState::Idle;

	} else {
		_next_entry++;
	}

	PX4LOG_WARN("MavlinkLogHandler::_log_send_listing id: %u count: %u last: %u size: %u date: %u status: %d",
		    response.id,
		    response.num_logs,
		    response.last_log_num,
		    response.size,
		    response.time_utc,
		    (uint32_t)_current_status);
	return sizeof(response);
}

//-------------------------------------------------------------------
size_t
MavlinkLogHandler::_log_send_data()
{
	mavlink_log_data_t response;
	memset(&response, 0, sizeof(response));
	uint32_t len = _current_log_data_remaining;

	if (len > sizeof(response.data)) {
		len = sizeof(response.data);
	}

	size_t read_size = _get_log_data(len, response.data);
	response.ofs     = _current_log_data_offset;
	response.id      = _current_log_index;
	response.count   = read_size;
	mavlink_msg_log_data_send_struct(_mavlink->get_channel(), &response);
	_current_log_data_offset    += read_size;
	_current_log_data_remaining -= read_size;

	if (read_size < sizeof(response.data) || _current_log_data_remaining == 0) {
		_current_status = LogHandlerState::Idle;
	}

	return sizeof(response);
}

//-------------------------------------------------------------------
void MavlinkLogHandler::_close_and_unlink_files()
{
	if (_current_log_filep) {
		::fclose(_current_log_filep);
		_reset_list_helper();
	}

	// Remove log data files (if any)
	unlink(kLogData);
	unlink(kTmpData);
}

//-------------------------------------------------------------------
bool
MavlinkLogHandler::_get_entry(int idx, uint32_t &size, uint32_t &date, char *filename, int filename_len)
{
	//-- Find log file in log list file created during init()
	size = 0;
	date = 0;
	bool result = false;
	//-- Open list of log files
	FILE *f = ::fopen(kLogData, "r");

	if (f) {
		//--- Find requested entry
		char line[160];
		int count = 0;

		while (fgets(line, sizeof(line), f)) {
			//-- Found our "index"
			if (count++ == idx) {
				char file[160];

				if (sscanf(line, "%u %u %s", &date, &size, file) == 3) {
					if (filename && filename_len > 0) {
						strncpy(filename, file, filename_len);
						filename[filename_len - 1] = 0; // ensure null-termination
					}

					result = true;
					break;
				}
			}
		}

		fclose(f);
	}

	return result;
}

//-------------------------------------------------------------------
bool
MavlinkLogHandler::_open_for_transmit()
{
	if (_current_log_filep) {
		::fclose(_current_log_filep);
		_current_log_filep = nullptr;
	}

	_current_log_filep = ::fopen(_current_log_filename, "rb");

	if (!_current_log_filep) {
		PX4LOG_WARN("MavlinkLogHandler::open_for_transmit Could not open %s", _current_log_filename);
		return false;
	}

	return true;
}

//-------------------------------------------------------------------
size_t
MavlinkLogHandler::_get_log_data(uint8_t len, uint8_t *buffer)
{
	if (!_current_log_filename[0]) {
		return 0;
	}

	if (!_current_log_filep) {
		PX4LOG_WARN("MavlinkLogHandler::get_log_data file not open %s", _current_log_filename);
		return 0;
	}

	long int offset = _current_log_data_offset - ftell(_current_log_filep);

	if (offset && fseek(_current_log_filep, offset, SEEK_CUR)) {
		fclose(_current_log_filep);
		_current_log_filep = nullptr;
		PX4LOG_WARN("MavlinkLogHandler::get_log_data Seek error in %s", _current_log_filename);
		return 0;
	}

	size_t result = fread(buffer, 1, len, _current_log_filep);
	return result;
}


void
MavlinkLogHandler::_reset_list_helper()
{
	_next_entry = 0;
	_last_entry = 0;
	_log_count = 0;
	_current_log_index = UINT16_MAX;
	_current_log_size = 0;
	_current_log_data_offset = 0;
	_current_log_data_remaining = 0;
	_current_log_filep = nullptr;
}

void
MavlinkLogHandler::_init_list_helper()
{
	/*

		When this helper is created, it scans the log directory
		and collects all log files found into one file for easy,
		subsequent access.
	*/

	_current_log_filename[0] = 0;

	// Remove old log data file (if any)
	unlink(kLogData);
	// Open log directory
	DIR *dp = opendir(kLogRoot);

	if (dp == nullptr) {
		// No log directory. Nothing to do.
		return;
	}

	// Create work file
	FILE *f = ::fopen(kTmpData, "w");

	if (!f) {
		PX4LOG_WARN("MavlinkLogHandler::init Error creating %s", kTmpData);
		closedir(dp);
		return;
	}

	// Scan directory and collect log files
	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {
		if (result->d_type == PX4LOG_DIRECTORY) {
			time_t tt = 0;
			char log_path[128];
			int ret = snprintf(log_path, sizeof(log_path), "%s/%s", kLogRoot, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_path));

			if (path_is_ok) {
				if (_get_session_date(log_path, result->d_name, tt)) {
					_scan_logs(f, log_path, tt);
				}
			}
		}
	}

	closedir(dp);
	fclose(f);

	// Rename temp file to data file
	if (rename(kTmpData, kLogData)) {
		PX4LOG_WARN("MavlinkLogHandler::init Error renaming %s", kTmpData);
		_log_count = 0;
	}
}

//-------------------------------------------------------------------
bool
MavlinkLogHandler::_get_session_date(const char *path, const char *dir, time_t &date)
{
	if (strlen(dir) > 4) {
		// Always try to get file time first
		if (stat_file(path, &date)) {
			// Try to prevent taking date if it's around 1970 (use the logic below instead)
			if (date > 60 * 60 * 24) {
				return true;
			}
		}

		// Convert "sess000" to 00:00 Jan 1 1970 (day per session)
		if (strncmp(dir, "sess", 4) == 0) {
			unsigned u;

			if (sscanf(&dir[4], "%u", &u) == 1) {
				date = u * 60 * 60 * 24;
				return true;
			}
		}
	}

	return false;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_scan_logs(FILE *f, const char *dir, time_t &date)
{
	DIR *dp = opendir(dir);

	if (dp) {
		struct dirent *result = nullptr;

		while ((result = readdir(dp))) {
			if (result->d_type == PX4LOG_REGULAR_FILE) {
				time_t  ldate = date;
				uint32_t size = 0;
				char log_file_path[128];
				int ret = snprintf(log_file_path, sizeof(log_file_path), "%s/%s", dir, result->d_name);
				bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_file_path));

				if (path_is_ok) {
					if (_get_log_time_size(log_file_path, result->d_name, ldate, size)) {
						//-- Write result->out to list file
						fprintf(f, "%u %u %s\n", (unsigned)ldate, (unsigned)size, log_file_path);
						_log_count++;
					}
				}
			}
		}

		closedir(dp);
	}
}

//-------------------------------------------------------------------
bool
MavlinkLogHandler::_get_log_time_size(const char *path, const char *file, time_t &date, uint32_t &size)
{
	if (file && file[0]) {
		if (strstr(file, ".px4log") || strstr(file, ".ulg")) {
			// Always try to get file time first
			if (stat_file(path, &date, &size)) {
				// Try to prevent taking date if it's around 1970 (use the logic below instead)
				if (date > 60 * 60 * 24) {
					return true;
				}
			}

			// Convert "log000" to 00:00 (minute per flight in session)
			if (strncmp(file, "log", 3) == 0) {
				unsigned u;

				if (sscanf(&file[3], "%u", &u) == 1) {
					date += (u * 60);

					if (stat_file(path, nullptr, &size)) {
						return true;
					}
				}
			}
		}
	}

	return false;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_delete_all(const char *dir)
{
	//-- Open log directory
	DIR *dp = opendir(dir);

	if (dp == nullptr) {
		return;
	}

	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {
		// no more entries?
		if (result == nullptr) {
			break;
		}

		if (result->d_type == PX4LOG_DIRECTORY && result->d_name[0] != '.') {
			char log_path[128];
			int ret = snprintf(log_path, sizeof(log_path), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_path));

			if (path_is_ok) {
				_delete_all(log_path); //Recursive call. TODO: consider add protection

				if (rmdir(log_path)) {
					PX4LOG_WARN("MavlinkLogHandler::delete_all Error removing %s", log_path);
				}
			}
		}

		if (result->d_type == PX4LOG_REGULAR_FILE) {
			char log_path[128];
			int ret = snprintf(log_path, sizeof(log_path), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_path));

			if (path_is_ok) {
				if (unlink(log_path)) {
					PX4LOG_WARN("MavlinkLogHandler::delete_all Error deleting %s", log_path);
				}
			}
		}
	}

	closedir(dp);
}
