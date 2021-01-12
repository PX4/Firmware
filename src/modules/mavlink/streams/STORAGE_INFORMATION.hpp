/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef STORAGE_INFORMATION_HPP
#define STORAGE_INFORMATION_HPP

#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

#include <math.h>

class MavlinkStreamStorageInformation : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamStorageInformation(mavlink); }

	static constexpr const char *get_name_static() { return "STORAGE_INFORMATION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_STORAGE_INFORMATION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool request_message(float param2, float param3, float param4,
			     float param5, float param6, float param7) override
	{
		_storage_id = (int)roundf(param2);
		return send();
	}
private:
	explicit MavlinkStreamStorageInformation(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	int _storage_id{0};

	bool send() override
	{
		mavlink_storage_information_t storage_info{};
		const char *microsd_dir = PX4_STORAGEDIR;

		if (_storage_id == 0 || _storage_id == 1) { // request is for all or the first storage
			storage_info.storage_id = 1;

			struct statfs statfs_buf;
			uint64_t total_bytes = 0;
			uint64_t avail_bytes = 0;

			if (statfs(microsd_dir, &statfs_buf) == 0) {
				total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;
				avail_bytes = (uint64_t)statfs_buf.f_bavail * statfs_buf.f_bsize;
			}

			if (total_bytes == 0) { // on NuttX we get 0 total bytes if no SD card is inserted
				storage_info.storage_count = 0;
				storage_info.status = 0; // not available

			} else {
				storage_info.storage_count = 1;
				storage_info.status = 2; // available & formatted
				storage_info.total_capacity = total_bytes / 1024. / 1024.;
				storage_info.available_capacity = avail_bytes / 1024. / 1024.;
				storage_info.used_capacity = (total_bytes - avail_bytes) / 1024. / 1024.;
			}

		} else {
			return false; // results in MAV_RESULT_DENIED
		}

		storage_info.time_boot_ms = hrt_absolute_time() / 1000;
		mavlink_msg_storage_information_send_struct(_mavlink->get_channel(), &storage_info);
		return true;
	}
};

#endif // STORAGE_INFORMATION_HPP
