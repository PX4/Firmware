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

#ifndef COMPONENT_INFORMATION_HPP
#define COMPONENT_INFORMATION_HPP

#include "../mavlink_stream.h"

#include <component_information/checksums.h>

#include <px4_platform_common/defines.h>

#include <sys/stat.h>

class MavlinkStreamComponentInformation : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamComponentInformation(mavlink); }

	static constexpr const char *get_name_static() { return "COMPONENT_INFORMATION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_COMPONENT_INFORMATION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return 0; // never streamed
	}

	bool request_message(float param2, float param3, float param4,
			     float param5, float param6, float param7) override
	{
		mavlink_component_information_t component_info{};
		PX4_DEBUG("COMPONENT_INFORMATION request");

		strncpy(component_info.general_metadata_uri, "mftp://etc/extras/component_general.json.xz",
			sizeof(component_info.general_metadata_uri) - 1);
		component_info.general_metadata_file_crc = component_information::component_general_crc;

		component_info.time_boot_ms = hrt_absolute_time() / 1000;
		mavlink_msg_component_information_send_struct(_mavlink->get_channel(), &component_info);

		return true;
	}
private:
	explicit MavlinkStreamComponentInformation(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	bool send() override
	{
		return false;
	}
};

#endif // COMPONENT_INFORMATION_HPP
