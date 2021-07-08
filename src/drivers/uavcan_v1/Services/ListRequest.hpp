/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file ListRequest.hpp
 *
 * Defines a List Service invoker and process List responses
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <version/version.h>

#include <uavcan/_register/List_1_0.h>

#include "ServiceRequest.hpp"
#include "../ParamManager.hpp"
#include "../Publishers/Publisher.hpp"

class UavcanListServiceRequest : public UavcanServiceRequest
{
public:
	UavcanListServiceRequest(CanardInstance &ins) :
		UavcanServiceRequest(ins, "List", uavcan_register_List_1_0_FIXED_PORT_ID_,
				     uavcan_register_List_Response_1_0_EXTENT_BYTES_) { };


	bool getIndex(CanardNodeID node_id, uint16_t index, UavcanServiceRequestInterface *handler)
	{
		uavcan_register_List_Request_1_0 msg;
		msg.index = index;

		uint8_t request_payload_buffer[uavcan_register_List_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

		CanardTransfer transfer = {
			.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindRequest,
			.port_id        = uavcan_register_List_1_0_FIXED_PORT_ID_, // This is the subject-ID.
			.remote_node_id = node_id,
			.transfer_id    = request_transfer_id,
			.payload_size   = uavcan_register_List_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
			.payload        = &request_payload_buffer,
		};

		if (uavcan_register_List_Request_1_0_serialize_(&msg, request_payload_buffer, &transfer.payload_size) == 0) {
			return request(&transfer, handler);

		} else {
			return false;
		}
	};


};
