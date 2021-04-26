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
 * @file GetInfo.hpp
 *
 * Defines response to a GetInfo request
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <version/version.h>

#include "../NodeManager.hpp"

#include <uavcan/node/ID_1_0.h>
#include <uavcan/node/GetInfo_1_0.h>

#include "../Subscribers/BaseSubscriber.hpp"

class UavcanGetInfoResponse : public UavcanBaseSubscriber
{
public:
	UavcanGetInfoResponse(CanardInstance &ins) :
		UavcanBaseSubscriber(ins, "GetInfo", 0) { };

	void subscribe() override
	{
		// Subscribe to requests uavcan.pnp.NodeIDAllocationData
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindRequest,
				  uavcan_node_GetInfo_1_0_FIXED_PORT_ID_,
				  uavcan_node_GetInfo_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_subj_sub._canard_sub);

	};

	void callback(const CanardTransfer &receive) override
	{
		PX4_INFO("GetInfo request");

		// Setup node.GetInfo response

		uavcan_node_GetInfo_Response_1_0 node_info;

		uavcan_node_GetInfo_Response_1_0_initialize_(&node_info);

		node_info.protocol_version.major = 1;
		node_info.protocol_version.minor = 0;

#if defined(BOARD_HAS_VERSIONING)
		node_info.hardware_version.major = (uint8_t)px4_board_hw_version();
		node_info.hardware_version.minor = (uint8_t)px4_board_hw_revision();
#endif

		unsigned fwver = px4_firmware_version();
		node_info.software_version.major = (fwver >> (8 * 3)) & 0xFF;
		node_info.software_version.minor = (fwver >> (8 * 2)) & 0xFF;

		node_info.software_vcs_revision_id = px4_firmware_version_binary();

		px4_guid_t px4_guid;
		board_get_px4_guid(px4_guid);
		memcpy(node_info.unique_id, px4_guid, sizeof(node_info.unique_id));

		//TODO proper name
		strncpy((char *)node_info.name.elements,
			px4_board_name(),
			uavcan_node_GetInfo_Response_1_0_name_ARRAY_CAPACITY_);

		node_info.name.count = strlen(px4_board_name());

		uint8_t response_payload_buffer[uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

		CanardTransfer response = {
			.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindResponse,
			.port_id        = uavcan_node_GetInfo_1_0_FIXED_PORT_ID_, // This is the subject-ID.
			.remote_node_id = receive.remote_node_id,       // Send back to request Node
			.transfer_id    = getinfo_response_transfer_id,
			.payload_size   = uavcan_node_GetInfo_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
			.payload        = &response_payload_buffer,
		};

		int32_t result = uavcan_node_GetInfo_Response_1_0_serialize_(&node_info, (uint8_t *)&response_payload_buffer,
				 &response.payload_size);

		if (result == 0) {
			// set the data ready in the buffer and chop if needed
			++getinfo_response_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
			result = canardTxPush(&_canard_instance, &response);
		}

		//TODO proper error handling
		if (result < 0) {
			// An error has occurred: either an argument is invalid or we've ran out of memory.
			// It is possible to statically prove that an out-of-memory will never occur for a given application if the
			// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
			// return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
		}

	};

private:
	CanardTransferID getinfo_response_transfer_id = 0;

};
