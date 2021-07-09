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
 * @file Access.hpp
 *
 * Defines response to a Access request
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <version/version.h>

#include "../ParamManager.hpp"

#include <uavcan/node/ID_1_0.h>
#include <uavcan/node/GetInfo_1_0.h>

#include "../Subscribers/BaseSubscriber.hpp"

class UavcanAccessResponse : public UavcanBaseSubscriber
{
public:
	UavcanAccessResponse(CanardInstance &ins, UavcanParamManager &pmgr) :
		UavcanBaseSubscriber(ins, "Access", 0),  _param_manager(pmgr) { };

	void subscribe() override
	{
		// Subscribe to requests uavcan.pnp.NodeIDAllocationData
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindRequest,
				  uavcan_register_Access_1_0_FIXED_PORT_ID_,
				  uavcan_register_Access_Response_1_0_EXTENT_BYTES_,
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_subj_sub._canard_sub);

	};

	void callback(const CanardTransfer &receive) override
	{
		PX4_INFO("Access request");

		uavcan_register_Access_Request_1_0 msg;
		uavcan_register_Access_Request_1_0_initialize_(&msg);

		size_t register_in_size_bits = receive.payload_size;
		uavcan_register_Access_Request_1_0_deserialize_(&msg, (const uint8_t *)receive.payload, &register_in_size_bits);

		int result {0};

		uavcan_register_Value_1_0 value = msg.value;
		uavcan_register_Name_1_0 name = msg.name;

		/// TODO: get/set parameter based on whether empty or not
		if (uavcan_register_Value_1_0_is_empty_(&value)) { // Tag Type: uavcan_primitive_Empty_1_0
			// Value is empty -- 'Get' only
			result = _param_manager.GetParamByName(name, value) ? 0 : -1;

		} else {
			// Set value
			result = _param_manager.SetParamByName(name, value) ? 0 : -1;

		}

		/// TODO: Access_Response
		uavcan_register_Access_Response_1_0 response {};
		response.value = value;

		uint8_t response_payload_buffer[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

		CanardTransfer transfer = {
			.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindResponse,
			.port_id        = uavcan_register_Access_1_0_FIXED_PORT_ID_,                // This is the subject-ID.
			.remote_node_id = receive.remote_node_id,       // Messages cannot be unicast, so use UNSET.
			.transfer_id    = access_response_transfer_id, /// TODO: track register Access _response_ separately?
			.payload_size   = uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
			.payload        = &response_payload_buffer,
		};

		result = uavcan_register_Access_Response_1_0_serialize_(&response, response_payload_buffer, &transfer.payload_size);

		if (result == 0) {
			// set the data ready in the buffer and chop if needed
			++access_response_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
			result = canardTxPush(&_canard_instance, &transfer);
		}

		//return result;

	};

private:
	UavcanParamManager &_param_manager;
	CanardTransferID access_response_transfer_id = 0;

};
