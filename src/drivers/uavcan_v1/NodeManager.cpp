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
 * @file NodeIDManager.cpp
 *
 * Defines basic implementation of UAVCAN PNP for dynamic Node ID
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#define RETRY_COUNT 10

#include "NodeManager.hpp"

void NodeManager::callback(const CanardTransfer &receive)
{
	if (_canard_instance.mtu_bytes == CANARD_MTU_CAN_FD) {
		uavcan_pnp_NodeIDAllocationData_2_0 node_id_alloc_msg {};
		size_t msg_size_in_bytes = receive.payload_size;
		uavcan_pnp_NodeIDAllocationData_2_0_deserialize_(&node_id_alloc_msg, (const uint8_t *)receive.payload,
				&msg_size_in_bytes);
		/// do something with the data
		HandleNodeIDRequest(node_id_alloc_msg);

	} else {
		uavcan_pnp_NodeIDAllocationData_1_0 node_id_alloc_msg {};
		size_t msg_size_in_bytes = receive.payload_size;
		uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(&node_id_alloc_msg, (const uint8_t *)receive.payload,
				&msg_size_in_bytes);
		/// do something with the data
		HandleNodeIDRequest(node_id_alloc_msg);
	}
}

bool NodeManager::HandleNodeIDRequest(uavcan_pnp_NodeIDAllocationData_1_0 &msg)
{
	if (msg.allocated_node_id.count == 0) {
		uint32_t i;

		msg.allocated_node_id.count = 1;
		msg.allocated_node_id.elements[0].value = CANARD_NODE_ID_UNSET;

		/* Search for an available NodeID to assign */
		for (i = 1; i < sizeof(nodeid_registry) / sizeof(nodeid_registry[0]); i++) {
			if (i == _canard_instance.node_id) {
				continue; // Don't give our NodeID to a node

			} else if (nodeid_registry[i].node_id == 0) { // Unused
				nodeid_registry[i].node_id = i;
				memcpy(&nodeid_registry[i].unique_id, &msg.unique_id_hash, 6);
				break;

			} else if (memcmp(&nodeid_registry[i].unique_id[0], &msg.unique_id_hash, 6) == 0) {
				msg.allocated_node_id.elements[0].value = nodeid_registry[i].node_id; // Existing NodeID
				break;
			}
		}

		nodeid_registry[i].register_setup = false; // Re-instantiate register setup
		nodeid_registry[i].register_index = 0;
		nodeid_registry[i].retry_count = 0;

		if (msg.allocated_node_id.elements[0].value != CANARD_NODE_ID_UNSET) {

			PX4_INFO("Received NodeID allocation request assigning %i", msg.allocated_node_id.elements[0].value);

			uint8_t node_id_alloc_payload_buffer[uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

			CanardTransfer transfer = {
				.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_, // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
				.transfer_id    = _uavcan_pnp_nodeidallocation_v1_transfer_id,
				.payload_size   = uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
				.payload        = &node_id_alloc_payload_buffer,
			};

			int result = uavcan_pnp_NodeIDAllocationData_1_0_serialize_(&msg, node_id_alloc_payload_buffer, &transfer.payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_uavcan_pnp_nodeidallocation_v1_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = canardTxPush(&_canard_instance, &transfer);
			}

			_register_request_last = hrt_absolute_time();

			return result >= 0;
		}
	}

	return false;
}

bool NodeManager::HandleNodeIDRequest(uavcan_pnp_NodeIDAllocationData_2_0 &msg)
{
	//TODO v2 node id
	return false;
}


void NodeManager::HandleListResponse(const CanardTransfer &receive)
{
	uavcan_register_List_Response_1_0 msg;

	size_t register_in_size_bits = receive.payload_size;
	uavcan_register_List_Response_1_0_deserialize_(&msg, (const uint8_t *)receive.payload, &register_in_size_bits);

	if (msg.name.name.count == 0) {
		// Index doesn't exist, we've parsed through all registers
		for (uint32_t i = 0; i < sizeof(nodeid_registry) / sizeof(nodeid_registry[0]); i++) {
			if (nodeid_registry[i].node_id == receive.remote_node_id) {
				nodeid_registry[i].register_setup = true; // Don't update anymore
			}
		}

	} else {
		for (uint32_t i = 0; i < sizeof(nodeid_registry) / sizeof(nodeid_registry[0]); i++) {
			if (nodeid_registry[i].node_id == receive.remote_node_id) {
				nodeid_registry[i].register_index++; // Increment index counter for next update()
				nodeid_registry[i].register_setup = false;
				nodeid_registry[i].retry_count = 0;
			}
		}

		if (_access_request.setPortId(receive.remote_node_id, msg.name, NULL)) { //FIXME confirm handler
			PX4_INFO("Set portID succesfull");

		} else {
			PX4_INFO("Register not found %.*s", msg.name.name.count, msg.name.name.elements);
		}
	}
}
void NodeManager::update()
{
	if (hrt_elapsed_time(&_register_request_last) >= hrt_abstime(2 *
			1000000ULL)) { // Compiler hates me here, some 1_s doesn't work
		for (uint32_t i = 0; i < sizeof(nodeid_registry) / sizeof(nodeid_registry[0]); i++) {
			if (nodeid_registry[i].node_id != 0 && nodeid_registry[i].register_setup == false) {
				//Setting up registers
				_list_request.getIndex(nodeid_registry[i].node_id, nodeid_registry[i].register_index, this);
				nodeid_registry[i].retry_count++;

				if (nodeid_registry[i].retry_count > RETRY_COUNT) {
					nodeid_registry[i].register_setup = true; // Don't update anymore
				}
			}
		}

		_register_request_last = hrt_absolute_time();
	}
}
