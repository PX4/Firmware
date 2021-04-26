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
 * @file NodeIDAllocationData.hpp
 *
 * Defines basic functionality of UAVCAN NodeIDAllocationData.1.0 subscription
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include "../NodeManager.hpp"

//Quick and Dirty PNP imlementation only V1 for now as well
#include <uavcan/node/ID_1_0.h>
#include <uavcan/pnp/NodeIDAllocationData_1_0.h>
#include <uavcan/pnp/NodeIDAllocationData_2_0.h>

#define PNP1_PORT_ID                                 uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_
#define PNP1_PAYLOAD_SIZE                            uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_
#define PNP2_PORT_ID                                 uavcan_pnp_NodeIDAllocationData_2_0_FIXED_PORT_ID_
#define PNP2_PAYLOAD_SIZE                            uavcan_pnp_NodeIDAllocationData_2_0_SERIALIZATION_BUFFER_SIZE_BYTES_

#include "BaseSubscriber.hpp"

class UavcanNodeIDAllocationDataSubscriber : public UavcanBaseSubscriber
{
public:
	UavcanNodeIDAllocationDataSubscriber(CanardInstance &ins, NodeManager &nmgr) :
		UavcanBaseSubscriber(ins, "NodeIDAllocationData", 0), _nmgr(nmgr) { };

	void subscribe() override
	{
		// Subscribe to messages uavcan.pnp.NodeIDAllocationData
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindMessage,
				  (_canard_instance.mtu_bytes == CANARD_MTU_CAN_FD ? PNP2_PORT_ID : PNP1_PORT_ID),  // The fixed Subject-ID
				  (_canard_instance.mtu_bytes == CANARD_MTU_CAN_FD ? PNP2_PAYLOAD_SIZE : PNP1_PAYLOAD_SIZE),
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_subj_sub._canard_sub);

	};

	void callback(const CanardTransfer &receive) override
	{
		PX4_INFO("NodeIDAllocationData");

		if (_canard_instance.mtu_bytes == CANARD_MTU_CAN_FD) {
			uavcan_pnp_NodeIDAllocationData_2_0 node_id_alloc_msg {};
			size_t msg_size_in_bytes = receive.payload_size;
			uavcan_pnp_NodeIDAllocationData_2_0_deserialize_(&node_id_alloc_msg, (const uint8_t *)receive.payload,
					&msg_size_in_bytes);
			/// do something with the data
			_nmgr.HandleNodeIDRequest(node_id_alloc_msg);

		} else {
			uavcan_pnp_NodeIDAllocationData_1_0 node_id_alloc_msg {};
			size_t msg_size_in_bytes = receive.payload_size;
			uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(&node_id_alloc_msg, (const uint8_t *)receive.payload,
					&msg_size_in_bytes);
			/// do something with the data
			_nmgr.HandleNodeIDRequest(node_id_alloc_msg);
		}

	};

private:
	NodeManager &_nmgr;

};
