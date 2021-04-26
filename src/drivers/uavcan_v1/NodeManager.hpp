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
 * @file NodeManager.hpp
 *
 * Defines basic implementation of UAVCAN PNP for dynamic Node ID
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once


#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "CanardInterface.hpp"

#include <uavcan/node/ID_1_0.h>
#include <uavcan/pnp/NodeIDAllocationData_1_0.h>
#include <uavcan/pnp/NodeIDAllocationData_2_0.h>


class NodeManager;

#include "Services/AccessRequest.hpp"
#include "Services/ListRequest.hpp"

//TODO make this an object instead?
typedef struct {
	uint8_t   node_id;
	uint8_t   unique_id[16];
	bool      register_setup;
	uint16_t  register_index;
} UavcanNodeEntry;

class NodeManager
{
public:
	NodeManager(CanardInstance &ins) : _canard_instance(ins), _access_request(ins), _list_request(ins) { };

	bool HandleNodeIDRequest(uavcan_pnp_NodeIDAllocationData_1_0 &msg);
	bool HandleNodeIDRequest(uavcan_pnp_NodeIDAllocationData_2_0 &msg);


	void HandleListResponse(CanardNodeID node_id, uavcan_register_List_Response_1_0 &msg);

	void update();

private:
	CanardInstance &_canard_instance;
	CanardTransferID _uavcan_pnp_nodeidallocation_v1_transfer_id{0};
	UavcanNodeEntry nodeid_registry[16] {0}; //TODO configurable or just rewrite

	UavcanAccessServiceRequest _access_request;
	UavcanListServiceRequest _list_request;

	bool nodeRegisterSetup = 0;

	hrt_abstime _register_request_last{0};

	//TODO work this out
	const char *gps_uorb_register_name = "uavcan.pub.gnss_uorb.id";
	const char *bms_status_register_name = "uavcan.pub.battery_status.id";
};
