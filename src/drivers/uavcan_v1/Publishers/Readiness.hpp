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
 * @file Readiness.hpp
 *
 * Defines the UAVCAN v1 readiness publisher
 * readiness state is used to command or report the availability status
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

// DS-15 Specification Messages
#include <reg/drone/service/common/Readiness_0_1.h>

#include "Publisher.hpp"

class UavcanReadinessPublisher : public UavcanPublisher
{
public:
	UavcanReadinessPublisher(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanPublisher(ins, pmgr, "readiness", instance)
	{

	};

	// Update the uORB Subscription and broadcast a UAVCAN message
	virtual void update() override
	{
		// Not sure if actuator_armed is a good indication of readiness but seems close to it
		if (_actuator_armed_sub.updated() && _port_id != CANARD_PORT_ID_UNSET && _port_id != 0) {
			actuator_armed_s armed {};
			_actuator_armed_sub.update(&armed);

			reg_drone_service_common_Readiness_0_1 readiness {};

			if (armed.armed) {
				readiness.value = reg_drone_service_common_Readiness_0_1_ENGAGED;

			} else {
				readiness.value = reg_drone_service_common_Readiness_0_1_STANDBY;
			}

			uint8_t readiness_payload_buffer[reg_drone_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

			CanardTransfer transfer = {
				.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id, // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id,
				.payload_size   = reg_drone_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
				.payload        = &readiness_payload_buffer,
			};

			int32_t result = reg_drone_service_common_Readiness_0_1_serialize_(&readiness, readiness_payload_buffer,
					 &transfer.payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = canardTxPush(&_canard_instance, &transfer);
			}
		}
	};

private:
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
};
