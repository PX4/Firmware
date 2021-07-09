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
 * @file actuator_outputs.hpp
 *
* Defines uORB over UAVCANv1 actuator_outputs publisher
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <uORB/topics/actuator_outputs.h>

#include "../Publisher.hpp"

class UORB_over_UAVCAN_actuator_outputs_Publisher : public UavcanPublisher
{
public:
	UORB_over_UAVCAN_actuator_outputs_Publisher(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanPublisher(ins, pmgr, "actuator_outputs", instance)
	{};

	// Update the uORB Subscription and broadcast a UAVCAN message
	// FIXME think about update and limiting
	virtual void update() override
	{
		// Not sure if actuator_armed is a good indication of readiness but seems close to it
		if (_actuator_outputs_sub.updated() && _port_id != CANARD_PORT_ID_UNSET && _port_id != 0) {
			actuator_outputs_s actuator_msg {};
			_actuator_outputs_sub.update(&actuator_msg);

			CanardTransfer transfer = {
				.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id, // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id,
				.payload_size   = actuator_payload_size(&actuator_msg),
				.payload        = &actuator_msg,
			};

			// set the data ready in the buffer and chop if needed
			++_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
			canardTxPush(&_canard_instance, &transfer);
		}
	};

private:

	// Remove unvalid output & padding from payload_size to save bandwidth
	size_t actuator_payload_size(actuator_outputs_s *msg)
	{
		return sizeof(struct actuator_outputs_s) - sizeof(msg->_padding0) -
		       ((sizeof(msg->output) / sizeof(msg->output[0]) - msg->noutputs) * sizeof(msg->output[0]));
	}

	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
};
