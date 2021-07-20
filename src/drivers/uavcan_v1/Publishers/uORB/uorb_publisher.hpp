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
 * @file uorb_template.hpp
 *
* Defines generic, templatized uORB over UAVCANv1 publisher
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include "../Publisher.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>

template <class T>
class uORB_over_UAVCAN_Publisher : public UavcanPublisher
{
public:
	uORB_over_UAVCAN_Publisher(CanardInstance &ins, UavcanParamManager &pmgr, const orb_metadata *meta,
				   uint8_t instance = 0) :
		UavcanPublisher(ins, pmgr, meta->o_name, instance),
		_uorb_meta{meta},
		_uorb_sub(meta)
	{};

	~uORB_over_UAVCAN_Publisher() override { };

	// Update the uORB Subscription and broadcast a UAVCAN message
	virtual void update() override
	{
		// Not sure if actuator_armed is a good indication of readiness but seems close to it
		if (_uorb_sub.updated() && _port_id != CANARD_PORT_ID_UNSET) {
			T data {};
			_uorb_sub.update(&data);

			CanardTransfer transfer = {
				.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id, // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id,
				.payload_size   = get_payload_size(&data),
				.payload        = &data,
			};

			// set the data ready in the buffer and chop if needed
			++_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
			canardTxPush(&_canard_instance, &transfer);
		}
	};

protected:
	// Default payload-size function -- can specialize in derived class
	size_t get_payload_size(T *msg)
	{
		(void)msg;
		return sizeof(T);
	}

private:
	const orb_metadata *_uorb_meta;
	uORB::Subscription _uorb_sub;
};

/* ---- Specializations of get_payload_size() to reduce wasted bandwidth where possible ---- */

template<>
size_t uORB_over_UAVCAN_Publisher<actuator_outputs_s>::get_payload_size(actuator_outputs_s *msg);
