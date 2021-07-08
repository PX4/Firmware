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
 * @file List.hpp
 *
 * Defines a List Service invoker and process List responses
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <version/version.h>

#include "../NodeManager.hpp"

#include <uavcan/_register/List_1_0.h>

#include "../Subscribers/BaseSubscriber.hpp"


class UavcanServiceRequestInterface
{
public:
	virtual void response_callback(const CanardTransfer &receive) = 0;
};

class UavcanServiceRequest : public UavcanBaseSubscriber
{
public:
	UavcanServiceRequest(CanardInstance &ins, const char *subject_name, CanardPortID portID, size_t extent) :
		UavcanBaseSubscriber(ins, subject_name, 0), _portID(portID), _extent(extent) { };


	void subscribe() override
	{
		// Subscribe to requests response
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindResponse,
				  _portID,
				  _extent,
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_subj_sub._canard_sub);
	};

	bool request(CanardTransfer *transfer, UavcanServiceRequestInterface *handler)
	{
		_response_callback = handler;
		++request_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
		return canardTxPush(&_canard_instance, transfer) > 0;
	}

	void callback(const CanardTransfer &receive) override
	{
		PX4_INFO("Response");

		if (_response_callback != NULL) {
			_response_callback->response_callback(receive);
		}
	};



protected:
	CanardTransferID request_transfer_id = 0;

	const CanardPortID _portID;
	const size_t _extent;
	UavcanServiceRequestInterface *_response_callback = NULL;

};
