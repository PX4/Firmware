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
 * @file Publisher.hpp
 *
 * Defines basic functionality of UAVCAN v1 publisher class
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>

#include <lib/parameters/param.h>

#include <uavcan/_register/Access_1_0.h>

#include "../CanardInterface.hpp"
#include "../ParamManager.hpp"

/* This is a default baseline timeout for publishers
 * Still it's recommended for implementers to check if their publisher
 * has timing requirements and if it should drop messages that are too old in favour of newer messages
 */
#define PUBLISHER_DEFAULT_TIMEOUT_USEC 100000UL

class UavcanPublisher
{
public:
	UavcanPublisher(CanardInstance &ins, UavcanParamManager &pmgr, const char *subject_name, uint8_t instance = 0) :
		_canard_instance(ins), _param_manager(pmgr), _subject_name(subject_name), _instance(instance) { };

	// Update the uORB Subscription and broadcast a UAVCAN message
	virtual void update() = 0;

	CanardPortID id() { return _port_id; };

	void updateParam()
	{
		char uavcan_param[256];
		snprintf(uavcan_param, sizeof(uavcan_param), "uavcan.pub.%s.%d.id", _subject_name, _instance);

		// Set _port_id from _uavcan_param
		uavcan_register_Value_1_0 value;

		if (_param_manager.GetParamByName(uavcan_param, value)) {
			uint16_t new_id = value.natural16.value.elements[0];

			if (_port_id != new_id) {
				if (new_id == CANARD_PORT_ID_UNSET) {
					PX4_INFO("Disabling publication of subject %s.%d", _subject_name, _instance);
					_port_id = CANARD_PORT_ID_UNSET;

				} else {
					_port_id = (CanardPortID)new_id;
					PX4_INFO("Enabling subject %s.%d on port %d", _subject_name, _instance, _port_id);
				}
			}
		}
	};

	void printInfo()
	{
		if (_port_id != CANARD_PORT_ID_UNSET) {
			PX4_INFO("Enabled subject %s.%d on port %d", _subject_name, _instance, _port_id);

		} else {
			PX4_INFO("Subject %s.%d disabled", _subject_name, _instance);
		}
	}

protected:
	CanardInstance &_canard_instance;
	UavcanParamManager &_param_manager;
	const char *_subject_name;
	uint8_t _instance {0};

	CanardPortID _port_id {CANARD_PORT_ID_UNSET};
	CanardTransferID _transfer_id {0};
};
