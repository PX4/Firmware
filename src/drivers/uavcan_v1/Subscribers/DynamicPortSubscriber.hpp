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
 * @file DynamicPortSubscriber.hpp
 *
 * Defines basic functionality of UAVCAN v1 subscriber class with Non-fixed unregulated port identifier
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>

#include <lib/parameters/param.h>

#include <uavcan/_register/Access_1_0.h>

#include "DynamicPortSubscriber.hpp"
#include "../CanardInterface.hpp"
#include "../ParamManager.hpp"

class UavcanDynamicPortSubscriber : public UavcanBaseSubscriber
{
public:
	UavcanDynamicPortSubscriber(CanardInstance &ins, UavcanParamManager &pmgr, const char *subject_name,
				    uint8_t instance = 0) :
		UavcanBaseSubscriber(ins, subject_name, instance), _param_manager(pmgr) { };

	void updateParam()
	{
		SubjectSubscription *curSubj = &_subj_sub;

		while (curSubj != NULL) {
			char uavcan_param[90];
			snprintf(uavcan_param, sizeof(uavcan_param), "uavcan.sub.%s.%d.id", curSubj->_subject_name, _instance);

			// Set _port_id from _uavcan_param
			uavcan_register_Value_1_0 value;
			_param_manager.GetParamByName(uavcan_param, value);
			int32_t new_id = value.integer32.value.elements[0];

			/* FIXME how about partial subscribing */
			if (curSubj->_canard_sub._port_id != new_id) {
				if (new_id == CANARD_PORT_ID_UNSET) {
					// Cancel subscription
					unsubscribe();

				} else {
					if (curSubj->_canard_sub._port_id != CANARD_PORT_ID_UNSET) {
						// Already active; unsubscribe first
						unsubscribe();
					}

					// Subscribe on the new port ID
					curSubj->_canard_sub._port_id = (CanardPortID)new_id;
					PX4_INFO("Subscribing %s.%d on port %d", curSubj->_subject_name, _instance, curSubj->_canard_sub._port_id);
					subscribe();
				}
			}

			curSubj = curSubj->next;
		}
	};

protected:
	UavcanParamManager &_param_manager;
};
