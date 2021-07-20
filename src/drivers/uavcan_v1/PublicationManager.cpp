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
 * @file PublicationManager.cpp
 *
 * Manages the dynamic (run-time configurable) UAVCAN publications
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */


#include "PublicationManager.hpp"

PublicationManager::~PublicationManager()
{
	UavcanPublisher *dynpub;

	while (_dynpublishers != NULL) {
		dynpub = _dynpublishers;
		_dynpublishers = dynpub->next();
		delete dynpub;
	}
}

void PublicationManager::updateDynamicPublications()
{
	for (auto &sub : _uavcan_pubs) {

		bool found_publisher = false;
		UavcanPublisher *dynpub = _dynpublishers;

		while (dynpub != NULL) {
			// Check if subscriber has already been created
			const char *subj_name = dynpub->getSubjectName();
			const uint8_t instance = dynpub->getInstance();

			if (strcmp(subj_name, sub.subject_name) == 0 && instance == sub.instance) {
				found_publisher = true;
				break;
			}

			dynpub = dynpub->next();
		}

		if (found_publisher) {
			continue;
		}

		char uavcan_param[90];
		snprintf(uavcan_param, sizeof(uavcan_param), "uavcan.pub.%s.%d.id", sub.subject_name, sub.instance);
		uavcan_register_Value_1_0 value;

		if (_param_manager.GetParamByName(uavcan_param, value)) {
			uint16_t port_id = value.natural16.value.elements[0];

			if (port_id <= CANARD_PORT_ID_MAX) { // PortID is set, create a subscriber
				dynpub = sub.create_pub(_canard_instance, _param_manager);

				if (dynpub == NULL) {
					PX4_ERR("Out of memory");
					return;
				}

				if (_dynpublishers == NULL) {
					// Set the head of our linked list
					_dynpublishers = dynpub;

				} else {
					// Append the new subscriber to our linked list
					UavcanPublisher *tmp = _dynpublishers;

					while (tmp->next() != NULL) {
						tmp = tmp->next();
					}

					tmp->setNext(dynpub);
				}

				dynpub->updateParam();
			}

		} else {
			PX4_ERR("Port ID param for publisher %s.%u not found", sub.subject_name, sub.instance);
			return;
		}
	}
}

void PublicationManager::printInfo()
{
	UavcanPublisher *dynpub = _dynpublishers;

	while (dynpub != NULL) {
		dynpub->printInfo();
		dynpub = dynpub->next();
	}
}

void PublicationManager::updateParams()
{
	UavcanPublisher *dynpub = _dynpublishers;

	while (dynpub != NULL) {
		dynpub->updateParam();
		dynpub = dynpub->next();
	}

	// Check for any newly-enabled publication
	updateDynamicPublications();
}

void PublicationManager::update()
{
	UavcanPublisher *dynpub = _dynpublishers;

	while (dynpub != NULL) {
		dynpub->update();
		dynpub = dynpub->next();
	}
}
