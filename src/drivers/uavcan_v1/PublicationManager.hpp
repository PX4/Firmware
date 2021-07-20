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
 * @file PublicationManager.hpp
 *
 * Manages the dynamic (run-time configurable) UAVCAN publications
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once


#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include "Publishers/Publisher.hpp"
#include "CanardInterface.hpp"

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/sensor_gps.h>

#include "Actuators/EscClient.hpp"
#include "Publishers/DS-015/Readiness.hpp"
#include "Publishers/DS-015/Gnss.hpp"
#include "Publishers/uORB/uorb_publisher.hpp"

typedef struct {
	UavcanPublisher *(*create_pub)(CanardInstance &ins, UavcanParamManager &pmgr) {};
	const char *subject_name;
	const uint8_t instance;
} UavcanDynPubBinder;

class PublicationManager
{
public:
	PublicationManager(CanardInstance &ins, UavcanParamManager &pmgr) : _canard_instance(ins), _param_manager(pmgr) {}
	~PublicationManager();

	void update();
	void printInfo();
	void updateParams();

private:
	void updateDynamicPublications();

	CanardInstance &_canard_instance;
	UavcanParamManager &_param_manager;
	UavcanPublisher *_dynpublishers {NULL};

	const UavcanDynPubBinder _uavcan_pubs[5] {
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new UavcanGnssPublisher(ins, pmgr, 0);
			},
			"gps",
			0
		},
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new UavcanEscController(ins, pmgr);
			},
			"esc",
			0
		},
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new UavcanReadinessPublisher(ins, pmgr, 0);
			},
			"readiness",
			0
		},
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new uORB_over_UAVCAN_Publisher<actuator_outputs_s>(ins, pmgr, ORB_ID(actuator_outputs));
			},
			"uorb.actuator_outputs",
			0
		},

		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new uORB_over_UAVCAN_Publisher<sensor_gps_s>(ins, pmgr, ORB_ID(sensor_gps));
			},
			"uorb.sensor_gps",
			0
		},
	};
};
