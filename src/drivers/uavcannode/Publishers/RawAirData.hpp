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

#pragma once

#include "UavcanPublisherBase.hpp"

#include <uavcan/equipment/air_data/RawAirData.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/differential_pressure.h>

namespace uavcannode
{

class RawAirData :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::air_data::RawAirData>
{
public:
	RawAirData(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::air_data::RawAirData::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(differential_pressure)),
		uavcan::Publisher<uavcan::equipment::air_data::RawAirData>(node)
	{}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::air_data::RawAirData::getDataTypeFullName(),
			       uavcan::equipment::air_data::RawAirData::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// differential_pressure -> uavcan::equipment::air_data::RawAirData
		differential_pressure_s diff_press;

		if (uORB::SubscriptionCallbackWorkItem::update(&diff_press)) {
			uavcan::equipment::air_data::RawAirData raw_air_data{};

			// raw_air_data.static_pressure =
			raw_air_data.differential_pressure = diff_press.differential_pressure_raw_pa;
			// raw_air_data.static_pressure_sensor_temperature =
			raw_air_data.differential_pressure_sensor_temperature = diff_press.temperature - CONSTANTS_ABSOLUTE_NULL_CELSIUS;
			raw_air_data.static_air_temperature = diff_press.temperature - CONSTANTS_ABSOLUTE_NULL_CELSIUS;
			// raw_air_data.pitot_temperature
			// raw_air_data.covariance
			uavcan::Publisher<uavcan::equipment::air_data::RawAirData>::broadcast(raw_air_data);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
