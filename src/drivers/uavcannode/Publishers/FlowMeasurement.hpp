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

#include <com/hex/equipment/flow/Measurement.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/optical_flow.h>

namespace uavcannode
{

class FlowMeasurement :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<com::hex::equipment::flow::Measurement>
{
public:
	FlowMeasurement(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(com::hex::equipment::flow::Measurement::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(optical_flow)),
		uavcan::Publisher<com::hex::equipment::flow::Measurement>(node)
	{}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       com::hex::equipment::flow::Measurement::getDataTypeFullName(),
			       com::hex::equipment::flow::Measurement::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// optical_flow -> com::hex::equipment::flow::Measurement
		optical_flow_s optical_flow;

		if (uORB::SubscriptionCallbackWorkItem::update(&optical_flow)) {
			com::hex::equipment::flow::Measurement measurement{};
			measurement.integration_interval  = optical_flow.integration_timespan * 1e-6f; // us -> s
			measurement.rate_gyro_integral[0] = optical_flow.gyro_x_rate_integral;
			measurement.rate_gyro_integral[1] = optical_flow.gyro_y_rate_integral;
			measurement.flow_integral[0] = optical_flow.pixel_flow_x_integral;
			measurement.flow_integral[1] = optical_flow.pixel_flow_y_integral;
			measurement.quality = optical_flow.quality;

			uavcan::Publisher<com::hex::equipment::flow::Measurement>::broadcast(measurement);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
