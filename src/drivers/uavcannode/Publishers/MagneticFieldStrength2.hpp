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

#include <uavcan/equipment/ahrs/MagneticFieldStrength2.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_mag.h>

namespace uavcannode
{

class MagneticFieldStrength2 :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::ahrs::MagneticFieldStrength2>
{
public:
	MagneticFieldStrength2(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::ahrs::MagneticFieldStrength2::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(sensor_mag)),
		uavcan::Publisher<uavcan::equipment::ahrs::MagneticFieldStrength2>(node)
	{}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::ahrs::MagneticFieldStrength2::getDataTypeFullName(),
			       uavcan::equipment::ahrs::MagneticFieldStrength2::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// sensor_mag -> uavcan::equipment::ahrs::MagneticFieldStrength2
		sensor_mag_s mag;

		if (uORB::SubscriptionCallbackWorkItem::update(&mag)) {
			uavcan::equipment::ahrs::MagneticFieldStrength2 magnetic_field{};
			magnetic_field.sensor_id = mag.device_id;
			magnetic_field.magnetic_field_ga[0] = mag.x;
			magnetic_field.magnetic_field_ga[1] = mag.y;
			magnetic_field.magnetic_field_ga[2] = mag.z;
			uavcan::Publisher<uavcan::equipment::ahrs::MagneticFieldStrength2>::broadcast(magnetic_field);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
