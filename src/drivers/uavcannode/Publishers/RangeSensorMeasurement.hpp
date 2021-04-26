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

#include <uavcan/equipment/range_sensor/Measurement.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/distance_sensor.h>

namespace uavcannode
{

class RangeSensorMeasurement :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::range_sensor::Measurement>
{
public:
	RangeSensorMeasurement(px4::WorkItem *work_item, uavcan::INode &node, uint8_t instance = 0) :
		UavcanPublisherBase(uavcan::equipment::range_sensor::Measurement::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(distance_sensor), instance),
		uavcan::Publisher<uavcan::equipment::range_sensor::Measurement>(node)
	{}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::range_sensor::Measurement::getDataTypeFullName(),
			       uavcan::equipment::range_sensor::Measurement::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// distance_sensor[] -> uavcan::equipment::range_sensor::Measurement
		distance_sensor_s dist;

		if (uORB::SubscriptionCallbackWorkItem::update(&dist)) {
			uavcan::equipment::range_sensor::Measurement range_sensor{};

			range_sensor.sensor_id = get_instance();
			range_sensor.range = dist.current_distance;
			range_sensor.field_of_view = dist.h_fov;

			// sensor type
			switch (dist.type) {
			case distance_sensor_s::MAV_DISTANCE_SENSOR_LASER:
				range_sensor.sensor_type = uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_LIDAR;
				break;

			case distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND:
				range_sensor.sensor_type = uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_SONAR;
				break;

			case distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR:
				range_sensor.sensor_type = uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_RADAR;
				break;

			case distance_sensor_s::MAV_DISTANCE_SENSOR_INFRARED:
			default:
				range_sensor.sensor_type = uavcan::equipment::range_sensor::Measurement::SENSOR_TYPE_UNDEFINED;
				break;
			}

			// reading_type
			if (dist.current_distance >= dist.max_distance) {
				range_sensor.reading_type = uavcan::equipment::range_sensor::Measurement::READING_TYPE_TOO_FAR;

			} else if (dist.current_distance <= dist.min_distance) {
				range_sensor.reading_type = uavcan::equipment::range_sensor::Measurement::READING_TYPE_TOO_CLOSE;

			} else if (dist.signal_quality != 0) {
				range_sensor.reading_type = uavcan::equipment::range_sensor::Measurement::READING_TYPE_VALID_RANGE;

			} else {
				range_sensor.reading_type = uavcan::equipment::range_sensor::Measurement::READING_TYPE_UNDEFINED;
			}

			uavcan::Publisher<uavcan::equipment::range_sensor::Measurement>::broadcast(range_sensor);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
