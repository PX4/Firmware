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

#include <uavcan/equipment/power/BatteryInfo.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/battery_status.h>

namespace uavcannode
{

class BatteryInfo :
	public UavcanPublisherBase,
	private uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::power::BatteryInfo>
{
public:
	BatteryInfo(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::power::BatteryInfo::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(battery_status)),
		uavcan::Publisher<uavcan::equipment::power::BatteryInfo>(node)
	{}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::power::BatteryInfo::getDataTypeFullName(),
			       uavcan::equipment::power::BatteryInfo::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		// battery_status -> uavcan::equipment::power::BatteryInfo
		battery_status_s battery;

		if (uORB::SubscriptionCallbackWorkItem::update(&battery)) {
			uavcan::equipment::power::BatteryInfo battery_info{};
			battery_info.voltage = battery.voltage_v;
			battery_info.current = fabs(battery.current_a);
			battery_info.temperature = battery.temperature - CONSTANTS_ABSOLUTE_NULL_CELSIUS; // convert from C to K
			battery_info.full_charge_capacity_wh = battery.capacity;
			battery_info.remaining_capacity_wh = battery.remaining * battery.capacity;
			battery_info.state_of_charge_pct = battery.remaining * 100;
			battery_info.state_of_charge_pct_stdev = battery.max_error;
			battery_info.model_instance_id = 0; // TODO: what goes here?
			battery_info.model_name = "ARK BMS Rev 0.2";
			battery_info.battery_id = battery.serial_number;
			battery_info.hours_to_full_charge = 0; // TODO: Read BQ40Z80_TIME_TO_FULL
			battery_info.state_of_health_pct = battery.state_of_health;

			if (battery.current_a > 0.0f) {
				battery_info.status_flags = uavcan::equipment::power::BatteryInfo::STATUS_FLAG_CHARGING;

			} else {
				battery_info.status_flags = uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
			}

			uavcan::Publisher<uavcan::equipment::power::BatteryInfo>::broadcast(battery_info);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcan
