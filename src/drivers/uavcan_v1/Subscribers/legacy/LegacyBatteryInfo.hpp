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
 * @file Battery.hpp
 *
 * Defines basic functionality of UAVCAN legacy Battery subscription
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <uORB/topics/battery_status.h>
#include <uORB/PublicationMulti.hpp>

// Legacy message from UAVCANv0
#include <legacy/equipment/power/BatteryInfo_1_0.h>

#include "../DynamicPortSubscriber.hpp"

class UavcanLegacyBatteryInfoSubscriber : public UavcanDynamicPortSubscriber
{
public:
	UavcanLegacyBatteryInfoSubscriber(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(ins, pmgr, "legacy_bms", instance) { };

	void subscribe() override
	{
		// Subscribe to messages reg.drone.service.battery.Status.0.1
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindMessage,
				  _subj_sub._canard_sub.port_id,
				  legacy_equipment_power_BatteryInfo_1_0_EXTENT_BYTES_,
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC * 100, //FIXME timeout caused by scheduler
				  &_subj_sub._canard_sub);
	};

	void callback(const CanardTransfer &receive) override
	{
		PX4_INFO("Legacy BmsCallback");

		legacy_equipment_power_BatteryInfo_1_0 bat_info {};
		size_t bat_info_size_in_bytes = receive.payload_size;
		legacy_equipment_power_BatteryInfo_1_0_deserialize_(&bat_info, (const uint8_t *)receive.payload,
				&bat_info_size_in_bytes);

		battery_status_s bat_status {0};
		bat_status.timestamp = hrt_absolute_time();
		bat_status.voltage_filtered_v = bat_info.voltage;
		bat_status.current_filtered_a = bat_info.current;
		bat_status.current_average_a = bat_info.average_power_10sec;
		bat_status.remaining = bat_info.state_of_charge_pct / 100.0f;
		bat_status.scale = -1;

		if (bat_info.status_flags & legacy_equipment_power_BatteryInfo_1_0_STATUS_FLAG_TEMP_HOT) {
			bat_status.temperature = 100;

		} else if (bat_info.status_flags & legacy_equipment_power_BatteryInfo_1_0_STATUS_FLAG_TEMP_COLD) {
			bat_status.temperature = -30;

		} else {
			bat_status.temperature = 20; // Temp okay ?
		}

		bat_status.cell_count = 0; // Unknown
		bat_status.connected = bat_info.status_flags & legacy_equipment_power_BatteryInfo_1_0_STATUS_FLAG_IN_USE;
		bat_status.source = 1; // External
		bat_status.capacity = bat_info.full_charge_capacity_wh;
		bat_status.serial_number = bat_info.model_instance_id & 0xFFFF; // Take first 16 bits
		bat_status.state_of_health = bat_info.state_of_health_pct; // External
		bat_status.id = bat_info.battery_id;

		/* Missing fields in UAVCANv0 legacy message
		 * temperature (partly)
		 * cell_count
		 * connected (partly)
		 * priority
		 * cycle_count
		 * run_time_to_empty
		 * average_time_to_empty
		 * manufacture_date
		 * max_error
		 * interface_error
		 * voltage_cell_v
		 * max_cell_voltage_delta
		 * is_powering_off
		 * warning
		 */


		_battery_status_pub.publish(bat_status);
		print_message(bat_status);
	};

private:
	uORB::PublicationMulti<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};

};
