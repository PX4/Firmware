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
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#include "ice_status.hpp"
#include <uORB/topics/internal_combustion_engine_status.h>

const char *const UavcanIceStatusBridge::NAME = "ice_status";

UavcanIceStatusBridge::UavcanIceStatusBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_ice_status", ORB_ID(internal_combustion_engine_status)),
	_sub_ice_status_data(node)
{ }

int UavcanIceStatusBridge::init()
{
	int res = _sub_ice_status_data.start(IceStatusCbBinder(this, &UavcanIceStatusBridge::ice_status_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanIceStatusBridge::ice_status_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::ice::reciprocating::Status> &msg)
{
	auto report = ::internal_combustion_engine_status_s();
	report.timestamp = hrt_absolute_time();
	report.state = msg.state;
	report.flags = msg.flags;
	report.engine_load_percent = msg.engine_load_percent;
	report.engine_speed_rpm = msg.engine_speed_rpm;
	report.spark_dwell_time_ms = msg.spark_dwell_time_ms;
	report.atmospheric_pressure_kpa = msg.atmospheric_pressure_kpa;
	report.intake_manifold_pressure_kpa = msg.intake_manifold_pressure_kpa;
	report.intake_manifold_temperature = msg.intake_manifold_temperature;
	report.coolant_temperature = msg.coolant_temperature;
	report.oil_pressure = msg.oil_pressure;
	report.oil_temperature = msg.oil_temperature;
	report.fuel_pressure = msg.fuel_pressure;
	report.fuel_consumption_rate_cm3pm = msg.fuel_consumption_rate_cm3pm;
	report.estimated_consumed_fuel_volume_cm3 = msg.estimated_consumed_fuel_volume_cm3;
	report.throttle_position_percent = msg.throttle_position_percent;
	report.ecu_index = msg.ecu_index;
	report.spark_plug_usage = msg.spark_plug_usage;

	if (msg.cylinder_status.size() > 0) {
		report.ignition_timing_deg = msg.cylinder_status[0].ignition_timing_deg;
		report.injection_time_ms = msg.cylinder_status[0].injection_time_ms;
		report.cylinder_head_temperature = msg.cylinder_status[0].cylinder_head_temperature;
		report.exhaust_gas_temperature = msg.cylinder_status[0].exhaust_gas_temperature;
		report.lambda_coefficient = msg.cylinder_status[0].lambda_coefficient;
	}

	publish(msg.getSrcNodeID().get(), &report);
}

int UavcanIceStatusBridge::init_driver(uavcan_bridge::Channel *channel)
{
	return PX4_OK;
}
