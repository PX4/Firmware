/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @author RJ Gritter <rjgritter657@gmail.com>
 */

#pragma once

#include "sensor_bridge.hpp"
#include <drivers/drv_airspeed.h>
#include <uORB/topics/airspeed.h>

#include <uavcan/equipment/air_data/IndicatedAirspeed.hpp>
#include <uavcan/equipment/air_data/TrueAirspeed.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>

class UavcanAirspeedBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanAirspeedBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	void ias_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::IndicatedAirspeed> &msg);
	void tas_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::TrueAirspeed> &msg);
	void oat_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature> &msg);

	typedef uavcan::MethodBinder < UavcanAirspeedBridge *,
		void (UavcanAirspeedBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::IndicatedAirspeed> &) >
		IASCbBinder;

	typedef uavcan::MethodBinder < UavcanAirspeedBridge *,
		void (UavcanAirspeedBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::TrueAirspeed> &) >
		TASCbBinder;

	typedef uavcan::MethodBinder < UavcanAirspeedBridge *,
		void (UavcanAirspeedBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature> &) >
		OATCbBinder;

	uavcan::Subscriber<uavcan::equipment::air_data::IndicatedAirspeed, IASCbBinder> _sub_ias_data;
	uavcan::Subscriber<uavcan::equipment::air_data::TrueAirspeed, TASCbBinder> _sub_tas_data;
	uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature, OATCbBinder> _sub_oat_data;

	float _last_tas_m_s{0.0f};
	float _last_outside_air_temp_k{0.0f};

};
