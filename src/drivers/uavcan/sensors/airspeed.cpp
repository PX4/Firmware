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

#include <drivers/drv_hrt.h>
#include "airspeed.hpp"
#include <math.h>
#include <lib/ecl/geo/geo.h> // For CONSTANTS_*

const char *const UavcanAirspeedBridge::NAME = "airspeed";

UavcanAirspeedBridge::UavcanAirspeedBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_airspeed", ORB_ID(airspeed)),
	_sub_ias_data(node),
	_sub_tas_data(node),
	_sub_oat_data(node)
{ }

int UavcanAirspeedBridge::init()
{
	int res = _sub_ias_data.start(IASCbBinder(this, &UavcanAirspeedBridge::ias_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	int res2 = _sub_tas_data.start(TASCbBinder(this, &UavcanAirspeedBridge::tas_sub_cb));

	if (res2 < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res2);
		return res2;
	}

	int res3 = _sub_oat_data.start(OATCbBinder(this, &UavcanAirspeedBridge::oat_sub_cb));

	if (res3 < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res3);
		return res3;
	}

	return 0;
}

void
UavcanAirspeedBridge::oat_sub_cb(const
				 uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature> &msg)
{
	_last_outside_air_temp_k = msg.static_temperature;
}

void
UavcanAirspeedBridge::tas_sub_cb(const
				 uavcan::ReceivedDataStructure<uavcan::equipment::air_data::TrueAirspeed> &msg)
{
	_last_tas_m_s = msg.true_airspeed;
}
void
UavcanAirspeedBridge::ias_sub_cb(const
				 uavcan::ReceivedDataStructure<uavcan::equipment::air_data::IndicatedAirspeed> &msg)
{
	airspeed_s report{};

	/*
	 * FIXME HACK
	 * This code used to rely on msg.getMonotonicTimestamp().toUSec() instead of HRT.
	 * It stopped working when the time sync feature has been introduced, because it caused libuavcan
	 * to use an independent time source (based on hardware TIM5) instead of HRT.
	 * The proper solution is to be developed.
	 */
	report.timestamp   		= hrt_absolute_time();
	report.indicated_airspeed_m_s   = msg.indicated_airspeed;
	report.true_airspeed_m_s   	= _last_tas_m_s;
	report.air_temperature_celsius 	= _last_outside_air_temp_k + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	publish(msg.getSrcNodeID().get(), &report);
}
