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
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#include "differential_pressure.hpp"

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <parameters/param.h>
#include <systemlib/err.h>

const char *const UavcanDifferentialPressureBridge::NAME = "differential_pressure";

UavcanDifferentialPressureBridge::UavcanDifferentialPressureBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_differential_pressure", ORB_ID(differential_pressure)),
	_sub_air(node)
{
}

int UavcanDifferentialPressureBridge::init()
{
	// Initialize the calibration offset
	param_get(param_find("SENS_DPRES_OFF"), &_diff_pres_offset);

	int res = _sub_air.start(AirCbBinder(this, &UavcanDifferentialPressureBridge::air_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanDifferentialPressureBridge::air_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::air_data::RawAirData> &msg)
{
	_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_UAVCAN;
	_device_id.devid_s.address = msg.getSrcNodeID().get() & 0xFF;

	float diff_press_pa = msg.differential_pressure;
	float temperature_c = msg.static_air_temperature + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	differential_pressure_s report = {
		.timestamp = hrt_absolute_time(),
		.error_count = 0,
		.differential_pressure_raw_pa = diff_press_pa - _diff_pres_offset,
		.differential_pressure_filtered_pa = _filter.apply(diff_press_pa) - _diff_pres_offset, /// TODO: Create filter
		.temperature = temperature_c,
		.device_id = _device_id.devid
	};

	publish(msg.getSrcNodeID().get(), &report);
}
