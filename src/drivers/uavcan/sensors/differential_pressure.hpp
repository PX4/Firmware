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

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include "sensor_bridge.hpp"

#include <uavcan/equipment/air_data/RawAirData.hpp>

class UavcanDifferentialPressureBridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanDifferentialPressureBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	float _diff_pres_offset{0.f};

	math::LowPassFilter2p _filter{10.f, 1.1f}; /// Adapted from MS5525 driver

	void air_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::RawAirData> &msg);

	typedef uavcan::MethodBinder < UavcanDifferentialPressureBridge *,
		void (UavcanDifferentialPressureBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::RawAirData> &) >
		AirCbBinder;

	uavcan::Subscriber<uavcan::equipment::air_data::RawAirData, AirCbBinder> _sub_air;
};
