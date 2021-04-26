/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @author CUAVcaijie <caijie@cuav.net>
 */

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/safety.h>

#include <uavcan/uavcan.hpp>
#include <ardupilot/indication/Button.hpp>
#include "sensor_bridge.hpp"

class UavcanSafetyBridge : public IUavcanSensorBridge
{
public:
	static const char *const NAME;

	UavcanSafetyBridge(uavcan::INode &node);
	~UavcanSafetyBridge() = default;

	const char *get_name() const override { return NAME; }

	int init() override;

	unsigned get_num_redundant_channels() const override;

	void print_status() const override;
private:
	safety_s _safety{};  //
	bool _safety_disabled{false};

	bool _safety_btn_off{false};		///< State of the safety button read from the HW button
	void safety_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::indication::Button> &msg);

	typedef uavcan::MethodBinder < UavcanSafetyBridge *,
		void (UavcanSafetyBridge::*)(const uavcan::ReceivedDataStructure<ardupilot::indication::Button> &) >
		SafetyCommandCbBinder;

	uavcan::INode &_node;
	uavcan::Subscriber<ardupilot::indication::Button, SafetyCommandCbBinder> _sub_safety;
	uavcan::Publisher<ardupilot::indication::Button> _pub_safety;

	uORB::PublicationMulti<safety_s> _safety_pub{ORB_ID(safety)};
};
