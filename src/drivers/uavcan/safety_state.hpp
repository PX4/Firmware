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
 * @file safety_state.hpp
 *
 * @author CUAVcaijie <caijie@cuav.net>
 *
 * @brief According to actuator_armed to control the CAN SafetyState led
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <ardupilot/indication/SafetyState.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>


class UavcanSafetyState
{
public:
	UavcanSafetyState(uavcan::INode &node);

	/*
	 * setup periodic updater
	 */
	int init();

private:
	/*
	 * Max update rate to avoid exessive bus traffic
	 */
	static constexpr unsigned MAX_RATE_HZ = 10;

	/*
	 * Setup timer and call back function for periodic updates
	 */
	void periodic_update(const uavcan::TimerEvent &);

	typedef uavcan::MethodBinder<UavcanSafetyState *, void (UavcanSafetyState::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	/*
	 * Publish CAN Safety state led
	 */
	uavcan::Publisher<ardupilot::indication::SafetyState> _safety_state_pub;

	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

};
