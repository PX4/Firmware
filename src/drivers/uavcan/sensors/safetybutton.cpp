/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include "safetybutton.hpp"
#include <cstdint>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <mathlib/mathlib.h>

using namespace time_literals;
const char *const UavcanSafetyBridge::NAME = "safety";

UavcanSafetyBridge::UavcanSafetyBridge(uavcan::INode &node) :
	_node(node),
	_sub_safety(node),
	_pub_safety(node)
{
}

int UavcanSafetyBridge::init()
{
	int res = _pub_safety.init(uavcan::TransferPriority::MiddleLower);

	if (res < 0) {
		printf("safety pub failed %i", res);
		return res;
	}

	res = _sub_safety.start(SafetyCommandCbBinder(this, &UavcanSafetyBridge::safety_sub_cb));

	if (res < 0) {
		printf("safety pub failed %i", res);
		return res;
	}

	return 0;
}

unsigned UavcanSafetyBridge::get_num_redundant_channels() const
{
	return 0;
}

void UavcanSafetyBridge::print_status() const
{
}

void UavcanSafetyBridge::safety_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::indication::Button> &msg)
{
	if (msg.press_time > 10 && msg.button == 1) {
		if (_safety_disabled) { return; }

		_safety_disabled = true;

	} else {

		_safety_disabled = false;
	}




}
