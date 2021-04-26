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

#include "UavcanSubscriberBase.hpp"

#include <uavcan/equipment/indication/BeepCommand.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/tune_control.h>

namespace uavcannode
{

class BeepCommand;

typedef uavcan::MethodBinder<BeepCommand *,
	void (BeepCommand::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::indication::BeepCommand>&)>
	BeepcommandBinder;

class BeepCommand :
	public UavcanSubscriberBase,
	private uavcan::Subscriber<uavcan::equipment::indication::BeepCommand, BeepcommandBinder>
{
public:
	BeepCommand(uavcan::INode &node) :
		UavcanSubscriberBase(uavcan::equipment::indication::BeepCommand::DefaultDataTypeID),
		uavcan::Subscriber<uavcan::equipment::indication::BeepCommand, BeepcommandBinder>(node)
	{}

	bool init()
	{
		if (start(BeepcommandBinder(this, &BeepCommand::callback)) < 0) {
			PX4_ERR("uavcan::equipment::indication::BeepCommand subscription failed");
			return false;
		}

		return true;
	}

	void PrintInfo() const override
	{
		printf("\t%s:%d -> %s\n",
		       uavcan::equipment::indication::BeepCommand::getDataTypeFullName(),
		       uavcan::equipment::indication::BeepCommand::DefaultDataTypeID,
		       _tune_control_pub.get_topic()->o_name);
	}

private:
	void callback(const uavcan::ReceivedDataStructure<uavcan::equipment::indication::BeepCommand> &msg)
	{
		tune_control_s tune_control{};
		tune_control.tune_id = 0;
		tune_control.frequency = (uint16_t)msg.frequency;
		tune_control.duration = uavcan::uint32_t(1000000 * msg.duration);
		tune_control.volume = 0xff;
		tune_control.timestamp = hrt_absolute_time();
		_tune_control_pub.publish(tune_control);
	}

	uORB::Publication<tune_control_s> _tune_control_pub{ORB_ID(tune_control)};
};
} // namespace uavcannode
