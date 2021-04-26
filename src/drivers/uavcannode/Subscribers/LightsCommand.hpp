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

#include <uavcan/equipment/indication/LightsCommand.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/led_control.h>

namespace uavcannode
{

class LightsCommand;

typedef uavcan::MethodBinder<LightsCommand *,
	void (LightsCommand::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::indication::LightsCommand>&)>
	LightscommandBinder;

class LightsCommand :
	public UavcanSubscriberBase,
	private uavcan::Subscriber<uavcan::equipment::indication::LightsCommand, LightscommandBinder>
{
public:
	LightsCommand(uavcan::INode &node) :
		UavcanSubscriberBase(uavcan::equipment::indication::LightsCommand::DefaultDataTypeID),
		uavcan::Subscriber<uavcan::equipment::indication::LightsCommand, LightscommandBinder>(node)
	{}

	bool init()
	{
		if (start(LightscommandBinder(this, &LightsCommand::callback)) < 0) {
			PX4_ERR("uavcan::equipment::indication::LightsCommand subscription failed");
			return false;
		}

		return true;
	}

	void PrintInfo() const override
	{
		printf("\t%s:%d -> %s\n",
		       uavcan::equipment::indication::LightsCommand::getDataTypeFullName(),
		       uavcan::equipment::indication::LightsCommand::DefaultDataTypeID,
		       _led_control_pub.get_topic()->o_name);
	}

private:
	void callback(const uavcan::ReceivedDataStructure<uavcan::equipment::indication::LightsCommand> &msg)
	{
		uavcan::uint32_t red = 0;
		uavcan::uint32_t green = 0;
		uavcan::uint32_t blue = 0;

		for (auto &cmd : msg.commands) {
			if (cmd.light_id == _self_light_index) {
				using uavcan::equipment::indication::RGB565;

				red = uavcan::uint32_t(float(cmd.color.red) *
						       (255.0F / float(RGB565::FieldTypes::red::max())) + 0.5F);

				green = uavcan::uint32_t(float(cmd.color.green) *
							 (255.0F / float(RGB565::FieldTypes::green::max())) + 0.5F);

				blue = uavcan::uint32_t(float(cmd.color.blue) *
							(255.0F / float(RGB565::FieldTypes::blue::max())) + 0.5F);

				red   = uavcan::min<uavcan::uint32_t>(red, 0xFFU);
				green = uavcan::min<uavcan::uint32_t>(green, 0xFFU);
				blue  = uavcan::min<uavcan::uint32_t>(blue, 0xFFU);

				led_control_s led_control{};
				led_control.num_blinks = 0;
				led_control.priority = led_control_s::MAX_PRIORITY;
				led_control.mode = led_control_s::MODE_OFF;
				led_control.led_mask = 0xff;
				led_control.color = led_control_s::COLOR_OFF;

				if (red != 0 && blue == 0 && green == 0) {
					led_control.color = led_control_s::COLOR_RED;

				} else if (red == 0 && blue != 0 && green == 0) {
					led_control.color = led_control_s::COLOR_BLUE;

				} else if (red == 0 && blue == 0 && green != 0) {
					led_control.color = led_control_s::COLOR_GREEN;

				} else if (red != 0 && blue == 0 && green != 0) {
					led_control.color = led_control_s::COLOR_YELLOW;

				} else if (red != 0 && blue != 0 && green == 0) {
					led_control.color = led_control_s::COLOR_PURPLE;

				} else if (red != 0 && blue == 0 && green != 0 && red > green) {
					led_control.color = led_control_s::COLOR_AMBER;

				} else if (red == 0 && blue != 0 && green != 0) {
					led_control.color = led_control_s::COLOR_CYAN;

				} else if (red != 0 && blue != 0 && green != 0) {
					led_control.color = led_control_s::COLOR_WHITE;
				}

				if (led_control.color !=  led_control_s::COLOR_OFF) {
					led_control.mode = led_control_s::MODE_ON;
				}

				led_control.timestamp = hrt_absolute_time();
				_led_control_pub.publish(led_control);
			}
		}
	}

	uORB::Publication<led_control_s> _led_control_pub{ORB_ID(led_control)};
	unsigned _self_light_index{0};
};
} // namespace uavcannode
