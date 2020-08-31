/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file esc.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "esc.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#define MOTOR_BIT(x) (1<<(x))

using namespace time_literals;

UavcanEscController::UavcanEscController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_uavcan_sub_status(node),
	_orb_timer(node)
{
	_uavcan_pub_raw_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
}

UavcanEscController::~UavcanEscController()
{
}

int
UavcanEscController::init()
{
	// ESC status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

	if (res < 0) {
		PX4_ERR("ESC status sub failed %i", res);
		return res;
	}

	// ESC status will be relayed from UAVCAN bus into ORB at this rate
	_orb_timer.setCallback(TimerCbBinder(this, &UavcanEscController::orb_pub_timer_cb));
	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ESC_STATUS_UPDATE_RATE_HZ));

	return res;
}

void
UavcanEscController::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	if (num_outputs > uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize) {
		num_outputs = uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize;
	}

	if (num_outputs > esc_status_s::CONNECTED_ESC_MAX) {
		num_outputs = esc_status_s::CONNECTED_ESC_MAX;
	}

	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	/*
	 * Fill the command message
	 * If unarmed, we publish an empty message anyway
	 */
	uavcan::equipment::esc::RawCommand msg;

	for (unsigned i = 0; i < num_outputs; i++) {
		if (stop_motors || outputs[i] == DISARMED_OUTPUT_VALUE) {
			msg.cmd.push_back(static_cast<unsigned>(0));

		} else {
			msg.cmd.push_back(static_cast<int>(outputs[i]));
		}
	}

	/*
	 * Remove channels that are always zero.
	 * The objective of this optimization is to avoid broadcasting multi-frame transfers when a single frame
	 * transfer would be enough. This is a valid optimization as the UAVCAN specification implies that all
	 * non-specified ESC setpoints should be considered zero.
	 * The positive outcome is a (marginally) lower bus traffic and lower CPU load.
	 *
	 * From the standpoint of the PX4 architecture, however, this is a hack. It should be investigated why
	 * the mixer returns more outputs than are actually used.
	 */
	for (int index = int(msg.cmd.size()) - 1; index >= _max_number_of_nonzero_outputs; index--) {
		if (msg.cmd[index] != 0) {
			_max_number_of_nonzero_outputs = index + 1;
			break;
		}
	}

	msg.cmd.resize(_max_number_of_nonzero_outputs);

	/*
	 * Publish the command message to the bus
	 * Note that for a quadrotor it takes one CAN frame
	 */
	_uavcan_pub_raw_cmd.broadcast(msg);
}

void
UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.esc_index];

		ref.esc_address = msg.getSrcNodeID().get();
		ref.timestamp       = hrt_absolute_time();
		ref.esc_voltage     = msg.voltage;
		ref.esc_current     = msg.current;
		ref.esc_temperature = msg.temperature;
		ref.esc_rpm         = msg.rpm;
		ref.esc_errorcount  = msg.error_count;
	}
}

void
UavcanEscController::orb_pub_timer_cb(const uavcan::TimerEvent &)
{
	_esc_status.timestamp = hrt_absolute_time();
	_esc_status.esc_count = _rotor_count;
	_esc_status.counter += 1;
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
	_esc_status.esc_online_flags = check_escs_status();
	_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;

	_esc_status_pub.publish(_esc_status);
}

uint8_t
UavcanEscController::check_escs_status()
{
	int esc_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		if (_esc_status.esc[index].timestamp > 0 && now - _esc_status.esc[index].timestamp < 1200_ms) {
			esc_status_flags |= (1 << index);
		}

	}

	return esc_status_flags;
}
