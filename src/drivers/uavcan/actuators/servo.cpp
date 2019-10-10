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
 * @file UavcanServoController.cpp
 *
 * @author Richard Kirby rkirby@kspresearch.com>
 */

#include "servo.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/topics/parameter_update.h>

#define MOTOR_BIT(x) (1<<(x))

UavcanServoController::UavcanServoController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_array_cmd(node),
	_uavcan_sub_status(node),
	_orb_timer(node),
	_t_param(-1),
	_param_update_force(false)
{
	_uavcan_pub_array_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);

	if (_perfcnt_invalid_input == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_invalid_input");
	}

	if (_perfcnt_scaling_error == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_scaling_error");
	}

	memset(_actuator_status.position, 0, sizeof(_actuator_status.position));
	memset(_actuator_status.power_percent, 0, sizeof(_actuator_status.power_percent));
}

UavcanServoController::~UavcanServoController()
{
	perf_free(_perfcnt_invalid_input);
	perf_free(_perfcnt_scaling_error);
}

int UavcanServoController::init()
{
	// Servo status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanServoController::servo_status_sub_cb));

	if (res < 0) {
		warnx("Servo status sub failed %i", res);
		return res;
	}

	// Servo status will be relayed from UAVCAN bus into ORB at this rate
	_orb_timer.setCallback(TimerCbBinder(this, &UavcanServoController::orb_pub_timer_cb));
	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ACTUATOR_STATUS_UPDATE_RATE_HZ));

	_t_param = orb_subscribe(ORB_ID(parameter_update));
	_param_update_force = true;

	return res;
}

void UavcanServoController::update_outputs(uint8_t actuator_id, float value)
{
	if (actuator_id > MAX_NUM_ACTUATORS) {
		perf_count(_perfcnt_invalid_input);
		return;
	}

	_actuator_outputs.noutputs = MAX_NUM_ACTUATORS;
	_actuator_outputs.timestamp = hrt_absolute_time();

	/*
	 * Rate limiting - we don't want to congest the bus
	 */

	if (_armed_mask & SERVO_BIT(actuator_id)) {
		const auto timestamp = _node.getMonotonicTime();

		// rate limit
		if ((timestamp - _prev_cmd_pub[actuator_id]).toUSec() < (1000000 / _rate_limit)) {
			return;
		}

		_prev_cmd_pub[actuator_id] = timestamp;

		/*
		* Fill the command message
		*/
		uavcan::equipment::actuator::ArrayCommand msg;
		uavcan::equipment::actuator::Command cmd;

		cmd.actuator_id = actuator_id;
		cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
		cmd.command_value = (value * _scale[actuator_id]) +
				    _trim[actuator_id];         // Trim values range from -0.25 to +0.25 scale ranges from 0.75 to 1.0
		cmd.command_value = (cmd.command_value > _max[actuator_id]) ? _max[actuator_id] :
				    cmd.command_value;              // just to be safe
		cmd.command_value = (cmd.command_value < _min[actuator_id]) ? _min[actuator_id] : cmd.command_value;

		_actuator_outputs.output[actuator_id] = cmd.command_value;
		msg.commands.push_back(cmd);

		/*
		* Publish the command message to the bus
		*/
		(void)_uavcan_pub_array_cmd.broadcast(msg);

	} else {
		/* check updates on uORB topics and handle it */
		bool updated = false;
		orb_check(_t_param, &updated);

		if (updated || _param_update_force) {
			_param_update_force = false;
			parameter_update_s pupdate;
			orb_copy(ORB_ID(parameter_update), _t_param, &pupdate);
			update_params();
		}

		// set to disarmed value
		uavcan::equipment::actuator::ArrayCommand msg;
		uavcan::equipment::actuator::Command cmd;
		cmd.actuator_id = actuator_id;
		cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
		cmd.command_value = _disarmed[actuator_id];
		_actuator_outputs.output[actuator_id] = cmd.command_value;
		msg.commands.push_back(cmd);
		(void)_uavcan_pub_array_cmd.broadcast(msg);
	}

	if (_actuator_outputs_pub != nullptr) {
		orb_publish(ORB_ID(actuator_outputs), _actuator_outputs_pub, &_actuator_outputs);

	} else {
		int instance;
		_actuator_outputs_pub = orb_advertise_multi(ORB_ID(actuator_outputs), &_actuator_outputs,
					&instance, ORB_PRIO_DEFAULT);
	}

}

void UavcanServoController::servo_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>
		&msg)
{
	if (msg.actuator_id < MAX_NUM_ACTUATORS) {
		_actuator_status.timestamp = hrt_absolute_time();
		_actuator_status.noutputs = MAX_NUM_ACTUATORS;
		_actuator_status.position[msg.actuator_id] = msg.position;
		_actuator_status.power_percent[msg.actuator_id] = msg.power_rating_pct;
	}
}

void UavcanServoController::orb_pub_timer_cb(const uavcan::TimerEvent &)
{
	_actuator_status.counter += 1;
	_actuator_status.actuator_connectiontype = actuator_status_s::ACTUATOR_CONNECTION_TYPE_CAN;

	if (_actuator_status_pub != nullptr) {
		(void)orb_publish(ORB_ID(actuator_status), _actuator_status_pub, &_actuator_status);

	} else {
		_actuator_status_pub = orb_advertise(ORB_ID(actuator_status), &_actuator_status);
	}

}


void UavcanServoController::arm_all_servos(bool arm)
{
	if (arm) {
		_armed_mask = -1;

	} else {
		_armed_mask = 0;
	}
}

void UavcanServoController::arm_single_servo(int num, bool arm)
{
	if (arm) {
		_armed_mask = MOTOR_BIT(num);

	} else {
		_armed_mask = 0;
	}
}

void UavcanServoController::update_params()
{
	char str[20];
	float default_disarmed;
	float default_max;
	float default_min;

	// Find default values for disabled, min and max
	(void)sprintf(str, "UAVCAN_DISARMED");

	if (!(OK == param_get(param_find(str), &default_disarmed))) {
		default_disarmed = DEFAULT_DISARMED;
	}

	(void)sprintf(str, "UAVCAN_MAX");

	if (!(OK == param_get(param_find(str), &default_max))) {
		default_max = DEFAULT_MAX;
	}

	(void)sprintf(str, "UAVCAN_MIN");

	if ((OK == param_get(param_find(str), &default_min))) {
		default_min = DEFAULT_MIN;
	}

	// Rate limit
	(void)sprintf(str, "UAVCAN_RATE");

	if (!(OK == param_get(param_find(str), &_rate_limit))) {
		_rate_limit = DEFAULT_RATE_LIMIT;
	}

	for (uint8_t i = 0; i < 8; i++) {
		(void)sprintf(str, "UAVCAN_TRIM%u", i + 1);

		if (!(OK == param_get(param_find(str), &_trim[i]))) {
			_trim[i] = DEFAULT_TRIM;
		}

		(void)sprintf(str, "UAVCAN_SCALE%u", i + 1);

		if (!(OK == param_get(param_find(str), &_scale[i]))) {
			_scale[i] = DEFAULT_SCALE;
		}

		(void)sprintf(str, "UAVCAN_DIS%u", i + 1);

		if (!(OK == param_get(param_find(str), &_disarmed[i]))) {
			_disarmed[i] = default_disarmed;
		}

		(void)sprintf(str, "UAVCAN_MAX%u", i + 1);

		if (!(OK == param_get(param_find(str), &_max[i]))) {
			_max[i] = default_max;
		}

		(void)sprintf(str, "UAVCAN_MIN%u", i + 1);

		if (!(OK == param_get(param_find(str), &_min[i]))) {
			_min[i] = default_min;
		}
	}
}
