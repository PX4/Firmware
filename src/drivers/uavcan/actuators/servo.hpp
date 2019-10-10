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
 * @file UavcanServoController.hpp
 *
 * UAVCAN <--> ORB bridge for servo motor deflection messages messages:
 *     TODO: figure out what these should be for the servo motors
 *     uavcan.equipment.esc.RawCommand
 *     uavcan.equipment.esc.RPMCommand
 *     uavcan.equipment.esc.Status
 *
 * @author Richard Kirby <rkirby@kspresearch.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <perf/perf_counter.h>
#include <uORB/topics/actuator_status.h>
#include <uORB/topics/actuator_outputs.h>

#define SERVO_BIT(x) (1<<(x))

class UavcanServoController
{
public:
	UavcanServoController(uavcan::INode &node);
	~UavcanServoController();

	int init();

	void update_outputs(uint8_t actuator_id, float position);


	void arm_all_servos(bool arm);
	void arm_single_servo(int num, bool arm);
	void update_params();

private:
	/**
	 * Servo status message reception will be reported via this callback.
	 */
	void servo_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status> &msg);

	/**
	 * Servo status will be published to ORB from this callback (fixed rate).
	 */
	void orb_pub_timer_cb(const uavcan::TimerEvent &event);


	static constexpr unsigned 	MAX_RATE_HZ = 1;			///< XXX make this configurable?
	///< TODO: change this to match servo update rate

	static constexpr unsigned 	ACTUATOR_STATUS_UPDATE_RATE_HZ = 10;    ///< TODO: figure out what this should be,
	///< I think it is 1Hz.

	static constexpr unsigned 	UAVCAN_COMMAND_TRANSFER_PRIORITY = 5;	///< 0..31, inclusive, 0 - highest, 31 - lowest
	static constexpr unsigned 	MAX_NUM_ACTUATORS = 8;			///< Aux mixer has 8 channels
	static constexpr float 		DEFAULT_DISARMED = 0.0f;		///< -1.0 - 1.0 can be set by parameters
	static constexpr float 		DEFAULT_MAX = 1.0f;			///< 0.0 - 1.0 can be set by parameters
	static constexpr float 		DEFAULT_MIN = -1.0f;			///< -1.0 - 0.0 can be set by parameters
	static constexpr unsigned 	DEFAULT_RATE_LIMIT = 50;		///< default matches 20mS PWM servos
	static constexpr float		DEFAULT_TRIM = 0.0;			///< -0.25 -0.25 can be set by parameters
	static constexpr float		DEFAULT_SCALE = 1.0;			///< 0.50 - 1.00 can be set by parameters

	typedef uavcan::MethodBinder<UavcanServoController *,
		void (UavcanServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>&)>
		StatusCbBinder;

	typedef uavcan::MethodBinder<UavcanServoController *, void (UavcanServoController::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	bool			_armed = false;
	bool			_run_at_idle_throttle_when_armed = false;
	actuator_status_s 	_actuator_status = {};
	actuator_outputs_s  	_actuator_outputs = {};
	orb_advert_t 		_actuator_outputs_pub = nullptr;
	orb_advert_t 		_actuator_status_pub = nullptr;

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_cmd_pub[8];   ///< rate limiting
	uavcan::INode								&_node;
	//uavcan::Publisher<uavcan::equipment::esc::RawCommand>			_uavcan_pub_raw_cmd;
	uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>		_uavcan_pub_array_cmd;
	uavcan::Subscriber<uavcan::equipment::actuator::Status, StatusCbBinder>	_uavcan_sub_status;
	uavcan::TimerEventForwarder<TimerCbBinder>				_orb_timer;

	/*
	 * servo states
	 */
	uint32_t 			_armed_mask = 0;
	uint8_t				_max_number_of_nonzero_outputs = 0;
	int				_t_param;							///< parameter update topic
	bool				_param_update_force;						///< force a parameter update

	float				_trim[8] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};		///< trims updated from parameters
	float				_scale[8] {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};		///< trims updated from parameters
	float				_disarmed[8] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};		///< disarmed output updated from parameters
	float				_max[8] {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};		///< max output updated from parameters
	float				_min[8] {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};	///< min output updated from parameters
	id_t				_rate_limit {DEFAULT_RATE_LIMIT};				///< rate limit updated from parameters

	/*
	 * Perf counters
	 */
	perf_counter_t _perfcnt_invalid_input = perf_alloc(PC_COUNT, "uavcan_esc_invalid_input");
	perf_counter_t _perfcnt_scaling_error = perf_alloc(PC_COUNT, "uavcan_esc_scaling_error");
};
