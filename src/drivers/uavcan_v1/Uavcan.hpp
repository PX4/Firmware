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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>

#include "o1heap/o1heap.h"

#include <canard.h>
#include <canard_dsdl.h>

#include "CanardInterface.hpp"

#include "Publishers/Publisher.hpp"
#include "Publishers/uORB/uorb_publisher.hpp"

#include "NodeManager.hpp"

#include "PublicationManager.hpp"
#include "SubscriptionManager.hpp"

#include "Actuators/EscClient.hpp" /// TODO: Add EscServer.hpp for node-side service

/**
 * UAVCAN mixing class.
 * It is separate from UavcanNode to have 2 WorkItems and therefore allowing independent scheduling
 * (I.e. UavcanMixingInterface runs upon actuator_control updates, whereas UavcanNode runs at
 * a fixed rate or upon bus updates).
 * Both work items are expected to run on the same work queue.
 */
class UavcanMixingInterface : public OutputModuleInterface
{
public:
	UavcanMixingInterface(pthread_mutex_t &node_mutex,
			      UavcanEscController &esc_controller) //, UavcanServoController &servo_controller)
		: OutputModuleInterface(MODULE_NAME "-actuators", px4::wq_configurations::uavcan),
		  _node_mutex(node_mutex),
		  _esc_controller(esc_controller)/*,
		  _servo_controller(servo_controller)*/ {}

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	void mixerChanged() override {};

	void printInfo() { _mixing_output.printStatus(); }

	MixingOutput &mixingOutput() { return _mixing_output; }

	const char *get_param_prefix() override { return "UCAN1_ACT"; }

protected:
	void Run() override;
private:
	friend class UavcanNode;
	pthread_mutex_t &_node_mutex;
	UavcanEscController &_esc_controller;
	// UavcanServoController &_servo_controller;
	MixingOutput _mixing_output{MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
};

class UavcanNode : public ModuleParams, public px4::ScheduledWorkItem
{
	/*
	* This memory is allocated for the 01Heap allocator used by
	* libcanard to store incoming/outcoming data
	* Current size of 8192 bytes is arbitrary, should be optimized further
	* when more nodes and messages are on the CAN bus
	*/
	static constexpr unsigned HeapSize = 8192;

	/*
	 * Base interval, has to be complemented with events from the CAN driver
	 * and uORB topics sending data, to decrease response time.
	 */
	static constexpr unsigned ScheduleIntervalMs = 10;

public:

	UavcanNode(CanardInterface *interface, uint32_t node_id);
	~UavcanNode() override;

	static int start(uint32_t node_id, uint32_t bitrate);

	void print_info();

	static UavcanNode *instance() { return _instance; }

	/* The bit rate that can be passed back to the bootloader */
	int32_t active_bitrate{0};

private:
	void init();
	void Run() override;
	void fill_node_info();

	void transmit();

	// Sends a heartbeat at 1s intervals
	void sendHeartbeat();

	void *_uavcan_heap{nullptr};

	CanardInterface *const _can_interface;

	CanardInstance _canard_instance;

	px4::atomic_bool _task_should_exit{false};	///< flag to indicate to tear down the CAN driver

	bool _initialized{false};		///< number of actuators currently available

	static UavcanNode *_instance;

	pthread_mutex_t _node_mutex;

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	// uavcan::node::Heartbeat_1_0
	uint8_t _uavcan_node_heartbeat_buffer[uavcan_node_Heartbeat_1_0_EXTENT_BYTES_];
	hrt_abstime _uavcan_node_heartbeat_last{0};
	CanardTransferID _uavcan_node_heartbeat_transfer_id{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UAVCAN_V1_ENABLE>) _param_uavcan_v1_enable,
		(ParamInt<px4::params::UAVCAN_V1_ID>) _param_uavcan_v1_id,
		(ParamInt<px4::params::UAVCAN_V1_BAUD>) _param_uavcan_v1_baud
	)

	UavcanParamManager _param_manager;

	NodeManager _node_manager {_canard_instance, _param_manager};

	PublicationManager _pub_manager {_canard_instance, _param_manager};
	SubscriptionManager _sub_manager {_canard_instance, _param_manager};

	/// TODO: Integrate with PublicationManager
	UavcanEscController _esc_controller {_canard_instance, _param_manager};

	UavcanMixingInterface _mixing_output {_node_mutex, _esc_controller};

};
