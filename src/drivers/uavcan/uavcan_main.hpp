/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file uavcan_main.hpp
 *
 * Defines basic functinality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *		 Andreas Jochum <Andreas@NicaDrone.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "beep.hpp"
#include "rgbled.hpp"
#include "safety_state.hpp"
#include "uavcan_driver.hpp"
#include "uavcan_servers.hpp"
#include "allocator.hpp"
#include "actuators/esc.hpp"
#include "actuators/hardpoint.hpp"
#include "sensors/sensor_bridge.hpp"

#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <uavcan/protocol/global_time_sync_master.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
#include <uavcan/protocol/RestartNode.hpp>

#include <lib/drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class UavcanNode;

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
	UavcanMixingInterface(pthread_mutex_t &node_mutex, UavcanEscController &esc_controller)
		: OutputModuleInterface(MODULE_NAME "-actuators", px4::wq_configurations::uavcan),
		  _node_mutex(node_mutex),
		  _esc_controller(esc_controller) {}

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	void mixerChanged() override;

	MixingOutput &mixingOutput() { return _mixing_output; }

protected:
	void Run() override;
private:
	friend class UavcanNode;
	pthread_mutex_t &_node_mutex;
	UavcanEscController &_esc_controller;
	MixingOutput _mixing_output{MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
};

/**
 * A UAVCAN node.
 */
class UavcanNode : public cdev::CDev, public px4::ScheduledWorkItem, public ModuleParams
{
	static constexpr unsigned MaxBitRatePerSec	= 1000000;
	static constexpr unsigned bitPerFrame		= 148;
	static constexpr unsigned FramePerSecond	= MaxBitRatePerSec / bitPerFrame;
	static constexpr unsigned FramePerMSecond	= ((FramePerSecond / 1000) + 1);

	static constexpr unsigned ScheduleIntervalMs		= 3;


	/*
	 * This memory is reserved for uavcan to use for queuing CAN frames.
	 * At 1Mbit there is approximately one CAN frame every 145 uS.
	 * The number of buffers sets how long you can go without calling
	 * node_spin_xxxx. Since our task is the only one running and the
	 * driver will light the callback when there is a CAN frame we can nun with
	 * a minimum number of buffers to conserver memory. Each buffer is
	 * 32 bytes. So 5 buffers costs 160 bytes and gives us a poll rate
	 * of ~1 mS
	 *  1000000/200
	 */

	static constexpr unsigned RxQueueLenPerIface	= FramePerMSecond * ScheduleIntervalMs; // At

public:
	typedef UAVCAN_DRIVER::CanInitHelper<RxQueueLenPerIface> CanInitHelper;
	enum eServerAction : int {None, Start, Stop, CheckFW, Busy};

	UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);

	virtual		~UavcanNode();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	static int	start(uavcan::NodeID node_id, uint32_t bitrate);

	uavcan::Node<>	&get_node() { return _node; }

	int		teardown();

	void		print_info();

	void		shrink();

	void		hardpoint_controller_set(uint8_t hardpoint_id, uint16_t command);

	static UavcanNode	*instance() { return _instance; }
	static int		 getHardwareVersion(uavcan::protocol::HardwareVersion &hwver);
	int			 fw_server(eServerAction action);
	void			 attachITxQueueInjector(ITxQueueInjector *injector) {_tx_injector = injector;}
	int			 list_params(int remote_node_id);
	int			 save_params(int remote_node_id);
	int			 set_param(int remote_node_id, const char *name, char *value);
	int			 get_param(int remote_node_id, const char *name);
	int			 reset_node(int remote_node_id);

	static void busevent_signal_trampoline();

protected:
	void Run() override;
private:
	void		fill_node_info();
	int		init(uavcan::NodeID node_id, UAVCAN_DRIVER::BusEvent &bus_events);
	void		node_spin_once();

	int		start_fw_server();
	int		stop_fw_server();
	int		request_fw_check();

	int		print_params(uavcan::protocol::param::GetSet::Response &resp);
	int		get_set_param(int nodeid, const char *name, uavcan::protocol::param::GetSet::Request &req);
	void 		update_params();

	void		set_setget_response(uavcan::protocol::param::GetSet::Response *resp) { _setget_response = resp; }
	void		free_setget_response(void) { _setget_response = nullptr; }

	void enable_idle_throttle_when_armed(bool value);

	px4::atomic_bool	_task_should_exit{false};	///< flag to indicate to tear down the CAN driver
	px4::atomic<int>	_fw_server_action{None};
	int			 _fw_server_status{-1};

	bool			_is_armed{false};		///< the arming status of the actuators on the bus

	unsigned		_output_count{0};		///< number of actuators currently available

	static UavcanNode	*_instance;			///< singleton pointer

	uavcan_node::Allocator	 _pool_allocator;

	uavcan::Node<>			_node;				///< library instance
	pthread_mutex_t			_node_mutex;
	px4_sem_t			_server_command_sem;
	UavcanEscController		_esc_controller;
	UavcanMixingInterface 		_mixing_interface{_node_mutex, _esc_controller};
	UavcanHardpointController	_hardpoint_controller;
	UavcanBeep			_beep_controller;
	UavcanSafetyState         	_safety_state_controller;
	UavcanRGBController             _rgbled_controller;
	uavcan::GlobalTimeSyncMaster	_time_sync_master;
	uavcan::GlobalTimeSyncSlave	_time_sync_slave;
	uavcan::NodeStatusMonitor	_node_status_monitor;

	List<IUavcanSensorBridge *>	_sensor_bridges;		///< List of active sensor bridges

	ITxQueueInjector		*_tx_injector{nullptr};

	bool 				_idle_throttle_when_armed{false};
	int32_t 			_idle_throttle_when_armed_param{0};

	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};

	perf_counter_t			_cycle_perf;
	perf_counter_t			_interval_perf;

	void handle_time_sync(const uavcan::TimerEvent &);

	typedef uavcan::MethodBinder<UavcanNode *, void (UavcanNode::*)(const uavcan::TimerEvent &)> TimerCallback;
	uavcan::TimerEventForwarder<TimerCallback> _master_timer;

	bool				_callback_success{false};

	uavcan::protocol::param::GetSet::Response *_setget_response{nullptr};

	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &)> GetSetCallback;
	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &)> ExecuteOpcodeCallback;
	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &)> RestartNodeCallback;

	void cb_setget(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);
	void cb_opcode(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result);
	void cb_restart(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &result);

};
