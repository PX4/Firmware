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

#include <px4_config.h>

#include "uavcan_driver.hpp"
#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <uavcan/protocol/global_time_sync_master.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
#include <uavcan/protocol/RestartNode.hpp>

#include <drivers/device/device.h>
#include <perf/perf_counter.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/actuator_direct.h>

#include "actuators/esc.hpp"
#include "actuators/hardpoint.hpp"
#include "actuators/servo.hpp"
#include "sensors/sensor_bridge.hpp"

#include "uavcan_servers.hpp"
#include "allocator.hpp"

#define NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN	4

// we add two to allow for actuator_direct and busevent
#define UAVCAN_NUM_POLL_FDS (NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN+2)

/**
 * A UAVCAN node.
 */
class UavcanNode : public cdev::CDev
{
	static constexpr unsigned MaxBitRatePerSec	= 1000000;
	static constexpr unsigned bitPerFrame		= 148;
	static constexpr unsigned FramePerSecond	= MaxBitRatePerSec / bitPerFrame;
	static constexpr unsigned FramePerMSecond	= ((FramePerSecond / 1000) + 1);

	static constexpr unsigned PollTimeoutMs		= 3;


	/*
	 * This memory is reserved for uavcan to use for queuing CAN frames.
	 * At 1Mbit there is approximately one CAN frame every 145 uS.
	 * The number of buffers sets how long you can go without calling
	 * node_spin_xxxx. Since our task is the only one running and the
	 * driver will light the fd when there is a CAN frame we can nun with
	 * a minimum number of buffers to conserver memory. Each buffer is
	 * 32 bytes. So 5 buffers costs 160 bytes and gives us a poll rate
	 * of ~1 mS
	 *  1000000/200
	 */

	static constexpr unsigned RxQueueLenPerIface	= FramePerMSecond * PollTimeoutMs; // At
	static constexpr unsigned StackSize		= 2400;

public:
	typedef UAVCAN_DRIVER::CanInitHelper<RxQueueLenPerIface> CanInitHelper;
	enum eServerAction {None, Start, Stop, CheckFW, Busy};

	UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);

	virtual		~UavcanNode();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	static int	start(uavcan::NodeID node_id, uint32_t bitrate);

	uavcan::Node<>	&get_node() { return _node; }

	// TODO: move the actuator mixing stuff into the ESC controller class
	static int	control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);

	void		subscribe();

	int		teardown();

	int		arm_actuators(bool arm);

	void		print_info();

	void		shrink();

	void		hardpoint_controller_set(uint8_t hardpoint_id, uint16_t command);

	static UavcanNode *instance() { return _instance; }
	static int		 getHardwareVersion(uavcan::protocol::HardwareVersion &hwver);
	int			 fw_server(eServerAction action);
	void			attachITxQueueInjector(ITxQueueInjector *injector) {_tx_injector = injector;}
	int			 list_params(int remote_node_id);
	int			 save_params(int remote_node_id);
	int			 set_param(int remote_node_id, const char *name, char *value);
	int			 get_param(int remote_node_id, const char *name);
	int			 reset_node(int remote_node_id);

private:
	void		fill_node_info();
	int		init(uavcan::NodeID node_id);
	void		node_spin_once();
	int		run();
	int		add_poll_fd(int fd);			///< add a fd to poll list, returning index into _poll_fds[]
	int		start_fw_server();
	int		stop_fw_server();
	int		request_fw_check();
	int		print_params(uavcan::protocol::param::GetSet::Response &resp);
	int		get_set_param(int nodeid, const char *name, uavcan::protocol::param::GetSet::Request &req);
	void 		update_params();
	void		set_setget_response(uavcan::protocol::param::GetSet::Response *resp)
	{
		_setget_response = resp;
	}
	void		free_setget_response(void)
	{
		_setget_response = nullptr;
	}

	int			_task = -1;			///< handle to the OS task
	bool			_task_should_exit = false;	///< flag to indicate to tear down the CAN driver
	volatile eServerAction	_fw_server_action;
	int			 _fw_server_status;
	int			_armed_sub = -1;		///< uORB subscription of the arming status
	actuator_armed_s	_armed = {};			///< the arming request of the system
	bool			_is_armed = false;		///< the arming status of the actuators on the bus

	int			_test_motor_sub = -1;   ///< uORB subscription of the test_motor status
	test_motor_s		_test_motor = {};
	bool			_test_in_progress = false;

	unsigned		_output_count = 0;		///< number of actuators currently available

	static UavcanNode	*_instance;			///< singleton pointer

	uavcan_node::Allocator	 _pool_allocator;

	uavcan::Node<>			_node;				///< library instance
	pthread_mutex_t			_node_mutex;
	px4_sem_t			_server_command_sem;
	UavcanEscController		_esc_controller;
	UavcanHardpointController	_hardpoint_controller;
	UavcanServoController           _servo_controller;
	uavcan::GlobalTimeSyncMaster	_time_sync_master;
	uavcan::GlobalTimeSyncSlave	_time_sync_slave;
	uavcan::NodeStatusMonitor	_node_status_monitor;

	List<IUavcanSensorBridge *>	_sensor_bridges;		///< List of active sensor bridges

	MixerGroup			*_mixers = nullptr;
	ITxQueueInjector		*_tx_injector;
	uint32_t			_groups_required = 0;
	uint32_t			_groups_subscribed = 0;
	int				_control_subs[NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN];
	actuator_controls_s		_controls[NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN] = {};
	orb_id_t			_control_topics[NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN] = {};
	pollfd				_poll_fds[UAVCAN_NUM_POLL_FDS] = {};
	unsigned			_poll_fds_num = 0;
	int32_t 			_idle_throttle_when_armed = 0;

	int				_actuator_direct_sub = -1;   ///< uORB subscription of the actuator_direct topic
	uint8_t				_actuator_direct_poll_fd_num = 0;
	actuator_direct_s		_actuator_direct = {};

	actuator_outputs_s		_outputs = {};

	perf_counter_t			_perf_control_latency;

	Mixer::Airmode 			_airmode = Mixer::Airmode::disabled;
	float 					_thr_mdl_factor = 0.0f;

	// index into _poll_fds for each _control_subs handle
	uint8_t				_poll_ids[NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN];

	void handle_time_sync(const uavcan::TimerEvent &);

	typedef uavcan::MethodBinder<UavcanNode *, void (UavcanNode::*)(const uavcan::TimerEvent &)> TimerCallback;
	uavcan::TimerEventForwarder<TimerCallback> _master_timer;

	bool _callback_success;
	uavcan::protocol::param::GetSet::Response *_setget_response;
	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &)> GetSetCallback;
	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &)> ExecuteOpcodeCallback;
	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &)> RestartNodeCallback;
	void cb_setget(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);
	void cb_opcode(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result);
	void cb_restart(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &result);

	void get_mixer_mapping();
	int _esc_channel_map {0};
	int _servo_channel_map {0};
};
