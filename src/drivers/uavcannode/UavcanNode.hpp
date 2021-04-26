/****************************************************************************
 *
 *   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
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
 * @file uavcan_main.hpp
 *
 * Defines basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *         David Sidrane <david_s5@nscdg.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "uavcan_driver.hpp"
#include "allocator.hpp"
#include "UavcanNodeParamManager.hpp"

#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>

#include <containers/IntrusiveSortedList.hpp>

#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include "Publishers/UavcanPublisherBase.hpp"
#include "Subscribers/UavcanSubscriberBase.hpp"

using namespace time_literals;

namespace uavcannode
{

/**
 * A UAVCAN node.
 */
class UavcanNode : public px4::ScheduledWorkItem
{
	/*
	 * This memory is reserved for uavcan to use as over flow for message
	 * Coming from multiple sources that my not be considered at development
	 * time.
	 *
	 * The call to getNumFreeBlocks will tell how many blocks there are
	 * free -and multiply it times getBlockSize to get the number of bytes
	 *
	 */
	static constexpr unsigned MemPoolSize = 2048;

	static constexpr unsigned MaxBitRatePerSec = 1000000;
	static constexpr unsigned bitPerFrame      = 148;
	static constexpr unsigned FramePerSecond   = MaxBitRatePerSec / bitPerFrame;
	static constexpr unsigned FramePerMSecond  = ((FramePerSecond / 1000) + 1);

	static constexpr unsigned ScheduleIntervalMs = 10;

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

	static constexpr unsigned RxQueueLenPerIface = FramePerMSecond * ScheduleIntervalMs;

public:
	typedef UAVCAN_DRIVER::CanInitHelper<RxQueueLenPerIface> CanInitHelper;
	typedef uavcan::protocol::file::BeginFirmwareUpdate BeginFirmwareUpdate;

	UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);

	virtual		~UavcanNode();

	static int	start(uavcan::NodeID node_id, uint32_t bitrate);

	uavcan::Node<>	&get_node() { return _node; }

	void		PrintInfo();

	void		shrink();

	static UavcanNode	*instance() { return _instance; }
	static int		 getHardwareVersion(uavcan::protocol::HardwareVersion &hwver);

	static void busevent_signal_trampoline();

	/* The bit rate that can be passed back to the bootloader */
	int32_t active_bitrate{0};

private:
	void Run() override;

	void fill_node_info();
	int init(uavcan::NodeID node_id, UAVCAN_DRIVER::BusEvent &bus_events);

	px4::atomic_bool	_task_should_exit{false};	///< flag to indicate to tear down the CAN driver

	bool		_initialized{false};		///< number of actuators currently available

	static UavcanNode	*_instance;			///< singleton pointer

	uavcan_node::Allocator	 _pool_allocator;

	uavcan::Node<>		_node;				///< library instance
	pthread_mutex_t		_node_mutex;

	uavcan::GlobalTimeSyncSlave _time_sync_slave;

	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request> &,
				     uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &)>
		BeginFirmwareUpdateCallBack;

	uavcan::ServiceServer<BeginFirmwareUpdate, BeginFirmwareUpdateCallBack> _fw_update_listner;
	void cb_beginfirmware_update(const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request> &req,
				     uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &rsp);

	IntrusiveSortedList<UavcanPublisherBase *> _publisher_list;
	IntrusiveSortedList<UavcanSubscriberBase *> _subscriber_list;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	UavcanNodeParamManager _param_manager;
	uavcan::ParamServer _param_server;

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

public:
	/* A timer used to reboot after the response is sent */
	uavcan::TimerEventForwarder<void (*)(const uavcan::TimerEvent &)> _reset_timer;
};
}; // namespace uavcannode
