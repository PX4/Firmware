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

#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength2.hpp>
#include <uavcan/equipment/air_data/RawAirData.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>
#include <uavcan/equipment/gnss/Fix2.hpp>
#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <uavcan/equipment/range_sensor/Measurement.hpp>


#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_gps_position.h>

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

	void		print_info();

	void		shrink();

	static UavcanNode	*instance() { return _instance; }
	static int		 getHardwareVersion(uavcan::protocol::HardwareVersion &hwver);

	static void busevent_signal_trampoline();

	/* The bit rate that can be passed back to the bootloader */
	int32_t active_bitrate{0};

protected:
	void Run() override;
private:
	void		fill_node_info();
	int		init(uavcan::NodeID node_id, UAVCAN_DRIVER::BusEvent &bus_events);

	px4::atomic_bool	_task_should_exit{false};	///< flag to indicate to tear down the CAN driver

	bool		_initialized{false};		///< number of actuators currently available

	static UavcanNode	*_instance;			///< singleton pointer

	uavcan_node::Allocator	 _pool_allocator;

	uavcan::Node<>			_node;				///< library instance
	pthread_mutex_t			_node_mutex;

	uavcan::GlobalTimeSyncSlave _time_sync_slave;

	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request> &,
				     uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &)>
		BeginFirmwareUpdateCallBack;

	uavcan::ServiceServer<BeginFirmwareUpdate, BeginFirmwareUpdateCallBack> _fw_update_listner;
	void cb_beginfirmware_update(const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request> &req,
				     uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &rsp);


	uavcan::Publisher<uavcan::equipment::ahrs::MagneticFieldStrength2> _ahrs_magnetic_field_strength2_publisher;
	uavcan::Publisher<uavcan::equipment::gnss::Fix2> _gnss_fix2_publisher;
	uavcan::Publisher<uavcan::equipment::power::BatteryInfo> _power_battery_info_publisher;
	uavcan::Publisher<uavcan::equipment::air_data::StaticPressure> _air_data_static_pressure_publisher;
	uavcan::Publisher<uavcan::equipment::air_data::StaticTemperature> _air_data_static_temperature_publisher;
	uavcan::Publisher<uavcan::equipment::air_data::RawAirData> _raw_air_data_publisher;
	uavcan::Publisher<uavcan::equipment::range_sensor::Measurement> _range_sensor_measurement;

	hrt_abstime _last_static_temperature_publish{0};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::SubscriptionCallbackWorkItem _battery_status_sub{this, ORB_ID(battery_status)};
	uORB::SubscriptionCallbackWorkItem _diff_pressure_sub{this, ORB_ID(differential_pressure)};
	uORB::SubscriptionCallbackWorkItem _distance_sensor_sub[ORB_MULTI_MAX_INSTANCES] {
		{this, ORB_ID(distance_sensor), 0},
		{this, ORB_ID(distance_sensor), 1},
		{this, ORB_ID(distance_sensor), 2},
		{this, ORB_ID(distance_sensor), 3},
	};
	uORB::SubscriptionCallbackWorkItem _sensor_baro_sub{this, ORB_ID(sensor_baro)};
	uORB::SubscriptionCallbackWorkItem _sensor_mag_sub{this, ORB_ID(sensor_mag)};
	uORB::SubscriptionCallbackWorkItem _vehicle_gps_position_sub{this, ORB_ID(vehicle_gps_position)};

	perf_counter_t _cycle_perf;
	perf_counter_t _interval_perf;

public:

	/* A timer used to reboot after the response is sent */
	uavcan::TimerEventForwarder<void (*)(const uavcan::TimerEvent &)> _reset_timer;
};
