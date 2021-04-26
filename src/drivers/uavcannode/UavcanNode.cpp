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

#include "UavcanNode.hpp"

#include "boot_app_shared.h"
#include "boot_alt_app_shared.h"

#include <drivers/drv_watchdog.h>
#include <lib/ecl/geo/geo.h>
#include <lib/version/version.h>

#include "Publishers/BatteryInfo.hpp"
#include "Publishers/FlowMeasurement.hpp"
#include "Publishers/GnssFix2.hpp"
#include "Publishers/MagneticFieldStrength2.hpp"
#include "Publishers/RangeSensorMeasurement.hpp"
#include "Publishers/RawAirData.hpp"
#include "Publishers/SafetyButton.hpp"
#include "Publishers/StaticPressure.hpp"
#include "Publishers/StaticTemperature.hpp"

#include "Subscribers/BeepCommand.hpp"
#include "Subscribers/LightsCommand.hpp"

using namespace time_literals;

namespace uavcannode
{

/**
 * @file uavcan_main.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *         David Sidrane <david_s5@nscdg.com>
 */

/*
 * This is the AppImageDescriptor used
 * by the make_can_boot_descriptor.py tool to set
 * the application image's descriptor so that the
 * uavcan bootloader has the ability to validate the
 * image crc, size etc of this application
*/
boot_app_shared_section app_descriptor_t AppDescriptor = {
	.signature = APP_DESCRIPTOR_SIGNATURE,
	{
		0,
	},
	.image_size = 0,
	.git_hash  = 0,
	.major_version = APP_VERSION_MAJOR,
	.minor_version = APP_VERSION_MINOR,
	.board_id = HW_VERSION_MAJOR << 8 | HW_VERSION_MINOR,
	.reserved = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};

UavcanNode *UavcanNode::_instance;

UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_node(can_driver, system_clock, _pool_allocator),
	_time_sync_slave(_node),
	_fw_update_listner(_node),
	_param_server(_node),
	_reset_timer(_node)
{
	int res = pthread_mutex_init(&_node_mutex, nullptr);

	if (res < 0) {
		std::abort();
	}
}

UavcanNode::~UavcanNode()
{
	if (_instance) {
		/* tell the task we want it to go away */
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 10;

		do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			if (--i == 0) {
				break;
			}

		} while (_instance);
	}

	_publisher_list.clear();
	_subscriber_list.clear();

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int UavcanNode::getHardwareVersion(uavcan::protocol::HardwareVersion &hwver)
{
	if (UavcanNode::instance()) {

		hwver.major = HW_VERSION_MAJOR;
		hwver.minor = HW_VERSION_MINOR;

		mfguid_t mfgid = {};
		board_get_mfguid(mfgid);
		uavcan::copy(mfgid, mfgid + sizeof(mfgid), hwver.unique_id.begin());
		return 0;
	}

	return -1;
}

int UavcanNode::start(uavcan::NodeID node_id, uint32_t bitrate)
{

	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	/*
	 * CAN driver init
	 * Note that we instantiate and initialize CanInitHelper only once, because the STM32's bxCAN driver
	 * shipped with libuavcan does not support deinitialization.
	 */
	static CanInitHelper *can = nullptr;

	if (can == nullptr) {

		can = new CanInitHelper();

		if (can == nullptr) {                    // We don't have exceptions so bad_alloc cannot be thrown
			PX4_ERR("Out of memory");
			return -1;
		}

		const int can_init_res = can->init(bitrate);

		if (can_init_res < 0) {
			PX4_ERR("CAN driver init failed %i", can_init_res);
			return can_init_res;
		}
	}

	// Node init
	_instance = new UavcanNode(can->driver, UAVCAN_DRIVER::SystemClock::instance());

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	const int node_init_res = _instance->init(node_id, can->driver.updateEvent());

	if (node_init_res < 0) {
		delete _instance;
		_instance = nullptr;
		PX4_ERR("Node init failed %i", node_init_res);
		return node_init_res;
	}

	// Keep the bit rate for reboots on BenginFirmware updates
	_instance->active_bitrate = bitrate;

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

	return PX4_OK;
}

void UavcanNode::fill_node_info()
{
	// software version
	uavcan::protocol::SoftwareVersion swver;

	// Extracting the first 8 hex digits of the git hash and converting them to int
	char fw_git_short[9] = {};
	std::memmove(fw_git_short, px4_firmware_version_string(), 8);
	char *end = nullptr;
	swver.vcs_commit = std::strtol(fw_git_short, &end, 16);
	swver.optional_field_flags |= swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
	swver.major = AppDescriptor.major_version;
	swver.minor = AppDescriptor.minor_version;
	swver.image_crc = AppDescriptor.image_crc;

	_node.setSoftwareVersion(swver);

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;
	getHardwareVersion(hwver);
	_node.setHardwareVersion(hwver);
}

void UavcanNode::busevent_signal_trampoline()
{
	if (_instance) {
		// trigger the work queue (Note, this is called from IRQ context)
		_instance->ScheduleNow();
	}
}

static void cb_reboot(const uavcan::TimerEvent &)
{
	watchdog_pet();
	board_reset(0);
}

void UavcanNode::cb_beginfirmware_update(const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request>
		&req, uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &rsp)
{
	static bool inprogress = false;

	rsp.error = rsp.ERROR_UNKNOWN;

	if (req.image_file_remote_path.path.size()) {
		rsp.error = rsp.ERROR_IN_PROGRESS;

		if (!inprogress) {
			inprogress = true;
#if defined(SUPPORT_ALT_CAN_BOOTLOADER)

			if (!board_booted_by_px4()) {
				bootloader_alt_app_shared_t shared_alt{0};
				shared_alt.fw_server_node_id = req.source_node_id;
				shared_alt.node_id = _node.getNodeID().get();
				strncat((char *)shared_alt.path, (const char *)req.image_file_remote_path.path.c_str(), sizeof(shared_alt.path) - 1);
				bootloader_alt_app_shared_write(&shared_alt);
				board_configure_reset(BOARD_RESET_MODE_CAN_BL, shared_alt.node_id);
			}

#endif

			bootloader_app_shared_t shared;
			shared.bus_speed = active_bitrate;
			shared.node_id = _node.getNodeID().get();
			bootloader_app_shared_write(&shared, App);
			//rgb_led(255, 128, 0, 5);
			_reset_timer.setCallback(cb_reboot);
			_reset_timer.startOneShotWithDelay(uavcan::MonotonicDuration::fromMSec(1000));
			rsp.error = rsp.ERROR_OK;
		}
	}
}

int UavcanNode::init(uavcan::NodeID node_id, UAVCAN_DRIVER::BusEvent &bus_events)
{
	_node.setName(HW_UAVCAN_NAME);

	// Was the node_id supplied by the bootloader?

	if (node_id != 0) {
		_node.setNodeID(node_id);
	}

	fill_node_info();

	if (_fw_update_listner.start(BeginFirmwareUpdateCallBack(this, &UavcanNode::cb_beginfirmware_update)) < 0) {
		PX4_ERR("firmware update listener start failed");
		return PX4_ERROR;
	}

	// TODO: make runtime (and build time?) configurable
	_publisher_list.add(new BatteryInfo(this, _node));
	_publisher_list.add(new FlowMeasurement(this, _node));
	_publisher_list.add(new GnssFix2(this, _node));
	_publisher_list.add(new MagneticFieldStrength2(this, _node));
	_publisher_list.add(new RangeSensorMeasurement(this, _node));
	_publisher_list.add(new RawAirData(this, _node));
	_publisher_list.add(new SafetyButton(this, _node));
	_publisher_list.add(new StaticPressure(this, _node));
	_publisher_list.add(new StaticTemperature(this, _node));

	_subscriber_list.add(new BeepCommand(_node));
	_subscriber_list.add(new LightsCommand(_node));

	for (auto &subscriber : _subscriber_list) {
		subscriber->init();
	}

	bus_events.registerSignalCallback(UavcanNode::busevent_signal_trampoline);

	int rv = _node.start();

	if (rv < 0) {
		return rv;
	}

	// If the node_id was not supplied by the bootloader do Dynamic Node ID allocation

	if (node_id == 0) {

		uavcan::DynamicNodeIDClient client(_node);

		int client_start_res = client.start(_node.getHardwareVersion().unique_id,    // USING THE SAME UNIQUE ID AS ABOVE
						    node_id);

		if (client_start_res < 0) {
			PX4_ERR("Failed to start the dynamic node ID client");
			return client_start_res;
		}

		watchdog_pet(); // If allocation takes too long reboot

		/*
		 * Waiting for the client to obtain a node ID.
		 * This may take a few seconds.
		 */

		while (!client.isAllocationComplete()) {
			const int res = _node.spin(uavcan::MonotonicDuration::fromMSec(200));    // Spin duration doesn't matter

			if (res < 0) {
				PX4_ERR("Transient failure: %d", res);
			}
		}

		_node.setNodeID(client.getAllocatedNodeID());
	}

	return rv;
}

// Restart handler
class RestartRequestHandler: public uavcan::IRestartRequestHandler
{
	bool handleRestartRequest(uavcan::NodeID request_source) override
	{
		PX4_INFO("UAVCAN: Restarting by request from %i\n", int(request_source.get()));
		usleep(20 * 1000 * 1000);
		board_reset(0);
		return true; // Will never be executed BTW
	}
} restart_request_handler;

void UavcanNode::Run()
{
	static  hrt_abstime up_time{0};
	pthread_mutex_lock(&_node_mutex);

	// Bootloader started it.

	watchdog_pet();

	if (!_initialized) {
		up_time = hrt_absolute_time();
		get_node().setRestartRequestHandler(&restart_request_handler);
		_param_server.start(&_param_manager);

		// Set up the time synchronization
		const int slave_init_res = _time_sync_slave.start();

		if (slave_init_res < 0) {
			PX4_ERR("Failed to start time_sync_slave");
			_task_should_exit.store(true);
		}

		_node.setModeOperational();

		_initialized = true;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
	}

	_node.spinOnce();

	for (auto &publisher : _publisher_list) {
		publisher->BroadcastAnyUpdates();
	}

	_node.spinOnce();

	// This is done only once to signify the node has run 30 seconds

	if (up_time && hrt_elapsed_time(&up_time) > 30_s) {
		up_time = 0;
		board_configure_reset(BOARD_RESET_MODE_RTC_BOOT_FWOK, 0);
	}

	perf_end(_cycle_perf);

	pthread_mutex_unlock(&_node_mutex);

	if (_task_should_exit.load()) {
		ScheduleClear();
		_instance = nullptr;
	}
}

void UavcanNode::PrintInfo()
{
	pthread_mutex_lock(&_node_mutex);

	// Memory status
	printf("Pool allocator status:\n");
	printf("\tCapacity hard/soft: %u/%u blocks\n",
	       _pool_allocator.getBlockCapacityHardLimit(), _pool_allocator.getBlockCapacity());
	printf("\tReserved:  %u blocks\n", _pool_allocator.getNumReservedBlocks());
	printf("\tAllocated: %u blocks\n", _pool_allocator.getNumAllocatedBlocks());

	printf("\n");

	// UAVCAN node perfcounters
	printf("UAVCAN node status:\n");
	printf("\tInternal failures: %llu\n", _node.getInternalFailureCount());
	printf("\tTransfer errors:   %llu\n", _node.getDispatcher().getTransferPerfCounter().getErrorCount());
	printf("\tRX transfers:      %llu\n", _node.getDispatcher().getTransferPerfCounter().getRxTransferCount());
	printf("\tTX transfers:      %llu\n", _node.getDispatcher().getTransferPerfCounter().getTxTransferCount());

	printf("\n");

	// CAN driver status
	for (unsigned i = 0; i < _node.getDispatcher().getCanIOManager().getCanDriver().getNumIfaces(); i++) {
		printf("CAN%u status:\n", unsigned(i + 1));

		auto iface = _node.getDispatcher().getCanIOManager().getCanDriver().getIface(i);
		printf("\tHW errors: %llu\n", iface->getErrorCount());

		auto iface_perf_cnt = _node.getDispatcher().getCanIOManager().getIfacePerfCounters(i);
		printf("\tIO errors: %llu\n", iface_perf_cnt.errors);
		printf("\tRX frames: %llu\n", iface_perf_cnt.frames_rx);
		printf("\tTX frames: %llu\n", iface_perf_cnt.frames_tx);
	}

	printf("\n");
	printf("Publishers:\n");

	for (const auto &publisher : _publisher_list) {
		publisher->PrintInfo();
	}

	printf("\n");
	printf("Subscribers:\n");

	for (const auto &subscriber : _subscriber_list) {
		subscriber->PrintInfo();
	}

	printf("\n");

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	pthread_mutex_unlock(&_node_mutex);
}

void UavcanNode::shrink()
{
	(void)pthread_mutex_lock(&_node_mutex);
	_pool_allocator.shrink();
	(void)pthread_mutex_unlock(&_node_mutex);
}

} // namespace uavcannode

static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tuavcannode {start|status|stop|arm|disarm}");
}

extern "C" int uavcannode_start(int argc, char *argv[])
{
	//board_app_initialize(nullptr);

	// Sarted byt the bootloader, we must pet it
	watchdog_pet();

	// CAN bitrate
	int32_t bitrate = 0;

	// Node ID
	int32_t node_id = 0;

	// Did the bootloader auto baud and get a node ID Allocated
	bootloader_app_shared_t shared;
	int valid = bootloader_app_shared_read(&shared, BootLoader);

	if (valid == 0) {

		bitrate = shared.bus_speed;
		node_id = shared.node_id;

		// Invalidate to prevent deja vu
		bootloader_app_shared_invalidate();

	} else {
		// Node ID
#if defined(SUPPORT_ALT_CAN_BOOTLOADER)
		if (!board_booted_by_px4()) {
			node_id = 0;
			bitrate = 1000000;

		} else
#endif
		{
			(void)param_get(param_find("CANNODE_NODE_ID"), &node_id);
			(void)param_get(param_find("CANNODE_BITRATE"), &bitrate);
		}
	}

	if (
#if defined(SUPPORT_ALT_CAN_BOOTLOADER)
		board_booted_by_px4() &&
#endif
		(node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast())) {
		PX4_ERR("Invalid Node ID %i", node_id);
		return 1;
	}

	// Start
	PX4_INFO("Node ID %u, bitrate %u", node_id, bitrate);
	int rv = uavcannode::UavcanNode::start(node_id, bitrate);

	return rv;
}

extern "C" __EXPORT int uavcannode_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!std::strcmp(argv[1], "start")) {
		if (uavcannode::UavcanNode::instance()) {
			PX4_ERR("already started");
			return 1;
		}

		return uavcannode_start(argc, argv);
	}

	/* commands below require the app to be started */
	uavcannode::UavcanNode *const inst = uavcannode::UavcanNode::instance();

	if (!inst) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		if (inst) {
			inst->PrintInfo();
		}

		return 0;
	}

	if (!std::strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}
