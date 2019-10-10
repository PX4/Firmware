/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @file uavcan_main.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author David Sidrane <david_s5@nscdg.com>
 * @author Andreas Jochum <Andreas@NicaDrone.com>
 *
 */

#include <px4_config.h>
#include <px4_tasks.h>

#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <lib/mixer/mixer.h>
#include <version/version.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>

#include "uavcan_module.hpp"
#include "uavcan_main.hpp"
#include <uavcan/util/templates.hpp>

#include <uavcan/protocol/param/ExecuteOpcode.hpp>

//todo:The Inclusion of file_server_backend is killing
// #include <sys/types.h> and leaving OK undefined
# define OK 0

/*
 * UavcanNode
 */
UavcanNode *UavcanNode::_instance;
UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
	CDev(UAVCAN_DEVICE_PATH),
	_node(can_driver, system_clock, _pool_allocator),
	_node_mutex(),
	_esc_controller(_node),
	_hardpoint_controller(_node),
	_servo_controller(_node),
	_time_sync_master(_node),
	_time_sync_slave(_node),
	_node_status_monitor(_node),
	_perf_control_latency(perf_alloc(PC_ELAPSED, "uavcan control latency")),
	_master_timer(_node),
	_setget_response(0)
{
	_task_should_exit = false;
	_fw_server_action = None;
	_fw_server_status = -1;
	_tx_injector = nullptr;
	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	for (int i = 0; i < NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN; ++i) {
		_control_subs[i] = -1;
	}

	int res = pthread_mutex_init(&_node_mutex, nullptr);

	if (res < 0) {
		std::abort();
	}

	res = px4_sem_init(&_server_command_sem, 0, 0);

	if (res < 0) {
		std::abort();
	}

	/* _server_command_sem use case is a signal */
	px4_sem_setprotocol(&_server_command_sem, SEM_PRIO_NONE);
}

UavcanNode::~UavcanNode()
{

	fw_server(Stop);

	if (_task != -1) {

		/* tell the task we want it to go away */
		_task_should_exit = true;

		unsigned i = 10;

		do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			/* if we have given up, kill it */
			if (--i == 0) {
				task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	(void)orb_unsubscribe(_armed_sub);
	(void)orb_unsubscribe(_test_motor_sub);
	(void)orb_unsubscribe(_actuator_direct_sub);

	// Removing the sensor bridges
	_sensor_bridges.clear();

	_instance = nullptr;

	pthread_mutex_destroy(&_node_mutex);
	px4_sem_destroy(&_server_command_sem);

	// Is it allowed to delete it like that?
	if (_mixers != nullptr) {
		delete _mixers;
	}

	perf_free(_perf_control_latency);
}

int UavcanNode::getHardwareVersion(uavcan::protocol::HardwareVersion &hwver)
{
	int rv = -1;

	if (UavcanNode::instance()) {
		if (!std::strncmp(px4_board_name(), "PX4_FMU_V2", 9)) {
			hwver.major = 2;

		} else {
			; // All other values of px4_board_name() resolve to zero
		}

		mfguid_t mfgid = {};
		board_get_mfguid(mfgid);
		uavcan::copy(mfgid, mfgid + sizeof(mfgid), hwver.unique_id.begin());
		rv = 0;
	}

	return rv;
}

int UavcanNode::print_params(uavcan::protocol::param::GetSet::Response &resp)
{
	if (resp.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
		return std::printf("name: %s %lld\n", resp.name.c_str(),
				   resp.value.to<uavcan::protocol::param::Value::Tag::integer_value>());

	} else if (resp.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
		return std::printf("name: %s %.4f\n", resp.name.c_str(),
				   static_cast<double>(resp.value.to<uavcan::protocol::param::Value::Tag::real_value>()));

	} else if (resp.value.is(uavcan::protocol::param::Value::Tag::boolean_value)) {
		return std::printf("name: %s %d\n", resp.name.c_str(),
				   resp.value.to<uavcan::protocol::param::Value::Tag::boolean_value>());

	} else if (resp.value.is(uavcan::protocol::param::Value::Tag::string_value)) {
		return std::printf("name: %s '%s'\n", resp.name.c_str(),
				   resp.value.to<uavcan::protocol::param::Value::Tag::string_value>().c_str());
	}

	return -1;
}

void UavcanNode::cb_opcode(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result)
{
	uavcan::protocol::param::ExecuteOpcode::Response resp;
	_callback_success = result.isSuccessful();
	resp = result.getResponse();
	_callback_success &= resp.ok;
}

int  UavcanNode::save_params(int remote_node_id)
{
	uavcan::protocol::param::ExecuteOpcode::Request opcode_req;
	opcode_req.opcode = opcode_req.OPCODE_SAVE;
	uavcan::ServiceClient<uavcan::protocol::param::ExecuteOpcode, ExecuteOpcodeCallback> client(_node);
	client.setCallback(ExecuteOpcodeCallback(this, &UavcanNode::cb_opcode));
	_callback_success = false;
	int call_res = client.call(remote_node_id, opcode_req);

	if (call_res >= 0) {
		while (client.hasPendingCalls()) {
			usleep(10000);
		}
	}

	if (!_callback_success) {
		std::printf("Failed to save parameters: %d\n", call_res);
		return -1;
	}

	return 0;
}

void UavcanNode::cb_restart(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &result)
{
	uavcan::protocol::RestartNode::Response resp;
	_callback_success = result.isSuccessful();
	resp = result.getResponse();
	_callback_success &= resp.ok;
}

int  UavcanNode::reset_node(int remote_node_id)
{
	uavcan::protocol::RestartNode::Request restart_req;
	restart_req.magic_number = restart_req.MAGIC_NUMBER;
	uavcan::ServiceClient<uavcan::protocol::RestartNode, RestartNodeCallback> client(_node);
	client.setCallback(RestartNodeCallback(this, &UavcanNode::cb_restart));
	_callback_success = false;
	int call_res = client.call(remote_node_id, restart_req);

	if (call_res >= 0) {
		while (client.hasPendingCalls()) {
			usleep(10000);
		}
	}

	if (!call_res) {
		std::printf("Failed to reset node: %d\n", remote_node_id);
		return -1;
	}

	return 0;
}

int  UavcanNode::list_params(int remote_node_id)
{
	int rv = 0;
	int index = 0;
	uavcan::protocol::param::GetSet::Response resp;
	set_setget_response(&resp);

	while (true) {
		uavcan::protocol::param::GetSet::Request req;
		req.index =  index++;
		_callback_success = false;
		int call_res = get_set_param(remote_node_id, nullptr, req);

		if (call_res < 0 || !_callback_success) {
			std::printf("Failed to get param: %d\n", call_res);
			rv = -1;
			break;
		}

		if (resp.name.empty()) { // Empty name means no such param, which means we're finished
			break;
		}

		print_params(resp);
	}

	free_setget_response();
	return rv;
}


void UavcanNode::cb_setget(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result)
{
	_callback_success = result.isSuccessful();
	*_setget_response = result.getResponse();
}

int  UavcanNode::get_set_param(int remote_node_id, const char *name, uavcan::protocol::param::GetSet::Request &req)
{
	if (name != nullptr) {
		req.name = name;
	}

	uavcan::ServiceClient<uavcan::protocol::param::GetSet, GetSetCallback> client(_node);
	client.setCallback(GetSetCallback(this, &UavcanNode::cb_setget));
	_callback_success = false;
	int call_res = client.call(remote_node_id, req);

	if (call_res >= 0) {

		while (client.hasPendingCalls()) {
			usleep(10000);
		}

		if (!_callback_success) {
			call_res = -1;
		}
	}

	return call_res;
}

int  UavcanNode::set_param(int remote_node_id, const char *name, char *value)
{
	uavcan::protocol::param::GetSet::Request req;
	uavcan::protocol::param::GetSet::Response resp;
	set_setget_response(&resp);
	int rv = get_set_param(remote_node_id, name, req);

	if (rv < 0 || resp.name.empty()) {
		std::printf("Failed to retrieve param: %s\n", name);
		rv = -1;

	} else {

		rv = 0;
		req = {};

		if (resp.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
			int64_t i = std::strtoull(value, NULL, 10);
			int64_t min = resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>();
			int64_t max = resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>();

			if (i >= min &&  i <= max) {
				req.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = i;

			} else {
				std::printf("Invalid value for: %s must be between %lld and %lld\n", name, min, max);
				rv = -1;
			}

		} else if (resp.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
			float f = static_cast<float>(std::atof(value));
			float min = resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>();
			float max = resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>();

			if (f >= min &&  f <= max) {
				req.value.to<uavcan::protocol::param::Value::Tag::real_value>() = f;

			} else {
				std::printf("Invalid value for: %s must be between %.4f and %.4f\n", name, static_cast<double>(min),
					    static_cast<double>(max));
				rv = -1;
			}

		} else if (resp.value.is(uavcan::protocol::param::Value::Tag::boolean_value)) {
			int8_t i = (value[0] == '1' || value[0] == 't') ? 1 : 0;
			req.value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = i;

		} else if (resp.value.is(uavcan::protocol::param::Value::Tag::string_value)) {
			req.value.to<uavcan::protocol::param::Value::Tag::string_value>() = value;
		}

		if (rv == 0) {
			rv = get_set_param(remote_node_id, name, req);

			if (rv < 0 || resp.name.empty()) {
				std::printf("Failed to set param: %s\n", name);
				return -1;
			}

			return 0;
		}
	}

	free_setget_response();
	return rv;
}

int  UavcanNode::get_param(int remote_node_id, const char *name)
{
	uavcan::protocol::param::GetSet::Request req;
	uavcan::protocol::param::GetSet::Response resp;
	set_setget_response(&resp);
	int rv = get_set_param(remote_node_id, name, req);

	if (rv < 0 || resp.name.empty()) {
		std::printf("Failed to get param: %s\n", name);
		rv = -1;

	} else {
		print_params(resp);
		rv = 0;
	}

	free_setget_response();
	return rv;
}

void UavcanNode::update_params()
{
	// multicopter air-mode
	param_t param_handle = param_find("MC_AIRMODE");

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &_airmode);
	}

	param_handle = param_find("THR_MDL_FAC");

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &_thr_mdl_factor);
	}
}

int UavcanNode::start_fw_server()
{
	int rv = -1;
	_fw_server_action = Busy;
	UavcanServers   *_servers = UavcanServers::instance();

	if (_servers == nullptr) {

		rv = UavcanServers::start(_node);

		if (rv >= 0) {
			/*
			 * Set our pointer to to the injector
			 *  This is a work around as
			 *  main_node.getDispatcher().installRxFrameListener(driver.get());
			 *  would require a dynamic cast and rtti is not enabled.
			 */
			UavcanServers::instance()->attachITxQueueInjector(&_tx_injector);
		}
	}

	_fw_server_action = None;
	px4_sem_post(&_server_command_sem);
	return rv;
}

int UavcanNode::request_fw_check()
{
	int rv = -1;
	_fw_server_action = Busy;
	UavcanServers   *_servers  = UavcanServers::instance();

	if (_servers != nullptr) {
		_servers->requestCheckAllNodesFirmwareAndUpdate();
		rv = 0;
	}

	_fw_server_action = None;
	px4_sem_post(&_server_command_sem);
	return rv;

}

int UavcanNode::stop_fw_server()
{
	int rv = -1;
	_fw_server_action = Busy;
	UavcanServers   *_servers  = UavcanServers::instance();

	if (_servers != nullptr) {
		/*
		 * Set our pointer to to the injector
		 *  This is a work around as
		 *  main_node.getDispatcher().remeveRxFrameListener();
		 *  would require a dynamic cast and rtti is not enabled.
		 */
		_tx_injector = nullptr;

		rv = _servers->stop();
	}

	_fw_server_action = None;
	px4_sem_post(&_server_command_sem);
	return rv;
}


int UavcanNode::fw_server(eServerAction action)
{
	int rv = -EAGAIN;

	switch (action) {
	case Start:
	case Stop:
	case CheckFW:
		if (_fw_server_action == None) {
			_fw_server_action = action;
			px4_sem_wait(&_server_command_sem);
			rv = _fw_server_status;
		}

		break;

	default:
		rv = -EINVAL;
		break;
	}

	return rv;
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

	/*
	 * Node init
	 */
	_instance = new UavcanNode(can->driver, UAVCAN_DRIVER::SystemClock::instance());

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	const int node_init_res = _instance->init(node_id);

	if (node_init_res < 0) {
		delete _instance;
		_instance = nullptr;
		PX4_ERR("Node init failed %i", node_init_res);
		return node_init_res;
	}

	/*
	 * Start the task. Normally it should never exit.
	 */
	static auto run_trampoline = [](int, char *[]) {return UavcanNode::_instance->run();};
	_instance->_task = px4_task_spawn_cmd("uavcan", SCHED_DEFAULT, SCHED_PRIORITY_ACTUATOR_OUTPUTS, StackSize,
					      static_cast<main_t>(run_trampoline), nullptr);

	if (_instance->_task < 0) {
		PX4_ERR("start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void UavcanNode::fill_node_info()
{
	/* software version */
	uavcan::protocol::SoftwareVersion swver;

	// Extracting the first 8 hex digits of the git hash and converting them to int
	char fw_git_short[9] = {};
	std::memmove(fw_git_short, px4_firmware_version_string(), 8);
	char *end = nullptr;
	swver.vcs_commit = std::strtol(fw_git_short, &end, 16);
	swver.optional_field_flags |= swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;

	// Too verbose for normal operation
	//PX4_INFO("SW version vcs_commit: 0x%08x", unsigned(swver.vcs_commit));

	_node.setSoftwareVersion(swver);

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;
	getHardwareVersion(hwver);
	_node.setHardwareVersion(hwver);
}


int UavcanNode::init(uavcan::NodeID node_id)
{
	int ret = -1;

	// Do regular cdev init
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	_node.setName("org.pixhawk.pixhawk");

	_node.setNodeID(node_id);

	fill_node_info();

	// Actuators
	ret = _esc_controller.init();

	if (ret < 0) {
		return ret;
	}

	{
		(void) param_get(param_find("UAVCAN_ESC_IDLT"), &_idle_throttle_when_armed);
		_esc_controller.enable_idle_throttle_when_armed(_idle_throttle_when_armed > 0);
	}

	ret = _hardpoint_controller.init();

	if (ret < 0) {
		return ret;
	}

	ret = _servo_controller.init();

	// Sensor bridges
	IUavcanSensorBridge::make_all(_node, _sensor_bridges);

	for (const auto &br : _sensor_bridges) {
		ret = br->init();

		if (ret < 0) {
			PX4_ERR("cannot init sensor bridge '%s' (%d)", br->get_name(), ret);
			return ret;
		}

		PX4_INFO("sensor bridge '%s' init ok", br->get_name());
	}

	/* find which mixer channels are assigned to ESCs and which channels are assigned to servo motors */
	get_mixer_mapping();

	/*  Start the Node   */
	return _node.start();
}

void UavcanNode::node_spin_once()
{
	const int spin_res = _node.spinOnce();

	if (spin_res < 0) {
		PX4_ERR("node spin error %i", spin_res);
	}


	if (_tx_injector != nullptr) {
		_tx_injector->injectTxFramesInto(_node);
	}
}

/*
  add a fd to the list of polled events. This assumes you want
  POLLIN for now.
 */
int UavcanNode::add_poll_fd(int fd)
{
	int ret = _poll_fds_num;

	if (_poll_fds_num >= UAVCAN_NUM_POLL_FDS) {
		errx(1, "uavcan: too many poll fds, exiting");
	}

	_poll_fds[_poll_fds_num]	= ::pollfd();
	_poll_fds[_poll_fds_num].fd	= fd;
	_poll_fds[_poll_fds_num].events	= POLLIN;
	_poll_fds_num += 1;
	return ret;
}


void UavcanNode::handle_time_sync(const uavcan::TimerEvent &)
{

	/*
	 * Check whether there are higher priority masters in the network.
	 * If there are, we need to activate the local slave in order to sync with them.
	 */
	if (_time_sync_slave.isActive()) { // "Active" means that the slave tracks at least one remote master in the network
		if (_node.getNodeID() < _time_sync_slave.getMasterNodeID()) {
			/*
			 * We're the highest priority master in the network.
			 * We need to suppress the slave now to prevent it from picking up unwanted sync messages from
			 * lower priority masters.
			 */
			_time_sync_slave.suppress(true);  // SUPPRESS

		} else {
			/*
			 * There is at least one higher priority master in the network.
			 * We need to allow the slave to adjust our local clock in order to be in sync.
			 */
			_time_sync_slave.suppress(false);  // UNSUPPRESS
		}

	} else {
		/*
		 * There are no other time sync masters in the network, so we're the only time source.
		 * The slave must be suppressed anyway to prevent it from disrupting the local clock if a new
		 * lower priority master suddenly appears in the network.
		 */
		_time_sync_slave.suppress(true);
	}

	/*
	 * Publish the sync message now, even if we're not a higher priority master.
	 * Other nodes will be able to pick the right master anyway.
	 */
	_time_sync_master.publish();
}



int UavcanNode::run()
{
	(void)pthread_mutex_lock(&_node_mutex);

	// XXX figure out the output count
	_output_count = 2;

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_test_motor_sub = orb_subscribe(ORB_ID(test_motor));
	_actuator_direct_sub = orb_subscribe(ORB_ID(actuator_direct));

	memset(&_outputs, 0, sizeof(_outputs));

	/*
	 * Set up the time synchronization
	 */

	const int slave_init_res = _time_sync_slave.start();

	if (slave_init_res < 0) {
		PX4_ERR("Failed to start time_sync_slave");
		_task_should_exit = true;
	}

	/* When we have a system wide notion of time update (i.e the transition from the initial
	 * System RTC setting to the GPS) we would call UAVCAN_DRIVER::clock::setUtc() when that
	 * happens, but for now we use adjustUtc with a correction of the hrt so that the
	 * time bases are the same
	 */
	UAVCAN_DRIVER::clock::adjustUtc(uavcan::UtcDuration::fromUSec(hrt_absolute_time()));
	_master_timer.setCallback(TimerCallback(this, &UavcanNode::handle_time_sync));
	_master_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000));

	_node_status_monitor.start();

	const int busevent_fd = ::open(UAVCAN_DRIVER::BusEvent::DevName, 0);

	if (busevent_fd < 0) {
		PX4_ERR("Failed to open %s", UAVCAN_DRIVER::BusEvent::DevName);
		_task_should_exit = true;
	}

	/*
	 * XXX Mixing logic/subscriptions shall be moved into UavcanEscController::update();
	 *	 IO multiplexing shall be done here.
	 */

	_node.setModeOperational();

	/*
	 * This event is needed to wake up the thread on CAN bus activity (RX/TX/Error).
	 * Please note that with such multiplexing it is no longer possible to rely only on
	 * the value returned from poll() to detect whether actuator control has timed out or not.
	 * Instead, all ORB events need to be checked individually (see below).
	 */
	add_poll_fd(busevent_fd);

	/*
	 * setup poll to look for actuator direct input if we are
	 * subscribed to the topic
	 */
	if (_actuator_direct_sub != -1) {
		_actuator_direct_poll_fd_num = add_poll_fd(_actuator_direct_sub);
	}

	update_params();

	uORB::Subscription parameter_update_sub{ORB_ID(parameter_update)};

	while (!_task_should_exit) {

		// check for parameter updates
		if (parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			update_params();
		}

		switch (_fw_server_action) {
		case Start:
			_fw_server_status = start_fw_server();
			break;

		case Stop:
			_fw_server_status = stop_fw_server();
			break;

		case CheckFW:
			_fw_server_status = request_fw_check();
			break;

		case None:
		default:
			break;
		}

		// update actuator controls subscriptions if needed
		if (_groups_subscribed != _groups_required) {
			subscribe();
			_groups_subscribed = _groups_required;
		}

		// Mutex is unlocked while the thread is blocked on IO multiplexing
		(void)pthread_mutex_unlock(&_node_mutex);

		const int poll_ret = ::poll(_poll_fds, _poll_fds_num, PollTimeoutMs);

		(void)pthread_mutex_lock(&_node_mutex);

		node_spin_once();  // Non-blocking

		bool new_output = false;

		// this would be bad...
		if (poll_ret < 0) {
			PX4_ERR("poll error %d", errno);
			continue;

		} else {
			// get controls for required topics
			bool controls_updated = false;

			for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
				if (_control_subs[i] >= 0) {
					if (_poll_fds[_poll_ids[i]].revents & POLLIN) {
						controls_updated = true;
						orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);
					}
				}
			}

			/*
			  see if we have any direct actuator updates
			 */
			if (_actuator_direct_sub != -1 &&
			    (_poll_fds[_actuator_direct_poll_fd_num].revents & POLLIN) &&
			    orb_copy(ORB_ID(actuator_direct), _actuator_direct_sub, &_actuator_direct) == OK &&
			    !_test_in_progress) {
				if (_actuator_direct.nvalues > actuator_outputs_s::NUM_ACTUATOR_OUTPUTS) {
					_actuator_direct.nvalues = actuator_outputs_s::NUM_ACTUATOR_OUTPUTS;
				}

				memcpy(&_outputs.output[0], &_actuator_direct.values[0],
				       _actuator_direct.nvalues * sizeof(float));
				_outputs.noutputs = _actuator_direct.nvalues;
				new_output = true;
			}

			// can we mix?
			if (_test_in_progress) {
				memset(&_outputs, 0, sizeof(_outputs));

				if (_test_motor.motor_number < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS) {
					_outputs.output[_test_motor.motor_number] = _test_motor.value * 2.0f - 1.0f;
					_outputs.noutputs = _test_motor.motor_number + 1;
				}

				new_output = true;

			} else if (controls_updated && (_mixers != nullptr)) {

				// XXX one output group has 8 outputs max,
				// but this driver could well serve multiple groups.
				unsigned num_outputs_max = 8;

				_mixers->set_thrust_factor(_thr_mdl_factor);
				_mixers->set_airmode(_airmode);

				// Do mixing
				_outputs.noutputs = _mixers->mix(&_outputs.output[0], num_outputs_max);

				//PX4_INFO("(controls_updated && (_mixers != nullptr");
				new_output = true;
			}
		}

		if (new_output) {
			// Are any channels mapped to ESCs?
			if (_esc_channel_map) {
				// Do we have any channels mapped to ESCs?
				// iterate actuators, checking for valid values
				for (uint8_t i = 0; i < _outputs.noutputs; i++) {


					// last resort: catch NaN, INF and out-of-band errors
					if (!isfinite(_outputs.output[i])) {
						/*
						* Value is NaN, INF or out of band - set to the minimum value.
						* This will be clearly visible on the servo status and will limit the risk of accidentally
						* spinning motors. It would be deadly in flight.
						*/
						_outputs.output[i] = -1.0f;
					}

					// never go below min
					if (_outputs.output[i] < -1.0f) {
						_outputs.output[i] = -1.0f;
					}

					// never go above max
					if (_outputs.output[i] > 1.0f) {
						_outputs.output[i] = 1.0f;
					}

					// Determine if this mixer channel is enabled and mapped to an ESC
					// Ideally we want _esc_controller.update_outputs() to accept single ESC commands with an ID, but that might break existing implementations
					// For now we just send a minimum value, but this adds unneccesary traffic to the CAN bus
					if (!(_esc_channel_map & (1 << i))) {
						_outputs.output[i] = -1.0f;
					}
				}

				// Output to the bus
				_esc_controller.update_outputs(_outputs.output, _outputs.noutputs);
			}

			_outputs.timestamp = hrt_absolute_time();

			/* From Mixer */
			for (uint8_t i = 0; i < _outputs.noutputs; i++) {
				if (_servo_channel_map & (1 << i)) {
					_servo_controller.update_outputs(i, _outputs.output[i]);
					// PX4_INFO("Servo Actuator output %d", _outputs.output[i]);
				}
			}

			// use first valid timestamp_sample for latency tracking
			for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
				const bool required = _groups_required & (1 << i);
				const hrt_abstime &timestamp_sample = _controls[i].timestamp_sample;

				if (required && (timestamp_sample > 0)) {
					perf_set_elapsed(_perf_control_latency, _outputs.timestamp - timestamp_sample);
					break;
				}
			}
		}


		// Check motor test state
		bool updated = false;
		orb_check(_test_motor_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(test_motor), _test_motor_sub, &_test_motor);

			// Update the test status and check that we're not locked down
			_test_in_progress = (_test_motor.value > 0);
			_esc_controller.arm_single_esc(_test_motor.motor_number, _test_in_progress);
		}

		// Check arming state
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

			// Update the armed status and check that we're not locked down and motor
			// test is not running
			bool set_armed = _armed.armed && !_armed.lockdown && !_armed.manual_lockdown && !_test_in_progress;

			arm_actuators(set_armed);

			if (_armed.soft_stop) {
				_esc_controller.enable_idle_throttle_when_armed(false);

			} else {
				_esc_controller.enable_idle_throttle_when_armed(_idle_throttle_when_armed > 0);
			}
		}
	}

	(void)::close(busevent_fd);

	teardown();

	exit(0);
}

int
UavcanNode::control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];
	return 0;
}

int
UavcanNode::teardown()
{
	px4_sem_post(&_server_command_sem);

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] >= 0) {
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	return (_armed_sub >= 0) ? orb_unsubscribe(_armed_sub) : 0;
}

int
UavcanNode::arm_actuators(bool arm)
{
	_is_armed = arm;
	_esc_controller.arm_all_escs(arm);
	_servo_controller.arm_all_servos(arm);
	return OK;
}

void
UavcanNode::subscribe()
{
	// Subscribe/unsubscribe to required actuator control groups
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;

	// the first fd used by CAN
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			PX4_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			PX4_DEBUG("unsubscribe from actuator_controls_%d", i);
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] >= 0) {
			_poll_ids[i] = add_poll_fd(_control_subs[i]);
			orb_set_interval(_control_subs[i], 1000 / UavcanEscController::MAX_RATE_HZ);
		}
	}
}

int
UavcanNode::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		arm_actuators(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		// these are no-ops, as no safety switch
		break;

	case PWM_SERVO_DISARM:
		arm_actuators(false);
		break;

	case MIXERIOCGETOUTPUTCOUNT:
		*(unsigned *)arg = _output_count;
		break;

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)_controls);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					PX4_ERR("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
					PX4_INFO("Groups required %d", _groups_required);

					int rotor_count = _mixers->get_multirotor_count();
					_esc_controller.set_rotor_count(rotor_count);
					PX4_INFO("Number of rotors %d", rotor_count);
				}
			}
		}
		break;


	case UAVCAN_IOCS_HARDPOINT_SET: {
			const auto &hp_cmd = *reinterpret_cast<uavcan::equipment::hardpoint::Command *>(arg);
			_hardpoint_controller.set_command(hp_cmd.hardpoint_id, hp_cmd.command);
		}
		break;

	case UAVCAN_IOCG_NODEID_INPROGRESS: {
			UavcanServers   *_servers = UavcanServers::instance();

			if (_servers == nullptr) {
				// status unavailable
				ret = -EINVAL;
				break;

			} else if (_servers->guessIfAllDynamicNodesAreAllocated()) {
				// node discovery complete
				ret = -ETIME;
				break;

			} else {
				// node discovery in progress
				ret = OK;
				break;
			}
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

void
UavcanNode::print_info()
{
	(void)pthread_mutex_lock(&_node_mutex);

	// Memory status
	printf("Pool allocator status:\n");
	printf("\tCapacity hard/soft: %u/%u blocks\n",
	       _pool_allocator.getBlockCapacityHardLimit(), _pool_allocator.getBlockCapacity());
	printf("\tReserved:  %u blocks\n", _pool_allocator.getNumReservedBlocks());
	printf("\tAllocated: %u blocks\n", _pool_allocator.getNumAllocatedBlocks());

	// UAVCAN node perfcounters
	printf("UAVCAN node status:\n");
	printf("\tInternal failures: %llu\n", _node.getInternalFailureCount());
	printf("\tTransfer errors:   %llu\n", _node.getDispatcher().getTransferPerfCounter().getErrorCount());
	printf("\tRX transfers:      %llu\n", _node.getDispatcher().getTransferPerfCounter().getRxTransferCount());
	printf("\tTX transfers:      %llu\n", _node.getDispatcher().getTransferPerfCounter().getTxTransferCount());

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

	// ESC mixer status
	printf("ESC actuators control groups: sub: %u / req: %u / fds: %u\n",
	       (unsigned)_groups_subscribed, (unsigned)_groups_required, _poll_fds_num);
	printf("ESC mixer: %s\n", (_mixers == nullptr) ? "NONE" : "OK");

	if (_outputs.noutputs != 0) {
		PX4_INFO("ESC output: ");

		for (uint8_t i = 0; i < _outputs.noutputs; i++) {
			printf("%d ", (int)(_outputs.output[i] * 1000));
		}

		printf("\n");

		// ESC status
		int esc_sub = orb_subscribe(ORB_ID(esc_status));
		esc_status_s esc = {};
		orb_copy(ORB_ID(esc_status), esc_sub, &esc);
		orb_unsubscribe(esc_sub);

		print_message(esc);
	}

	// Sensor bridges
	for (const auto &br : _sensor_bridges) {
		printf("Sensor '%s':\n", br->get_name());
		br->print_status();
		printf("\n");
	}

	// Printing all nodes that are online
	std::printf("Online nodes (Node ID, Health, Mode):\n");
	_node_status_monitor.forEachNode([](uavcan::NodeID nid, uavcan::NodeStatusMonitor::NodeStatus ns) {
		static constexpr const char *HEALTH[] = {
			"OK", "WARN", "ERR", "CRIT"
		};
		static constexpr const char *MODES[] = {
			"OPERAT", "INIT", "MAINT", "SW_UPD", "?", "?", "?", "OFFLN"
		};
		std::printf("\t% 3d %-10s %-10s\n", int(nid.get()), HEALTH[ns.health], MODES[ns.mode]);
	});

	(void)pthread_mutex_unlock(&_node_mutex);
}

void UavcanNode::shrink()
{
	(void)pthread_mutex_lock(&_node_mutex);
	_pool_allocator.shrink();
	(void)pthread_mutex_unlock(&_node_mutex);
}

void UavcanNode::hardpoint_controller_set(uint8_t hardpoint_id, uint16_t command)
{
	(void)pthread_mutex_lock(&_node_mutex);
	_hardpoint_controller.set_command(hardpoint_id, command);
	(void)pthread_mutex_unlock(&_node_mutex);
}

void UavcanNode::get_mixer_mapping()
{
	char str[20];
	int value;

	for (uint8_t i = 1; i < 9; i++) {
		(void)sprintf(str, "UAVCAN_MIX_CH%u", i);

		if (OK == param_get(param_find(str), &value)) {
			if ((value == 1) || (value == 3)) {
				_esc_channel_map += 1 << (i - 1);
				PX4_INFO("AUX Mixer ch%u mapped to ESC", i);
			}

			if ((value == 2) || (value == 3)) {
				_servo_channel_map += 1 << (i - 1);
				PX4_INFO("AUX Mixer ch%u mapped to servo", i);
			}
		}
	}
}

/*
 * App entry point
 */
static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tuavcan {start [fw]|status|stop [all|fw]|shrink|arm|disarm|update fw|\n"
		 "\t        param [set|get|list|save] <node-id> <name> <value>|reset <node-id>|\n"
		 "\t        hardpoint set <id> <command>}");
}

extern "C" __EXPORT int uavcan_main(int argc, char *argv[]);

int uavcan_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	bool fw = argc > 2 && !std::strcmp(argv[2], "fw");

	if (!std::strcmp(argv[1], "start")) {
		if (UavcanNode::instance()) {
			if (fw && UavcanServers::instance() == nullptr) {
				int rv = UavcanNode::instance()->fw_server(UavcanNode::Start);

				if (rv < 0) {
					PX4_ERR("Firmware Server Failed to Start %d", rv);
					::exit(rv);
				}

				::exit(0);
			}

			// Already running, no error
			PX4_INFO("already started");
			::exit(0);
		}

		// Node ID
		int32_t node_id = 1;
		(void)param_get(param_find("UAVCAN_NODE_ID"), &node_id);

		if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
			PX4_ERR("Invalid Node ID %i", node_id);
			::exit(1);
		}

		// CAN bitrate
		int32_t bitrate = 1000000;
		(void)param_get(param_find("UAVCAN_BITRATE"), &bitrate);

		// Start
		PX4_INFO("Node ID %u, bitrate %u", node_id, bitrate);
		return UavcanNode::start(node_id, bitrate);
	}

	/* commands below require the app to be started */
	UavcanNode *const inst = UavcanNode::instance();

	if (!inst) {
		errx(1, "application not running");
	}

	if (fw && !std::strcmp(argv[1], "update")) {
		if (UavcanServers::instance() == nullptr) {
			errx(1, "firmware server is not running");
		}

		int rv = UavcanNode::instance()->fw_server(UavcanNode::CheckFW);
		::exit(rv);
	}

	if (fw && (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info"))) {
		printf("Firmware Server is %s\n", UavcanServers::instance() ? "Running" : "Stopped");
		::exit(0);
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		inst->print_info();
		::exit(0);
	}

	if (!std::strcmp(argv[1], "shrink")) {
		inst->shrink();
		::exit(0);
	}

	if (!std::strcmp(argv[1], "arm")) {
		inst->arm_actuators(true);
		::exit(0);
	}

	if (!std::strcmp(argv[1], "disarm")) {
		inst->arm_actuators(false);
		::exit(0);
	}

	/*
	 * Parameter setting commands
	 *
	 *  uavcan param list <node>
	 *  uavcan param save <node>
	 *  uavcan param get <node> <name>
	 *  uavcan param set <node> <name> <value>
	 *
	 */
	int node_arg = !std::strcmp(argv[1], "reset") ? 2 : 3;

	if (!std::strcmp(argv[1], "param") || node_arg == 2) {
		if (argc < node_arg + 1) {
			errx(1, "Node id required");
		}

		int nodeid = atoi(argv[node_arg]);

		if (nodeid  == 0 || nodeid  > 127 || nodeid  == inst->get_node().getNodeID().get()) {
			errx(1, "Invalid Node id");
		}

		if (node_arg == 2) {

			return inst->reset_node(nodeid);

		} else if (!std::strcmp(argv[2], "list")) {

			return inst->list_params(nodeid);

		} else if (!std::strcmp(argv[2], "save")) {

			return inst->save_params(nodeid);

		} else if (!std::strcmp(argv[2], "get")) {
			if (argc < 5) {
				errx(1, "Name required");
			}

			return inst->get_param(nodeid, argv[4]);

		} else if (!std::strcmp(argv[2], "set")) {
			if (argc < 5) {
				errx(1, "Name required");
			}

			if (argc < 6) {
				errx(1, "Value required");
			}

			return inst->set_param(nodeid, argv[4], argv[5]);
		}
	}

	if (!std::strcmp(argv[1], "hardpoint")) {
		if (!std::strcmp(argv[2], "set") && argc > 4) {
			const int hardpoint_id = atoi(argv[3]);
			const int command = atoi(argv[4]);

			// Sanity check - weed out negative values, check against maximums
			if (hardpoint_id >= 0 && hardpoint_id < 256 &&
			    command >= 0 && command < 65536) {
				inst->hardpoint_controller_set((uint8_t) hardpoint_id, (uint16_t) command);

			} else {
				errx(1, "Invalid argument");
			}

		} else {
			errx(1, "Invalid hardpoint command");
		}

		::exit(0);
	}

	if (!std::strcmp(argv[1], "stop")) {
		if (fw) {

			int rv = inst->fw_server(UavcanNode::Stop);

			/* Let's recover any memory we can */

			inst->shrink();

			if (rv < 0) {
				PX4_ERR("Firmware Server Failed to Stop %d", rv);
				::exit(rv);
			}

			::exit(0);

		} else {
			delete inst;
			::exit(0);
		}
	}

	print_usage();
	::exit(1);
}
