/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>

#include <nuttx/config.h>

#include <cstdlib>
#include <cstring>
#include <cctype>
#include <fcntl.h>
#include <dirent.h>
#include <pthread.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <version/version.h>

#include <arch/chip/chip.h>

#include "uavcan_main.hpp"
#include "uavcan_servers.hpp"
#include "uavcan_virtual_can_driver.hpp"

#include <uavcan_posix/dynamic_node_id_server/file_event_tracer.hpp>
#include <uavcan_posix/dynamic_node_id_server/file_storage_backend.hpp>
#include <uavcan_posix/firmware_version_checker.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/uavcan_parameter_request.h>

/**
 * @file uavcan_servers.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *         David Sidrane <david_s5@nscdg.com>
 */

/*
 * UavcanNode
 */

UavcanServers *UavcanServers::_instance;

UavcanServers::UavcanServers(uavcan::INode &main_node) :
	_vdriver(NumIfaces, UAVCAN_DRIVER::SystemClock::instance(), main_node.getAllocator(), VirtualIfaceBlockAllocationQuota),
	_subnode(_vdriver, UAVCAN_DRIVER::SystemClock::instance(), main_node.getAllocator()),
	_main_node(main_node),
	_server_instance(_subnode, _storage_backend, _tracer),
	_fileserver_backend(_subnode),
	_node_info_retriever(_subnode),
	_fw_upgrade_trigger(_subnode, _fw_version_checker),
	_fw_server(_subnode, _fileserver_backend),
	_param_getset_client(_subnode),
	_param_opcode_client(_subnode),
	_param_restartnode_client(_subnode),
	_beep_pub(_subnode),
	_enumeration_indication_sub(_subnode),
	_enumeration_client(_subnode),
	_enumeration_getset_client(_subnode),
	_enumeration_save_client(_subnode)
{
}

UavcanServers::~UavcanServers()
{
	if (_mutex_inited) {
		(void)Lock::deinit(_subnode_mutex);
	}

	_main_node.getDispatcher().removeRxFrameListener();
}

int
UavcanServers::stop()
{
	UavcanServers *server = instance();

	if (server == nullptr) {
		PX4_INFO("Already stopped");
		return -1;
	}

	if (server->_subnode_thread) {
		PX4_INFO("stopping fw srv thread...");
		server->_subnode_thread_should_exit.store(true);
		(void)pthread_join(server->_subnode_thread, NULL);
	}

	_instance = nullptr;

	server->_main_node.getDispatcher().removeRxFrameListener();

	delete server;
	return 0;
}

int
UavcanServers::start(uavcan::INode &main_node)
{
	if (_instance != nullptr) {
		PX4_INFO("Already started");
		return -1;
	}

	/*
	 * Node init
	 */
	_instance = new UavcanServers(main_node);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -2;
	}

	int rv = _instance->init();

	if (rv < 0) {
		PX4_ERR("Node init failed: %d", rv);
		delete _instance;
		_instance = nullptr;
		return rv;
	}

	/*
	 * Start the thread. Normally it should never exit.
	 */
	pthread_attr_t tattr;
	struct sched_param param;

	pthread_attr_init(&tattr);
	(void)pthread_attr_getschedparam(&tattr, &param);
	tattr.stacksize = PX4_STACK_ADJUSTED(StackSize);
	param.sched_priority = Priority;

	if (pthread_attr_setschedparam(&tattr, &param)) {
		PX4_ERR("setting sched params failed");
	}

	static auto run_trampoline = [](void *) {return UavcanServers::_instance->run(_instance);};

	rv = pthread_create(&_instance->_subnode_thread, &tattr, static_cast<pthread_startroutine_t>(run_trampoline), NULL);

	if (rv != 0) {
		rv = -rv;
		PX4_ERR("pthread_create() failed: %d", rv);
		delete _instance;
		_instance = nullptr;
	}

	return rv;
}

int
UavcanServers::init()
{
	errno = 0;

	/*
	 * Initialize the mutex.
	 * giving it its path
	 */
	int ret = Lock::init(_subnode_mutex);

	if (ret < 0) {
		PX4_ERR("Lock init: %d", errno);
		return ret;
	}

	_mutex_inited = true;

	_subnode.setNodeID(_main_node.getNodeID());
	_main_node.getDispatcher().installRxFrameListener(&_vdriver);

	/*
	 * Initialize the fw version checker.
	 * giving it its path
	 */
	ret = _fw_version_checker.createFwPaths(UAVCAN_FIRMWARE_PATH, UAVCAN_ROMFS_FW_PATH);

	if (ret < 0) {
		PX4_ERR("FirmwareVersionChecker init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* Start fw file server back */

	ret = _fw_server.start(UAVCAN_FIRMWARE_PATH, UAVCAN_ROMFS_FW_PATH);

	if (ret < 0) {
		PX4_ERR("BasicFileServer init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* Initialize storage back end for the node allocator using UAVCAN_NODE_DB_PATH directory */

	ret = _storage_backend.init(UAVCAN_NODE_DB_PATH);

	if (ret < 0) {
		PX4_ERR("FileStorageBackend init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* Initialize trace in the UAVCAN_NODE_DB_PATH directory */

	ret = _tracer.init(UAVCAN_LOG_FILE);

	if (ret < 0) {
		PX4_ERR("FileEventTracer init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;
	UavcanNode::getHardwareVersion(hwver);

	/* Initialize the dynamic node id server  */
	ret = _server_instance.init(hwver.unique_id);

	if (ret < 0) {
		PX4_ERR("CentralizedServer init: %d", ret);
		return ret;
	}

	/* Start node info retriever to fetch node info from new nodes */
	ret = _node_info_retriever.start();

	if (ret < 0) {
		PX4_ERR("NodeInfoRetriever init: %d", ret);
		return ret;
	}

	/* Start the fw version checker   */
	ret = _fw_upgrade_trigger.start(_node_info_retriever);

	if (ret < 0) {
		PX4_ERR("FirmwareUpdateTrigger init: %d", ret);
		return ret;
	}

	/*  Start the Node   */
	return 0;
}

pthread_addr_t
UavcanServers::run(pthread_addr_t)
{
	prctl(PR_SET_NAME, "uavcan_fw_srv", 0);

	Lock lock(_subnode_mutex);

	/*
	Check for firmware in the root directory, move it to appropriate location on
	the SD card, as defined by the APDesc.
	*/
	migrateFWFromRoot(UAVCAN_FIRMWARE_PATH, UAVCAN_SD_ROOT_PATH);

	/* the subscribe call needs to happen in the same thread,
	 * so not in the constructor */
	uORB::Subscription armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription vcmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription param_request_sub{ORB_ID(uavcan_parameter_request)};

	/* Set up shared service clients */
	_param_getset_client.setCallback(GetSetCallback(this, &UavcanServers::cb_getset));
	_param_opcode_client.setCallback(ExecuteOpcodeCallback(this, &UavcanServers::cb_opcode));
	_param_restartnode_client.setCallback(RestartNodeCallback(this, &UavcanServers::cb_restart));
	_enumeration_client.setCallback(EnumerationBeginCallback(this, &UavcanServers::cb_enumeration_begin));
	_enumeration_indication_sub.start(EnumerationIndicationCallback(this, &UavcanServers::cb_enumeration_indication));
	_enumeration_getset_client.setCallback(GetSetCallback(this, &UavcanServers::cb_enumeration_getset));
	_enumeration_save_client.setCallback(ExecuteOpcodeCallback(this, &UavcanServers::cb_enumeration_save));

	_count_in_progress = _param_in_progress = _param_list_in_progress = _cmd_in_progress = _param_list_all_nodes = false;
	memset(_param_counts, 0, sizeof(_param_counts));

	_esc_enumeration_active = false;
	memset(_esc_enumeration_ids, 0, sizeof(_esc_enumeration_ids));
	_esc_enumeration_index = 0;

	while (!_subnode_thread_should_exit.load()) {

		if (_check_fw == true) {
			_check_fw = false;
			_node_info_retriever.invalidateAll();
		}

		const int spin_res = _subnode.spin(uavcan::MonotonicDuration::fromMSec(10));

		if (spin_res < 0) {
			PX4_ERR("node spin error %i", spin_res);
		}

		// Check for parameter requests (get/set/list)
		if (param_request_sub.updated() && !_param_list_in_progress && !_param_in_progress && !_count_in_progress) {
			uavcan_parameter_request_s request{};
			param_request_sub.copy(&request);

			if (_param_counts[request.node_id]) {
				/*
				 * We know how many parameters are exposed by this node, so
				 * process the request.
				 */
				if (request.message_type == uavcan_parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_READ) {
					uavcan::protocol::param::GetSet::Request req;

					if (request.param_index >= 0) {
						req.index = request.param_index;

					} else {
						req.name = (char *)request.param_id;
					}

					int call_res = _param_getset_client.call(request.node_id, req);

					if (call_res < 0) {
						PX4_ERR("couldn't send GetSet: %d", call_res);

					} else {
						_param_in_progress = true;
						_param_index = request.param_index;
					}

				} else if (request.message_type == uavcan_parameter_request_s::MESSAGE_TYPE_PARAM_SET) {
					uavcan::protocol::param::GetSet::Request req;

					if (request.param_index >= 0) {
						req.index = request.param_index;

					} else {
						req.name = (char *)request.param_id;
					}

					if (request.param_type == uavcan_parameter_request_s::PARAM_TYPE_REAL32) {
						req.value.to<uavcan::protocol::param::Value::Tag::real_value>() = request.real_value;

					} else if (request.param_type == uavcan_parameter_request_s::PARAM_TYPE_UINT8) {
						req.value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = request.int_value;

					} else {
						req.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = request.int_value;
					}

					// Set the dirty bit for this node
					set_node_params_dirty(request.node_id);

					int call_res = _param_getset_client.call(request.node_id, req);

					if (call_res < 0) {
						PX4_ERR("couldn't send GetSet: %d", call_res);

					} else {
						_param_in_progress = true;
						_param_index = request.param_index;
					}

				} else if (request.message_type == uavcan_parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_LIST) {
					// This triggers the _param_list_in_progress case below.
					_param_index = 0;
					_param_list_in_progress = true;
					_param_list_node_id = request.node_id;
					_param_list_all_nodes = false;

					PX4_DEBUG("starting component-specific param list");
				}

			} else if (request.node_id == uavcan_parameter_request_s::NODE_ID_ALL) {
				if (request.message_type == uavcan_parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_LIST) {
					/*
					 * This triggers the _param_list_in_progress case below,
					 * but additionally iterates over all active nodes.
					 */
					_param_index = 0;
					_param_list_in_progress = true;
					_param_list_node_id = get_next_active_node_id(0);
					_param_list_all_nodes = true;

					PX4_DEBUG("starting global param list with node %hhu", _param_list_node_id);

					if (_param_counts[_param_list_node_id] == 0) {
						param_count(_param_list_node_id);
					}
				}

			} else {
				/*
				 * Need to know how many parameters this node has before we can
				 * continue; count them now and then process the request.
				 */
				param_count(request.node_id);
			}
		}

		// Handle parameter listing index/node ID advancement
		if (_param_list_in_progress && !_param_in_progress && !_count_in_progress) {
			if (_param_index >= _param_counts[_param_list_node_id]) {
				PX4_DEBUG("completed param list for node %hhu", _param_list_node_id);
				// Reached the end of the current node's parameter set.
				_param_list_in_progress = false;

				if (_param_list_all_nodes) {
					// We're listing all parameters for all nodes -- get the next node ID
					uint8_t next_id = get_next_active_node_id(_param_list_node_id);

					if (next_id < 128) {
						_param_list_node_id = next_id;

						/*
						 * If there is a next node ID, check if that node's parameters
						 * have been counted before. If not, do it now.
						 */
						if (_param_counts[_param_list_node_id] == 0) {
							param_count(_param_list_node_id);
						}

						// Keep on listing.
						_param_index = 0;
						_param_list_in_progress = true;
						PX4_DEBUG("started param list for node %hhu", _param_list_node_id);
					}
				}
			}
		}

		// Check if we're still listing, and need to get the next parameter
		if (_param_list_in_progress && !_param_in_progress && !_count_in_progress) {
			// Ready to request the next value -- _param_index is incremented
			// after each successful fetch by cb_getset
			uavcan::protocol::param::GetSet::Request req;
			req.index = _param_index;

			int call_res = _param_getset_client.call(_param_list_node_id, req);

			if (call_res < 0) {
				_param_list_in_progress = false;
				PX4_ERR("couldn't send param list GetSet: %d", call_res);

			} else {
				_param_in_progress = true;
			}
		}

		// Check for ESC enumeration commands
		if (vcmd_sub.updated() && !_cmd_in_progress) {
			bool acknowledge = false;
			vehicle_command_s cmd{};
			vcmd_sub.copy(&cmd);

			uint8_t cmd_ack_result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_UAVCAN) {
				acknowledge = true;
				int command_id = static_cast<int>(cmd.param1 + 0.5f);
				int node_id = static_cast<int>(cmd.param2 + 0.5f);
				int call_res;

				PX4_DEBUG("received UAVCAN command ID %d, node ID %d", command_id, node_id);

				switch (command_id) {
				case 0:
				case 1: {
						_esc_enumeration_active = command_id;
						_esc_enumeration_index = 0;
						_esc_count = 0;
						uavcan::protocol::enumeration::Begin::Request req;
						// TODO: Incorrect implementation; the parameter name field should be left empty.
						//       Leaving it as-is to avoid breaking compatibility with non-compliant nodes.
						req.parameter_name = "esc_index";
						req.timeout_sec = _esc_enumeration_active ? 65535 : 0;
						call_res = _enumeration_client.call(get_next_active_node_id(0), req);

						if (call_res < 0) {
							PX4_ERR("UAVCAN ESC enumeration: couldn't send initial Begin request: %d", call_res);
							beep(BeepFrequencyError);
							cmd_ack_result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;

						} else {
							beep(BeepFrequencyGenericIndication);
						}

						break;
					}

				default: {
						PX4_ERR("unknown command ID %d", command_id);
						cmd_ack_result = vehicle_command_ack_s::VEHICLE_RESULT_UNSUPPORTED;
						break;
					}
				}

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE) {
				acknowledge = true;
				int command_id = static_cast<int>(cmd.param1 + 0.5f);

				PX4_DEBUG("received storage command ID %d", command_id);

				switch (command_id) {
				case 1: {
						// Param save request
						int node_id;
						node_id = get_next_dirty_node_id(1);

						if (node_id < 128) {
							_param_save_opcode = uavcan::protocol::param::ExecuteOpcode::Request::OPCODE_SAVE;
							param_opcode(node_id);
						}

						break;
					}

				case 2: {
						// Command is a param erase request -- apply it to all active nodes by setting the dirty bit
						_param_save_opcode = uavcan::protocol::param::ExecuteOpcode::Request::OPCODE_ERASE;

						for (int i = 1; i < 128; i = get_next_active_node_id(i)) {
							set_node_params_dirty(i);
						}

						param_opcode(get_next_dirty_node_id(1));
						break;
					}
				}
			}

			if (acknowledge) {
				// Acknowledge the received command
				vehicle_command_ack_s ack{};
				ack.command = cmd.command;
				ack.result = cmd_ack_result;
				ack.target_system = cmd.source_system;
				ack.target_component = cmd.source_component;
				ack.timestamp = hrt_absolute_time();
				_command_ack_pub.publish(ack);
			}
		}

		// Shut down once armed
		// TODO (elsewhere): start up again once disarmed?
		if (armed_sub.updated()) {
			actuator_armed_s armed;

			if (armed_sub.copy(&armed)) {
				if (armed.armed) {
					_subnode_thread_should_exit.store(true);
				}
			}
		}
	}

	return (pthread_addr_t) 0;
}

void
UavcanServers::cb_getset(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result)
{
	if (_count_in_progress) {
		/*
		 * Currently in parameter count mode:
		 * Iterate over all parameters for the node to which the request was
		 * originally sent, in order to find the maximum parameter ID. If a
		 * request fails, set the node's parameter count to zero.
		 */
		uint8_t node_id = result.getCallID().server_node_id.get();

		if (result.isSuccessful()) {
			uavcan::protocol::param::GetSet::Response resp = result.getResponse();

			if (resp.name.size()) {
				_count_index++;
				_param_counts[node_id] = _count_index;

				uavcan::protocol::param::GetSet::Request req;
				req.index = _count_index;

				int call_res = _param_getset_client.call(result.getCallID().server_node_id, req);

				if (call_res < 0) {
					_count_in_progress = false;
					_count_index = 0;
					PX4_ERR("couldn't send GetSet during param count: %d", call_res);
					beep(BeepFrequencyError);
				}

			} else {
				_count_in_progress = false;
				_count_index = 0;
				PX4_DEBUG("completed param count for node %hhu: %hhu", node_id, _param_counts[node_id]);
				beep(BeepFrequencyGenericIndication);
			}

		} else {
			_param_counts[node_id] = 0;
			_count_in_progress = false;
			_count_index = 0;
			PX4_ERR("GetSet error during param count");
		}

	} else {
		/*
		 * Currently in parameter get/set mode:
		 * Publish a uORB uavcan_parameter_value message containing the current value
		 * of the parameter.
		 */
		if (result.isSuccessful()) {
			uavcan::protocol::param::GetSet::Response param = result.getResponse();

			struct uavcan_parameter_value_s response;
			response.node_id = result.getCallID().server_node_id.get();
			strncpy(response.param_id, param.name.c_str(), sizeof(response.param_id) - 1);
			response.param_id[16] = '\0';
			response.param_index = _param_index;
			response.param_count = _param_counts[response.node_id];

			if (param.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
				response.param_type = uavcan_parameter_request_s::PARAM_TYPE_INT64;
				response.int_value = param.value.to<uavcan::protocol::param::Value::Tag::integer_value>();

			} else if (param.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
				response.param_type = uavcan_parameter_request_s::PARAM_TYPE_REAL32;
				response.real_value = param.value.to<uavcan::protocol::param::Value::Tag::real_value>();

			} else if (param.value.is(uavcan::protocol::param::Value::Tag::boolean_value)) {
				response.param_type = uavcan_parameter_request_s::PARAM_TYPE_UINT8;
				response.int_value = param.value.to<uavcan::protocol::param::Value::Tag::boolean_value>();
			}

			_param_response_pub.publish(response);

		} else {
			PX4_ERR("GetSet error");
		}

		_param_in_progress = false;
		_param_index++;
	}
}

void
UavcanServers::param_count(uavcan::NodeID node_id)
{
	uavcan::protocol::param::GetSet::Request req;
	req.index = 0;
	int call_res = _param_getset_client.call(node_id, req);

	if (call_res < 0) {
		PX4_ERR("couldn't start parameter count: %d", call_res);

	} else {
		_count_in_progress = true;
		_count_index = 0;
		PX4_DEBUG("starting param count");
	}
}

void
UavcanServers::param_opcode(uavcan::NodeID node_id)
{
	uavcan::protocol::param::ExecuteOpcode::Request opcode_req;
	opcode_req.opcode = _param_save_opcode;
	int call_res = _param_opcode_client.call(node_id, opcode_req);

	if (call_res < 0) {
		PX4_ERR("couldn't send ExecuteOpcode: %d", call_res);

	} else {
		_cmd_in_progress = true;
		PX4_INFO("sent ExecuteOpcode");
	}
}

void
UavcanServers::cb_opcode(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result)
{
	bool success = result.isSuccessful();
	uint8_t node_id = result.getCallID().server_node_id.get();
	uavcan::protocol::param::ExecuteOpcode::Response resp = result.getResponse();
	success &= resp.ok;
	_cmd_in_progress = false;

	if (!result.isSuccessful()) {
		PX4_ERR("save request for node %hhu timed out.", node_id);

	} else if (!result.getResponse().ok) {
		PX4_ERR("save request for node %hhu rejected.", node_id);

	} else {
		PX4_INFO("save request for node %hhu completed OK, restarting.", node_id);

		uavcan::protocol::RestartNode::Request restart_req;
		restart_req.magic_number = restart_req.MAGIC_NUMBER;
		int call_res = _param_restartnode_client.call(node_id, restart_req);

		if (call_res < 0) {
			PX4_ERR("couldn't send RestartNode: %d", call_res);

		} else {
			PX4_ERR("sent RestartNode");
			_cmd_in_progress = true;
		}
	}

	if (!_cmd_in_progress) {
		/*
		 * Something went wrong, so cb_restart is never going to be called as a result of this request.
		 * To ensure we try to execute the opcode on all nodes that permit it, get the next dirty node
		 * ID and keep processing here. The dirty bit on the current node is still set, so the
		 * save/erase attempt will occur when the next save/erase command is received over MAVLink.
		 */
		node_id = get_next_dirty_node_id(node_id);

		if (node_id < 128) {
			param_opcode(node_id);
		}
	}
}

void
UavcanServers::cb_restart(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &result)
{
	bool success = result.isSuccessful();
	uint8_t node_id = result.getCallID().server_node_id.get();
	uavcan::protocol::RestartNode::Response resp = result.getResponse();
	success &= resp.ok;
	_cmd_in_progress = false;

	if (success) {
		PX4_DEBUG("restart request for node %hhu completed OK.", node_id);

		// Clear the dirty flag
		clear_node_params_dirty(node_id);

	} else {
		PX4_ERR("restart request for node %hhu failed.", node_id);
	}

	// Get the next dirty node ID and send the same command to it
	node_id = get_next_dirty_node_id(node_id);

	if (node_id < 128) {
		param_opcode(node_id);
	}
}

uint8_t
UavcanServers::get_next_active_node_id(uint8_t base)
{
	base++;

	for (; base < 128 && (!_node_info_retriever.isNodeKnown(base) ||
			      _subnode.getNodeID().get() == base); base++);

	return base;
}

uint8_t
UavcanServers::get_next_dirty_node_id(uint8_t base)
{
	base++;

	for (; base < 128 && !are_node_params_dirty(base); base++);

	return base;
}

void
UavcanServers::beep(float frequency)
{
	uavcan::equipment::indication::BeepCommand cmd;
	cmd.frequency = frequency;
	cmd.duration = 0.1F;              // We don't want to incapacitate ESC for longer time that this
	(void)_beep_pub.broadcast(cmd);
}

void
UavcanServers::cb_enumeration_begin(const uavcan::ServiceCallResult<uavcan::protocol::enumeration::Begin> &result)
{
	uint8_t next_id = get_next_active_node_id(result.getCallID().server_node_id.get());

	if (!result.isSuccessful()) {
		PX4_ERR("UAVCAN ESC enumeration: begin request for node %hhu timed out.", result.getCallID().server_node_id.get());

	} else if (result.getResponse().error) {
		PX4_ERR("UAVCAN ESC enumeration: begin request for node %hhu rejected: %hhu", result.getCallID().server_node_id.get(),
			result.getResponse().error);

	} else {
		_esc_count++;
		PX4_INFO("UAVCAN ESC enumeration: begin request for node %hhu completed OK.", result.getCallID().server_node_id.get());
	}

	if (next_id < 128) {
		// Still other active nodes to send the request to
		uavcan::protocol::enumeration::Begin::Request req;
		// TODO: Incorrect implementation; the parameter name field should be left empty.
		//       Leaving it as-is to avoid breaking compatibility with non-compliant nodes.
		req.parameter_name = "esc_index";
		req.timeout_sec = _esc_enumeration_active ? 65535 : 0;

		int call_res = _enumeration_client.call(next_id, req);

		if (call_res < 0) {
			PX4_ERR("UAVCAN ESC enumeration: couldn't send Begin request: %d", call_res);

		} else {
			PX4_INFO("UAVCAN ESC enumeration: sent Begin request");
		}

	} else {
		PX4_INFO("UAVCAN ESC enumeration: begun enumeration on all nodes.");
	}
}

void
UavcanServers::cb_enumeration_indication(const uavcan::ReceivedDataStructure<uavcan::protocol::enumeration::Indication>
		&msg)
{
	// Called whenever an ESC thinks it has received user input.
	PX4_INFO("UAVCAN ESC enumeration: got indication");

	if (!_esc_enumeration_active) {
		// Ignore any messages received when we're not expecting them
		return;
	}

	// First, check if we've already seen an indication from this ESC. If so, just ignore this indication.
	int i = 0;

	for (; i < _esc_enumeration_index; i++) {
		if (_esc_enumeration_ids[i] == msg.getSrcNodeID().get()) {
			PX4_INFO("UAVCAN ESC enumeration: already enumerated ESC ID %hhu as index %d, ignored", _esc_enumeration_ids[i], i);
			return;
		}
	}

	uavcan::protocol::param::GetSet::Request req;
	req.name = msg.parameter_name;	// 'esc_index' or something alike, the name is not standardized
	req.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = i;

	int call_res = _enumeration_getset_client.call(msg.getSrcNodeID(), req);

	if (call_res < 0) {
		PX4_ERR("UAVCAN ESC enumeration: couldn't send GetSet: %d", call_res);

	} else {
		PX4_INFO("UAVCAN ESC enumeration: sent GetSet to node %hhu (index %d)", _esc_enumeration_ids[i], i);
	}
}

void
UavcanServers::cb_enumeration_getset(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result)
{
	if (!result.isSuccessful()) {
		PX4_ERR("UAVCAN ESC enumeration: save request for node %hhu timed out.", result.getCallID().server_node_id.get());

	} else {
		PX4_INFO("UAVCAN ESC enumeration: save request for node %hhu completed OK.", result.getCallID().server_node_id.get());

		uavcan::protocol::param::GetSet::Response resp = result.getResponse();
		uint8_t esc_index = (uint8_t)resp.value.to<uavcan::protocol::param::Value::Tag::integer_value>();
		esc_index = math::min((uint8_t)(uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize - 1), esc_index);
		_esc_enumeration_index = math::max(_esc_enumeration_index, (uint8_t)(esc_index + 1));

		_esc_enumeration_ids[esc_index] = result.getCallID().server_node_id.get();

		uavcan::protocol::param::ExecuteOpcode::Request opcode_req;
		opcode_req.opcode = opcode_req.OPCODE_SAVE;
		int call_res = _enumeration_save_client.call(result.getCallID().server_node_id, opcode_req);

		if (call_res < 0) {
			PX4_ERR("UAVCAN ESC enumeration: couldn't send ExecuteOpcode: %d", call_res);

		} else {
			PX4_INFO("UAVCAN ESC enumeration: sent ExecuteOpcode to node %hhu (index %hhu)", _esc_enumeration_ids[esc_index],
				 esc_index);
		}
	}
}

void
UavcanServers::cb_enumeration_save(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result)
{
	const bool this_is_the_last_one =
		(_esc_enumeration_index >= uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize - 1) ||
		(_esc_enumeration_index >= _esc_count);

	if (!result.isSuccessful()) {
		PX4_ERR("UAVCAN ESC enumeration: save request for node %hhu timed out.", result.getCallID().server_node_id.get());
		beep(BeepFrequencyError);

	} else if (!result.getResponse().ok) {
		PX4_ERR("UAVCAN ESC enumeration: save request for node %hhu rejected", result.getCallID().server_node_id.get());
		beep(BeepFrequencyError);

	} else {
		PX4_INFO("UAVCAN ESC enumeration: save request for node %hhu completed OK.", result.getCallID().server_node_id.get());
		beep(this_is_the_last_one ? BeepFrequencySuccess : BeepFrequencyGenericIndication);
	}

	PX4_INFO("UAVCAN ESC enumeration: completed %hhu of %hhu", _esc_enumeration_index, _esc_count);

	if (this_is_the_last_one) {
		_esc_enumeration_active = false;

		// Tell all ESCs to stop enumerating
		uavcan::protocol::enumeration::Begin::Request req;

		// TODO: Incorrect implementation; the parameter name field should be left empty.
		//       Leaving it as-is to avoid breaking compatibility with non-compliant nodes.
		req.parameter_name = "esc_index";
		req.timeout_sec = 0;
		int call_res = _enumeration_client.call(get_next_active_node_id(0), req);

		if (call_res < 0) {
			PX4_ERR("UAVCAN ESC enumeration: couldn't send Begin request to stop enumeration: %d", call_res);

		} else {
			PX4_INFO("UAVCAN ESC enumeration: sent Begin request to stop enumeration");
		}
	}
}


void
UavcanServers::migrateFWFromRoot(const char *sd_path, const char *sd_root_path)
{
	/*
	Copy Any bin files with APDes into appropriate location on SD card
	overriding any firmware the user has already loaded there.

	The SD firmware directory structure is along the lines of:

	  /fs/microsd/ufw
	     nnnnn.bin - where n is the board_id
	*/

	const size_t maxlen = UAVCAN_MAX_PATH_LENGTH;
	const size_t sd_root_path_len = strlen(sd_root_path);
	struct stat sb;
	int rv;
	char dstpath[maxlen + 1];
	char srcpath[maxlen + 1];

	DIR *const sd_root_dir = opendir(sd_root_path);

	if (!sd_root_dir) {
		return;
	}

	if (stat(sd_path, &sb) != 0 || !S_ISDIR(sb.st_mode)) {
		rv = mkdir(sd_path, S_IRWXU | S_IRWXG | S_IRWXO);

		if (rv != 0) {
			PX4_ERR("dev: couldn't create '%s'", sd_path);
			return;
		}
	}

	// Iterate over all bin files in root directory
	struct dirent *dev_dirent = NULL;

	while ((dev_dirent = readdir(sd_root_dir)) != nullptr) {

		uavcan_posix::FirmwareVersionChecker::AppDescriptor descriptor;

		// Looking for all uavcan.bin files.

		if (DIRENT_ISFILE(dev_dirent->d_type) && strstr(dev_dirent->d_name, ".bin") != nullptr) {

			// Make sure the path fits

			size_t filename_len = strlen(dev_dirent->d_name);
			size_t srcpath_len = sd_root_path_len + 1 + filename_len;

			if (srcpath_len > maxlen) {
				PX4_WARN("file: srcpath '%s%s' too long", sd_root_path, dev_dirent->d_name);
				continue;
			}

			snprintf(srcpath, sizeof(srcpath), "%s%s", sd_root_path, dev_dirent->d_name);

			if (uavcan_posix::FirmwareVersionChecker::getFileInfo(srcpath, descriptor, 1024) != 0) {
				continue;
			}

			if (descriptor.image_crc == 0) {
				continue;
			}

			snprintf(dstpath, sizeof(dstpath), "%s/%d.bin", sd_path, descriptor.board_id);

			if (copyFw(dstpath, srcpath) >= 0) {
				unlink(srcpath);
			}
		}
	}

	if (dev_dirent != nullptr) {
		(void)closedir(dev_dirent);
	}
}

int
UavcanServers::copyFw(const char *dst, const char *src)
{
	int rv = 0;
	uint8_t buffer[512] {};

	int dfd = open(dst, O_WRONLY | O_CREAT, 0666);

	if (dfd < 0) {
		PX4_ERR("copyFw: couldn't open dst");
		return -errno;
	}

	int sfd = open(src, O_RDONLY, 0);

	if (sfd < 0) {
		(void)close(dfd);
		PX4_ERR("copyFw: couldn't open src");
		return -errno;
	}

	ssize_t size = 0;

	do {
		size = read(sfd, buffer, sizeof(buffer));

		if (size < 0) {
			PX4_ERR("copyFw: couldn't read");
			rv = -errno;

		} else if (size > 0) {
			rv = 0;
			ssize_t remaining = size;
			ssize_t total_written = 0;
			ssize_t written = 0;

			do {
				written = write(dfd, &buffer[total_written], remaining);

				if (written < 0) {
					PX4_ERR("copyFw: couldn't write");
					rv = -errno;

				} else {
					total_written += written;
					remaining -=  written;
				}
			} while (written > 0 && remaining > 0);
		}
	} while (rv == 0 && size != 0);

	(void)close(dfd);
	(void)close(sfd);

	return rv;
}
