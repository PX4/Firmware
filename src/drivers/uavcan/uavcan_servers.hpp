/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
#include "uavcan_driver.hpp"
#include <drivers/device/Device.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/uavcan_parameter_value.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <uavcan/node/sub_node.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>

#include <uavcan/protocol/dynamic_node_id_server/centralized.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>
#include <uavcan_posix/basic_file_server_backend.hpp>
#include <uavcan/protocol/firmware_update_trigger.hpp>
#include <uavcan/protocol/file_server.hpp>
#include <uavcan_posix/dynamic_node_id_server/file_event_tracer.hpp>
#include <uavcan_posix/dynamic_node_id_server/file_storage_backend.hpp>
#include <uavcan_posix/firmware_version_checker.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/indication/BeepCommand.hpp>
#include <uavcan/protocol/enumeration/Begin.hpp>
#include <uavcan/protocol/enumeration/Indication.hpp>

#include "uavcan_module.hpp"
#include "uavcan_virtual_can_driver.hpp"

/**
 * @file uavcan_servers.hpp
 *
 * Defines basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author David Sidrane <david_s5@nscdg.com>
 */

/**
 * A UAVCAN Server Sub node.
 */
class __EXPORT UavcanServers
{
	static constexpr unsigned NumIfaces = 1;  // UAVCAN_STM32_NUM_IFACES

	static constexpr unsigned StackSize  = 6000;
	static constexpr unsigned Priority  =  120;

	static constexpr unsigned VirtualIfaceBlockAllocationQuota =  80;

	static constexpr float BeepFrequencyGenericIndication       = 1000.0F;
	static constexpr float BeepFrequencySuccess                 = 2000.0F;
	static constexpr float BeepFrequencyError                   = 100.0F;

public:
	UavcanServers(uavcan::INode &main_node);

	virtual		~UavcanServers();

	static int      start(uavcan::INode &main_node);
	static int      stop();

	uavcan::SubNode<> &get_node() { return _subnode; }

	static UavcanServers *instance() { return _instance; }

	/*
	 *  Set the main node's pointer to to the injector
	 *  This is a work around as main_node.getDispatcher().remeveRxFrameListener();
	 *  would require a dynamic cast and rtti is not enabled.
	 */
	void attachITxQueueInjector(ITxQueueInjector **injector) {*injector = &_vdriver;}

	void requestCheckAllNodesFirmwareAndUpdate() { _check_fw = true; }

	bool guessIfAllDynamicNodesAreAllocated() { return _server_instance.guessIfAllDynamicNodesAreAllocated(); }

private:
	pthread_t         _subnode_thread{-1};
	pthread_mutex_t   _subnode_mutex{};
	px4::atomic_bool  _subnode_thread_should_exit{false};

	int		init();

	pthread_addr_t	run(pthread_addr_t);

	static UavcanServers	*_instance;            ///< singleton pointer

	VirtualCanDriver _vdriver;

	uavcan::SubNode<>  _subnode;
	uavcan::INode      &_main_node;

	uavcan_posix::dynamic_node_id_server::FileEventTracer _tracer;
	uavcan_posix::dynamic_node_id_server::FileStorageBackend _storage_backend;
	uavcan_posix::FirmwareVersionChecker _fw_version_checker;
	uavcan::dynamic_node_id_server::CentralizedServer _server_instance;  ///< server singleton pointer
	uavcan_posix::BasicFileServerBackend  _fileserver_backend;
	uavcan::NodeInfoRetriever   _node_info_retriever;
	uavcan::FirmwareUpdateTrigger   _fw_upgrade_trigger;
	uavcan::BasicFileServer         _fw_server;

	/*
	 * The MAVLink parameter bridge needs to know the maximum parameter index
	 * of each node so that clients can determine when parameter listings have
	 * finished. We do that by querying a node's entire parameter set whenever
	 * a parameter message is received for a node with a zero _param_count,
	 * and storing the count here. If a node doesn't respond, or doesn't have
	 * any parameters, its count will stay at zero and we'll try to query the
	 * set again next time.
	 *
	 * The node's UAVCAN ID is used as the index into the _param_counts array.
	 */
	uint8_t _param_counts[128] = {};
	bool _count_in_progress = false;
	uint8_t _count_index = 0;

	bool _param_in_progress = false;
	uint8_t _param_index = 0;
	bool _param_list_in_progress = false;
	bool _param_list_all_nodes = false;
	uint8_t _param_list_node_id = 1;

	uint32_t _param_dirty_bitmap[4] = {};
	uint8_t _param_save_opcode = 0;

	bool _cmd_in_progress = false;

	// uORB topic handle for MAVLink parameter responses
	uORB::Publication<uavcan_parameter_value_s> _param_response_pub{ORB_ID(uavcan_parameter_value)};
	uORB::Publication<vehicle_command_ack_s>	_command_ack_pub{ORB_ID(vehicle_command_ack)};

	typedef uavcan::MethodBinder<UavcanServers *,
		void (UavcanServers::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &)> GetSetCallback;
	typedef uavcan::MethodBinder<UavcanServers *,
		void (UavcanServers::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &)>
		ExecuteOpcodeCallback;
	typedef uavcan::MethodBinder<UavcanServers *,
		void (UavcanServers::*)(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &)> RestartNodeCallback;

	void cb_getset(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);
	void cb_count(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);
	void cb_opcode(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result);
	void cb_restart(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &result);

	uavcan::ServiceClient<uavcan::protocol::param::GetSet, GetSetCallback> _param_getset_client;
	uavcan::ServiceClient<uavcan::protocol::param::ExecuteOpcode, ExecuteOpcodeCallback> _param_opcode_client;
	uavcan::ServiceClient<uavcan::protocol::RestartNode, RestartNodeCallback> _param_restartnode_client;
	void param_count(uavcan::NodeID node_id);
	void param_opcode(uavcan::NodeID node_id);

	uint8_t get_next_active_node_id(uint8_t base);
	uint8_t get_next_dirty_node_id(uint8_t base);
	void set_node_params_dirty(uint8_t node_id) { _param_dirty_bitmap[node_id >> 5] |= 1 << (node_id & 31); }
	void clear_node_params_dirty(uint8_t node_id) { _param_dirty_bitmap[node_id >> 5] &= ~(1 << (node_id & 31)); }
	bool are_node_params_dirty(uint8_t node_id) const { return bool((_param_dirty_bitmap[node_id >> 5] >> (node_id & 31)) & 1); }

	void beep(float frequency);

	bool _mutex_inited = false;
	volatile bool _check_fw = false;

	// ESC enumeration
	bool _esc_enumeration_active = false;
	uint8_t _esc_enumeration_ids[uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize];
	uint8_t _esc_enumeration_index = 0;
	uint8_t _esc_count = 0;

	typedef uavcan::MethodBinder<UavcanServers *,
		void (UavcanServers::*)(const uavcan::ServiceCallResult<uavcan::protocol::enumeration::Begin> &)>
		EnumerationBeginCallback;
	typedef uavcan::MethodBinder<UavcanServers *,
		void (UavcanServers::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::enumeration::Indication>&)>
		EnumerationIndicationCallback;
	void cb_enumeration_begin(const uavcan::ServiceCallResult<uavcan::protocol::enumeration::Begin> &result);
	void cb_enumeration_indication(const uavcan::ReceivedDataStructure<uavcan::protocol::enumeration::Indication> &msg);
	void cb_enumeration_getset(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);
	void cb_enumeration_save(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result);

	uavcan::Publisher<uavcan::equipment::indication::BeepCommand> _beep_pub;
	uavcan::Subscriber<uavcan::protocol::enumeration::Indication, EnumerationIndicationCallback>
	_enumeration_indication_sub;
	uavcan::ServiceClient<uavcan::protocol::enumeration::Begin, EnumerationBeginCallback> _enumeration_client;
	uavcan::ServiceClient<uavcan::protocol::param::GetSet, GetSetCallback> _enumeration_getset_client;
	uavcan::ServiceClient<uavcan::protocol::param::ExecuteOpcode, ExecuteOpcodeCallback> _enumeration_save_client;

	void unpackFwFromROMFS(const char *sd_path, const char *romfs_path);
	void migrateFWFromRoot(const char *sd_path, const char *sd_root_path);
	int copyFw(const char *dst, const char *src);
};
