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

#include "Uavcan.hpp"

#include <lib/ecl/geo/geo.h>
#include <lib/version/version.h>

using namespace time_literals;

UavcanNode *UavcanNode::_instance;

O1HeapInstance *uavcan_allocator{nullptr};

static void *memAllocate(CanardInstance *const ins, const size_t amount) { return o1heapAllocate(uavcan_allocator, amount); }
static void memFree(CanardInstance *const ins, void *const pointer) { o1heapFree(uavcan_allocator, pointer); }

#if defined(__PX4_NUTTX)
# if defined(CONFIG_NET_CAN)
#  include "CanardSocketCAN.hpp"
# elif defined(CONFIG_CAN)
#  include "CanardNuttXCDev.hpp"
# endif // CONFIG_CAN
#endif // NuttX

UavcanNode::UavcanNode(CanardInterface *interface, uint32_t node_id) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_can_interface(interface)
{
	pthread_mutex_init(&_node_mutex, nullptr);

	_uavcan_heap = memalign(O1HEAP_ALIGNMENT, HeapSize);
	uavcan_allocator = o1heapInit(_uavcan_heap, HeapSize, nullptr, nullptr);

	if (uavcan_allocator == nullptr) {
		PX4_ERR("o1heapInit failed with size %u", HeapSize);
	}

	_canard_instance = canardInit(&memAllocate, &memFree);

	_canard_instance.node_id = node_id; // Defaults to anonymous; can be set up later at any point.

	bool can_fd = false;

	if (can_fd) {
		_canard_instance.mtu_bytes = CANARD_MTU_CAN_FD;

	} else {
		_canard_instance.mtu_bytes = CANARD_MTU_CAN_CLASSIC;
	}

	_node_manager.subscribe();

	for (auto &publisher : _publishers) {
		publisher->updateParam();
	}

	_sub_manager.subscribe();

	_mixing_output.mixingOutput().updateSubscriptions(false, false);
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

	delete _can_interface;

	perf_free(_cycle_perf);
	perf_free(_interval_perf);

	//delete _uavcan_heap;
}

int UavcanNode::start(uint32_t node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

#if defined(__PX4_NUTTX)
# if defined(CONFIG_NET_CAN)
	CanardInterface *interface = new CanardSocketCAN();
# elif defined(CONFIG_CAN)
	CanardInterface *interface = new CanardNuttXCDev();
# endif // CONFIG_CAN
#endif // NuttX

	_instance = new UavcanNode(interface, node_id);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	// Keep the bit rate for reboots on BenginFirmware updates
	_instance->active_bitrate = bitrate;

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

	return PX4_OK;
}

void UavcanNode::init()
{
	// interface init
	if (_can_interface) {
		if (_can_interface->init() == PX4_OK) {
			_initialized = true;
		}
	}
}

void UavcanNode::Run()
{
	pthread_mutex_lock(&_node_mutex);

	if (!_initialized) {
		init();

		// return early if still not initialized
		if (!_initialized) {
			pthread_mutex_unlock(&_node_mutex);
			return;
		}
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		for (auto &publisher : _publishers) {
			// Have the publisher update its associated port-id parameter
			// Setting to 0 disable publication
			publisher->updateParam();
		}

		_sub_manager.updateParams();

		_mixing_output.updateParams();

		_mixing_output.mixingOutput().updateSubscriptions(false, false);
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// send uavcan::node::Heartbeat_1_0 @ 1 Hz
	sendHeartbeat();

	// Check all publishers
	for (auto &publisher : _publishers) {
		publisher->update();
	}

	_node_manager.update();

	transmit();

	/* Process received messages */

	uint8_t data[64] {};
	CanardFrame received_frame{};
	received_frame.payload = &data;

	while (_can_interface->receive(&received_frame) > 0) {
		CanardTransfer receive{};
		CanardRxSubscription *subscription = NULL;
		int32_t result = canardRxAccept2(&_canard_instance, &received_frame, 0, &receive, &subscription);

		if (result < 0) {
			// An error has occurred: either an argument is invalid or we've ran out of memory.
			// It is possible to statically prove that an out-of-memory will never occur for a given application if
			// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
			// Reception of an invalid frame is NOT an error.
			PX4_ERR("Receive error %" PRId32" \n", result);

		} else if (result == 1) {
			// A transfer has been received, process it.
			// PX4_INFO("received Port ID: %d", receive.port_id);

			if (subscription != NULL) {
				UavcanBaseSubscriber *sub_instance = (UavcanBaseSubscriber *)subscription->user_reference;
				sub_instance->callback(receive);

			} else {
				PX4_ERR("No matching sub for %d", receive.port_id);
			}

			// Deallocate the dynamic memory afterwards.
			_canard_instance.memory_free(&_canard_instance, (void *)receive.payload);

		} else {
			//PX4_INFO("RX canard %d", result);
		}
	}

	// Pop canardTx queue to send out responses to requets
	transmit();

	perf_end(_cycle_perf);

	if (_task_should_exit.load()) {
		_can_interface->close();

		ScheduleClear();
		_instance = nullptr;
	}

	pthread_mutex_unlock(&_node_mutex);
}

void UavcanNode::transmit()
{
	// Look at the top of the TX queue.
	for (const CanardFrame *txf = nullptr; (txf = canardTxPeek(&_canard_instance)) != nullptr;) {
		// Attempt transmission only if the frame is not yet timed out while waiting in the TX queue.
		// Otherwise just drop it and move on to the next one.
		const hrt_abstime now = hrt_absolute_time();

		if (txf->timestamp_usec == 0 || txf->timestamp_usec > now) {
			// Send the frame. Redundant interfaces may be used here.
			const int tx_res = _can_interface->transmit(*txf);

			if (tx_res < 0) {
				// Failure - drop the frame and report
				canardTxPop(&_canard_instance);

				// Deallocate the dynamic memory afterwards.
				_canard_instance.memory_free(&_canard_instance, (CanardFrame *)txf);
				PX4_ERR("Transmit error %d, frame dropped, errno '%s'", tx_res, strerror(errno));

			} else if (tx_res > 0) {
				// Success - just drop the frame
				canardTxPop(&_canard_instance);

				// Deallocate the dynamic memory afterwards.
				_canard_instance.memory_free(&_canard_instance, (CanardFrame *)txf);

			} else {
				// Timeout - just exit and try again later
				break;
			}

		} else if (txf->timestamp_usec <= now) {
			// Transmission timed out -- remove from queue and deallocate its memory
			canardTxPop(&_canard_instance);

			_canard_instance.memory_free(&_canard_instance, (CanardFrame *)txf);
		}
	}
}

void UavcanNode::print_info()
{
	pthread_mutex_lock(&_node_mutex);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	O1HeapDiagnostics heap_diagnostics = o1heapGetDiagnostics(uavcan_allocator);

	PX4_INFO("Heap status %zu/%zu Peak alloc %zu Peak req %zu OOM count %" PRIu64,
		 heap_diagnostics.allocated, heap_diagnostics.capacity,
		 heap_diagnostics.peak_allocated, heap_diagnostics.peak_request_size,
		 heap_diagnostics.oom_count);

	for (auto &publisher : _publishers) {
		publisher->printInfo();
	}

	CanardRxSubscription *rxs = _canard_instance.rx_subscriptions[CanardTransferKindMessage];

	while (rxs != NULL) {
		if (rxs->user_reference == NULL) {
			PX4_INFO("Message port id %d", rxs->port_id);

		} else {
			((UavcanBaseSubscriber *)rxs->user_reference)->printInfo();
		}

		rxs = rxs->next;
	}

	rxs = _canard_instance.rx_subscriptions[CanardTransferKindRequest];

	while (rxs != NULL) {
		if (rxs->user_reference == NULL) {
			PX4_INFO("Service response port id %d", rxs->port_id);

		} else {
			((UavcanBaseSubscriber *)rxs->user_reference)->printInfo();
		}

		rxs = rxs->next;
	}

	rxs = _canard_instance.rx_subscriptions[CanardTransferKindResponse];

	while (rxs != NULL) {
		if (rxs->user_reference == NULL) {
			PX4_INFO("Service request port id %d", rxs->port_id);

		} else {
			((UavcanBaseSubscriber *)rxs->user_reference)->printInfo();
		}

		rxs = rxs->next;
	}

	_mixing_output.printInfo();

	pthread_mutex_unlock(&_node_mutex);
}

static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tuavcannode {start|status|stop}");
}

extern "C" __EXPORT int uavcan_v1_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (UavcanNode::instance()) {
			PX4_ERR("already started");
			return 1;
		}

		// CAN bitrate
		int32_t bitrate = 0;
		param_get(param_find("UAVCAN_V1_BAUD"), &bitrate);

		// Node ID
		int32_t node_id = 0;
		param_get(param_find("UAVCAN_V1_ID"), &node_id);

		// Start
		PX4_INFO("Node ID %" PRIu32 ", bitrate %" PRIu32, node_id, bitrate);
		return UavcanNode::start(node_id, bitrate);
	}

	/* commands below require the app to be started */
	UavcanNode *const inst = UavcanNode::instance();

	if (!inst) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!strcmp(argv[1], "status") || !strcmp(argv[1], "info")) {
		inst->print_info();
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}

void UavcanNode::sendHeartbeat()
{
	if (hrt_elapsed_time(&_uavcan_node_heartbeat_last) >= 1_s) {

		uavcan_node_Heartbeat_1_0 heartbeat{};
		heartbeat.uptime = _uavcan_node_heartbeat_transfer_id; // TODO: use real uptime
		heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
		heartbeat.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
		const hrt_abstime now = hrt_absolute_time();

		CanardTransfer transfer = {
			.timestamp_usec = now + PUBLISHER_DEFAULT_TIMEOUT_USEC,
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindMessage,
			.port_id        = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
			.remote_node_id = CANARD_NODE_ID_UNSET,
			.transfer_id    = _uavcan_node_heartbeat_transfer_id++,
			.payload_size   = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
			.payload        = &_uavcan_node_heartbeat_buffer,
		};

		uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, _uavcan_node_heartbeat_buffer, &transfer.payload_size);

		int32_t result = canardTxPush(&_canard_instance, &transfer);

		if (result < 0) {
			// An error has occurred: either an argument is invalid or we've ran out of memory.
			// It is possible to statically prove that an out-of-memory will never occur for a given application if the
			// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
			PX4_ERR("Heartbeat transmit error %" PRId32 "", result);
		}

		_uavcan_node_heartbeat_last = now;
	}
}

bool UavcanMixingInterface::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	// Note: This gets called from MixingOutput from within its update() function
	// Hence, the mutex lock in UavcanMixingInterface::Run() is in effect

	/// TODO: Need esc/servo metadata / bitmask(s)
	_esc_controller.update_outputs(stop_motors, outputs, num_outputs);
	// _servo_controller.update_outputs(stop_motors, outputs, num_outputs);

	return true;
}

void UavcanMixingInterface::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false, false);
	pthread_mutex_unlock(&_node_mutex);
}
