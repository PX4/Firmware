/****************************************************************************
 * examples/canard/canard_main.c
 *
 *   Copyright (C) 2016 ETH Zuerich. All rights reserved.
 *   Author: Matthias Renner <rennerm@ethz.ch>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <canard.h>
#include <canard_dsdl.h>

#include <sched.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <net/if.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <poll.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include "socketcan.h"
#include "o1heap.h"
#include "uorb_converter.h"

#include "libcancl/pnp.h"
#include "libcancl/registerinterface.h"

#include "boot_app_shared.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Arena for memory allocation, used by the library */
#define O1_HEAP_SIZE CONFIG_EXAMPLES_LIBCANARDV1_NODE_MEM_POOL_SIZE
#define UNIQUE_ID_LENGTH_BYTES                   16

/****************************************************************************
 * Private Data
 ****************************************************************************/
/*
 * This is the AppImageDescriptor used
 * by the make_can_boot_descriptor.py tool to set
 * the application image's descriptor so that the
 * uavcan bootloader has the ability to validate the
 * image crc, size etc of this application
*/
boot_app_shared_section app_descriptor_t AppDescriptor = {
	.signature = APP_DESCRIPTOR_SIGNATURE,
	.image_crc = 0,
	.image_size = 0,
	.git_hash  = 0,
	.major_version = APP_VERSION_MAJOR,
	.minor_version = APP_VERSION_MINOR,
	.board_id = HW_VERSION_MAJOR << 8 | HW_VERSION_MINOR,
	.reserved = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};

O1HeapInstance *my_allocator;
static uint8_t uavcan_heap[O1_HEAP_SIZE]
__attribute__((aligned(O1HEAP_ALIGNMENT)));

static uint8_t unique_id[UNIQUE_ID_LENGTH_BYTES] = { //FIXME real HW ID
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01
};

/* Node status variables */

static bool g_canard_daemon_started;

static int16_t gps_uorb_port_id = -1;
static int16_t gps_fix_port_id = -1;
static int16_t gps_aux_port_id = -1;

struct pollfd fd;

/****************************************************************************
 * private functions
 ****************************************************************************/


//TODO move this to a seperate file probably

int32_t set_gps_uorb_port_id(uavcan_register_Value_1_0 *value)
{
	if (uavcan_register_Value_1_0_is_natural16_(value) && value->natural16.value.count == 1) { // Natural 16
		//TODO check validity
		printf("Master: set uORB portID to %i\n", value->natural16.value.elements[0]);
		gps_uorb_port_id = value->natural16.value.elements[0];
		return 0;
	}

	return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
}

uavcan_register_Value_1_0 get_gps_uorb_port_id()
{
	void *dataReturn;
	uavcan_register_Value_1_0 value;

	value.natural16.value.elements[0] = gps_uorb_port_id;
	value.natural16.value.count = 1;
	value._tag_ = 10; // TODO does nunavut generate ENUM/defines for this??
	return value;
}

int32_t set_gps_fix_port_id(uavcan_register_Value_1_0 *value)
{
	if (uavcan_register_Value_1_0_is_natural16_(value) && value->natural16.value.count == 1) { // Natural 16
		//TODO check validity
		printf("Master: set FIX portID to %i\n", value->natural16.value.elements[0]);
		gps_fix_port_id = value->natural16.value.elements[0];
		return 0;
	}

	return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
}

uavcan_register_Value_1_0 get_gps_fix_port_id()
{
	void *dataReturn;
	uavcan_register_Value_1_0 value;

	value.natural16.value.elements[0] = gps_fix_port_id;
	value.natural16.value.count = 1;
	value._tag_ = 10; // TODO does nunavut generate ENUM/defines for this??
	return value;
}

int32_t set_gps_aux_port_id(uavcan_register_Value_1_0 *value)
{
	if (uavcan_register_Value_1_0_is_natural16_(value) && value->natural16.value.count == 1) { // Natural 16
		//TODO check validity
		printf("Master: set AUX portID to %i\n", value->natural16.value.elements[0]);
		gps_fix_port_id = value->natural16.value.elements[0];
		return 0;
	}

	return -UAVCAN_REGISTER_ERROR_SERIALIZATION;
}

uavcan_register_Value_1_0 get_gps_aux_port_id()
{
	void *dataReturn;
	uavcan_register_Value_1_0 value;

	value.natural16.value.elements[0] = gps_aux_port_id;
	value.natural16.value.count = 1;
	value._tag_ = 10; // TODO does nunavut generate ENUM/defines for this??
	return value;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memAllocate
 *
 * Description:
 *
 ****************************************************************************/
static void *memAllocate(CanardInstance *const ins, const size_t amount)
{
	(void) ins;
	return o1heapAllocate(my_allocator, amount);
}

/****************************************************************************
 * Name: memFree
 *
 * Description:
 *
 ****************************************************************************/

static void memFree(CanardInstance *const ins, void *const pointer)
{
	(void) ins;
	o1heapFree(my_allocator, pointer);
}

/****************************************************************************
 * Name: getMonotonicTimestampUSec
 *
 * Description:
 *
 ****************************************************************************/


uint64_t getMonotonicTimestampUSec(void)
{
	int ret;
	struct timespec ts;

	memset(&ts, 0, sizeof(ts));

	ret = clock_gettime(CLOCK_MONOTONIC, &ts);

	if (ret != 0) {
		PX4_ERR("clock error %i", ret);
		abort();
	}

	return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

/****************************************************************************
 * Name: processTxRxOnce
 *
 * Description:
 *   Transmits all frames from the TX queue, receives up to one frame.
 *
 ****************************************************************************/

void processTxRxOnce(CanardInstance *ins, CanardSocketInstance *sock_ins, int timeout_msec)
{
	int32_t result;

	/* Transmitting */


	for (const CanardFrame *txf = NULL; (txf = canardTxPeek(ins)) != NULL;) { // Look at the top of the TX queue.
		if (txf->timestamp_usec > getMonotonicTimestampUSec()) { // Check if the frame has timed out.
			if (socketcanTransmit(sock_ins, txf) == 0) {           // Send the frame. Redundant interfaces may be used here.
				break;                             // If the driver is busy, break and retry later.
			}

		} else {
			printf("Timeout??\n");
		}

		canardTxPop(ins);                         // Remove the frame from the queue after it's transmitted.
		ins->memory_free(ins, (CanardFrame *)txf); // Deallocate the dynamic memory afterwards.
	}


	/* Poll receive */
	if (poll(&fd, 1, timeout_msec) <= 0) {
		return;
	}

	/* Receiving */
	CanardFrame received_frame;

	socketcanReceive(sock_ins, &received_frame);

	CanardTransfer receive;
	result = canardRxAccept(ins,
				&received_frame, // The CAN frame received from the bus.
				0,               // If the transport is not redundant, use 0.
				&receive);

	if (result < 0) {
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if
		// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
		// Reception of an invalid frame is NOT an error.
		fprintf(stderr, "Receive error %d\n", result);

	} else if (result == 1) {
		printf("Receive portId %i\n", receive.port_id);

		if (receive.port_id == PNPGetPortID(ins)) {
			PNPProcess(ins, &receive);

		} else {
			uavcan_register_interface_process(ins, &receive);
		}

		ins->memory_free(ins, (void *)receive.payload); // Deallocate the dynamic memory afterwards.

	} else {
		// printf("RX canard %d\r\n", result);
		// Nothing to do.
		// The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
		// Reception of an invalid frame is NOT reported as an error because it is not an error.
	}

}

/****************************************************************************
 * Name: canard_daemon
 *
 * Description:
 *
 ****************************************************************************/

static int canard_daemon(int argc, char *argv[])
{
	int errval = 0;
	int can_fd = 0;

	if (argc > 2) {
		for (int args = 2; args < argc; args++) {
			if (!strcmp(argv[args], "canfd")) {
				can_fd = 1;
			}
		}
	}

	my_allocator = o1heapInit(&uavcan_heap, O1_HEAP_SIZE, NULL, NULL);

	if (my_allocator == NULL) {
		printf("o1heapInit failed with size %d\n", O1_HEAP_SIZE);
		errval = 2;
		goto errout_with_dev;
	}

	CanardInstance ins = canardInit(&memAllocate, &memFree);

	if (can_fd) {
		ins.mtu_bytes = CANARD_MTU_CAN_FD;

	} else {
		ins.mtu_bytes = CANARD_MTU_CAN_CLASSIC;
	}

	/* Open the CAN device for reading */
	CanardSocketInstance sock_ins;
	socketcanOpen(&sock_ins, CONFIG_EXAMPLES_LIBCANARDV1_DEV, can_fd);

	/* setup poll fd */
	fd.fd = sock_ins.s;
	fd.events = POLLIN;

	if (sock_ins.s < 0) {
		printf("canard_daemon: ERROR: open %s failed: %d\n",
		       CONFIG_EXAMPLES_LIBCANARDV1_DEV, errno);
		errval = 2;
		goto errout_with_dev;
	}

	/* libcancl functions */

	/* Dynamic NodeId */

	/* Init UAVCAN register interfaces */
	uavcan_node_GetInfo_Response_1_0 node_information; // TODO ADD INFO
	uavcan_register_interface_init(&ins, &node_information);
	uavcan_register_interface_add_entry("gnss_uorb", set_gps_uorb_port_id, get_gps_uorb_port_id);
	uavcan_register_interface_add_entry("gnss_fix", set_gps_fix_port_id, get_gps_fix_port_id);
	uavcan_register_interface_add_entry("gnss_aux", set_gps_aux_port_id, get_gps_aux_port_id);

	initPNPAllocatee(&ins, unique_id);

	uint32_t random_no;
	random_no = ((float)rand() / RAND_MAX) * (1000000);

	uint64_t next_alloc_req = getMonotonicTimestampUSec() + random_no;

	while (ins.node_id == CANARD_NODE_ID_UNSET) {
		// process the TX and RX buffer
		processTxRxOnce(&ins, &sock_ins, 10); //10Ms

		const uint64_t ts = getMonotonicTimestampUSec();

		if (ts >= next_alloc_req) {
			next_alloc_req += ((float)rand() / RAND_MAX) * (1000000);
			int32_t result = PNPAllocRequest(&ins);

			if (result) {
				ins.node_id = PNPGetNodeID();
			}
		}
	}

	printf("canard_daemon: canard initialized\n");
	printf("start node (ID: %d MTU: %d)\n", ins.node_id,
	       ins.mtu_bytes);

	/* Initialize uORB publishers & subscribers */
	uorbConverterInit(&ins, &gps_uorb_port_id, &gps_fix_port_id, &gps_aux_port_id);

	g_canard_daemon_started = true;
	uint64_t next_1hz_service_at = getMonotonicTimestampUSec();

	for (;;) {
		processTxRxOnce(&ins, &sock_ins, 10);

		uorbProcessSub(10);

	}

errout_with_dev:

	g_canard_daemon_started = false;
	printf("canard_daemon: Terminating!\n");
	fflush(stdout);
	return errval;
}


/****************************************************************************
 * Name: canard_main
 *
 * Description:
 *
 ****************************************************************************/

int uavcannode_gps_demo_main(int argc, FAR char *argv[])
{
	int ret;

	printf("canard_main: Starting canard_daemon\n");

	if (g_canard_daemon_started) {
		printf("canard_main: receive and send task already running\n");
		return EXIT_SUCCESS;
	}

	ret = task_create("canard_daemon", CONFIG_EXAMPLES_LIBCANARDV1_DAEMON_PRIORITY,
			  CONFIG_EXAMPLES_LIBCANARDV1_DAEMON_STACK_SIZE, canard_daemon,
			  argv);

	if (ret < 0) {
		int errcode = errno;
		printf("canard_main: ERROR: Failed to start canard_daemon: %d\n",
		       errcode);
		return EXIT_FAILURE;
	}

	printf("canard_main: canard_daemon started\n");
	return EXIT_SUCCESS;
}
