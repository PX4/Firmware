/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

#include <nuttx/config.h>
#include "boot_config.h"

#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "chip.h"
#include "nvic.h"

#include "board.h"
#include "flash.h"
#include "timer.h"
#include "blsched.h"
#include "can.h"
#include "uavcan.h"
#include "random.h"

#include <drivers/bootloaders/boot_app_shared.h>
#include <drivers/bootloaders/boot_alt_app_shared.h>
#include <drivers/drv_watchdog.h>
#include <lib/systemlib/crc.h>

//#define DEBUG_APPLICATION_INPLACE    1 /* Never leave defined */
#define DEBUG_NO_FW_UPDATE           1 /* With DEBUG_APPLICATION_INPLACE
                                        * prevents fw update
                                        */

/* Using 2 character text in  LogMessage */

#define LOGMESSAGE_STAGE_INIT     'I'
#define LOGMESSAGE_STAGE_GET_INFO 'G'
#define LOGMESSAGE_STAGE_ERASE    'E'
#define LOGMESSAGE_STAGE_READ     'R'
#define LOGMESSAGE_STAGE_PROGRAM  'P'
#define LOGMESSAGE_STAGE_VALIDATE 'V'
#define LOGMESSAGE_STAGE_FINALIZE 'F'

#define LOGMESSAGE_RESULT_START   's'
#define LOGMESSAGE_RESULT_FAIL    'f'
#define LOGMESSAGE_RESULT_OK      'o'

#if defined(DEBUG_APPLICATION_INPLACE)
#pragma message "******** DANGER DEBUG_APPLICATION_INPLACE is DEFINED ******"
#endif

typedef volatile struct bootloader_t {
	can_speed_t bus_speed;
	volatile uint8_t health;
	volatile uint8_t mode;
	volatile uint8_t sub_mode;
	volatile bool app_valid;
	volatile uint32_t uptime;
	volatile app_descriptor_t *fw_image_descriptor;
	volatile uint32_t *fw_image;
	bool  wait_for_getnodeinfo;
	bool  app_bl_request;
	bool sent_node_info_response;
	uint16_t percentage_done;
	union {
		uint32_t l;
		uint8_t b[sizeof(uint32_t)];
	} fw_word0[LATER_FLAHSED_WORDS];

} bootloader_t;

bootloader_t bootloader;

const uint8_t debug_log_source[uavcan_byte_count(LogMessage, source)] = {'B', 'o', 'o', 't'};

/****************************************************************************
 * Name: early_start_the_watch_dog
 *
 * Description:
 *   This function will start the hardware watchdog. Once stated the code must
 *   kick it before it time out a reboot will occur.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void early_start_the_watch_dog(void)
{
#ifdef OPT_ENABLE_WD
#endif
}

/****************************************************************************
 * Name: app_start_the_watch_dog
 *
 * Description:
 *   This function will start the hardware watchdog. Once stated the code must
 *   kick it before it time out a reboot will occur.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void app_start_the_watch_dog(void)
{
	watchdog_init();
	watchdog_pet();
}
/****************************************************************************
 * Name: kick_the_watch_dog
 *
 * Description:
 *   This function will reset the watchdog timeout
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void kick_the_watch_dog(void)
{
#ifdef OPT_ENABLE_WD
#endif
}

/****************************************************************************
 * Name: uptime_process
 *
 * Description:
 *   Run as a timer callback off the system tick ISR. This function counts
 *   Seconds of up time.
 *
 * Input Parameters:
 *   Not Used.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void uptime_process(bl_timer_id id, void *context)
{
	bootloader.uptime++;
}

/****************************************************************************
 * Name: node_info_process
 *
 * Description:
 *   Run as a timer callback off the system tick ISR.
 *   Once we have a node id, this process is started and handles the
 *   GetNodeInfo replies.  This function uses the fifoGetNodeInfo and
 *   MBGetNodeInfo of the CAN so it can coexist regardless of
 *   application state and what is it doing.
 *
 * Input Parameters:
 *   Not Used.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void node_info_process(bl_timer_id id, void *context)
{
	uavcan_GetNodeInfo_response_t response;
	uavcan_GetNodeInfo_request_t  request;

	uavcan_protocol_t protocol;

	response.nodes_status.uptime_sec = bootloader.uptime;
	response.nodes_status.u8 = uavcan_pack(bootloader.sub_mode, NodeStatus, sub_mode)
				   | uavcan_pack(bootloader.mode, NodeStatus, mode)
				   | uavcan_pack(bootloader.health, NodeStatus, health);
	response.nodes_status.vendor_specific_status_code = 0u;

	(void)board_get_hardware_version(&response.hardware_version);
	response.name_length = board_get_product_name(response.name, sizeof(response.name));

	memset(&response.software_version, 0, sizeof(response.software_version));

	if (bootloader.app_valid) {

		response.software_version.major =
			bootloader.fw_image_descriptor->major_version;

		response.software_version.minor =
			bootloader.fw_image_descriptor->minor_version;

		response.software_version.vcs_commit =
			bootloader.fw_image_descriptor->git_hash;

		response.software_version.image_crc =
			bootloader.fw_image_descriptor->image_crc;

		response.software_version.optional_field_flags = OPTIONAL_FIELD_FLAG_IMAGE_CRC | OPTIONAL_FIELD_FLAG_VCS_COMMIT;
	}

	size_t length = sizeof(uavcan_GetNodeInfo_request_t);
	size_t send_length = uavcan_pack_GetNodeInfo_response(&response);

	/*
	 * Do a passive receive attempt on the GetNodeInfo fifo
	 * If it matches send the GetNodeInfo response
	 */

	protocol.id.u32 = ANY_NODE_ID;

	if (UavcanOk == uavcan_rx_dsdl(DSDLReqGetNodeInfo, &protocol, (uint8_t *) &request, &length, 0)) {
		if (UavcanOk == uavcan_tx_dsdl(DSDLRspGetNodeInfo, &protocol, (const uint8_t *) &response, send_length)) {
			bootloader.sent_node_info_response = true;
		}
	}
}

/****************************************************************************
 * Name: node_status_process
 *
 * Description:
 *   Run as a timer callback off the system tick ISR.
 *   Once we have a node id, this process is started and sends the
 *   NodeStatus messages.  This function uses the MBNodeStatus
 *   of the CAN so it can coexist regardless of application state ant
 *   what is it doing.
 *
 * Input Parameters:
 *   Not Used.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void node_status_process(bl_timer_id id, void *context)
{
	static uint8_t transfer_id;

	uavcan_NodeStatus_t message;
	uavcan_protocol_t protocol;

	protocol.tail.transfer_id = transfer_id++;

	message.uptime_sec = bootloader.uptime;
	message.u8 = uavcan_pack(bootloader.sub_mode, NodeStatus, sub_mode)
		     | uavcan_pack(bootloader.mode, NodeStatus, mode)
		     | uavcan_pack(bootloader.health, NodeStatus, health);
	message.vendor_specific_status_code = bootloader.percentage_done;
	uavcan_tx_dsdl(DSDLMsgNodeStatus, &protocol, (const uint8_t *) &message, sizeof(uavcan_NodeStatus_t));
}

/****************************************************************************
 * Name: find_descriptor
 *
 * Description:
 *   This functions looks through the application image in flash on 8 byte
 *   aligned boundaries to find the Application firmware descriptor.
 *   Once it is found the bootloader.fw_image_descriptor is set to point to
 *   it.
 *
 *
 * Input Parameters:
 *   None
 *
 * Returned State:
 *   If found bootloader.fw_image_descriptor points to the app_descriptor_t
 *   of the application firmware image.
 *   If not found bootloader.fw_image_descriptor = NULL
 *
 ****************************************************************************/
static void find_descriptor(void)
{
	uint64_t *p = (uint64_t *)APPLICATION_LOAD_ADDRESS;
	app_descriptor_t *descriptor = NULL;
	union {
		uint64_t ull;
		uint8_t bytes[sizeof(uint64_t)];
	} sig = {
		.bytes = APP_DESCRIPTOR_SIGNATURE
	};

	do {
		if (*p == sig.ull) {
			descriptor = (app_descriptor_t *)p;
			break;
		}
	} while (++p < APPLICATION_LAST_64BIT_ADDRRESS);

	bootloader.fw_image_descriptor = (volatile app_descriptor_t *)descriptor;
}

/****************************************************************************
 * Name: is_app_valid
 *
 * Description:
 *   This functions validates the applications image based on the validity of
 *   the Application firmware descriptor's 2 crcs and the value of
 *   the first word in the FLASH image.
 *
 *
 * Input Parameters:
 *   first_word - pointer the value read from the first 2 words of the Application's
 *   in FLASH image.
 *
 * Returned Value:
 *   true if the application in flash is valid., false otherwise.
 *
 ****************************************************************************/
static bool is_app_valid(volatile uint32_t *first_words)
{
	uint32_t block_crc1;
	uint32_t block_crc2;
	size_t length;

	find_descriptor();

	if (!bootloader.fw_image_descriptor || first_words[0] == 0xFFFFFFFFu) {
		return false;
	}

	length = bootloader.fw_image_descriptor->image_size;

	if (length > APPLICATION_SIZE || length == 0) {
		return false;
	}

	size_t block2_len = bootloader.fw_image_descriptor->image_size - ((size_t)&bootloader.fw_image_descriptor->major_version
			    - (size_t)bootloader.fw_image);

	if (block2_len > APPLICATION_SIZE || block2_len == 0) {
		return false;
	}

	block_crc1 = crc32_signature(0, LATER_FLAHSED_WORDS * sizeof(uint32_t), (const uint8_t *)first_words);
	block_crc1 = crc32_signature(block_crc1, (size_t)(&bootloader.fw_image_descriptor->crc32_block1) -
				     (size_t)(bootloader.fw_image + LATER_FLAHSED_WORDS), (const uint8_t *)(bootloader.fw_image + LATER_FLAHSED_WORDS));

	block_crc2 = crc32_signature(0, block2_len,
				     (const uint8_t *) &bootloader.fw_image_descriptor->major_version);

#if defined(DEBUG_APPLICATION_INPLACE)
	return true;
#endif

	return block_crc1 == bootloader.fw_image_descriptor->crc32_block1
	       && block_crc2 == bootloader.fw_image_descriptor->crc32_block2;
}

/****************************************************************************
 * Name: get_dynamic_node_id
 *
 * Description:
 *   This functions performs The allocatee  side of the uavcan specification
 *   for dynamic node allocation.
 *
 * Input Parameters:
 *   tboot             - The id of the timer to use at the tBoot timeout.
 *   allocated_node_id - A pointer to the location that should receive the
 *                       allocated node id.
 * Returned Value:
 *   CAN_OK            - Indicates the a node id has been allocated to this
 *                       node.
 *   CAN_BOOT_TIMEOUT - Indicates that tboot expired before an allocation
 *                      was done.
 *
 ****************************************************************************/
static int get_dynamic_node_id(bl_timer_id tboot, uint32_t *allocated_node_id)
{
	uavcan_HardwareVersion_t hw_version;

	struct {
		uint8_t node_id;
		uavcan_Allocation_t allocation_message;
	} server;

	/* Get the Hw info, (struct will be zeroed by board_get_hardware_version ) */

	size_t  rx_len = board_get_hardware_version(&hw_version);
	uint16_t random  = (uint16_t) timer_hrt_read();
	random = crc16_signature(random, rx_len, hw_version.unique_id);
	util_srand(random);
	memset(&server, 0, sizeof(server));

	/*
	 * Rule A: on initialization, The allocatee  subscribes to uavcan.protocol.dynamic_node_id.Allocation and
	 * starts a Request Timer with interval of Trequestinterval = the random value between MIN_REQUEST_PERIOD_MS
	 * MAX_REQUEST_PERIOD_MS
	 *
	 */

	bl_timer_id trequest = timer_allocate(modeTimeout | modeStarted, util_random(MIN_REQUEST_PERIOD_MS,
					      MAX_REQUEST_PERIOD_MS), 0);

	uavcan_protocol_t protocol;

	do {
		/*
		 *  Rule B. On expiration of trequest:
		 * 1. Request Timer restarts with a random interval of Trequest.
		 * 2. The allocatee broadcasts a first-stage Allocation request message, where the fields are assigned following values:
		 *    node_id                 - preferred node ID, or zero if the allocatee doesn't have any preference
		 *    first_part_of_unique_id - true
		 *    unique_id               - first MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST bytes of unique ID
		 */

		if (timer_expired(trequest)) {
			uavcan_tx_allocation_message(*allocated_node_id, sizeof_member(uavcan_HardwareVersion_t, unique_id),
						     hw_version.unique_id,
						     0u, random);
restart:
			timer_restart(trequest, util_random(MIN_REQUEST_PERIOD_MS, MAX_REQUEST_PERIOD_MS));
			server.node_id = ANY_NODE_ID;
		}

		/*
		 * Do we have a frame and it is of DTIDAllocation that can be Anonymous or a Message
		 * (It can not be s service) It is possibly not to us.
		 */

		protocol.ana.source_node_id = server.node_id;
		rx_len = sizeof(server.allocation_message);

		if (UavcanOk == uavcan_rx_dsdl(DSDLMsgAllocation, &protocol, (uint8_t *) &server.allocation_message, &rx_len, 50)) {

			rx_len -= uavcan_byte_count(Allocation, node_id);

			/*
			 * Rule C. On any Allocation message, even if other rules also match:
			 *      1. Request Timer restarts with a random interval of Trequest.
			 */

			timer_restart(trequest, util_random(MIN_REQUEST_PERIOD_MS, MAX_REQUEST_PERIOD_MS));

			/*
			Skip this message if it's anonymous (from another client),
			*/
			if (protocol.msg.source_node_id == UavcanAnonymousNodeID) {

				continue;

				/*
				 *  If we do not have a server set or the transfer id
				 *  does not match and this is the first frame
				 */

			} else if (0 == server.node_id) {
				server.node_id = protocol.msg.source_node_id;
			}

			/*
			 *
			 * Rule D. On an Allocation message WHERE (source node ID is non-anonymous)
			 * AND (client's unique ID starts with the bytes available in the field unique_id)
			 * AND (unique_id is less than 16 bytes long):
			 *
			 * 1. The client waits for Tfollowup units of time, while listening for other Allocation messages (anon or otherwise).
			 *  If an Allocation message is received during this time, the execution of this rule will be terminated.
			 *  Also see rule C.
			 * 2. The client broadcasts a second-stage Allocation request message, where the fields are assigned following
			 *    values:
			 *      node_id                 - same value as in the first-stage
			 *      first_part_of_unique_id - false
			 *      unique_id               - at most MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST bytes of local unique ID with an offset
			 *                                equal to number of bytes in the received unique ID
			 *
			 *
			 *
			 * Rule E. On an Allocation message WHERE (source node ID is non-anonymous)
			 * AND (unique_id fully matches client's unique ID)
			 * AND (node_id in the received message is not zero):
			 * 1. Request Timer stops.
			 * 2. The client initializes its node_id with the received value.
			 * 3. The client terminates subscription to Allocation messages.
			 * 4. Exit.
			 */
			/* Count the number of unique ID bytes matched */

			size_t max_compare = uavcan_byte_count(Allocation, unique_id);

			if (max_compare > rx_len) {
				max_compare = rx_len;
			}

			uint8_t unique_id_matched;

			for (unique_id_matched = 0; unique_id_matched < max_compare
			     && hw_version.unique_id[unique_id_matched] == server.allocation_message.unique_id[unique_id_matched];
			     unique_id_matched++);

			if (unique_id_matched < rx_len) {

				/* Abort if we didn't match the whole unique ID */

				goto restart;

				/* All frames are received, yet what was sent by was not the complete id yet */

			} else if (unique_id_matched == uavcan_byte_count(Allocation, unique_id)) {

				/* Case E */
				*allocated_node_id =  uavcan_runpack(server.allocation_message, Allocation, node_id);
				break;

			} else {
				/* Case D */
				uint8_t rx_payload[CanPayloadLength];

				/* Case D.1 */
				protocol.id.u32 = ANY_NODE_ID;

				if (UavcanOk == uavcan_rx_dsdl(DSDLMsgAllocation, &protocol, (uint8_t *) &rx_payload, &rx_len,
							       util_random(MIN_FOLLOWUP_DELAY_MS, MAX_FOLLOWUP_DELAY_MS))) {
					goto restart;
				}

				/* Sending the next chunk */

				uavcan_tx_allocation_message(*allocated_node_id, sizeof_member(uavcan_HardwareVersion_t, unique_id),
							     hw_version.unique_id,
							     unique_id_matched, random);

			}
		}
	} while (!timer_expired(tboot));

	timer_free(trequest);
	return *allocated_node_id == ANY_NODE_ID ? CAN_BOOT_TIMEOUT : CAN_OK;
}

/****************************************************************************
 * Name: wait_for_beginfirmwareupdate
 *
 * Description:
 *   This functions performs The allocatee side of the uavcan specification
 *   for begin firmware update.
 *
 * Input Parameters:
 *   tboot              - The id of the timer to use at the tBoot timeout.
 *   fw_path            - A pointer to the location that should receive the
 *                        path of the firmware file to read.
 *   fw_path_length    -  A pointer to return the path length in.
 *
 * Returned Value:
 *   UavcanOk          - Indicates the a beginfirmwareupdate was received and
 *                       processed successful.
 *   UavcanBootTimeout - Indicates that tboot expired before a
 *                       beginfirmwareupdate was received.
 *
 ****************************************************************************/
static uavcan_error_t wait_for_beginfirmwareupdate(bl_timer_id tboot, uavcan_Path_t *fw_path, size_t *fw_path_length)
{
	uavcan_BeginFirmwareUpdate_request request;
	uavcan_protocol_t protocol;

	uavcan_error_t status = UavcanError;
	size_t rx_length;
	fw_path->u8[0] = 0;
	*fw_path_length = 0;

	g_server_node_id = ANY_NODE_ID;

	while (status != UavcanOk) {

		if (timer_expired(tboot)) {
			return UavcanBootTimeout;
		}

		protocol.id.u32 = ANY_NODE_ID;
		rx_length = sizeof(uavcan_BeginFirmwareUpdate_request);
		status = uavcan_rx_dsdl(DSDLReqBeginFirmwareUpdate, &protocol,
					(uint8_t *) &request, &rx_length,
					UavcanServiceTimeOutMs);
	}

	if (UavcanOk == status) {
		/* Update the priority on the BeginFirmwareUpdate received Request  */
		g_uavcan_priority = protocol.ser.priority;

		/* Send an ERROR_OK response */
		uavcan_BeginFirmwareUpdate_response response;

		response.error = ERROR_OK;

		/* We do not care if this send fails */
		uavcan_tx_dsdl(DSDLRspBeginFirmwareUpdate, &protocol,
			       (uint8_t *)&response, sizeof(uavcan_BeginFirmwareUpdate_response));

		rx_length = rx_length - uavcan_byte_count(BeginFirmwareUpdate, source_node_id);
		memcpy(fw_path, &request.image_file_remote_path, sizeof(uavcan_Path_t));
		g_server_node_id = request.source_node_id;
		*fw_path_length = rx_length;
	}

	return status;
}

/****************************************************************************
 * Name: file_getinfo
 *
 * Description:
 *   This functions performs The allocatee  side of the uavcan specification
 *   for getinfo (for a file).
 *
 * Input Parameters:
 *
 *   fw_path           - A pointer to the path of the firmware file that's
 *                       info is being requested.
 *   fw_path_length    - The path length of the firmware file that's
 *                       info is being requested.
 *   fw_image_size     - A pointer to the location that should receive the
 *                       firmware image size.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
static void file_getinfo(const uavcan_Path_t *fw_path, size_t fw_path_length, size_t *fw_image_size)
{
	uavcan_GetInfo_request_t   request;
	uavcan_GetInfo_response_t  response;
	uavcan_protocol_t protocol;

	protocol.tail_init.u8  = 0;

	uint8_t retries = UavcanServiceRetries;

	memcpy(&request.path, fw_path, sizeof(uavcan_Path_t));

	*fw_image_size = 0;

	while (retries--) {

		protocol.ser.source_node_id = g_server_node_id;
		size_t length =  FixedSizeGetInfoRequest + fw_path_length;

		if (UavcanOk == uavcan_tx_dsdl(DSDLReqGetInfo, &protocol,
					       (uint8_t *)&request, length)) {

			length = sizeof(response);
			protocol.ser.source_node_id = g_server_node_id;
			uavcan_error_t status = uavcan_rx_dsdl(DSDLRspGetInfo,
							       &protocol,
							       (uint8_t *) &response,
							       &length,
							       UavcanServiceTimeOutMs);

			protocol.tail.transfer_id++;

			/* UAVCANBootloader_v0.3 #27: validateFileInfo(file_info, &errorcode) */
			if (status == UavcanOk && response.error.value == FILE_ERROR_OK &&
			    (response.entry_type.flags & (ENTRY_TYPE_FLAG_FILE | ENTRY_TYPE_FLAG_READABLE)) &&
			    response.size > 0 && response.size < OPT_APPLICATION_IMAGE_LENGTH) {
				*fw_image_size = response.size;
				break;
			}
		}
	}
}

/****************************************************************************
 * Name: file_read_and_program
 *
 * Description:
 *   This functions performs The allocatee  side of the uavcan specification
 *   for file read and programs the flash.
 *
 * Input Parameters:
 *
 *   fw_path           - A pointer to the path of the firmware file that
 *                       is being read.
 *   fw_path_length    - The path length of the firmware file that is
 *                       being read.
 *   fw_image_size     - The size the fw image file should be.
 *
 * Returned Value:
 *   FLASH_OK          - Indicates that the correct amount of data has
 *                       been programmed to the FLASH.
 *                       processed successful.
 *   FLASH_ERROR       - Indicates that an error occurred
 *
 * From Read 218.Read.uavcan updated 5/16/2015
 *
 *      There are two possible outcomes of a successful service call:
 *        1. Data array size equals its capacity. This means that the
 *           end of the file is not reached yet.
 *        2. Data array size is less than its capacity, possibly zero. This
 *           means that the end of file is reached.
 *
 *       Thus, if The allocatee  needs to fetch the entire file, it should
 *       repeatedly call this service while increasing the offset,
 *       until incomplete data is returned.
 *
 *       If the object pointed by 'path' cannot be read (e.g. it is a
 *       directory or it does not exist), appropriate error code
 *       will be returned, and data array will be empty.
 *
 ****************************************************************************/
static flash_error_t file_read_and_program(const uavcan_Path_t *fw_path, uint8_t fw_path_length, size_t fw_image_size)
{
	uavcan_Read_request_t request;
	uavcan_Read_response_t response;
	uavcan_protocol_t protocol;

	uavcan_error_t uavcan_status;
	flash_error_t flash_status;

	uint8_t *data;
	uint32_t flash_address = (uint32_t) bootloader.fw_image;

	memset(&request, 0, sizeof(request));
	memset(&response, 0, sizeof(response));

	/* Set up the read request */
	memcpy(&request.path, fw_path, sizeof(uavcan_Path_t));

	bootloader.percentage_done = 0;

	uint8_t retries = UavcanServiceRetries;

	size_t length;

	protocol.tail_init.u8  = 0;
	int a_percent = fw_image_size / 100;

	do {
		/* reset the rate limit */
		retries = UavcanServiceRetries;
		uavcan_status = UavcanError;

		while (retries && uavcan_status != UavcanOk) {

			length = FixedSizeReadRequest + fw_path_length;
			protocol.ser.source_node_id = g_server_node_id;
			uavcan_status = uavcan_tx_dsdl(DSDLReqRead, &protocol,
						       (uint8_t *)&request, length);

			if (uavcan_status == UavcanOk) {
				length = sizeof(uavcan_Read_response_t);
				protocol.ser.source_node_id = g_server_node_id;
				uavcan_status = uavcan_rx_dsdl(DSDLRspRead,
							       &protocol,
							       (uint8_t *) &response,
							       &length,
							       UavcanServiceTimeOutMs);

				protocol.tail.transfer_id++;
			}

			if (uavcan_status != UavcanOk) {

				retries--;

			} else {

				if (length > sizeof_member(uavcan_Read_response_t, error)) {

					length -= sizeof_member(uavcan_Read_response_t, error);

				} else if (response.error.value != FILE_ERROR_OK) {
					uavcan_status = UavcanError;

					retries--;

					board_indicate(fw_update_invalid_response);

					uavcan_tx_log_message(LOGMESSAGE_LEVELERROR,
							      LOGMESSAGE_STAGE_PROGRAM,
							      LOGMESSAGE_RESULT_FAIL);
				}
			}
		}

		/* Exhausted retries */
		if (uavcan_status != UavcanOk) {
			board_indicate(fw_update_timeout);
			break;
		}

		data = response.data;

		/*
		 * STM32 flash addresses  must be word aligned If the packet is the
		 * last and an odd length add an 0xff but do not count it in length
		 * The is OK to do because the uavcan Read will fill all data payloads
		 * until the last one
		 */
		if (length & 1) {
			data[length] = 0xff;
		}

		/* Save the first words off */
		if (request.offset == 0u) {
			uint32_t *datal = (uint32_t *)data;
			bootloader.fw_word0[0].l = datal[0];
			datal[0] = 0xffffffff;
#if LATER_FLAHSED_WORDS > 1
			bootloader.fw_word0[1].l = datal[1];
			datal[1] = 0xffffffff;
#endif
		}

		flash_status = bl_flash_write(flash_address + request.offset,
					      data,
					      length + (length & 1));

		request.offset  += length;
		bootloader.percentage_done = (request.offset / a_percent);

	} while (request.offset < fw_image_size &&
		 length == sizeof(response.data)  &&
		 flash_status == FLASH_OK);

	/*
	 * Return success if the last read succeeded, the last write succeeded, the
	 * correct number of bytes were written, and the length of the last response
	 * was not. */
	if (uavcan_status == UavcanOk && flash_status == FLASH_OK
	    && request.offset == fw_image_size && length != 0) {
		return FLASH_OK;

	} else {
		return FLASH_ERROR;
	}
}

/****************************************************************************
 * Name: do_jump
 *
 * Description:
 *   This functions begins the execution of the application firmware.
 *
 * Input Parameters:
 *   stacktop   - The value that should be loaded into the stack pointer.
 *   entrypoint - The address to jump to.
 *
 * Returned Value:
 *    Does not return.
 *
 ****************************************************************************/
static void do_jump(uint32_t stacktop, uint32_t entrypoint)
{
	asm volatile("msr msp, %[stacktop]    \n"
		     "bx     %[entrypoint]      \n"
		     :: [stacktop] "r"(stacktop), [entrypoint] "r"(entrypoint):);

	// just to keep noreturn happy
	for (;;);
}

/****************************************************************************
 * Name: application_run
 *
 * Description:
 *   This functions will test the application image is valid by
 *   checking the value of the first word for != 0xffffffff and the
 *   second word for an address that lies inside od the application
 *   fw image in flash.
 *
 * Input Parameters:
 *   fw_image_size   - The size the fw image is.
 *
 * Returned Value:
 *    If the Image is valid this code does not return, but runs the application
 *    via do_jump.
 *    If the image is invalid the function returns.
 *
 ****************************************************************************/
static void application_run(size_t fw_image_size, bootloader_app_shared_t *common)
{
	/*
	 * We refuse to program the first word of the app until the upload is marked
	 * complete by the host.  So if it's not 0xffffffff, we should try booting it.

	 * The second word of the app is the entrypoint; it must point within the
	 * flash area (or we have a bad flash).
	 */

#if defined(DEBUG_APPLICATION_INPLACE)
	fw_image_size = FLASH_SIZE - OPT_BOOTLOADER_SIZE_IN_K;
#endif

	uint32_t fw_image[2] = {bootloader.fw_image[0], bootloader.fw_image[1]};

	if (fw_image[0] != 0xffffffff
	    && fw_image[1] > APPLICATION_LOAD_ADDRESS
	    && fw_image[1] < (APPLICATION_LOAD_ADDRESS + fw_image_size)) {

		/* We want to disable interrupts regardless of whether NuttX
		 * is configured for CONFIG_ARMV7M_USEBASEPRI
		 */

		__asm__ __volatile__("\tcpsid  i\n");

		board_deinitialize();

		/* kill the systick interrupt */
		putreg32(0, NVIC_SYSTICK_CTRL);

		/* and set a specific LED pattern */
		board_indicate(jump_to_app);

		/* Update the shared memory and make it valid to tell the
		 * App are node ID and Can bit rate.
		 */

		if (common->crc.valid) {
			bootloader_app_shared_write(common, BootLoader);
		}


		/* the interface */

		/* switch exception handlers to the application */
		__asm volatile("dsb");
		__asm volatile("isb");
		putreg32(APPLICATION_LOAD_ADDRESS, NVIC_VECTAB);
		__asm volatile("dsb");
		/* extract the stack and entrypoint from the app vector table and go */
		app_start_the_watch_dog();
		do_jump(fw_image[0], fw_image[1]);
	}
}

/****************************************************************************
 * Name: autobaud_and_get_dynamic_node_id
 *
 * Description:
 *   This helper functions wraps the auto baud and node id allocation
 *
 * Input Parameters:
 *   tboot   - The id of the timer to use at the tBoot timeout.
 *   speed   - A pointer to the location to receive the speed detected by
 *             the auto baud function.
 *   node_id - A pointer to the location to receive the allocated node_id
 *             from the dynamic node allocation.
 *
 * Returned Value:
 *   CAN_OK - on Success or a CAN_BOOT_TIMEOUT
 *
 ****************************************************************************/
static int autobaud_and_get_dynamic_node_id(bl_timer_id tboot, can_speed_t *speed, uint32_t *node_id)
{
	board_indicate(autobaud_start);
	bool autobaud_only = *speed == CAN_UNDEFINED;
	int rv = can_autobaud(speed, tboot);

	if (rv != CAN_BOOT_TIMEOUT) {
		board_indicate(autobaud_end);

		if (autobaud_only) {
			return rv;
		}

		board_indicate(allocation_start);
#if defined(DEBUG_APPLICATION_INPLACE)
		*node_id = 125;
		return rv;
#endif
		rv = get_dynamic_node_id(tboot, node_id);

		if (rv != CAN_BOOT_TIMEOUT) {
			board_indicate(allocation_end);
		}
	}

	return rv;
}

/****************************************************************************
 * Name: main
 *
 * Description:
 *   Called by the os_start code
 *
 * Input Parameters:
 *   Not used.
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/
__EXPORT int main(int argc, char *argv[])
{
	size_t fw_image_size = 0;
	uavcan_Path_t fw_path;
	size_t  fw_path_length;
	uint8_t error_log_stage;
	flash_error_t status;
	bootloader_app_shared_t common;

	early_start_the_watch_dog();

	/* Begin with all data zeroed */
	memset((void *)&bootloader, 0, sizeof(bootloader));

	/* Begin with a node id of zero for Allocation */
	g_this_node_id = ANY_NODE_ID;

	bootloader.health = HEALTH_OK;
	bootloader.mode = MODE_INITIALIZATION;
	bootloader.sub_mode = 0;

	error_log_stage = LOGMESSAGE_STAGE_INIT;

	bootloader.fw_image = (volatile uint32_t *)(APPLICATION_LOAD_ADDRESS);

	/*
	 *  This Option is set to 1 ensure a provider of firmware has an
	 *  opportunity update the node's firmware.
	 *  This Option is the default policy and can be overridden by
	 *  a jumper
	 *  When this Policy is set, the node will ignore tboot and
	 *  wait indefinitely for a GetNodeInfo request before booting.
	 *
	 *  OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO_INVERT is used to allow
	 *  the polarity of the jumper to be True Active
	 *
	 *  wait  OPT_WAIT_FOR_GETNODEINFO  OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO
	 *                                                 Jumper
	 *   yes           1                       0         x
	 *   yes           1                       1       Active
	 *   no            1                       1       Not Active
	 *   no            0                       0         X
	 *   yes           0                       1       Active
	 *   no            0                       1       Not Active
	 */
	bootloader.wait_for_getnodeinfo = OPT_WAIT_FOR_GETNODEINFO;

	/*
	 *  This Option allows the compile timed option to be overridden
	 *  by the state of the jumper
	 *
	 */
#if defined(OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO)
	bootloader.wait_for_getnodeinfo = (px4_arch_gpioread(GPIO_GETNODEINFO_JUMPER) ^
					   OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO_INVERT);
#endif

	/* Is the memory in the Application space occupied by a valid application? */
	bootloader.app_valid = is_app_valid(bootloader.fw_image);

	board_indicate(reset);

	/* Was this boot a result of the Application being told it has a FW update ? */
	bootloader.app_bl_request = (OK == bootloader_app_shared_read(&common, App)) &&
				    common.bus_speed && common.node_id;

#if defined(SUPPORT_ALT_CAN_BOOTLOADER)
	/* Was this boot a result of An Alternate Application being told it has a FW update ? */

	bootloader_alt_app_shared_t *ps = (bootloader_alt_app_shared_t *) &_sapp_bl_shared;

	if (!bootloader.app_bl_request && ps->signature == BL_ALT_APP_SHARED_SIGNATURE) {

		common.node_id = ps->node_id;
		common.bus_speed = CAN_UNDEFINED;
		bootloader.app_bl_request = ps->node_id != 0;
		ps->signature = 0;
	}

#endif
	/*
	 * Mark CRC to say this is not from
	 * auto baud and Node Allocation
	 */
	common.crc.valid = false;

	/* Either way prevent Deja vu by invalidating the struct*/
	bootloader_app_shared_invalidate();

	/* Set up the Timers */
	bl_timer_cb_t p = null_cb;
	p.cb = uptime_process;

	/* Uptime is always on*/
	timer_allocate(modeRepeating | modeStarted, 1000, &p);

	/*
	 * NodeInfo is a controlled process that will be run once a node Id is
	 * established to process the received NodeInfo response.
	 */
	p.cb = node_info_process;
	bl_timer_id tinfo = timer_allocate(modeRepeating, OPT_NODE_INFO_RATE_MS, &p);

	/*
	 * NodeStatus is a controlled process that will sent the requisite NodeStatus
	 * once a node Id is established
	 */
	p.cb = node_status_process;
	bl_timer_id tstatus = timer_allocate(modeRepeating, OPT_NODE_STATUS_RATE_MS, &p);

	/*
	 * tBoot is a controlled timer, that is run to allow the application, if
	 * valid to be booted. tBoot is gated on by the application being valid
	 * and the OPT_WAIT_FOR_GETNODEINFOxxxx settings
	 *
	 */
	bl_timer_id tboot = timer_allocate(modeTimeout, OPT_TBOOT_MS, 0);

	/*
	 * If the Application is Valid and we are not in a reboot from the
	 * running Application requesting to Bootload or we are not configured
	 * to wait for NodeStatus then start tBoot
	 */
	if (bootloader.app_valid && !bootloader.wait_for_getnodeinfo && !bootloader.app_bl_request) {
		timer_start(tboot);
	}

	/*
	 * If this is a reboot from the running Application requesting to Bootload
	 * then use the CAN parameters supplied by the running Application to init
	 * the CAN device.
	 */
	if (bootloader.app_bl_request) {

		bootloader.bus_speed = common.bus_speed;

		/* if the the bootloader_alt_app_shared_t was used there  is not bit rate.
		 * So let auto baud only as signaled by bootloader.bus_speed == CAN_UNDEFINED
		 */

		if (common.bus_speed == CAN_UNDEFINED) {
			if (CAN_OK != autobaud_and_get_dynamic_node_id(tboot, (can_speed_t *)&bootloader.bus_speed, &common.node_id)) {
				/*
				 * It is OK that node ID is set to the preferred Appl Node ID because
				 *  common.crc.valid is not true yet
				 */

				goto boot;

			}
		}

		can_init(can_freq2speed(common.bus_speed), CAN_Mode_Normal);
		/*
		 * Mark CRC to say this is from
		 * auto baud and Node Allocation
		 */
		common.crc.valid = true;


	} else {

		/*
		 * It is a regular boot, So we need to autobaud and get a node ID
		 * If the tBoot was started, we will boot normal if the auto baud
		 * or the Node allocation runs longer the tBoot
		 */

		/* Preferred Node Address */

		common.node_id = OPT_PREFERRED_NODE_ID;

		if (CAN_OK != autobaud_and_get_dynamic_node_id(tboot, (can_speed_t *)&bootloader.bus_speed, &common.node_id)) {

			/*
			 * It is OK that node ID is set to the preferred Node ID because
			 *  common.crc.valid is not true yet
			 */

			goto boot;
		}

		/* We have autobauded and got a Node ID. So reset uptime
		 * and save the speed and node_id in both the common and
		 * and bootloader data sets
		 */

		bootloader.uptime = 0;
		common.bus_speed = can_speed2freq(bootloader.bus_speed);

		/*
		 * Mark CRC to say this is from
		 * auto baud and Node Allocation
		 */
		common.crc.valid = true;

		/* Auto bauding may have taken a long time, so restart the tboot time*/

		if (bootloader.app_valid && !bootloader.wait_for_getnodeinfo) {
			timer_start(tboot);
		}


	}

	/* Now that we have a node Id configure the uavcan library */
	g_this_node_id = common.node_id;

	/* Now start the processes that were defendant on a node ID */
	timer_start(tinfo);
	timer_start(tstatus);

	/*
	 * Now since we are sending NodeStatus messages the Node Status monitor on the
	 * bus will see us as a new node or one whose uptime has gone backwards
	 * So we wait for the NodeInfoRequest and our response to be sent of if
	 * tBoot is running a time tBoot out.
	 */
	while (!bootloader.sent_node_info_response) {
		if (timer_expired(tboot)) {
			goto boot;
		}
	}

	/*
	 * Now we have seen and responded to the NodeInfoRequest
	 * If we have a Valid App and we are processing and the running App's
	 * re-boot to bootload request or we are configured to wait
	 * we start the tBoot time out.
	 *
	 * This effectively extends the time the FirmwareServer has to tell us
	 * to Begin a FW update. To tBoot from NodeInfoRequest as opposed to
	 * tBoot from Reset.
	 */
	if (bootloader.app_valid &&
	    (bootloader.wait_for_getnodeinfo ||
	     bootloader.app_bl_request)) {

		timer_start(tboot);
	}

#if defined(DEBUG_APPLICATION_INPLACE) && defined(DEBUG_NO_FW_UPDATE)
	goto boot;
#endif

	/*
	 * Now We wait up to tBoot for a begin firmware update from the FirmwareServer
	 * on the bus. tBoot will only be running if the app is valid.
	 */

	do {
		if (UavcanBootTimeout == wait_for_beginfirmwareupdate(tboot, &fw_path,
				&fw_path_length)) {
			goto boot;
		}
	} while (fw_path_length == 0);

	/* We received a begin firmware update */
	timer_stop(tboot);

	board_indicate(fw_update_start);
	bootloader.mode = MODE_SOFTWARE_UPDATE;

	file_getinfo(&fw_path, fw_path_length, &fw_image_size);

	//todo:Check this
	if (fw_image_size < sizeof(app_descriptor_t)) {
		error_log_stage = LOGMESSAGE_STAGE_GET_INFO;
		goto failure;
	}

	/* LogMessage the Erase  */
	uavcan_tx_log_message(LOGMESSAGE_LEVELINFO,
			      LOGMESSAGE_STAGE_ERASE,
			      LOGMESSAGE_RESULT_START);

	/* Need to signal that the app is no longer valid  if Node Info Request are done */
	bootloader.app_valid = false;

	status = bl_flash_erase(APPLICATION_LOAD_ADDRESS, APPLICATION_SIZE);

	if (status != FLASH_OK) {
		/* UAVCANBootloader_v0.3 #28.8: [Erase
		 * Failed]:INDICATE_FW_UPDATE_ERASE_FAIL */
		board_indicate(fw_update_erase_fail);

		error_log_stage = LOGMESSAGE_STAGE_ERASE;
		goto failure;
	}

	status = file_read_and_program(&fw_path, fw_path_length, fw_image_size);

	if (status != FLASH_OK) {
		error_log_stage = LOGMESSAGE_STAGE_PROGRAM;
		goto failure;
	}

	/* Did we program a valid image ?*/
	if (!is_app_valid(&bootloader.fw_word0[0].l)) {
		bootloader.app_valid = 0u;

		board_indicate(fw_update_invalid_crc);

		error_log_stage = LOGMESSAGE_STAGE_VALIDATE;
		goto failure;
	}

	/* Yes Commit the first word(s) to location 0 of the Application image in flash */
	status = bl_flash_write((uint32_t) bootloader.fw_image, (uint8_t *) &bootloader.fw_word0[0].b[0],
				sizeof(bootloader.fw_word0));

	if (status != FLASH_OK) {
		error_log_stage = LOGMESSAGE_STAGE_FINALIZE;
		goto failure;
	}

	bootloader.percentage_done = 100;

	/* Send a completion log allocation_message */
	uavcan_tx_log_message(LOGMESSAGE_LEVELINFO,
			      LOGMESSAGE_STAGE_FINALIZE,
			      LOGMESSAGE_RESULT_OK);



	/* Boot the application */

boot:

	kick_the_watch_dog();

	application_run(bootloader.fw_image_descriptor->image_size, &common);

	/* We will fall thru if the Image is bad */

failure:

	uavcan_tx_log_message(LOGMESSAGE_LEVELERROR,
			      error_log_stage,
			      LOGMESSAGE_RESULT_FAIL);


	bootloader.health = HEALTH_CRITICAL;

	bl_timer_id tmr = timer_allocate(modeTimeout | modeStarted, OPT_RESTART_TIMEOUT_MS, 0);

	while (!timer_expired(tmr)) {
		;
	}

	timer_free(tmr);
	up_systemreset();
}

/****************************************************************************
 * Name: sched_yield()
 *
 * Description:
 *   This function should be called in situation were the cpu may be
 *   busy for a long time without interrupts or the ability to run code
 *   to insure that the timer based process will be run.
 *
 *
 * Input Parameters:
 *   None
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/
#if defined(OPT_USE_YIELD)
void bl_sched_yield(void)
{
	/*
	 *  TODO: The uptime will be stalled so consider having the caller or
	 *  this code track the stall time and accumulate it to kee the uptime
	 *  moving
	 */

	node_status_process(0, 0);
}
#endif
