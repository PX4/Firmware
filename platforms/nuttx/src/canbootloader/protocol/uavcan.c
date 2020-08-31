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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "boot_config.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "chip.h"
#include "stm32.h"

#include "timer.h"
#include "uavcan.h"
#include "can.h"
#include "crc.h"

#define CAN_REQUEST_TIMEOUT 1000
#define ANY_NODE_ID 0
#define THIS_NODE   1
#define FW_SERVER   2
#define REQ_NODE    3

typedef begin_packed_struct struct dsdl_t {
	uavcan_protocol_t prototype;
	uint16_t signature_crc16;
	uint8_t intail;
	uint8_t outtail;
	uint8_t mailbox : 2;
	uint8_t fifo : 2;
} end_packed_struct dsdl_t;

/* Values used in filter initialization */

typedef enum uavcan_transfer_stage_t {
	Initialization,
	OnStartOfTransfer
} uavcan_transfer_stage_t;

typedef enum  uavcan_dsdl_ignore_t {
	sizeDSDLTransfers = SizeDSDL - SizeDSDLComponents + 1
} uavcan_dsdl_ignore_t;


/* Forward declaration */
extern const dsdl_t dsdl_table[sizeDSDLTransfers];

uint8_t g_uavcan_priority = PriorityAllocation;
uint8_t g_this_node_id = 0;
uint8_t g_server_node_id = 0;

static inline uavcan_dsdl_t getDSDLOffset(uavcan_dsdl_t full) { return full - (SizeDSDLComponents + 1); }

/****************************************************************************
 * Name: uavcan_init_comapare_masks
 *
 * Description:
 *   This function builds the masks needed for filtering Transfers
 *
 * Input Parameters:
 *   stage       - Either Initialization, OnStartOfTransfer
 *   protocol   - A pointer to a uavcan_protocol_t to configure the filters
 *   masks      - A pointer to a uavcan_protocol_t return the filters
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/
static void uavcan_init_comapare_masks(uavcan_transfer_stage_t stage, uavcan_protocol_t *protocol,
				       uavcan_protocol_t *masks)
{
	switch (stage) {
	default:
		break;

	case OnStartOfTransfer:
		masks->id.u32 |= uavcan_protocol_mask(UavCanMessageSourceNodeID);
		masks->tail_init.u8      |= uavcan_protocol_mask(UavCanTransferID);
		break;


	case Initialization:

		masks->tail_init.u8  =    uavcan_protocol_mask(UavCanToggle)
					  | uavcan_protocol_mask(UavCanStartOfTransfer);

		if (protocol->msg.service_not_message) {

			/* Specific Filter initialization for a Services */


			masks->id.u32        =    uavcan_protocol_mask(UavCanServiceTypeID)
						  | uavcan_protocol_mask(UavCanServiceRequestNotResponse)
						  | uavcan_protocol_mask(UavCanServiceDestinationNodeID)
						  | uavcan_protocol_mask(UavCanServiceServiceNotMessage);

			if (protocol->ser.source_node_id != ANY_NODE_ID) {

				masks->id.u32 |= uavcan_protocol_mask(UavCanMessageSourceNodeID);

			}


		} else  {

			/* Specific Filter initialization for a Message */

			/* Is it anonymous? */

			if (protocol->msg.source_node_id == ANY_NODE_ID) {

				/* Intentional use of UavCanMessageTypeID because the response
				 * to the anonymous message is a message and the discriminator
				 * will be 0
				 */
				masks->id.u32       =  uavcan_protocol_mask(UavCanMessageTypeID)
						       | uavcan_protocol_mask(UavCanAnonMessageServiceNotMessage);

			} else {


				masks->id.u32  =        uavcan_protocol_mask(UavCanMessageTypeID)
							| uavcan_protocol_mask(UavCanMessageServiceNotMessage)
							| uavcan_protocol_mask(UavCanMessageSourceNodeID);
			}
		}

		break;
	}
}

/****************************************************************************
 * Name: load_dsdl_protocol
 *
 * Description:
 *   This function builds the protocol for a given dsdl
 *   N.B The value set for a service is request.
 *
 * Input Parameters:
 *   dsdl       - An uavcan DSDL Identifier (Auto Generated)
 *   direction_in_not_out - in or out bound
 *   protocol   - A pointer to a uavcan_protocol_t to configure the filters
 *   masks      - A pointer to a uavcan_protocol_t return the filters
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/
static const dsdl_t *load_dsdl_protocol(uavcan_dsdl_t dsdl, uavcan_direction_t rx_not_tx, uavcan_protocol_t *protocol,
					uint8_t that_node_id)
{
	const dsdl_t *pdsdl =  &dsdl_table[getDSDLOffset(dsdl)];

	protocol->id.u32 = pdsdl->prototype.id.u32;
	protocol->msg.priority = g_uavcan_priority;

	/* Preserver the transfer_id */
	protocol->tail_init.u8 &= uavcan_protocol_mask(UavCanTransferID);

	if (rx_not_tx) {
		/* Rx */
		protocol->tail_init.u8 |=  pdsdl->intail;

		/* Service */
		if (pdsdl->prototype.msg.service_not_message) {

			protocol->ser.dest_node_id = g_this_node_id;
			protocol->ser.source_node_id = that_node_id;
		}

	} else {
		/*
		 * Tx
		 * All Transfers sent have .source_node_id set
		 * to this node's id
		 * During allocation this value will be
		 * 0
		 */
		protocol->tail_init.u8 |= pdsdl->outtail;
		protocol->msg.source_node_id = g_this_node_id;

		/*
		 * All Service Transfer sent have
		 * The ser.dest_node_id
		 */
		if (pdsdl->prototype.msg.service_not_message) {
			protocol->ser.dest_node_id = that_node_id;
		}
	}

	return pdsdl;
}

/****************************************************************************
 * Name: uavcan_tx
 *
 * Description:
 *   This function sends a single uavcan protocol based frame applying
 *   the tail byte.
 *
 * Input Parameters:
 *   protocol   - A pointer to a uavcan_protocol_t to configure the send
 *   frame_data - A pointer to 8 bytes of data to be sent (all 8 bytes will be
 *                loaded into the CAN transmitter but only length bytes will
 *                be sent.
 *   length     - The number of bytes of the frame_date (DLC field)
 *   mailbox    - A can_fifo_mailbox_t MBxxx value to choose the outgoing
 *                mailbox.
 *
 * Returned value:
 *   The UavcanOk of the data sent. Anything else indicates if a timeout
 *   occurred.
 *
 ****************************************************************************/
CCASSERT((int)UavcanOk == (int)CAN_OK);

static uavcan_error_t uavcan_tx(uavcan_protocol_t *protocol, uint8_t *frame_data, size_t length, uint8_t mailbox)
{
	frame_data[length++] = protocol->tail_init.u8;
	return can_tx(protocol->id.u32, length, frame_data, mailbox);
}

/****************************************************************************
 * Name: uavcan_tx_dsdl
 *
 * Description:
 *   This function sends a uavcan protocol transfer. For a Service
 *   The natural case for a Service Response is to send it to the Requestor
 *
 *   Therefore, this function assumes that id this is a Service Transfer,
 *   the protocol object has the destination node id set in
 *   protocol->ser.source_node_id.
 *
 * Input Parameters:
 *   dsdl       - An Uavcan DSDL Identifier (Auto Generated)
 *   protocol   - A pointer to a uavcan_protocol_t to configure the send,
 *                For a service transfer the desination node id set in
 *                ser.source_node_id to that of the node we are sending to.
 *   transfer    - A pointer to the packed data of the transfer to be sent.
 *   length     - The number of bytes of data
 *
 * Returned value:
 *   The UavcanOk of the data sent. Anything else indicates if a timeout
 *   occurred.
 *
 ****************************************************************************/

uavcan_error_t uavcan_tx_dsdl(uavcan_dsdl_t dsdl, uavcan_protocol_t *protocol, const uint8_t *transfer,
			      size_t transfer_length)
{
	/*
	 * Since we do not discriminate between sending a Message or a
	 * Service (Request or Response)
	 * We assume this is a response from a rx request and msg.source_node_id
	 * is that of the requester and that will become the ser.dest_node_id
	 */
	const dsdl_t *pdsdl = load_dsdl_protocol(dsdl, OutBound, protocol, protocol->ser.source_node_id);

	uint8_t payload[CanPayloadLength];

	/*
	 *  Only prepend CRC if the transfer will not fit within a single frame
	 */
	uint32_t m = 0;

	if (transfer_length > MaxUserPayloadLength) {
		uint16_t transfer_crc = crc16_signature(pdsdl->signature_crc16, transfer_length, transfer);
		payload[PayloadOffsetCRClsb] = (uint8_t)transfer_crc;
		payload[PayloadOffsetCRCmsb] = (uint8_t)(transfer_crc >> 8u);
		m = PayloadOffsetCRCdata;
	}

	for (uint32_t i = 0; i < transfer_length; i++) {

		payload[m++] = transfer[i];

		/* Is this the last byte? */

		protocol->tail.eot = (i == (transfer_length - 1));

		/* Either end of user portion of payload or last byte */
		if (m == MaxUserPayloadLength || protocol->tail.eot) {
			uavcan_error_t rv = uavcan_tx(protocol, payload, m, pdsdl->mailbox);

			if (rv != UavcanOk) {
				return rv;
			}

			/* Increment 1 bit frame sequence number */

			protocol->tail.toggle ^= true;
			protocol->tail.sot    = false;

			m = 0;
		}
	}

	return UavcanOk;
}

/****************************************************************************
 * Name: uavcan_rx
 *
 * Description:
 *   This function is called to receive a single CAN frame from the supplied
 *   fifo. It does not block if there is not available, but returns 0
 *
 * Input Parameters:
 *   protocol   - A pointer to a uavcan_protocol_t to return the CAN ID and
 *                Tail byte
 *   frame_data - A pointer to return 8 bytes of the frame's data
 *                (all 8 bytes will be read from the CAN fifo but
 *                 only length bytes will valid.
 *   length     - A pointer to return the number of bytes of the frame_data
 *                (DLC field)
 *   fifo         A can_fifo_mailbox_t fifixxx value to choose the incoming fifo.
 *
 * Returned value:
 *   The length of the data read or 0 if the fifo was empty
 *
 ****************************************************************************/
//#define DEBUG_DTID 1
#define DEBUG_DTID_TRIGGER 4
#ifdef DEBUG_DTID
int trigger = DEBUG_DTID_TRIGGER;
int id = DEBUG_DTID;
bool msg = true;
bool svc = false;

#endif

static uint8_t uavcan_rx(uavcan_protocol_t *protocol, uint8_t *frame_data, size_t *length,  uint8_t fifo)
{
	uint8_t rv = can_rx(&protocol->id.u32, length, frame_data, fifo);

	if (rv) {

		/* Remove the tail byte from length */

		*length -= 1;

		/* Save the tail byte from the last byte of frame*/

		protocol->tail_init.u8  = frame_data[*length];
#ifdef DEBUG_DTID
#pragma message("!!!!!!! DEBUG_DTID is enabled !!!!")
		static volatile int count = 0;

		if ((msg && protocol->msg.type_id == id)
		    || (svc && protocol->ser.type_id == id)
		   ) {
			if (count++ == trigger) {
				count = 0;
				static volatile int j = 0;
				j++;
			}
		}

#endif
	}

	return rv;
}


/****************************************************************************
 * Name: uavcan_rx_dsdl
 *
 * Description:
 *   This function receives a uavcan protocol transfer.
 *   For a Service the natural case for a Service Request is to receive from
 *   1) Any Node
 *   2) The established Server.
 *
 *   Therefore, this function assumes that id this is a Service Transfer,
 *   the protocol object has the destination node id set in
 *   protocol->ser.source_node_id.
 *
 *
 * Input Parameters:
 *   dsdl       - An Uavcan DSDL Identifier (Auto Generated)
 *   protocol   - A pointer to a uavcan_protocol_t to configure the receive,
 *                based the dsdl for the DTID Service.
 *                If the request must come from a specific server
 *                then protocol->ser.source_node_id, should be set
 *                to that node id;
 *
 *   in_out_transfer_length - The number of bytes of data to receive and the
 *                            number received.
 *   timeout_ms - The amount of time in mS to wait for the initial transfer
 *
 * Returned value:
 *   uavcan_error_t
 *
 ****************************************************************************/

uavcan_error_t uavcan_rx_dsdl(uavcan_dsdl_t dsdl, uavcan_protocol_t *protocol,
			      uint8_t *transfer, size_t *in_out_transfer_length,
			      uint32_t timeout_ms)
{
	const dsdl_t *pdsdl = load_dsdl_protocol(dsdl, InBound, protocol, protocol->msg.source_node_id);
	bl_timer_id timer = timer_allocate(modeTimeout | modeStarted, timeout_ms, 0);

	/*
	 *  Set up comparison masks
	 *  In the tail we care about the initial state sot, eot and toggle
	 *  In the CAN ID We care about the Service Type ID, RequestNotResponse,
	 *  ServiceNotMessage, and the destination id
	 *  If the source id is not ANY_NODE_ID, that that must match too.
	 */

	uavcan_protocol_t masks;

	uavcan_init_comapare_masks(Initialization, protocol, &masks);

	uint8_t timeout = false;
	uint16_t transfer_crc = 0;
	size_t total_rx = 0;
	uavcan_protocol_t rx_protocol;

	do {
		uint8_t payload[CanPayloadLength];
		size_t rx_length;

		if (!uavcan_rx(&rx_protocol, payload, &rx_length, pdsdl->fifo)
		    || BadTailState == (rx_protocol.tail_init.u8 & BadTailState)
		    || ((rx_protocol.id.u32 ^ protocol->id.u32) & masks.id.u32)
		    || ((rx_protocol.tail_init.u8 ^ protocol->tail_init.u8) & masks.tail_init.u8)) {
			continue;
		}


		timer_restart(timer, timeout_ms);

		/*
		If this is the first frame, capture the actual source node ID and
		transfer ID for future comparisons
		*/
		size_t payload_index = 0;

		/* Is this the start of transfer? */
		if (rx_protocol.tail.sot) {

			/*
			 * We expect only one Start Of Transfer  per transfer
			 * So knock it down
			 */
			protocol->tail.sot = false;

			/* Discard any data */
			total_rx  = 0;

			/*
			 *  Capture the source node id
			 *  and the Transfer Id
			 */
			protocol->msg.source_node_id = rx_protocol.msg.source_node_id;
			protocol->tail.transfer_id = rx_protocol.tail.transfer_id;

			/*
			 * This is the start of transfer - update the
			 * masks for this phase or the transfer were
			 * source_node_id is known and transfer_id
			 * matters. From now on match source node and
			 * transfer ID too
			 */
			uavcan_init_comapare_masks(OnStartOfTransfer, protocol, &masks);

			/* Is this a multi-frame transfer? */
			if (rx_protocol.tail.eot == false) {

				/* Capture the frame CRC */
				transfer_crc = uavcan_make_uint16(payload[PayloadOffsetCRClsb], payload[PayloadOffsetCRCmsb]);

				/*
				 * When the CRC is prepended to the payload
				 * the index of the data is past the CRC
				 */
				payload_index = PayloadOffsetCRCdata;
				rx_length    -= PayloadOffsetCRCdata;
			}
		}


		if (rx_protocol.tail.transfer_id > protocol->tail.transfer_id
		    || total_rx >= *in_out_transfer_length) {

			uavcan_init_comapare_masks(Initialization, protocol, &masks);
			protocol->tail.sot = true;
			protocol->tail.toggle = false;

		} else if (rx_protocol.tail.transfer_id < protocol->tail.transfer_id) {
			continue;
		}

		/* Copy transfer bytes to the transfer */

		if (total_rx + rx_length <= *in_out_transfer_length) {
			memcpy(&transfer[total_rx], &payload[payload_index], rx_length);
			total_rx += rx_length;
		}

		/* Increment 1 bit frame sequence number */

		protocol->tail.toggle ^= true;

		/* Is this the end of the transfer ?*/

		if (rx_protocol.tail.eot) {

			/* Return length of data received */

			*in_out_transfer_length = total_rx;

			break;
		}
	} while (!(timeout = timer_expired(timer)));

	timer_free(timer);

	return (!timeout && (rx_protocol.tail.sot
			     || transfer_crc == crc16_signature(pdsdl->signature_crc16, total_rx, transfer))
		? UavcanOk : UavcanError);
}

/****************************************************************************
 * Name: uavcan_pack_GetNodeInfo_response
 *
 * Description:
 *   This function packs the data of a uavcan_NodeStatus_t into
 *   a uavcan_GetNodeInfo_response_t structure as array of bytes.
 *   Then it packs the uavcan_GetNodeInfo_response_t
 *
 * Input Parameters:
 *   response   The uavcan_GetNodeInfo_response_t to be packed
 *   node_status - The uavcan_NodeStatus_t that is part of the composition
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/
size_t uavcan_pack_GetNodeInfo_response(uavcan_GetNodeInfo_response_t *response)
{
	size_t contiguous_length = FixedSizeGetNodeInfo + \
				   response->hardware_version.certificate_of_authenticity_length;
	/*
	 * Move name so it's contiguous with the end of certificate_of_authenticity[length]
	 * which very well may be just after certificate_of_authenticity_length if the length
	 * of the certificate_of_authenticity is 0
	 */
	memcpy(&((uint8_t *)response)[contiguous_length], response->name, response->name_length);
	return contiguous_length + response->name_length;
}

/****************************************************************************
 * Name: uavcan_pack_LogMessage
 *
 * Description:
 *   This function formats the data of a uavcan_logmessage_t structure into
 *   an array of bytes.
 *
 * Input Parameters:
 *   external    - The array of bytes to populate.
 *   internal - The uavcan_logmessage_t to pack into the data
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/
static size_t uavcan_pack_LogMessage(uint8_t *external, const uavcan_LogMessage_t *internal)
{
	/* Pack the 3 bit level in top bits followed by the length of source */
	external[uavcan_byte_offset(LogMessage, level)] = uavcan_ppack(internal, LogMessage, level) \
			|  uavcan_pack(uavcan_byte_count(LogMessage, source), LogMessage, source_length);
	memcpy(&external[uavcan_byte_offset(LogMessage, source)], internal->source,
	       PackedSizeMsgLogMessage - sizeof_member(uavcan_LogMessage_t, level));
	return PackedSizeMsgLogMessage;
}

/****************************************************************************
 * Name: uavcan_tx_log_message
 *
 * Description:
 *   This functions sends uavcan LogMessage type data. The Source will be
 *   taken from the application defined debug_log_source
 *
 * Input Parameters:
 *   level   - Log Level of the LogMessage Constants DEBUG, INFO, WARN, ERROR
 *   stage   - The Stage the application is at. see Aplication defined
 *             LOGMESSAGE_STAGE_x
 *   status  - The status of that stage. See Application defined
 *             LOGMESSAGE_RESULT_x
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void uavcan_tx_log_message(uavcan_LogMessageConsts_t level, uint8_t stage, uint8_t status)
{
	static uint8_t transfer_id;

	uavcan_LogMessage_t message;
	uavcan_protocol_t protocol;

	protocol.tail.transfer_id = transfer_id++;

	const dsdl_t *dsdl = load_dsdl_protocol(DSDLMsgLogMessage, MessageOut, &protocol, 0);

	message.level = level;
	memcpy(&message.source, debug_log_source, sizeof(message.source));
	message.text[0] = stage;
	message.text[1] = status;

	uint8_t payload[CanPayloadLength];
	size_t frame_len = uavcan_pack_LogMessage(payload, &message);
	uavcan_tx(&protocol, payload, frame_len, dsdl->mailbox);
}

/****************************************************************************
 * Name: uavcan_tx_allocation_message
 *
 * Description:
 *   This function sends a uavcan allocation message.
 *
 * Input Parameters:
 *   requested_node_id - This node's preferred node id 0 for no preference.
 *   unique_id_length  - This node's length of it's unique identifier.
 *   unique_id         - A pointer to the bytes that represent unique
 *                       identifier.
 *   unique_id_offset  - The offset equal 0 or the number of bytes in the
 *                       the last received message that matched the unique ID
 *                       field.
 *   random             - The value to use as the discriminator of the
 *                        anonymous message
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/
void uavcan_tx_allocation_message(uint8_t requested_node_id, size_t unique_id_length, const uint8_t *unique_id,
				  uint8_t unique_id_offset, uint16_t random)
{
	static uint8_t transfer_id;
	uavcan_protocol_t protocol;
	protocol.tail.transfer_id = transfer_id++;

	const dsdl_t *dsdl = load_dsdl_protocol(DSDLMsgAllocation, MessageOut, &protocol, 0);

	/* Override defaults */
	protocol.ana.discriminator = random;

	uint8_t payload[CanPayloadLength];

	size_t max_copy = MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;
	size_t remaining = unique_id_length - unique_id_offset;

	if (max_copy > remaining) {
		max_copy = remaining;
	}

	payload[uavcan_byte_offset(Allocation, node_id)] = uavcan_pack(requested_node_id, Allocation, node_id) |
			(unique_id_offset ? 0 : uavcan_bit_mask(Allocation, first_part_of_unique_id));
	/*
	 * Copy in the remaining bytes of payload, either filling it
	 * or on the last chunk partially filling it
	 */
	memcpy(&payload[uavcan_byte_offset(Allocation, unique_id)], &unique_id[unique_id_offset], max_copy);

	/* Account for the payload[0] */
	max_copy++;
	uavcan_tx(&protocol, payload, max_copy, dsdl->mailbox);
	can_cancel_on_error(dsdl->mailbox);
}

/****************************************************************************
 * Private Data - This table is positioned here to not mess up the line
 * numbers for the debugger.
 ****************************************************************************/
#define NA 0
#define END_COMPONENTS

#define UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length)

#define UAVCAN_DSDL_MESG_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	{ \
		{\
			.msg = \
			       { \
				 .source_node_id = 0, \
				 .service_not_message = (false), \
				 .type_id = (dtid), \
				 .priority = 0, \
			       }, \
			       { \
				 .tail_init = \
					      { \
						.u8 = (outbound), \
					      } \
			       } \
		}, \
		signature, \
		inbound, \
		outbound, \
		mailbox, \
		fifo, \
	},

#define UAVCAN_DSDL_SREQ_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	{ \
		{\
			.ser = \
			       { \
				 .source_node_id = 0, \
				 .service_not_message = (true), \
				 .dest_node_id = 0, \
				 .request_not_response = (true), \
				 .type_id = (dtid), \
				 .priority = 0, \
			       }, \
			       { \
				 .tail_init = \
					      { \
						.u8 = (outbound), \
					      } \
			       } \
		}, \
		signature, \
		inbound, \
		outbound, \
		mailbox, \
		fifo, \
	},

#define UAVCAN_DSDL_SRSP_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	{ \
		{\
			.ser = \
			       { \
				 .source_node_id = 0, \
				 .service_not_message = (true), \
				 .dest_node_id = 0, \
				 .request_not_response = (false), \
				 .type_id = (dtid), \
				 .priority = 0, \
			       }, \
			       { \
				 .tail_init = \
					      { \
						.u8 = (outbound), \
					      } \
			       } \
		}, \
		signature, \
		inbound, \
		outbound, \
		mailbox, \
		fifo, \
	},


#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound)

const dsdl_t dsdl_table[sizeDSDLTransfers] = {
#include "uavcan_dsdl_defs.h"
};
#undef UAVCAN_DSDL_TYPE_DEF
#undef UAVCAN_DSDL_SRSP_DEF
#undef UAVCAN_DSDL_SREQ_DEF
#undef UAVCAN_DSDL_MESG_DEF
#undef UAVCAN_DSDL_BIT_DEF
#undef FW_SERVER
#undef REQ_NODE
#undef THIS_NODE
#undef ANY_NODE_ID
#undef END_COMPONENTS
#undef NA
