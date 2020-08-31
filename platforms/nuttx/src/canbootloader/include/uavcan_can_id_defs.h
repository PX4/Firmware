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

/* This file not a typical h file, is defines the UAVCAN dsdl
 * usage and may be included several times in header or source
 * file
 */

/* UAVCAN_BIT_DEFINE(    field_name,                    lsb_pos,             length) */

/* Message Frame Format */
UAVCAN_BIT_DEFINE(UavCanMessagePriority,                   24,              5)
UAVCAN_BIT_DEFINE(UavCanMessageTypeID,                      8,             16)
UAVCAN_BIT_DEFINE(UavCanMessageServiceNotMessage,           7,              1)
UAVCAN_BIT_DEFINE(UavCanMessageSourceNodeID,                0,              7)
/* Anonymous message Frame Format */
UAVCAN_BIT_DEFINE(UavCanAnonMessagePriority,               24,              5)
UAVCAN_BIT_DEFINE(UavCanAnonMessageDiscriminator,          10,             14)
UAVCAN_BIT_DEFINE(UavCanAnonMessageTypeID,                  8,              2)
UAVCAN_BIT_DEFINE(UavCanAnonMessageServiceNotMessage,       7,              1)
UAVCAN_BIT_DEFINE(UavCanAnonMessageSourceNodeID,            0,              7)
/* Service Frame Format */
UAVCAN_BIT_DEFINE(UavCanServicePriority,                   24,              5)
UAVCAN_BIT_DEFINE(UavCanServiceTypeID,                     16,              8)
UAVCAN_BIT_DEFINE(UavCanServiceRequestNotResponse,         15,              1)
UAVCAN_BIT_DEFINE(UavCanServiceDestinationNodeID,           8,              7)
UAVCAN_BIT_DEFINE(UavCanServiceServiceNotMessage,           7,              1)
UAVCAN_BIT_DEFINE(UavCanServiceSourceNodeID,                0,              7)
/* Tail Format */
UAVCAN_BIT_DEFINE(UavCanStartOfTransfer,                    7,              1)
UAVCAN_BIT_DEFINE(UavCanEndOfTransfer,                      6,              1)
UAVCAN_BIT_DEFINE(UavCanToggle,                             5,              1)
UAVCAN_BIT_DEFINE(UavCanTransferID,                         0,              5)
