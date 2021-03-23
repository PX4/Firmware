/****************************************************************************
 *
 *   Copyright (c) 2012-2014, 2021 PX4 Development Team. All rights reserved.
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
 * @file mavlink_bridge_header
 * MAVLink bridge header for UART access.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

/* MAVLink adapter header */
#ifndef MAVLINK_BRIDGE_HEADER_H
#define MAVLINK_BRIDGE_HEADER_H

#define MAVLINK_NO_CONVERSION_HELPERS

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

/* use efficient approach, see mavlink_helpers.h */
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes

#define MAVLINK_START_UART_SEND mavlink_start_uart_send
#define MAVLINK_END_UART_SEND mavlink_end_uart_send

#define MAVLINK_GET_CHANNEL_BUFFER mavlink_get_channel_buffer
#define MAVLINK_GET_CHANNEL_STATUS mavlink_get_channel_status

#if !defined(CONSTRAINED_MEMORY)
# define MAVLINK_COMM_NUM_BUFFERS 6
# define MAVLINK_COMM_4 static_cast<mavlink_channel_t>(4)
# define MAVLINK_COMM_5 static_cast<mavlink_channel_t>(5)
#endif

#include <v2.0/mavlink_types.h>
#include <unistd.h>

__BEGIN_DECLS

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */
extern mavlink_system_t mavlink_system;

/**
 * @brief Send multiple chars (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length);

void mavlink_start_uart_send(mavlink_channel_t chan, int length);
void mavlink_end_uart_send(mavlink_channel_t chan, int length);

extern mavlink_status_t *mavlink_get_channel_status(uint8_t chan);
extern mavlink_message_t *mavlink_get_channel_buffer(uint8_t chan);

#define MAVLINK_DEVELOPMENT_BUILD

#ifdef MAVLINK_DEVELOPMENT_BUILD
#include <v2.0/development/mavlink.h>
#else
#include <v2.0/standard/mavlink.h>
#endif

__END_DECLS

#endif /* MAVLINK_BRIDGE_HEADER_H */
