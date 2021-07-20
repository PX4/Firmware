/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

/*
 * tap_esc_common.h
 *
 */

#pragma once

#include <stdint.h>

#include "drv_tap_esc.h"

namespace tap_esc_common
{
/**
 *  Select tap esc responder data for serial interface by 74hct151. GPIOs to be
 *  defined in board_config.h
 *  @param sel ID of the ESC (responder) of which feedback is requested
 */
void select_responder(uint8_t sel);

/**
 *  Opens a device for use as UART.
 *  @param device UNIX path of UART device
 *  @param uart_fd file-descriptor of UART device
 *  @return 0 on success, -1 on error
 */
int initialise_uart(const char *const device, int &uart_fd);

/**
 *  Closes a device previously opened with initialise_uart().
 *  @param uart_fd file-descriptor of UART device as provided by initialise_uart()
 *  @return 0 on success, -1 on error
 */
int deinitialise_uart(int &uart_fd);

/**
 *  Enables/disables flow control for open UART device.
 *  @param uart_fd file-descriptor of UART device
 *  @param enabled Set true for enabling and false for disabling flow control
 *  @return 0 on success, -1 on error
 */
int enable_flow_control(int uart_fd, bool enabled);

/**
 *  Sends a packet to all ESCs and requests a specific ESC to respond
 *  @param uart_fd file-descriptor of UART device
 *  @param packet Packet to be sent to ESCs. CRC information will be added.
 *  @param responder ID of the ESC (responder) that should return feedback
 *  @return On success number of bytes written, on error -1
 */
int send_packet(int uart_fd, EscPacket &packet, int responder);

/**
 *  Read data from the UART into a buffer
 *  @param uart_fd file-descriptor of UART device
 *  @param uart_buf Buffer where incoming data will be stored
 *  @return 0 on success, -1 on error
 */
int read_data_from_uart(int uart_fd, ESC_UART_BUF *const uart_buf);

/**
 *  Parse feedback from an ESC
 *  @param serial_buf Buffer where incoming data will be stored
 *  @param packetdata Packet that will be populated with information from buffer
 *  @return 0 on success, -1 on error
 */
int parse_tap_esc_feedback(ESC_UART_BUF *const serial_buf, EscPacket *const packetdata);

/**
 *  Lookup-table for faster CRC computation when sending ESC packets.
 */
extern const uint8_t crc_table[];
} /* tap_esc_common */
