/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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
 * tap_esc_common.cpp
 *
 */

#include "tap_esc_common.h"

#include <fcntl.h>
#include <termios.h>

#include <systemlib/px4_macros.h> // arraySize
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#ifndef B250000
#define B250000 250000
#endif

#if defined(GPIO_CAN1_SILENT_S0)
#  define ESC_MUX_SELECT0 GPIO_CAN1_SILENT_S0
#  define ESC_MUX_SELECT1 GPIO_CAN2_SILENT_S1
#  define ESC_MUX_SELECT2 GPIO_CAN3_SILENT_S2
#endif

namespace tap_esc_common
{
static uint8_t crc8_esc(uint8_t *p, uint8_t len);
static uint8_t crc_packet(EscPacket &p);

void select_responder(uint8_t sel)
{
#if defined(ESC_MUX_SELECT0)
	px4_arch_gpiowrite(ESC_MUX_SELECT0, sel & 1);
	px4_arch_gpiowrite(ESC_MUX_SELECT1, sel & 2);
	px4_arch_gpiowrite(ESC_MUX_SELECT2, sel & 4);
#endif
}

int initialise_uart(const char *const device, int &uart_fd)
{
	// open uart
	uart_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	int termios_state = -1;

	if (uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	// set baud rate
	int speed = B250000;
	struct termios uart_config;
	tcgetattr(uart_fd, &uart_config);

	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// set baud rate
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baudrate for %s: %d\n", device, termios_state);
		close(uart_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("tcsetattr failed for %s\n", device);
		close(uart_fd);
		return -1;
	}

	// setup output flow control
	if (enable_flow_control(uart_fd, false)) {
		PX4_WARN("hardware flow disable failed");
	}

	return 0;
}

int deinitialise_uart(int &uart_fd)
{
	int ret = close(uart_fd);

	if (ret == 0) {
		uart_fd = -1;
	}

	return ret;
}

int enable_flow_control(int uart_fd, bool enabled)
{
	struct termios uart_config;

	int ret = tcgetattr(uart_fd, &uart_config);

	if (ret != 0) {
		PX4_ERR("error getting uart configuration");
		return ret;
	}

	if (enabled) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;
	}

	return tcsetattr(uart_fd, TCSANOW, &uart_config);
}

int send_packet(int uart_fd, EscPacket &packet, int responder)
{
	if (responder >= 0) {
		select_responder(responder);
	}

	int packet_len = crc_packet(packet);
	int ret = ::write(uart_fd, &packet.head, packet_len);

	if (ret != packet_len) {
		PX4_WARN("TX ERROR: ret: %d, errno: %d", ret, errno);
	}

	return ret;
}

int read_data_from_uart(int uart_fd, ESC_UART_BUF *const uart_buf)
{
	uint8_t tmp_serial_buf[UART_BUFFER_SIZE];

	int len =::read(uart_fd, tmp_serial_buf, arraySize(tmp_serial_buf));

	if (len > 0 && (uart_buf->dat_cnt + len < UART_BUFFER_SIZE)) {
		for (int i = 0; i < len; i++) {
			uart_buf->esc_feedback_buf[uart_buf->tail++] = tmp_serial_buf[i];
			uart_buf->dat_cnt++;

			if (uart_buf->tail >= UART_BUFFER_SIZE) {
				uart_buf->tail = 0;
			}
		}

	} else if (len < 0) {
		return len;
	}

	return 0;
}

int parse_tap_esc_feedback(ESC_UART_BUF *const serial_buf, EscPacket *const packetdata)
{
	static PARSR_ESC_STATE state = HEAD;
	static uint8_t data_index = 0;
	static uint8_t crc_data_cal;

	if (serial_buf->dat_cnt > 0) {
		int count = serial_buf->dat_cnt;

		for (int i = 0; i < count; i++) {
			switch (state) {
			case HEAD:
				if (serial_buf->esc_feedback_buf[serial_buf->head] == PACKET_HEAD) {
					packetdata->head = PACKET_HEAD; //just_keep the format
					state = LEN;
				}

				break;

			case LEN:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < sizeof(packetdata->d)) {
					packetdata->len = serial_buf->esc_feedback_buf[serial_buf->head];
					state = ID;

				} else {
					state = HEAD;
				}

				break;

			case ID:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < ESCBUS_MSG_ID_MAX_NUM) {
					packetdata->msg_id = serial_buf->esc_feedback_buf[serial_buf->head];
					data_index = 0;
					state = DATA;

				} else {
					state = HEAD;
				}

				break;

			case DATA:
				packetdata->d.bytes[data_index++] = serial_buf->esc_feedback_buf[serial_buf->head];

				if (data_index >= packetdata->len) {

					crc_data_cal = crc8_esc((uint8_t *)(&packetdata->len), packetdata->len + 2);
					state = CRC;
				}

				break;

			case CRC:
				if (crc_data_cal == serial_buf->esc_feedback_buf[serial_buf->head]) {
					packetdata->crc_data = serial_buf->esc_feedback_buf[serial_buf->head];

					if (++serial_buf->head >= UART_BUFFER_SIZE) {
						serial_buf->head = 0;
					}

					serial_buf->dat_cnt--;
					state = HEAD;
					return 0;
				}

				state = HEAD;
				break;

			default:
				state = HEAD;
				break;

			}

			if (++serial_buf->head >= UART_BUFFER_SIZE) {
				serial_buf->head = 0;
			}

			serial_buf->dat_cnt--;
		}
	}

	return -1;
}

static uint8_t crc8_esc(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++) {
		crc = crc_table[crc^*p++];
	}

	return crc;
}

static uint8_t crc_packet(EscPacket &p)
{
	/* Calculate the crc over Len,ID,data */
	p.d.bytes[p.len] = crc8_esc(&p.len, p.len + 2);
	return p.len + offsetof(EscPacket, d) + 1;
}


const uint8_t crc_table[256] = {
	0x00, 0xE7, 0x29, 0xCE, 0x52, 0xB5, 0x7B, 0x9C, 0xA4, 0x43, 0x8D, 0x6A,
	0xF6, 0x11, 0xDF, 0x38, 0xAF, 0x48, 0x86, 0x61, 0xFD, 0x1A, 0xD4, 0x33,
	0x0B, 0xEC, 0x22, 0xC5, 0x59, 0xBE, 0x70, 0x97, 0xB9, 0x5E, 0x90, 0x77,
	0xEB, 0x0C, 0xC2, 0x25, 0x1D, 0xFA, 0x34, 0xD3, 0x4F, 0xA8, 0x66, 0x81,
	0x16, 0xF1, 0x3F, 0xD8, 0x44, 0xA3, 0x6D, 0x8A, 0xB2, 0x55, 0x9B, 0x7C,
	0xE0, 0x07, 0xC9, 0x2E, 0x95, 0x72, 0xBC, 0x5B, 0xC7, 0x20, 0xEE, 0x09,
	0x31, 0xD6, 0x18, 0xFF, 0x63, 0x84, 0x4A, 0xAD, 0x3A, 0xDD, 0x13, 0xF4,
	0x68, 0x8F, 0x41, 0xA6, 0x9E, 0x79, 0xB7, 0x50, 0xCC, 0x2B, 0xE5, 0x02,
	0x2C, 0xCB, 0x05, 0xE2, 0x7E, 0x99, 0x57, 0xB0, 0x88, 0x6F, 0xA1, 0x46,
	0xDA, 0x3D, 0xF3, 0x14, 0x83, 0x64, 0xAA, 0x4D, 0xD1, 0x36, 0xF8, 0x1F,
	0x27, 0xC0, 0x0E, 0xE9, 0x75, 0x92, 0x5C, 0xBB, 0xCD, 0x2A, 0xE4, 0x03,
	0x9F, 0x78, 0xB6, 0x51, 0x69, 0x8E, 0x40, 0xA7, 0x3B, 0xDC, 0x12, 0xF5,
	0x62, 0x85, 0x4B, 0xAC, 0x30, 0xD7, 0x19, 0xFE, 0xC6, 0x21, 0xEF, 0x08,
	0x94, 0x73, 0xBD, 0x5A, 0x74, 0x93, 0x5D, 0xBA, 0x26, 0xC1, 0x0F, 0xE8,
	0xD0, 0x37, 0xF9, 0x1E, 0x82, 0x65, 0xAB, 0x4C, 0xDB, 0x3C, 0xF2, 0x15,
	0x89, 0x6E, 0xA0, 0x47, 0x7F, 0x98, 0x56, 0xB1, 0x2D, 0xCA, 0x04, 0xE3,
	0x58, 0xBF, 0x71, 0x96, 0x0A, 0xED, 0x23, 0xC4, 0xFC, 0x1B, 0xD5, 0x32,
	0xAE, 0x49, 0x87, 0x60, 0xF7, 0x10, 0xDE, 0x39, 0xA5, 0x42, 0x8C, 0x6B,
	0x53, 0xB4, 0x7A, 0x9D, 0x01, 0xE6, 0x28, 0xCF, 0xE1, 0x06, 0xC8, 0x2F,
	0xB3, 0x54, 0x9A, 0x7D, 0x45, 0xA2, 0x6C, 0x8B, 0x17, 0xF0, 0x3E, 0xD9,
	0x4E, 0xA9, 0x67, 0x80, 0x1C, 0xFB, 0x35, 0xD2, 0xEA, 0x0D, 0xC3, 0x24,
	0xB8, 0x5F, 0x91, 0x76
};

} /* tap_esc_common */
