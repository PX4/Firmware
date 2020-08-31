/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <sys/socket.h>
#include <cstdlib>
#if __has_include("px4_platform_common/log.h") && __has_include("px4_platform_common/time.h")
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#endif

#include "microRTPS_transport.h"

/** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
uint16_t const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

Transport_node::Transport_node(const bool _debug):
    rx_buff_pos(0),
    debug(_debug)
{
}

Transport_node::~Transport_node()
{
}

uint16_t Transport_node::crc16_byte(uint16_t crc, const uint8_t data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

uint16_t Transport_node::crc16(uint8_t const *buffer, size_t len)
{
	uint16_t crc = 0;

	while (len--) {
		crc = crc16_byte(crc, *buffer++);
	}

	return crc;
}

ssize_t Transport_node::read(uint8_t *topic_ID, char out_buffer[], size_t buffer_len)
{
	if (nullptr == out_buffer || nullptr == topic_ID || !fds_OK()) {
		return -1;
	}

	*topic_ID = 255;

	ssize_t len = node_read((void *)(rx_buffer + rx_buff_pos), sizeof(rx_buffer) - rx_buff_pos);

	if (len < 0) {
		int errsv = errno;

		if (errsv && EAGAIN != errsv && ETIMEDOUT != errsv) {
#ifndef PX4_DEBUG
			if (debug) printf("\033[0;31m[ micrortps_transport ]\tRead fail %d\033[0m\n", errsv);
#else
			if (debug) PX4_DEBUG("Read fail %d", errsv);
#endif /* PX4_DEBUG */
		}

		return len;
	}

	rx_buff_pos += len;

	// We read some
	size_t header_size = sizeof(struct Header);

	// but not enough
	if (rx_buff_pos < header_size) {
		return 0;
	}

	uint32_t msg_start_pos = 0;

	for (msg_start_pos = 0; msg_start_pos <= rx_buff_pos - header_size; ++msg_start_pos) {
		if ('>' == rx_buffer[msg_start_pos] && memcmp(rx_buffer + msg_start_pos, ">>>", 3) == 0) {
			break;
		}
	}

	// Start not found
	if (msg_start_pos > (rx_buff_pos - header_size)) {
#ifndef PX4_DEBUG
		if (debug) printf("\033[1;33m[ micrortps_transport ]\t                                (↓↓ %u)\033[0m\n", msg_start_pos);
#else
		if (debug) PX4_DEBUG("                               (↓↓ %u)", msg_start_pos);
#endif /* PX4_DEBUG */

		// All we've checked so far is garbage, drop it - but save unchecked bytes
		memmove(rx_buffer, rx_buffer + msg_start_pos, rx_buff_pos - msg_start_pos);
		rx_buff_pos -= msg_start_pos;
		return -1;
	}

	// [>,>,>,topic_ID,seq,payload_length_H,payload_length_L,CRCHigh,CRCLow,payloadStart, ... ,payloadEnd]
	struct Header *header = (struct Header *)&rx_buffer[msg_start_pos];
	uint32_t payload_len = ((uint32_t)header->payload_len_h << 8) | header->payload_len_l;

	// The message won't fit the buffer.
	if (buffer_len < header_size + payload_len) {
		// Drop the message and continue with the read buffer
		memmove(rx_buffer, rx_buffer + msg_start_pos + 1, rx_buff_pos - (msg_start_pos + 1));
		rx_buff_pos -= (msg_start_pos + 1);
		return -EMSGSIZE;
	}

	// We do not have a complete message yet
	if (msg_start_pos + header_size + payload_len > rx_buff_pos) {
		// If there's garbage at the beginning, drop it
		if (msg_start_pos > 0) {
#ifndef PX4_DEBUG
			if (debug) printf("\033[1;33m[ micrortps_transport ]\t                                (↓ %u)\033[0m\n", msg_start_pos);
#else
			if (debug) PX4_DEBUG("                             (↓ %u)", msg_start_pos);
#endif /* PX4_DEBUG */
			memmove(rx_buffer, rx_buffer + msg_start_pos, rx_buff_pos - msg_start_pos);
			rx_buff_pos -= msg_start_pos;
		}

		return 0;
	}

	uint16_t read_crc = ((uint16_t)header->crc_h << 8) | header->crc_l;
	uint16_t calc_crc = crc16((uint8_t *)rx_buffer + msg_start_pos + header_size, payload_len);

	if (read_crc != calc_crc) {
#ifndef PX4_DEBUG
		if (debug) printf("\033[0;31m[ micrortps_transport ]\tBad CRC %u != %u\t\t(↓ %lu)\033[0m\n", read_crc, calc_crc, (unsigned long)(header_size + payload_len));
#else
		if (debug) PX4_DEBUG("Bad CRC %u != %u\t\t(↓ %lu)", read_crc, calc_crc, (unsigned long)(header_size + payload_len));
#endif /* PX4_DEBUG */

		// Drop garbage up just beyond the start of the message
		memmove(rx_buffer, rx_buffer + (msg_start_pos + 1), rx_buff_pos);

		// If there is a CRC error, the payload len cannot be trusted
		rx_buff_pos -= (msg_start_pos + 1);

		len = -1;

	} else {
		// copy message to outbuffer and set other return values
		memmove(out_buffer, rx_buffer + msg_start_pos + header_size, payload_len);
		*topic_ID = header->topic_ID;
		len = payload_len + header_size;

		// discard message from rx_buffer
		rx_buff_pos -= msg_start_pos + header_size + payload_len;
		memmove(rx_buffer, rx_buffer + msg_start_pos + header_size + payload_len, rx_buff_pos);
	}

	return len;
}

size_t Transport_node::get_header_length()
{
    return sizeof(struct Header);
}

ssize_t Transport_node::write(const uint8_t topic_ID, char buffer[], size_t length)
{
	if (!fds_OK()) {
		return -1;
	}

	static struct Header header = {{'>', '>', '>'}, 0u, 0u, 0u, 0u, 0u, 0u};

	// [>,>,>,topic_ID,seq,payload_length,CRCHigh,CRCLow,payload_start, ... ,payload_end]
	uint16_t crc = crc16((uint8_t *)&buffer[sizeof(header)], length);

	header.topic_ID = topic_ID;
	header.seq = _seq_number++;
	header.payload_len_h = (length >> 8) & 0xff;
	header.payload_len_l = length & 0xff;
	header.crc_h = (crc >> 8) & 0xff;
	header.crc_l = crc & 0xff;

	/* Headroom for header is created in client */
	/* Fill in the header in the same payload buffer to call a single node_write */
	memcpy(buffer, &header, sizeof(header));
	ssize_t len = node_write(buffer, length + sizeof(header));
	if (len != ssize_t(length + sizeof(header))) {
		return len;
	}
	return len + sizeof(header);
}

UART_node::UART_node(const char *_uart_name, const uint32_t _baudrate,
					 const uint32_t _poll_ms, const bool _hw_flow_control,
					 const bool _sw_flow_control, const bool _debug):
	Transport_node(_debug),
	uart_fd(-1),
	baudrate(_baudrate),
	poll_ms(_poll_ms),
	hw_flow_control(_hw_flow_control),
	sw_flow_control(_sw_flow_control)
{

	if (nullptr != _uart_name) {
		strcpy(uart_name, _uart_name);
	}
}

UART_node::~UART_node()
{
	close();
}

int UART_node::init()
{
	// Open a serial port
	uart_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (uart_fd < 0) {
#ifndef PX4_ERR
		printf("\033[0;31m[ micrortps_transport ]\tUART transport: Failed to open device: %s (%d)\033[0m\n", uart_name, errno);
#else
		PX4_ERR("UART transport: Failed to open device: %s (%d)", uart_name, errno);
#endif /* PX4_ERR */
		return -errno;
	}

	// If using shared UART, no need to set it up
	if (baudrate == 0) {
		return uart_fd;
	}

	// Try to set baud rate
	struct termios uart_config;
	int termios_state;

	// Back up the original uart configuration to restore it after exit
	if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
		int errno_bkp = errno;
#ifndef PX4_ERR
		printf("\033[0;31m[ micrortps_transport ]\tUART transport: ERR GET CONF %s: %d (%d)\n\033[0m", uart_name, termios_state, errno);
#else
		PX4_ERR("UART transport: ERR GET CONF %s: %d (%d)", uart_name, termios_state, errno);
#endif /* PX4_ERR */
		close();
		return -errno_bkp;
	}

	// Set up the UART for non-canonical binary communication: 8 bits, 1 stop bit, no parity.
	uart_config.c_iflag &= !(INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXANY | IXOFF);
	uart_config.c_iflag |= IGNBRK | IGNPAR;

	uart_config.c_oflag &= !(OPOST | ONLCR | OCRNL | ONOCR | ONLRET | OFILL | NLDLY | VTDLY);
	uart_config.c_oflag |= NL0 | VT0;

	uart_config.c_cflag &= !(CSIZE | CSTOPB | PARENB);
	uart_config.c_cflag |= CS8 | CREAD | CLOCAL;

	uart_config.c_lflag &= !(ISIG | ICANON | ECHO | TOSTOP | IEXTEN);

	// Flow control
	if (hw_flow_control) {
		// HW flow control
		uart_config.c_lflag |= CRTSCTS;
	} else if (sw_flow_control) {
		// SW flow control
		uart_config.c_lflag |= (IXON | IXOFF | IXANY);
	}

	// Set baud rate
	speed_t speed;

	if (!baudrate_to_speed(baudrate, &speed)) {
#ifndef PX4_ERR
		printf("\033[0;31m[ micrortps_transport ]\tUART transport: ERR SET BAUD %s: Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 921600, 1000000\033[0m\n",
			uart_name, baudrate);
#else
		PX4_ERR("UART transport: ERR SET BAUD %s: Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 921600, 1000000\n",
			uart_name, baudrate);
#endif /* PX4_ERR */
		close();
		return -EINVAL;
	}

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		int errno_bkp = errno;
#ifndef PX4_ERR
		printf("\033[0;31m[ micrortps_transport ]\tUART transport: ERR SET BAUD %s: %d (%d)\033[0m\n", uart_name, termios_state, errno);
#else
		PX4_ERR("ERR SET BAUD %s: %d (%d)", uart_name, termios_state, errno);
#endif /* PX4_ERR */
		close();
		return -errno_bkp;
	}

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		int errno_bkp = errno;
#ifndef PX4_ERR
		printf("\033[0;31m[ micrortps_transport ]\tUART transport: ERR SET CONF %s (%d)\033[0m\n", uart_name, errno);
#else
		PX4_ERR("UART transport: ERR SET CONF %s (%d)", uart_name, errno);
#endif /* PX4_ERR */
		close();
		return -errno_bkp;
	}

	char aux[64];
	bool flush = false;

	while (0 < ::read(uart_fd, (void *)&aux, 64)) {
		flush = true;
/**
 * According to px4_time.h, px4_usleep() is only defined when lockstep is set
 * to be used
 */
#ifndef px4_usleep
		usleep(1000);
#else
		px4_usleep(1000);
#endif /* px4_usleep */
	}

	if (flush) {
#ifndef PX4_DEBUG
		if (debug) printf("[ micrortps_transport ]\tUART transport: Flush\n");
#else
		if (debug) PX4_DEBUG("UART transport: Flush");
#endif /* PX4_DEBUG */
	} else {
#ifndef PX4_DEBUG
		if (debug) printf("[ micrortps_transport ]\tUART transport: No flush\n");
#else
		if (debug) PX4_DEBUG("UART transport: No flush");
#endif /* PX4_INFO */
	}

	poll_fd[0].fd = uart_fd;
	poll_fd[0].events = POLLIN;

	return uart_fd;
}

bool UART_node::fds_OK()
{
	return (-1 != uart_fd);
}

uint8_t UART_node::close()
{
	if (-1 != uart_fd) {
#ifndef PX4_WARN
		printf("\033[1;33m[ micrortps_transport ]\tClosed UART.\n\033[0m");
#else
		PX4_WARN("Closed UART.");
#endif /* PX4_WARN */
		::close(uart_fd);
		uart_fd = -1;
		memset(&poll_fd, 0, sizeof(poll_fd));
	}

	return 0;
}

ssize_t UART_node::node_read(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	ssize_t ret = 0;
	int r = poll(poll_fd, 1, poll_ms);

	if (r == 1 && (poll_fd[0].revents & POLLIN)) {
		ret = ::read(uart_fd, buffer, len);
	}

	return ret;
}

ssize_t UART_node::node_write(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	return ::write(uart_fd, buffer, len);
}

bool UART_node::baudrate_to_speed(uint32_t bauds, speed_t *speed)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

#ifndef B1500000
#define B1500000 1500000
#endif

#ifndef B2000000
#define B2000000 2000000
#endif

	switch (bauds) {
	case 0:      *speed = B0;		break;

	case 50:     *speed = B50;		break;

	case 75:     *speed = B75;		break;

	case 110:    *speed = B110;		break;

	case 134:    *speed = B134;		break;

	case 150:    *speed = B150;		break;

	case 200:    *speed = B200;		break;

	case 300:    *speed = B300;		break;

	case 600:    *speed = B600;		break;

	case 1200:   *speed = B1200;		break;

	case 1800:   *speed = B1800;		break;

	case 2400:   *speed = B2400;		break;

	case 4800:   *speed = B4800;		break;

	case 9600:   *speed = B9600;		break;

	case 19200:  *speed = B19200;		break;

	case 38400:  *speed = B38400;		break;

	case 57600:  *speed = B57600;		break;

	case 115200: *speed = B115200;		break;

	case 230400: *speed = B230400;		break;

	case 460800: *speed = B460800;		break;

	case 500000: *speed = B500000;		break;

	case 921600: *speed = B921600;		break;

	case 1000000: *speed = B1000000;	break;

	case 1500000: *speed = B1500000;	break;

	case 2000000: *speed = B2000000;	break;

#ifdef B3000000

	case 3000000: *speed = B3000000;    break;
#endif

#ifdef B3500000

	case 3500000: *speed = B3500000;    break;
#endif

#ifdef B4000000

	case 4000000: *speed = B4000000;    break;
#endif

	default:
		return false;
	}

	return true;
}

UDP_node::UDP_node(const char* _udp_ip, uint16_t _udp_port_recv,
				   uint16_t _udp_port_send, const bool _debug):
	Transport_node(_debug),
	sender_fd(-1),
	receiver_fd(-1),
	udp_port_recv(_udp_port_recv),
	udp_port_send(_udp_port_send)
{
    if (nullptr != _udp_ip) {
            strcpy(udp_ip, _udp_ip);
    }
}

UDP_node::~UDP_node()
{
	close();
}

int UDP_node::init()
{
	if (0 > init_receiver(udp_port_recv) || 0 > init_sender(udp_port_send)) {
		return -1;
	}

	return 0;
}

bool UDP_node::fds_OK()
{
	return (-1 != sender_fd && -1 != receiver_fd);
}

int UDP_node::init_receiver(uint16_t udp_port)
{
#if !defined (__PX4_NUTTX) || (defined (CONFIG_NET) && defined (__PX4_NUTTX))
	// udp socket data
	memset((char *)&receiver_inaddr, 0, sizeof(receiver_inaddr));
	receiver_inaddr.sin_family = AF_INET;
	receiver_inaddr.sin_port = htons(udp_port);
	receiver_inaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	if ((receiver_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
#ifndef PX4_ERR
		printf("\033[0;31m[ micrortps_transport ]\tUDP transport: Create socket failed\033[0m\n");
#else
		PX4_ERR("UDP transport: Create socket failed");
#endif /* PX4_ERR */
		return -1;
	}
#ifndef PX4_INFO
	printf("[ micrortps_transport ]\tUDP transport: Trying to connect...");
#else
	PX4_INFO("UDP transport: Trying to connect...");
#endif /* PX4_INFO */

	if (bind(receiver_fd, (struct sockaddr *)&receiver_inaddr, sizeof(receiver_inaddr)) < 0) {
#ifndef PX4_ERR
		printf("\033[0;31m[ micrortps_transport ]\tUDP transport: Bind failed\033[0m\n");
#else
		PX4_ERR("UDP transport: Bind failed");
#endif /* PX4_ERR */
		return -1;
	}
#ifndef PX4_INFO
	printf("[ micrortps_transport ]\tUDP transport: Connected to server!\n\n");
#else
	PX4_INFO("UDP transport: Connected to server!");
#endif /* PX4_INFO */
#endif /* __PX4_NUTTX */
	return 0;
}

int UDP_node::init_sender(uint16_t udp_port)
{
#if !defined (__PX4_NUTTX) || (defined (CONFIG_NET) && defined (__PX4_NUTTX))

	if ((sender_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
#ifndef PX4_ERR
		printf("\033[0;31m[ micrortps_transport ]\tUDP transport: Create socket failed\033[0m\n");
#else
		PX4_ERR("UDP transport: Create socket failed");
#endif /* PX4_ERR */
		return -1;
	}

	memset((char *) &sender_outaddr, 0, sizeof(sender_outaddr));
	sender_outaddr.sin_family = AF_INET;
	sender_outaddr.sin_port = htons(udp_port);

	if (inet_aton(udp_ip, &sender_outaddr.sin_addr) == 0) {
#ifndef PX4_ERR
		printf("\033[0;31m[ micrortps_transport ]\tUDP transport: inet_aton() failed\033[0m\n");
#else
		PX4_ERR("UDP transport: inet_aton() failed");
#endif /* PX4_ERR */
		return -1;
	}

#endif /* __PX4_NUTTX */

	return 0;
}

uint8_t UDP_node::close()
{
#if !defined (__PX4_NUTTX) || (defined (CONFIG_NET) && defined (__PX4_NUTTX))

	if (sender_fd != -1) {
#ifndef PX4_WARN
		printf("\033[1;33m[ micrortps_transport ]\tUDP transport: Closed sender socket!\033[0m\n");
#else
		PX4_WARN("UDP transport: Closed sender socket!");
#endif /* PX4_WARN */
		shutdown(sender_fd, SHUT_RDWR);
		::close(sender_fd);
		sender_fd = -1;
	}

	if (receiver_fd != -1) {
#ifndef PX4_WARN
		printf("\033[1;33m[ micrortps_transport ]\tUDP transport: Closed receiver socket!\033[0m\n");
#else
		PX4_WARN("UDP transport: Closed receiver socket!");
#endif /* PX4_WARN */
		shutdown(receiver_fd, SHUT_RDWR);
		::close(receiver_fd);
		receiver_fd = -1;
	}

#endif /* __PX4_NUTTX */
	return 0;
}

ssize_t UDP_node::node_read(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	int ret = 0;
#if !defined (__PX4_NUTTX) || (defined (CONFIG_NET) && defined (__PX4_NUTTX))
	// Blocking call
	static socklen_t addrlen = sizeof(receiver_outaddr);
	ret = recvfrom(receiver_fd, buffer, len, 0, (struct sockaddr *) &receiver_outaddr, &addrlen);
#endif /* __PX4_NUTTX */
	return ret;
}

ssize_t UDP_node::node_write(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	int ret = 0;
#if !defined (__PX4_NUTTX) || (defined (CONFIG_NET) && defined (__PX4_NUTTX))
	ret = sendto(sender_fd, buffer, len, 0, (struct sockaddr *)&sender_outaddr, sizeof(sender_outaddr));
#endif /* __PX4_NUTTX */
	return ret;
}
