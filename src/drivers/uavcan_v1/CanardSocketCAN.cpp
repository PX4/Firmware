/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "CanardSocketCAN.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>

#include <px4_platform_common/log.h>

int CanardSocketCAN::init()
{
	const char *const can_iface_name = "can0";

	struct sockaddr_can addr;
	struct ifreq ifr;

	//FIXME HOTFIX to make this code compile
	bool can_fd = 0;

	_can_fd = can_fd;

	/* open socket */
	if ((_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		PX4_ERR("socket");
		return -1;
	}

	strncpy(ifr.ifr_name, can_iface_name, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		PX4_ERR("if_nametoindex");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	const int on = 1;
	/* RX Timestamping */

	if (setsockopt(_fd, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
		PX4_ERR("SO_TIMESTAMP is disabled");
		return -1;
	}

	/* NuttX Feature: Enable TX deadline when sending CAN frames
	 * When a deadline occurs the driver will remove the CAN frame
	 */

	if (setsockopt(_fd, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &on, sizeof(on)) < 0) {
		PX4_ERR("CAN_RAW_TX_DEADLINE is disabled");
		return -1;
	}

	if (can_fd) {
		if (setsockopt(_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
			PX4_ERR("no CAN FD support");
			return -1;
		}
	}

	if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind");
		return -1;
	}

	// Setup TX msg
	_send_iov.iov_base = &_send_frame;

	if (_can_fd) {
		_send_iov.iov_len = sizeof(struct canfd_frame);

	} else {
		_send_iov.iov_len = sizeof(struct can_frame);
	}

	memset(&_send_control, 0x00, sizeof(_send_control));

	_send_msg.msg_iov    = &_send_iov;
	_send_msg.msg_iovlen = 1;
	_send_msg.msg_control = &_send_control;
	_send_msg.msg_controllen = sizeof(_send_control);

	_send_cmsg = CMSG_FIRSTHDR(&_send_msg);
	_send_cmsg->cmsg_level = SOL_CAN_RAW;
	_send_cmsg->cmsg_type = CAN_RAW_TX_DEADLINE;
	_send_cmsg->cmsg_len = sizeof(struct timeval);
	_send_tv = (struct timeval *)CMSG_DATA(&_send_cmsg);

	// Setup RX msg
	_recv_iov.iov_base = &_recv_frame;

	if (can_fd) {
		_recv_iov.iov_len = sizeof(struct canfd_frame);

	} else {
		_recv_iov.iov_len = sizeof(struct can_frame);
	}

	memset(_recv_control, 0x00, sizeof(_recv_control));

	_recv_msg.msg_iov = &_recv_iov;
	_recv_msg.msg_iovlen = 1;
	_recv_msg.msg_control = &_recv_control;
	_recv_msg.msg_controllen = sizeof(_recv_control);
	_recv_cmsg = CMSG_FIRSTHDR(&_recv_msg);

	return 0;
}

int16_t CanardSocketCAN::transmit(const CanardFrame &txf, int timeout_ms)
{
	/* Copy CanardFrame to can_frame/canfd_frame */
	if (_can_fd) {
		_send_frame.can_id = txf.extended_can_id | CAN_EFF_FLAG;
		_send_frame.len = txf.payload_size;
		memcpy(&_send_frame.data, txf.payload, txf.payload_size);

	} else {
		struct can_frame *frame = (struct can_frame *)&_send_frame;
		frame->can_id = txf.extended_can_id | CAN_EFF_FLAG;
		frame->can_dlc = txf.payload_size;
		memcpy(&frame->data, txf.payload, txf.payload_size);
	}

	/* Set CAN_RAW_TX_DEADLINE timestamp  */
	_send_tv->tv_usec = txf.timestamp_usec % 1000000ULL;
	_send_tv->tv_sec = (txf.timestamp_usec - _send_tv->tv_usec) / 1000000ULL;

	return sendmsg(_fd, &_send_msg, 1000);
}

int16_t CanardSocketCAN::receive(CanardFrame *rxf)
{
	int32_t result = recvmsg(_fd, &_recv_msg, 1000);

	if (result < 0) {
		return result;
	}

	/* Copy CAN frame to CanardFrame */

	if (_can_fd) {
		struct canfd_frame *recv_frame = (struct canfd_frame *)&_recv_frame;
		rxf->extended_can_id = recv_frame->can_id & CAN_EFF_MASK;
		rxf->payload_size = recv_frame->len;
		rxf->payload = &recv_frame->data;

	} else {
		struct can_frame *recv_frame = (struct can_frame *)&_recv_frame;
		rxf->extended_can_id = recv_frame->can_id & CAN_EFF_MASK;
		rxf->payload_size = recv_frame->can_dlc;
		rxf->payload = &recv_frame->data; //FIXME either copy or clearly state the pointer reference
	}

	/* Read SO_TIMESTAMP value */

	if (_recv_cmsg->cmsg_level == SOL_SOCKET && _recv_cmsg->cmsg_type == SO_TIMESTAMP) {
		struct timeval *tv = (struct timeval *)CMSG_DATA(_recv_cmsg);
		rxf->timestamp_usec = tv->tv_sec * 1000000ULL + tv->tv_usec;
	}

	return result;
}
