#include "socketcan.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>

int16_t socketcanOpen(CanardSocketInstance *ins, const char *const can_iface_name, const bool can_fd)
{
	struct sockaddr_can addr;
	struct ifreq ifr;

	ins->can_fd = can_fd;

	/* open socket */
	if ((ins->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return -1;
	}

	strncpy(ifr.ifr_name, can_iface_name, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		perror("if_nametoindex");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	const int on = 1;
	/* RX Timestamping */

	if (setsockopt(ins->s, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
		perror("SO_TIMESTAMP is disabled");
		return -1;
	}

	/* NuttX Feature: Enable TX deadline when sending CAN frames
	 * When a deadline occurs the driver will remove the CAN frame
	 */

	if (setsockopt(ins->s, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &on, sizeof(on)) < 0) {
		perror("CAN_RAW_TX_DEADLINE is disabled");
		return -1;
	}

	if (can_fd) {
		if (setsockopt(ins->s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
			perror("no CAN FD support");
			return -1;
		}
	}

	if (bind(ins->s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return -1;
	}

	// Setup TX msg
	ins->send_iov.iov_base = &ins->send_frame;

	if (ins->can_fd) {
		ins->send_iov.iov_len = sizeof(struct canfd_frame);

	} else {
		ins->send_iov.iov_len = sizeof(struct can_frame);
	}

	memset(&ins->send_control, 0x00, sizeof(ins->send_control));

	ins->send_msg.msg_iov    = &ins->send_iov;
	ins->send_msg.msg_iovlen = 1;
	ins->send_msg.msg_control = &ins->send_control;
	ins->send_msg.msg_controllen = sizeof(ins->send_control);

	ins->send_cmsg = CMSG_FIRSTHDR(&ins->send_msg);
	ins->send_cmsg->cmsg_level = SOL_CAN_RAW;
	ins->send_cmsg->cmsg_type = CAN_RAW_TX_DEADLINE;
	ins->send_cmsg->cmsg_len = sizeof(struct timeval);
	ins->send_tv = (struct timeval *)CMSG_DATA(&ins->send_cmsg);

	// Setup RX msg
	ins->recv_iov.iov_base = &ins->recv_frame;

	if (can_fd) {
		ins->recv_iov.iov_len = sizeof(struct canfd_frame);

	} else {
		ins->recv_iov.iov_len = sizeof(struct can_frame);
	}

	memset(&ins->recv_control, 0x00, sizeof(ins->recv_control));

	ins->recv_msg.msg_iov    = &ins->recv_iov;
	ins->recv_msg.msg_iovlen = 1;
	ins->recv_msg.msg_control = &ins->recv_control;
	ins->recv_msg.msg_controllen = sizeof(ins->recv_control);
	ins->recv_cmsg = CMSG_FIRSTHDR(&ins->recv_msg);

	return 0;
}

int16_t socketcanTransmit(CanardSocketInstance *ins, const CanardFrame *txf)
{
	/* Copy CanardFrame to can_frame/canfd_frame */

	if (ins->can_fd) {
		ins->send_frame.can_id = txf->extended_can_id;
		ins->send_frame.can_id |= CAN_EFF_FLAG;
		ins->send_frame.len = txf->payload_size;
		memcpy(&ins->send_frame.data, txf->payload, txf->payload_size);

	} else {
		struct can_frame *frame = (struct can_frame *)&ins->send_frame;
		frame->can_id = txf->extended_can_id;
		frame->can_id |= CAN_EFF_FLAG;
		frame->can_dlc = txf->payload_size;
		memcpy(&frame->data, txf->payload, txf->payload_size);
	}

	/* Set CAN_RAW_TX_DEADLINE timestamp  */

	ins->send_tv->tv_usec = txf->timestamp_usec % 1000000ULL;
	ins->send_tv->tv_sec = (txf->timestamp_usec - ins->send_tv->tv_usec) / 1000000ULL;

	return sendmsg(ins->s, &ins->send_msg, 0);
}

int16_t socketcanReceive(CanardSocketInstance *ins, CanardFrame *rxf)
{
	int32_t result = recvmsg(ins->s, &ins->recv_msg, 0);

	if (result < 0) {
		return result;
	}

	/* Copy CAN frame to CanardFrame */

	if (ins->can_fd) {
		struct canfd_frame *recv_frame = (struct canfd_frame *)&ins->recv_frame;
		rxf->extended_can_id = recv_frame->can_id & CAN_EFF_MASK;
		rxf->payload_size = recv_frame->len;
		rxf->payload = &recv_frame->data;

	} else {
		struct can_frame *recv_frame = (struct can_frame *)&ins->recv_frame;
		rxf->extended_can_id = recv_frame->can_id & CAN_EFF_MASK;
		rxf->payload_size = recv_frame->can_dlc;
		rxf->payload = &recv_frame->data; //FIXME either copy or clearly state the pointer reference
	}

	/* Read SO_TIMESTAMP value */

	if (ins->recv_cmsg->cmsg_level == SOL_SOCKET && ins->recv_cmsg->cmsg_type == SO_TIMESTAMP) {
		struct timeval *tv = (struct timeval *)CMSG_DATA(ins->recv_cmsg);
		rxf->timestamp_usec = tv->tv_sec * 1000000ULL + tv->tv_usec;
	}

	return result;
}


/* TODO implement corresponding IOCTL */

int16_t socketcanConfigureFilter(const fd_t fd, const size_t num_filters, const struct can_filter *filters)
{
	return -1;
}
