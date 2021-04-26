
#ifndef SOCKETCAN_H_INCLUDED
#define SOCKETCAN_H_INCLUDED

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <sys/time.h>
#include <sys/socket.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include <canard.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef struct CanardSocketInstance CanardSocketInstance;
typedef int fd_t;

struct CanardSocketInstance {
	fd_t               s;
	bool               can_fd;

	//// Send msg structure
	struct iovec       send_iov;
	struct canfd_frame send_frame;
	struct msghdr      send_msg;
	struct cmsghdr     *send_cmsg;
	struct timeval     *send_tv;   /* TX deadline timestamp */
	uint8_t            send_control[sizeof(struct cmsghdr) + sizeof(struct timeval)];

	//// Receive msg structure
	struct iovec       recv_iov;
	struct canfd_frame recv_frame;
	struct msghdr      recv_msg;
	struct cmsghdr     *recv_cmsg;
	uint8_t            recv_control[sizeof(struct cmsghdr) + sizeof(struct timeval)];
};

/// Creates a SocketCAN socket for corresponding iface can_iface_name
/// Also sets up the message structures required for socketcanTransmit & socketcanReceive
/// can_fd determines to use CAN FD frame when is 1, and classical CAN frame when is 0
/// The return value is 0 on succes and -1 on error
int16_t socketcanOpen(CanardSocketInstance *ins, const char *const can_iface_name, const bool can_fd);

/// Send a CanardFrame to the CanardSocketInstance socket
/// This function is blocking
/// The return value is number of bytes transferred, negative value on error.
int16_t socketcanTransmit(CanardSocketInstance *ins, const CanardFrame *txf);

/// Receive a CanardFrame from the CanardSocketInstance socket
/// This function is blocking
/// The return value is number of bytes received, negative value on error.
int16_t socketcanReceive(CanardSocketInstance *ins, CanardFrame *rxf);

// TODO implement ioctl for CAN filter
int16_t socketcanConfigureFilter(const fd_t fd, const size_t num_filters, const struct can_filter *filters);


#ifdef __cplusplus
}
#endif

#endif //SOCKETCAN_H_INCLUDED
