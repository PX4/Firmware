/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdint.h>

#include <battery/battery.h>

#include <drivers/device/device.h>
#include "ringbuffer.h"

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/input_rc.h>

#include "syslink.h"
#include "crtp.h"

using namespace time_literals;

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

typedef enum {
	BAT_DISCHARGING = 0,
	BAT_CHARGING = 1,
	BAT_CHARGED = 2
} battery_state;


class SyslinkBridge;
class SyslinkMemory;

class Syslink
{
public:
	Syslink();

	int start();

	int set_datarate(uint8_t datarate);
	int set_channel(uint8_t channel);
	int set_address(uint64_t addr);

	int is_good(int i) { return _params_ack[i] != 0; }

	int pktrate;
	int nullrate;
	int rxrate;
	int txrate;

private:
	friend class SyslinkBridge;
	friend class SyslinkMemory;

	int open_serial(const char *dev);

	void handle_message(syslink_message_t *msg);
	void handle_raw(syslink_message_t *sys);
	void handle_radio(syslink_message_t *sys);
	void handle_bootloader(syslink_message_t *sys);

	// Handles other types of messages that we don't really care about, but
	// will be maintained with the bare minimum implementation for supporting
	// other crazyflie libraries
	void handle_raw_other(syslink_message_t *sys);

	int send_bytes(const void *data, size_t len);

	// Checksums and transmits a syslink message
	int send_message(syslink_message_t *msg);

	int send_queued_raw_message();

	void update_params(bool force_set);

	int _syslink_task;
	bool _task_running;
	bool _bootloader_mode;

	int _count;
	int _null_count;
	int _count_in;
	int _count_out;
	hrt_abstime _lasttime;
	hrt_abstime _lasttxtime; // Last time a radio message was sent
	hrt_abstime _lastrxtime; // Last time a radio message was recieved

	int _fd;

	// For receiving raw syslink messages to send from other processes
	ringbuffer::RingBuffer _queue;

	// Stores data that was needs to be written as a raw message
	ringbuffer::RingBuffer _writebuffer;
	SyslinkBridge *_bridge;
	SyslinkMemory *_memory;

	int _params_sub;

	// Current parameter values
	int32_t _channel, _rate;
	uint64_t _addr;
	hrt_abstime _params_update[3]; // Time at which the parameters were updated
	hrt_abstime _params_ack[3]; // Time at which the parameters were acknowledged by the nrf module

	uORB::PublicationMulti<input_rc_s>		_rc_pub{ORB_ID(input_rc)};

	// nrf chip schedules battery updates with SYSLINK_SEND_PERIOD_MS
	static constexpr uint32_t SYSLINK_BATTERY_STATUS_INTERVAL_US = 10_ms;
	Battery _battery{1, nullptr, SYSLINK_BATTERY_STATUS_INTERVAL_US};

	int32_t _rssi;
	battery_state _bstate;

	px4_sem_t memory_sem;

	syslink_message_t memory_msg;

	static int task_main_trampoline(int argc, char *argv[]);

	void task_main();
};


class SyslinkBridge : public cdev::CDev
{

public:
	SyslinkBridge(Syslink *link);
	virtual ~SyslinkBridge() = default;

	virtual int	init();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);
	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

	// Makes the message available for reading to processes reading from the bridge
	void pipe_message(crtp_message_t *msg);

protected:
	virtual pollevent_t poll_state(struct file *filp);

private:
	Syslink *_link;

	// Stores data that was received from syslink but not yet read by another driver
	ringbuffer::RingBuffer _readbuffer;

	crtp_message_t _msg_to_send;
	int _msg_to_send_size_remaining;
};


class SyslinkMemory : public cdev::CDev
{

public:
	SyslinkMemory(Syslink *link);
	virtual ~SyslinkMemory() = default;

	virtual int	init();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);
	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	friend class Syslink;

	Syslink *_link;

	int _activeI;

	syslink_message_t msgbuf;

	uint8_t scan();
	void getinfo(int i);

	int read(int i, uint16_t addr, char *buf, int length);
	int write(int i, uint16_t addr, const char *buf, int length);

	void sendAndWait();
};
