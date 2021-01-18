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

/**
 * @file syslink_main.cpp
 * Entry point for syslink module used to communicate with the NRF module on a Crazyflie
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <mqueue.h>

#include <drivers/drv_rc_input.h>
#include <drivers/drv_board_led.h>

#include <systemlib/err.h>

#include <board_config.h>

#include "crtp.h"
#include "syslink_main.h"
#include "drv_deck.h"



__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

extern "C" { __EXPORT int syslink_main(int argc, char *argv[]); }


Syslink *g_syslink = nullptr;

Syslink::Syslink() :
	pktrate(0),
	nullrate(0),
	rxrate(0),
	txrate(0),
	_syslink_task(-1),
	_task_running(false),
	_bootloader_mode(false),
	_count(0),
	_null_count(0),
	_count_in(0),
	_count_out(0),
	_lasttime(0),
	_lasttxtime(0),
	_lastrxtime(0),
	_fd(0),
	_queue(2, sizeof(syslink_message_t)),
	_writebuffer(16, sizeof(crtp_message_t)),
	_rssi(RC_INPUT_RSSI_MAX),
	_bstate(BAT_DISCHARGING)
{
	px4_sem_init(&memory_sem, 0, 0);
	/* memory_sem use case is a signal */
	px4_sem_setprotocol(&memory_sem, SEM_PRIO_NONE);
}


int
Syslink::start()
{
	_task_running = true;
	_syslink_task = px4_task_spawn_cmd(
				"syslink",
				SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT,
				1500,
				Syslink::task_main_trampoline,
				NULL
			);

	return 0;
}


int
Syslink::set_datarate(uint8_t datarate)
{
	syslink_message_t msg;
	msg.type = SYSLINK_RADIO_DATARATE;
	msg.length = 1;
	msg.data[0] = datarate;
	return send_message(&msg);
}

int
Syslink::set_channel(uint8_t channel)
{
	syslink_message_t msg;
	msg.type = SYSLINK_RADIO_CHANNEL;
	msg.length = 1;
	msg.data[0] = channel;
	return send_message(&msg);
}

int
Syslink::set_address(uint64_t addr)
{
	syslink_message_t msg;
	msg.type = SYSLINK_RADIO_ADDRESS;
	msg.length = 5;
	memcpy(&msg.data, &addr, 5);
	return send_message(&msg);
}

int
Syslink::send_queued_raw_message()
{
	if (_writebuffer.empty()) {
		return 0;
	}

	_lasttxtime = hrt_absolute_time();

	syslink_message_t msg;
	msg.type = SYSLINK_RADIO_RAW;

	_count_out++;

	_writebuffer.get(&msg.length, sizeof(crtp_message_t));

	return send_message(&msg);
}


void
Syslink::update_params(bool force_set)
{
	param_t _param_radio_channel = param_find("SLNK_RADIO_CHAN");
	param_t _param_radio_rate = param_find("SLNK_RADIO_RATE");
	param_t _param_radio_addr1 = param_find("SLNK_RADIO_ADDR1");
	param_t _param_radio_addr2 = param_find("SLNK_RADIO_ADDR2");


	// reading parameter values into temp variables

	int32_t channel, rate, addr1, addr2;
	uint64_t addr = 0;

	param_get(_param_radio_channel, &channel);
	param_get(_param_radio_rate, &rate);
	param_get(_param_radio_addr1, &addr1);
	param_get(_param_radio_addr2, &addr2);

	memcpy(&addr, &addr2, 4);
	memcpy(((char *)&addr) + 4, &addr1, 4);


	hrt_abstime t = hrt_absolute_time();

	// request updates if needed

	if (channel != this->_channel || force_set) {
		this->_channel = channel;
		set_channel(channel);
		this->_params_update[0] = t;
		this->_params_ack[0] = 0;
	}

	if (rate != this->_rate || force_set) {
		this->_rate = rate;
		set_datarate(rate);
		this->_params_update[1] = t;
		this->_params_ack[1] = 0;
	}

	if (addr != this->_addr || force_set) {
		this->_addr = addr;
		set_address(addr);
		this->_params_update[2] = t;
		this->_params_ack[2] = 0;
	}
}

// 1M 8N1 serial connection to NRF51
int
Syslink::open_serial(const char *dev)
{
#ifndef B1000000
#define B1000000 1000000
#endif

	int rate = B1000000;

	// open uart
	int fd = px4_open(dev, O_RDWR | O_NOCTTY);
	int termios_state = -1;

	if (fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	// set baud rate
	struct termios config;
	tcgetattr(fd, &config);

	// clear ONLCR flag (which appends a CR for every LF)
	config.c_oflag = 0;
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Disable hardware flow control
	config.c_cflag &= ~CRTSCTS;


	/* Set baud rate */
	if (cfsetispeed(&config, rate) < 0 || cfsetospeed(&config, rate) < 0) {
		warnx("ERR SET BAUD %s: %d\n", dev, termios_state);
		px4_close(fd);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", dev);
		px4_close(fd);
		return -1;
	}

	return fd;
}

int
Syslink::task_main_trampoline(int argc, char *argv[])
{
	g_syslink->task_main();
	return 0;
}

void
Syslink::task_main()
{
	_bridge = new SyslinkBridge(this);
	_bridge->init();

	_memory = new SyslinkMemory(this);
	_memory->init();

	/* Open serial port */
	const char *dev = "/dev/ttyS2";
	_fd = open_serial(dev);

	if (_fd < 0) {
		err(1, "can't open %s", dev);
		return;
	}

	/* Set non-blocking */
	/*
	int flags = fcntl(_fd, F_GETFL, 0);
	fcntl(_fd, F_SETFL, flags | O_NONBLOCK);
	*/

	px4_arch_configgpio(GPIO_NRF_TXEN);

	px4_pollfd_struct_t fds[2];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	fds[1].fd = _params_sub;
	fds[1].events = POLLIN;

	int error_counter = 0;

	char buf[64];
	int nread;

	syslink_parse_state state;
	syslink_message_t msg;

	syslink_parse_init(&state);

	//setup initial parameters
	update_params(true);

	while (_task_running) {
		int poll_ret = px4_poll(fds, 2, 500);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* timeout: this means none of our providers is giving us data */

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("[syslink] ERROR return value from poll(): %d"
					, poll_ret);
			}

			error_counter++;

		} else {
			if (fds[0].revents & POLLIN) {
				if ((nread = read(_fd, buf, sizeof(buf))) < 0) {
					continue;
				}

				for (int i = 0; i < nread; i++) {
					if (syslink_parse_char(&state, buf[i], &msg)) {
						handle_message(&msg);
					}
				}
			}

			if (fds[1].revents & POLLIN) {
				parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), _params_sub, &update);
				update_params(false);
			}
		}

	}

	close(_fd);
}

void
Syslink::handle_message(syslink_message_t *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - _lasttime > 1000000) {
		pktrate = _count;
		rxrate = _count_in;
		txrate = _count_out;
		nullrate = _null_count;

		_lasttime = t;
		_count = 0;
		_null_count = 0;
		_count_in = 0;
		_count_out = 0;
	}

	_count++;

	if (msg->type == SYSLINK_PM_ONOFF_SWITCHOFF) {
		// When the power button is hit
	} else if (msg->type == SYSLINK_PM_BATTERY_STATE) {

		if (msg->length != 9) {
			return;
		}

		uint8_t flags = msg->data[0];
		int charging = flags & 1;
		int powered = flags & 2;

		float vbat; //, iset;
		memcpy(&vbat, &msg->data[1], sizeof(float));
		//memcpy(&iset, &msg->data[5], sizeof(float));

		_battery.updateBatteryStatus(vbat);

		// Update battery charge state
		if (charging) {
			_bstate = BAT_CHARGING;
		}

		/* With the usb plugged in and battery disconnected, it appears to be charged. The voltage check ensures that a battery is connected  */
		else if (powered && !charging && vbat > 3.7f) {
			_bstate = BAT_CHARGED;

		} else {
			_bstate = BAT_DISCHARGING;
		}

	} else if (msg->type == SYSLINK_RADIO_RSSI) {
		uint8_t rssi = msg->data[0]; // Between 40 and 100 meaning -40 dBm to -100 dBm
		_rssi = 140 - rssi * 100 / (100 - 40);

	} else if (msg->type == SYSLINK_RADIO_RAW) {
		handle_raw(msg);
		_lastrxtime = t;

	} else if ((msg->type & SYSLINK_GROUP) == SYSLINK_RADIO) {
		handle_radio(msg);

	} else if ((msg->type & SYSLINK_GROUP) == SYSLINK_OW) {
		memcpy(&_memory->msgbuf, msg, sizeof(syslink_message_t));
		px4_sem_post(&memory_sem);

	} else {
		PX4_INFO("GOT %d", msg->type);
	}

	//Send queued messages
	if (!_queue.empty()) {
		_queue.get(msg, sizeof(syslink_message_t));
		send_message(msg);
	}


	float p = (t % 500000) / 500000.0f;

	/* Use LED_GREEN for charging indicator */
	if (_bstate == BAT_CHARGED) {
		led_on(LED_GREEN);

	} else if (_bstate == BAT_CHARGING && p < 0.25f) {
		led_on(LED_GREEN);

	} else {
		led_off(LED_GREEN);
	}

	/* Alternate RX/TX LEDS when transfering */
	bool rx = t - _lastrxtime < 200000,
	     tx = t - _lasttxtime < 200000;


	if (rx && p < 0.25f) {
		led_on(LED_RX);

	} else {
		led_off(LED_RX);
	}

	if (tx && p > 0.5f && p > 0.75f) {
		led_on(LED_TX);

	} else {
		led_off(LED_TX);
	}


	// resend parameters if they haven't been acknowledged
	if (_params_ack[0] == 0 && t - _params_update[0] > 10000) {
		set_channel(_channel);

	} else if (_params_ack[1] == 0 && t - _params_update[1] > 10000) {
		set_datarate(_rate);

	} else if (_params_ack[2] == 0 && t - _params_update[2] > 10000) {
		set_address(_addr);
	}
}

void
Syslink::handle_radio(syslink_message_t *sys)
{
	hrt_abstime t = hrt_absolute_time();

	// record acknowlegements to parameter messages
	if (sys->type == SYSLINK_RADIO_CHANNEL) {
		_params_ack[0] = t;

	} else if (sys->type == SYSLINK_RADIO_DATARATE) {
		_params_ack[1] = t;

	} else if (sys->type == SYSLINK_RADIO_ADDRESS) {
		_params_ack[2] = t;
	}
}

void
Syslink::handle_raw(syslink_message_t *sys)
{
	crtp_message_t *c = (crtp_message_t *) &sys->length;

	if (CRTP_NULL(*c)) {
		if (c->size >= 3) {
			handle_bootloader(sys);
		}

		_null_count++;

	} else if (c->port == CRTP_PORT_COMMANDER) {

		crtp_commander *cmd = (crtp_commander *) &c->data[0];

		input_rc_s rc = {};

		rc.timestamp = hrt_absolute_time();
		rc.timestamp_last_signal = rc.timestamp;
		rc.channel_count = 5;
		rc.rc_failsafe = false;
		rc.rc_lost = false;
		rc.rc_lost_frame_count = 0;
		rc.rc_total_frame_count = 1;
		rc.rc_ppm_frame_length = 0;
		rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
		rc.rssi = _rssi;


		double pitch = cmd->pitch, roll = cmd->roll, yaw = cmd->yaw;

		/* channels (scaled to rc limits) */
		rc.values[0] = pitch * 500 / 20 + 1500;
		rc.values[1] = roll * 500 / 20 + 1500;
		rc.values[2] = yaw * 500 / 150 + 1500;
		rc.values[3] = cmd->thrust * 1000 / USHRT_MAX + 1000;
		rc.values[4] = 1000; // Dummy channel as px4 needs at least 5

		_rc_pub.publish(rc);

	} else if (c->port == CRTP_PORT_MAVLINK) {
		_count_in++;
		/* Pipe to Mavlink bridge */
		_bridge->pipe_message(c);

	} else {
		handle_raw_other(sys);
	}

	// Block all non-requested messaged in bootloader mode
	if (_bootloader_mode) {
		return;
	}

	// Allow one raw message to be sent from the queue
	send_queued_raw_message();
}

void
Syslink::handle_bootloader(syslink_message_t *sys)
{
	// Minimal bootloader emulation for being detectable
	// To the bitcraze utilities, the STM32 will appear to have no flashable pages
	// Upon receiving a bootloader message, all outbound packets are blocked in 'bootloader mode' due to how fragile the aforementioned utilities are to extra packets

	crtp_message_t *c = (crtp_message_t *) &sys->length;

	uint8_t target = c->data[0];
	uint8_t cmd = c->data[1];

	if (target != 0xFF) { // CF2 STM32 target
		return;
	}

	_bootloader_mode = true;

	if (cmd == 0x10) { // GET_INFO

		c->size = 1 + 23;
		memset(&c->data[2], 0, 21);
		c->data[22] = 0x10; // Protocol version
		send_message(sys);
	}
}

void
Syslink::handle_raw_other(syslink_message_t *sys)
{
	// This function doesn't actually do anything
	// It is just here to return null responses to most standard messages

	crtp_message_t *c = (crtp_message_t *) &sys->length;

	if (c->port == CRTP_PORT_LOG) {

		PX4_INFO("Log: %d %d", c->channel, c->data[0]);

		if (c->channel == 0) { // Table of Contents Access

			uint8_t cmd = c->data[0];

			if (cmd == 0) { // GET_ITEM
				//int id = c->data[1];
				memset(&c->data[2], 0, 3);
				c->data[2] = 1; // type
				c->size = 1 + 5;
				send_message(sys);

			} else if (cmd == 1) { // GET_INFO
				memset(&c->data[1], 0, 7);
				c->size = 1 + 8;
				send_message(sys);
			}

		} else if (c->channel == 1) { // Log control

			uint8_t cmd = c->data[0];

			PX4_INFO("Responding to cmd: %d", cmd);
			c->data[2] = 0; // Success
			c->size = 3 + 1;

			// resend message
			send_message(sys);

		} else if (c->channel == 2) { // Log data

		}
	} else if (c->port == CRTP_PORT_MEM) {
		if (c->channel == 0) { // Info
			int cmd = c->data[0];

			if (cmd == 1) { // GET_NBR_OF_MEMS
				c->data[1] = 0;
				c->size = 2 + 1;

				// resend message
				send_message(sys);
			}
		}

	} else if (c->port == CRTP_PORT_PARAM) {
		if (c->channel == 0) { // TOC Access
			//	uint8_t msgId = c->data[0];

			c->data[1] = 0; // Last parameter (id = 0)
			memset(&c->data[2], 0, 10);
			c->size = 1 + 8;
			send_message(sys);
		}

		else if (c->channel == 1) { // Param read
			// 0 is ok
			c->data[1] = 0; // value
			c->size = 1 + 2;
			send_message(sys);
		}

	} else {
		PX4_INFO("Got raw: %d", c->port);
	}
}

int
Syslink::send_bytes(const void *data, size_t len)
{
	// TODO: This could be way more efficient
	//       Using interrupts/DMA/polling would be much better
	for (size_t i = 0; i < len; i++) {
		// Block until we can send a byte
		while (px4_arch_gpioread(GPIO_NRF_TXEN)) ;

		write(_fd, ((const char *)data) + i, 1);
	}

	return 0;
}

int
Syslink::send_message(syslink_message_t *msg)
{
	syslink_compute_cksum(msg);
	send_bytes(syslink_magic, 2);
	send_bytes(&msg->type, sizeof(msg->type));
	send_bytes(&msg->length, sizeof(msg->length));
	send_bytes(&msg->data, msg->length);
	send_bytes(&msg->cksum, sizeof(msg->cksum));
	return 0;
}


namespace syslink
{

void usage();
void start();
void status();
void test();
void attached(int pid);

void usage()
{
	warnx("missing command: try 'start', 'status', 'attached', 'test'");
}

void start()
{
	if (g_syslink != nullptr) {
		printf("Already started\n");
		exit(1);
	}

	g_syslink = new Syslink();
	g_syslink->start();

	// Wait for task and bridge to start
	usleep(5000);


	warnx("Started syslink on /dev/ttyS2\n");
	exit(0);

}

void status()
{
	if (g_syslink == nullptr) {
		printf("Please start syslink first\n");
		exit(1);
	}

	printf("Connection activity:\n");
	printf("- total rx: %d p/s\n", g_syslink->pktrate);
	printf("- radio rx: %d p/s (%d null)\n", g_syslink->rxrate, g_syslink->nullrate);
	printf("- radio tx: %d p/s\n\n", g_syslink->txrate);

	printf("Parameter Status:\n");
	const char *goodParam = "good";
	const char *badParam = "fail!";
	printf("- channel: %s\n", g_syslink->is_good(0) ? goodParam : badParam);
	printf("- data rate: %s\n", g_syslink->is_good(1) != 0 ? goodParam : badParam);
	printf("- address: %s\n\n", g_syslink->is_good(2) != 0 ? goodParam : badParam);


	int deckfd = open(DECK_DEVICE_PATH, O_RDONLY);
	int ndecks = 0; ioctl(deckfd, DECKIOGNUM, (unsigned long) &ndecks);
	printf("Deck scan: (found %d)\n", ndecks);

	for (int i = 0; i < ndecks; i++) {
		ioctl(deckfd, DECKIOSNUM, (unsigned long) &i);

		uint8_t *id;
		int idlen = ioctl(deckfd, DECKIOID, (unsigned long) &id);

		// TODO: Validate the ID
		printf("%i: ROM ID: ", i);

		for (int idi = 0; idi < idlen; idi++) {
			printf("%02X", id[idi]);
		}

		deck_descriptor_t desc;
		read(deckfd, &desc, sizeof(desc));

		printf(", VID: %02X , PID: %02X\n", desc.header, desc.vendorId, desc.productId);

		// Print pages of memory
		for (size_t di = 0; di < sizeof(desc); di++) {
			if (di % 16 == 0) {
				printf("\n");
			}

			printf("%02X ", ((uint8_t *)&desc)[di]);

		}

		printf("\n\n");
	}

	close(deckfd);
	exit(0);
}

void attached(int pid)
{
	bool found = false;

	int deckfd = open(DECK_DEVICE_PATH, O_RDONLY);
	int ndecks = 0; ioctl(deckfd, DECKIOGNUM, (unsigned long) &ndecks);

	for (int i = 0; i < ndecks; i++) {
		ioctl(deckfd, DECKIOSNUM, (unsigned long) &i);

		deck_descriptor_t desc;
		read(deckfd, &desc, sizeof(desc));

		if (desc.productId == pid) {
			found = true;
			break;
		}
	}

	close(deckfd);
	exit(found ? 1 : 0);
}

void test()
{
	// TODO: Ensure battery messages are recent
	// TODO: Read and write from memory to ensure it is working
}

} // namespace syslink

int syslink_main(int argc, char *argv[])
{
	if (argc < 2) {
		syslink::usage();
		exit(1);
	}

	const char *verb = argv[1];

	if (!strcmp(verb, "start")) {
		syslink::start();
	}

	if (!strcmp(verb, "status")) {
		syslink::status();
	}

	if (!strcmp(verb, "attached")) {
		if (argc != 3) {
			errx(1, "usage: syslink attached [deck_pid]");
		}

		int pid = atoi(argv[2]);
		syslink::attached(pid);
	}

	if (!strcmp(verb, "test")) {
		syslink::test();
	}

	syslink::usage();
	exit(1);

	return 0;
}
