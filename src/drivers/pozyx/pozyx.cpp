/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file pozyx.cpp
 * @author Andreas Antener <andreas@uaventure.com>
 *
 * Driver for connecting the Pozyx localization system (1) via an Arduino
 * over serial. The Arduino is expected to run the referenced sketch from APM (2)(3).
 *
 * 1) https://www.pozyx.io/
 * 2) http://ardupilot.org/copter/docs/common-pozyx.html
 * 3) https://github.com/ArduPilot/ardupilot/blob/master/Tools/Pozyx/IndoorLoiter/IndoorLoiter.ino
 */

#include <termios.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pozyx.h>
#include <drivers/device/device.h>

#include <uORB/topics/pozyx_report.h>

#define DEFAULT_PORT		"/dev/ttyS2"	// telem2 on Pixhawk

extern "C" __EXPORT int pozyx_main(int argc, char *argv[]);

class Pozyx : public device::CDev
{
public:
	Pozyx(const char *port = DEFAULT_PORT);
	virtual ~Pozyx();

	virtual int 			init();

	int				start();

private:
	bool				_task_should_exit;
	int 				_task_handle;
	char 				_port[20];
	int				_class_instance;
	int				_orb_class_instance;
	orb_advert_t			_pozyx_report_topic;

	unsigned 	_head;
	unsigned 	_tail;
	uint8_t 	_buf[POZYX_BUF_LEN];

	enum ParsingState {
		POZYX_HEADER = 0,
		POZYX_ID,
		POZYX_LEN,
		POZYX_MSG,
	} _state;

	uint8_t 	_msg_id;
	uint8_t 	_msg_len;
	uint8_t 	_msg_chksum;

	static void task_main_trampoline(int argc, char *argv[]);
	void task_main();

	uint8_t get_next_index(uint8_t current);
	uint8_t get_next_byte();
	bool read_and_parse(uint8_t *buf, int len);
	void send_pozyx_report(struct pozyx_position_s &pozyx_position);
};

namespace pozyx
{
Pozyx	*g_dev;
}

Pozyx::Pozyx(const char *port) :
	CDev("Pozyx", POZYX0_DEVICE_PATH),
	_task_should_exit(false),
	_task_handle(-1),
	_class_instance(-1),
	_orb_class_instance(-1),
	_pozyx_report_topic(nullptr),
	_head(0),
	_tail(0),
	_state(POZYX_HEADER),
	_msg_id(0),
	_msg_len(0),
	_msg_chksum(0)
{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	// disable debug() calls
	_debug_enabled = false;

	memset(&_buf[0], 0, sizeof(_buf));
}

Pozyx::~Pozyx()
{

	if (_class_instance != -1) {
		unregister_class_devname(POZYX_BASE_DEVICE_PATH, _class_instance);
	}

	if (_task_handle != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_task_handle);
				break;
			}
		} while (_task_handle != -1);
	}

	if (_class_instance != -1) {
		unregister_class_devname(POZYX_BASE_DEVICE_PATH, _class_instance);
	}
}

int
Pozyx::init()
{
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) {
			PX4_WARN("cdev init failed");
			break;
		}

		int fd = px4_open(POZYX0_DEVICE_PATH, 0);

		if (fd < 0) {
			PX4_WARN("failed to open range finder device");
			ret = 1;
			break;
		}

		px4_close(fd);

		/* open fd */
		fd = px4_open(_port, O_RDWR | O_NOCTTY);

		if (fd < 0) {
			PX4_WARN("failed to open serial device");
			ret = 1;
			break;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag	 &= ~(CSTOPB | PARENB);

		unsigned speed = B115200;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d ISPD", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d OSPD\n", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
			PX4_WARN("ERR baud %d ATTR", termios_state);
			ret = 1;
			break;
		}

		px4_close(fd);

		_class_instance = register_class_devname(POZYX_BASE_DEVICE_PATH);

		struct pozyx_report_s pozyx_report = {};
		pozyx_report.timestamp = hrt_absolute_time();

		_pozyx_report_topic = orb_advertise_multi(ORB_ID(pozyx_report), &pozyx_report,
				      &_orb_class_instance, ORB_PRIO_HIGH);

		if (_pozyx_report_topic == nullptr) {
			DEVICE_LOG("failed to create pozyx_report object. Did you start uOrb?");
			ret = 1;
			break;
		}

	} while (0);

	return ret;
}

void
Pozyx::task_main_trampoline(int argc, char *argv[])
{
	pozyx::g_dev->task_main();
}

int
Pozyx::start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("pozyx",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX - 30,
					  800,
					  (px4_main_t)&Pozyx::task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

uint8_t
Pozyx::get_next_index(uint8_t current)
{
	uint8_t ret = current + 1;

	if (ret == POZYX_BUF_LEN) {
		ret = 0;
	}

	return ret;
}

uint8_t
Pozyx::get_next_byte()
{
	_tail = get_next_index(_tail);
	return _buf[_tail];
}

bool
Pozyx::read_and_parse(uint8_t *buf, int len)
{
	printf("parsing");

	// write new data into a ring buffer
	for (int i = 0; i < len; i++) {
		_head++;

		if (_head >= POZYX_BUF_LEN) {
			_head = 0;
		}

		if (_tail == _head) {
			_tail = (_tail == POZYX_BUF_LEN - 1) ? 0 : _head + 1;
		}

		_buf[_head] = buf[i];
	}

	// check how many bytes are in the buffer
	int num_bytes = _head >= _tail ? (_head - _tail) : (_head + POZYX_BUF_LEN - _tail);
	//PX4_WARN("read len %d, h %d, t %d, n %d", len, _head, _tail, num_bytes);

	for (; num_bytes > 0; num_bytes--) {
		switch (_state) {
		case ParsingState::POZYX_HEADER: {
				uint8_t c = get_next_byte();

				//PX4_WARN("header %d: %d", _tail, c);
				if (c == POZYX_MSG_HEADER) {
					_msg_chksum = 0;
					_state = ParsingState::POZYX_ID;
				}

				break;
			}

		case ParsingState::POZYX_ID:
			_msg_id = get_next_byte();

			//PX4_WARN("id %d", _msg_id);

			if (_msg_id == POZYX_MSGID_POSITION ||
			    _msg_id == POZYX_MSGID_BEACON_CONFIG ||
			    _msg_id == POZYX_MSGID_BEACON_DIST) {
				_msg_chksum ^= _msg_id;
				_state = ParsingState::POZYX_LEN;

			} else {
				// not supported message
				_state = ParsingState::POZYX_HEADER;
			}

			break;

		case ParsingState::POZYX_LEN:
			_msg_len = get_next_byte();

			//PX4_WARN("len %d", _msg_len);

			if (_msg_len > 1) {
				// has data and checksum
				_msg_chksum ^= _msg_len;
				_state = ParsingState::POZYX_MSG;

			} else {
				// looks invalid
				_state = ParsingState::POZYX_HEADER;
			}

			break;

		case ParsingState::POZYX_MSG:

			//PX4_WARN("msg");
			// we want to be able to read the complete message once we know what we're looking for
			if (num_bytes < _msg_len) {
				return false;
			}

			// start from the beginning after this, no matter if we can parse or not
			_state = ParsingState::POZYX_HEADER;

			// verify checksum
			int index = _tail;

			for (; _msg_len > 1; _msg_len--) {
				index = get_next_index(index);
				_msg_chksum ^= _buf[index];
			}

			index = get_next_index(index);

			if (_msg_chksum != _buf[index]) {
				// failed
				//PX4_WARN("wrong checksum");
				_tail = index;
				return false;
			}

			// consume
			if (_msg_id == POZYX_MSGID_POSITION) {
				struct pozyx_position_s pozyx_position = {};
				pozyx_position.x = (uint32_t)get_next_byte() << 0 |
						   (uint32_t)get_next_byte() << 8 |
						   (uint32_t)get_next_byte() << 16 |
						   (uint32_t)get_next_byte() << 24;

				pozyx_position.y = (uint32_t)get_next_byte() << 0 |
						   (uint32_t)get_next_byte() << 8 |
						   (uint32_t)get_next_byte() << 16 |
						   (uint32_t)get_next_byte() << 24;

				pozyx_position.z = (uint32_t)get_next_byte() << 0 |
						   (uint32_t)get_next_byte() << 8 |
						   (uint32_t)get_next_byte() << 16 |
						   (uint32_t)get_next_byte() << 24;

				pozyx_position.position_error = (uint32_t)get_next_byte() << 0 |
								(uint32_t)get_next_byte() << 8;
				send_pozyx_report(pozyx_position);
			}

			// skip checksum
			_tail = get_next_index(_tail);

			return true;

		}
	}

	return false;
}

void
Pozyx::send_pozyx_report(struct pozyx_position_s &pozyx_position)
{
	struct pozyx_report_s pozyx_report = {};
	pozyx_report.timestamp = hrt_absolute_time();
	pozyx_report.pos_x = 0.001f * pozyx_position.x;
	pozyx_report.pos_y = 0.001f * pozyx_position.y;
	pozyx_report.pos_z = 0.001f * pozyx_position.z;
	pozyx_report.cov_xy = pozyx_position.position_error;
	//PX4_WARN("pos %d %d %d", pozyx_report.cov_xy, pozyx_position.x, pozyx_position.y);
	orb_publish(ORB_ID(pozyx_report), _pozyx_report_topic, &pozyx_report);
}

void
Pozyx::task_main()
{
	int fd = px4_open(_port, O_RDWR | O_NOCTTY);

	// we poll on data from the serial port
	px4_pollfd_struct_t fds[1];
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	uint8_t buf[POZYX_READ_LEN];

	while (!_task_should_exit) {
		// wait for up to 100ms for data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out
		if (pret == 0) {
			printf("Serial Timeout \n");
			continue;
		}

		if (pret < 0) {
			PX4_DEBUG("Pozyx serial port poll error");
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			memset(&buf[0], 0, sizeof(buf));
			int len = px4_read(fd, &buf[0], sizeof(buf));

			if (len <= 0) {
				PX4_DEBUG("error reading from Pozyx");
			}

			read_and_parse(&buf[0], len);
		}
	}

	px4_close(fd);
}

int pozyx_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (pozyx::g_dev != nullptr) {
			PX4_WARN("driver already started");
			return 0;
		}

		if (argc > 2) {
			pozyx::g_dev = new Pozyx(argv[2]);

		} else {
			pozyx::g_dev = new Pozyx(DEFAULT_PORT);
		}

		if (pozyx::g_dev == nullptr) {
			PX4_ERR("failed to create instance of Pozyx");
			return 1;

		} else {
			PX4_WARN("pozyx driver started on %s", (argc > 2) ? argv[2] : DEFAULT_PORT);
		}

		if (PX4_OK != pozyx::g_dev->init()) {
			delete pozyx::g_dev;
			pozyx::g_dev = nullptr;
			return 1;
		}

		if (OK != pozyx::g_dev->start()) {
			delete pozyx::g_dev;
			pozyx::g_dev = nullptr;
			return 1;
		}

		return 0;
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		if (pozyx::g_dev != nullptr) {
			delete pozyx::g_dev;
			pozyx::g_dev = nullptr;

		} else {
			PX4_WARN("driver not running");
		}

		return 0;
	}

	if (!strcmp(argv[1], "info")) {
		PX4_INFO("Pozyx");
		return 0;

	}

	PX4_WARN("unrecognized arguments, try: start [device_name], stop, info ");
	return 1;
}
