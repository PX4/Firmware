/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @author Stone White <stone@thone.io>
 *
 * Driver for the Upixels UPFLOW LC302 optical flow sensor
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <math.h>
#include <arch/board/board.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>

#include <lib/perf/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <board_config.h>


#define UPFLOW_MAX_DISTANCE 5.01f
#define UPFLOW_MIN_DISTANCE 0.0f

#define NAME "upflow"
#define DEVICE_PATH "/dev/" NAME

#define SENSOR_READING_FREQ 10.0f
#define READING_USEC_PERIOD (unsigned long)(1000000.0f / SENSOR_READING_FREQ)
#define OVERSAMPLE 6
#define WORK_USEC_INTERVAL READING_USEC_PERIOD / OVERSAMPLE
#define COLLECT_USEC_TIMEOUT READING_USEC_PERIOD / (OVERSAMPLE / 2)

/* 0.05sec */
#define PROBE_USEC_TIMEOUT 500000

#define LC302_HEAD_FIRST 0x0FE
#define LC302_HEAD_SECOND 0x0A
#define READING_START_ADDR 0x14
#define READING_LEN 0xA


struct __attribute__((__packed__)) reading_msg {
	uint8_t head_first;		// The beginning of the packet (fixed value 0xFE)
	uint8_t head_second;		// Number of packet bytes (fixed value 0x0A)
	int16_t flow_x_integral;	// Accumulated displacement during the accumulation of X pixels, (radians*10000) [The actual displacement(mm) is divided by 10000, and then multiplies the height(mm)]
	int16_t flow_y_integral; 	// Accumulated displacement during the accumulation of Y pixels, (radians*10000) [The actual displacement(mm) is divided by 10000, and then multiplies the height(mm)]
	uint16_t integration_timespan;  // The accumulative time (us) from the last transmission of optical data to the current transmission of optical data.
	uint16_t ground_distance;	// Reserved. Default value is 999(0x03E7)
	uint8_t valid;			// State value: 0 (0x00), optical-flow data is invalid; 245 (0xF5) is optical-flow data is valid.
	uint8_t version; 		// Version of the optical-flow module
	uint8_t crc_xor;		// Check value (Byte 3~12 Bytes XOR)
	uint8_t tail;			// Ending packet identifier (fixed value 0x55)

};

class upflow : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	upflow(const char *device_path, const char *serial_port, uint8_t rotation);
	virtual ~upflow();

	virtual int init() override;
	void start();
	void stop();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);

private:
	uint8_t _sonar_rotation;

	ringbuffer::RingBuffer *_reports = nullptr;

	int _fd = -1;
	const char *_serial_port;
	uint8_t _buffer[sizeof(struct reading_msg)];
	uint8_t _buffer_len = 0;

	orb_advert_t _upflow_topic = nullptr;
	orb_advert_t _distance_sensor_topic;

	enum {
		state_waiting_reading = 0,
		state_reading_requested
	} _state = state_waiting_reading;
	hrt_abstime _timeout_usec = 0;

	perf_counter_t _collect_timeout_perf;
	perf_counter_t _comm_error;
	perf_counter_t _sample_perf;

	optical_flow_s _report = {};
	//struct reading_msg _report_msg={};
	int _fd_open();
	bool _request();
	int _collect();
	void _publish(struct reading_msg *msg);
	int _cycle();

	void Run() override;
};

extern "C" __EXPORT int upflow_main(int argc, char *argv[]);

static void help()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the Upixels UPFLOW LC302.

### Examples

Attempt to start driver on a specified serial device.
$ upflow start -d /dev/ttyS2
Stop driver
$ upflow stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("upflow", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("optical_flow");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test","Test driver (basic functional tests)");

}

int upflow_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	const char *serial_port = "/dev/ttyS2";
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	static upflow *inst = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:r", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			serial_port = myoptarg;
			break;

		case 'r':
			rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting upflow sonar orientation to %d", (int)rotation);
			break;

		default:
			PX4_WARN("Unknown option!");
			help();
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		help();
		return PX4_ERROR;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		inst = new upflow(DEVICE_PATH, serial_port, rotation);

		if (!inst) {
			PX4_ERR("No memory to allocate " NAME);
			return PX4_ERROR;
		}else{
			PX4_INFO("new upflow success");
		}

		if (inst->init() != PX4_OK) {
			delete inst;
			return PX4_ERROR;
		}

		inst->start();

	} else if (!strcmp(verb, "stop")) {
		if(!inst){
			delete inst;
			inst = nullptr;
			PX4_INFO("upflow stop");
		}else{
			PX4_INFO("upflow not start");
		}


	} else if (!strcmp(verb, "test")) {
		int fd = open(DEVICE_PATH, O_RDONLY);
		ssize_t sz;
		struct reading_msg reportmsg;
		struct optical_flow_s report;
		if (fd < 0) {
			PX4_ERR("Unable to open %s", DEVICE_PATH);
			return PX4_ERROR;
		}

		sz = read(fd, &reportmsg, sizeof(reportmsg));
		close(fd);

		if (sz != sizeof(reportmsg)) {
			PX4_ERR("No sample available in %s", DEVICE_PATH);
			return PX4_ERROR;
		}
		report.timestamp = hrt_absolute_time();
		report.pixel_flow_x_integral = static_cast<float>(reportmsg.flow_x_integral) / 10000.0f;//convert to radians
		report.pixel_flow_y_integral = static_cast<float>(reportmsg.flow_y_integral) / 10000.0f;//convert to radians
		report.frame_count_since_last_readout=(uint16_t)NAN;
		report.ground_distance_m= static_cast<float>NAN;
		report.quality = reportmsg.valid; //0:bad ; 255 max quality
		report.gyro_x_rate_integral= static_cast<float>NAN;
		report.gyro_y_rate_integral= static_cast<float>NAN;
		report.gyro_z_rate_integral= static_cast<float>NAN;
		report.integration_timespan=reportmsg.integration_timespan;
		report.time_since_last_sonar_update=(uint32_t)NAN;
		report.gyro_temperature=(int16_t)NAN;
		report.sensor_id=0;
		report.max_flow_rate=(float)NAN;
		report.min_ground_distance=(float)NAN;
		report.max_ground_distance=(float)NAN;
		print_message(report);

	} else {
		help();
		PX4_ERR("unrecognized command");
		return PX4_ERROR;
	}

	return PX4_OK;
}

upflow::upflow(const char *device_path, const char *serial_port, uint8_t rotation):
	CDev(device_path),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
	_sonar_rotation(rotation),
	_collect_timeout_perf(perf_alloc(PC_COUNT, "upflow_collect_timeout")),
	_comm_error(perf_alloc(PC_COUNT, "upflow_comm_errors")),
	_sample_perf(perf_alloc(PC_ELAPSED, "upflow_sample"))
{
	_serial_port = strdup(serial_port);
}

upflow::~upflow()
{
	stop();

	free((char *)_serial_port);

	if (_fd > -1) {
		::close(_fd);
	}

	if (_reports) {
		delete _reports;
	}

	if (_upflow_topic) {
		orb_unadvertise(_upflow_topic);
	}

	perf_free(_sample_perf);
	perf_free(_collect_timeout_perf);
	perf_free(_comm_error);
}

int upflow::_fd_open()
{
	if (!_serial_port) {
		return -1;
	}

	_fd = ::open(_serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("open upflow_serial_port:%s faild",_serial_port);
		return -1;
	}

	struct termios config;

	int r = tcgetattr(_fd, &config);

	if (r) {
		PX4_ERR("Unable to get termios from %s.", _serial_port);
		goto error;
	}

	/* clear: data bit size, two stop bits, parity, hardware flow control */
	config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
	/* set: 8 data bits, enable receiver, ignore modem status lines */
	config.c_cflag |= (CS8 | CREAD | CLOCAL);
	/* turn off output processing */
	config.c_oflag = 0;
	/* clear: echo, echo new line, canonical input and extended input */
	config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	r = cfsetispeed(&config, B19200);
	r |= cfsetospeed(&config, B19200);

	if (r) {
		PX4_ERR("Unable to set baudrate");
		goto error;
	}

	r = tcsetattr(_fd, TCSANOW, &config);

	if (r) {
		PX4_ERR("Unable to set termios to %s", _serial_port);
		goto error;
	}

	return PX4_OK;

error:
	::close(_fd);
	_fd = -1;
	return PX4_ERROR;
}

int upflow::init()
{
	if (CDev::init()) {
		PX4_ERR("Unable to initialize device\n");
		return -1;
	}else{
		PX4_INFO("able to initialize device");
	}

	_reports = new ringbuffer::RingBuffer(2, sizeof(optical_flow_s));

	if (!_reports) {
		PX4_ERR("No memory to allocate RingBuffer");
		return -1;
	}else{
		PX4_INFO("new ringbuffer success");
	}

	if (_fd_open()) {
		PX4_ERR("_fd_open faild");
		return PX4_ERROR;
	}else{
		PX4_INFO("_fd_open success ");
	}

	hrt_abstime timeout_usec, now;

	for (now = hrt_absolute_time(), timeout_usec = now + PROBE_USEC_TIMEOUT;
	     now < timeout_usec;
	     now = hrt_absolute_time()) {

		//PX4_INFO("now:%d",(int)now);
		if (_cycle() > 0) {
			printf(NAME " found\n");
			return PX4_OK;
		}

		px4_usleep(1000);
	}

	PX4_ERR("No readings from " NAME);

	return PX4_ERROR;
}

void
upflow::stop()
{
	ScheduleClear();
}

void upflow::start()
{
	/*
	 * file descriptor can only be accessed by the process that opened it
	 * so closing here and it will be opened from the High priority kernel
	 * process
	 */
	::close(_fd);
	_fd = -1;

	ScheduleDelayed(WORK_USEC_INTERVAL);
}

void
upflow::Run()
{
	if (_fd != -1) {
		_cycle();

	} else {
		_fd_open();
	}

	ScheduleDelayed(WORK_USEC_INTERVAL);
}

void upflow::_publish(struct reading_msg* msg)
{
	_report.timestamp = hrt_absolute_time();
	_report.pixel_flow_x_integral = static_cast<float>(msg->flow_x_integral) / 10000.0f; // convert to radians
	_report.pixel_flow_y_integral = static_cast<float>(msg->flow_y_integral) / 10000.0f; // convert to radians
	_report.frame_count_since_last_readout = (uint16_t)NAN;				     // microseconds(uint16_t)NAN;
	_report.ground_distance_m =  static_cast<float>NAN; // convert to meters
	_report.quality = msg->valid;                       // 0:bad ; 255 max quality
	_report.gyro_x_rate_integral=static_cast<float>NAN;
	_report.gyro_y_rate_integral=static_cast<float>NAN;
	_report.gyro_z_rate_integral=static_cast<float>NAN;
	_report.integration_timespan=msg->integration_timespan;
	_report.time_since_last_sonar_update=(uint32_t)NAN;
	_report.gyro_temperature=(int16_t)NAN;
	_report.sensor_id=0;
	_report.max_flow_rate = 5.0f;        // Datasheet: 7.4 rad/s;
	_report.min_ground_distance =  0.1f; // Datasheet: 80mm(float)NAN;
	_report.max_ground_distance = 30.0f; // Datasheet: infinity;


	if (_upflow_topic == nullptr) {
		_upflow_topic = orb_advertise(ORB_ID(optical_flow), &_report);

	} else {
		orb_publish(ORB_ID(optical_flow), _upflow_topic, &_report);
	}
}

bool upflow::_request()
{
	/* flush anything in RX buffer */
	tcflush(_fd, TCIFLUSH);

	//int r = ::write(_fd, request_reading_msg, sizeof(request_reading_msg));
	return true;
}

static uint8_t _calc_crc8(struct reading_msg *msg)
{

	uint8_t crc_xor=(((msg->flow_x_integral)>>8)&0xff)^
			((msg->flow_x_integral)&0xff)^
			(((msg->flow_y_integral)>>8)&0xff)^
			((msg->flow_y_integral)&0xff)^
			(((msg->integration_timespan)>>8)&0xff)^
			((msg->integration_timespan)&0xff)^
			(((msg->ground_distance)>>8)&0xff)^
			((msg->ground_distance)&0xff)^
			msg->valid^
			msg->version;

	return crc_xor;
}

/*
 * returns 0 when still waiting for reading_msg, 1 when frame was read or
 * -1 in case of error
 */
int upflow::_collect()
{
	struct reading_msg *msg;

	int r = ::read(_fd, _buffer + _buffer_len, sizeof(_buffer) - _buffer_len);
	//PX4_INFO("r=%d",r);
	if (r < 1) {
		return 0;
	}

	_buffer_len += r;

	if (_buffer_len < sizeof(struct reading_msg)) {
		return 0;
	}

	msg = (struct reading_msg *)_buffer;
	//PX4_INFO("msg->head_first:%d",(int)msg->head_first);
	//PX4_INFO("msg->head_second:%d",(int)msg->head_second);
	//PX4_INFO("msg->flow_x_integral:%d",(int)msg->flow_x_integral);
	//PX4_INFO("msg->flow_y_integral:%d",(int)msg->flow_y_integral);
	//PX4_INFO("msg->valid:%d",(int)msg->valid);
	if (msg->head_first != LC302_HEAD_FIRST || msg->head_second != LC302_HEAD_SECOND) {
		return -1;
	}
#if 0
	const uint16_t crc16_calc = _calc_crc16(_buffer, _buffer_len - 2);

	if (crc16_calc != msg->crc) {
		return -1;
	}

	/* NOTE: little-endian support only */
	_publish(msg->first_dist_high_byte << 8 | msg->first_dist_low_byte);
	return 1;

	////////////////////////////////

	//PX4_INFO("sz=%d,sizeof(_report_msg)=%d",sz,sizeof(_report_msg));
	if (r != sizeof(_report_msg)) {
		//PX4_ERR("No sample available in %s", DEVICE_PATH);
		return 0;
	}
	//PX4_INFO("msg.head_first:%d",(int)_report_msg.head_first);
	//PX4_INFO("msg.head_second:%d",(int)_report_msg.head_second);
	//PX4_INFO("msg.flow_x_integral:%d",(int)_report_msg.flow_x_integral);
	//PX4_INFO("msg.flow_y_integral:%d",(int)_report_msg.flow_y_integral);
	//PX4_INFO("msg.valid:%d",(int)_report_msg.valid);
#endif
	if (msg->head_first != LC302_HEAD_FIRST || msg->head_second != LC302_HEAD_SECOND) {
		return -1;
	}

	const uint8_t crc8_calc = _calc_crc8(msg);

	if (crc8_calc != msg->crc_xor) {
		return -1;
	}

	/* NOTE: little-endian support only */
	_publish(msg);
	return 1;
}

int upflow::_cycle()
{
	int ret = 0;
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case state_waiting_reading:
		if (now > _timeout_usec) {
			if (_request()) {
				perf_begin(_sample_perf);
				_buffer_len = 0;
				_state = state_reading_requested;
				_timeout_usec = now + COLLECT_USEC_TIMEOUT;
			}
		}

		break;

	case state_reading_requested:
		ret = _collect();

		if (ret == 1) {
			perf_end(_sample_perf);
			_state = state_waiting_reading;
			_timeout_usec = now + READING_USEC_PERIOD;

		} else {
			if (ret == 0 && now < _timeout_usec) {
				/* still waiting for reading */
				break;
			}

			if (ret == -1) {
				perf_count(_comm_error);

			} else {
				perf_count(_collect_timeout_perf);
			}

			perf_cancel(_sample_perf);
			_state = state_waiting_reading;
			_timeout_usec = 0;
			return _cycle();
		}
	}

	return ret;
}

ssize_t upflow::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct reading_msg);
	struct reading_msg *rbuf = reinterpret_cast<struct reading_msg *>(buffer);
	int ret = 0;

	if (count < 1) {
		return -ENOSPC;
	}

	while (count--) {
		if (_reports->get(rbuf)) {
			ret += sizeof(*rbuf);
			rbuf++;
		}
	}

	return ret ? ret : -EAGAIN;
}
