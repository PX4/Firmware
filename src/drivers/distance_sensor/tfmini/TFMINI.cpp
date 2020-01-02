/****************************************************************************
 *
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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

#include "TFMINI.hpp"

#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>

TFMINI::TFMINI(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_TFMINI;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
}

TFMINI::~TFMINI()
{
	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
TFMINI::init()
{
	int hw_model = 0;
	param_get(param_find("SENS_EN_TFMINI"), &hw_model);

	_hw_model = static_cast<TFMINI_MODEL>(hw_model);

	switch (_hw_model) {
	case TFMINI_MODEL::MODEL_UNKNOWN: // Other TFMINI models (12m, 100 Hz, FOV 2.3)
		// Note: Sensor specification shows 0.3m as minimum, but in practice
		// 0.3m is too close to minimum so chattering of invalid sensor decision
		// is happening sometimes. this cause EKF to believe inconsistent range readings.
		// So we set 0.4m as valid minimum.
		_px4_rangefinder.set_min_distance(0.4f);
		_px4_rangefinder.set_min_distance(0.4f);
		_px4_rangefinder.set_max_distance(12.0f);
		_px4_rangefinder.set_fov(math::radians(2.3f));
		break;

	case TFMINI_MODEL::MODEL_TFMINI: // TFMINI (12m, 100 Hz, FOV 2.3)
		_px4_rangefinder.set_min_distance(0.4f);
		_px4_rangefinder.set_max_distance(12.0f);
		_px4_rangefinder.set_fov(math::radians(2.3f));
		break;

	case TFMINI_MODEL::MODEL_TFMINIPLUS: // TFMINI-PLUS (12m, 100 Hz, FOV 3.6)
		_px4_rangefinder.set_min_distance(0.1f);
		_px4_rangefinder.set_max_distance(12.0f);
		_px4_rangefinder.set_fov(math::radians(3.6f));
		break;

	default:
		PX4_ERR("invalid HW model %u.", (uint8_t)_hw_model);
		return -1;
	}

	// status
	int ret = 0;

	do { // create a scope to handle exit conditions using break

		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return -1;
		}

		// baudrate 115200, 8 bits, no parity, 1 stop bit
		unsigned speed = B115200;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;			// 8-bit characters
		uart_config.c_cflag &= ~PARENB;			// no parity bit
		uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}
	} while (0);

	// close the fd
	::close(_fd);
	_fd = -1;

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

void
TFMINI::autosetup_tfmini_process()
{
	if (_hw_model != TFMINI_MODEL::MODEL_TFMINI) {
		return;
	}

	if (_tfmini_setup._setup_step == TFMINI_SETUP_STEP::STEP4_SAVE_CONFIRMED) {
		return;
	}

	_tfmini_setup._counter = 0;

	// informations about commands and responses to/from TFMini has been found in SJ-PM-TFmini-T-01_A05 Product Mannual_EN.pdf
	switch (_tfmini_setup._setup_step) {
	case TFMINI_SETUP_STEP::STEP0_UNCONFIGURED:

		// we have written _tfmini_setup._com_enter and we expect reading 42 57 02 01 00 00 01 02
		if (_command_response[3] == 0x01 && _command_response[6] == 0x01 && _command_response[7] == 0x02) {
			_tfmini_setup._setup_step = TFMINI_SETUP_STEP::STEP1_ENTER_CONFIRMED;
		}

		break;

	case TFMINI_SETUP_STEP::STEP1_ENTER_CONFIRMED:

		// we have written _tfmini_setup._com_mode and we expect reading 42 57 02 01 00 00 01 06
		if (_command_response[3] == 0x01 && _command_response[6] == 0x01 && _command_response[7] == 0x06) {
			_tfmini_setup._setup_step = TFMINI_SETUP_STEP::STEP2_MODE_CONFIRMED;
		}

		break;

	case TFMINI_SETUP_STEP::STEP2_MODE_CONFIRMED:

		// we have written _tfmini_setup._com_unit and we expect reading 42 57 02 01 00 00 01 1A
		if (_command_response[3] == 0x01 && _command_response[6] == 0x01 && _command_response[7] == 0x1A) {
			_tfmini_setup._setup_step = TFMINI_SETUP_STEP::STEP3_UNIT_CONFIRMED;
		}

		break;

	case TFMINI_SETUP_STEP::STEP3_UNIT_CONFIRMED:

		// we have written _tfmini_setup._com_save and we expect reading 42 57 02 01 00 00 00 02
		if (_command_response[3] == 0x01 && _command_response[6] == 0x00 && _command_response[7] == 0x02) {
			_tfmini_setup._setup_step = TFMINI_SETUP_STEP::STEP4_SAVE_CONFIRMED;
		}

		break;

	default:
		break;
	}
}

void
TFMINI::autosetup_tfmini()
{
	if (_hw_model != TFMINI_MODEL::MODEL_TFMINI) {
		return;
	}

	if (_tfmini_setup._setup_step == TFMINI_SETUP_STEP::STEP4_SAVE_CONFIRMED) {
		return;
	}

	if (_fd < 0) {
		return;
	}

	_tfmini_setup._counter++;

	if (_tfmini_setup._counter != 1) {
		return;
	}

	int ret = 0;

	// informations about commands and responses to/from TFMini- has been found in SJ-PM-TFmini-T-01_A05 Product Mannual_EN.pdf
	switch (_tfmini_setup._setup_step) {
	case TFMINI_SETUP_STEP::STEP0_UNCONFIGURED:
		ret = ::write(_fd, _tfmini_setup._com_enter, sizeof(_tfmini_setup._com_enter));
		break;

	case TFMINI_SETUP_STEP::STEP1_ENTER_CONFIRMED:
		ret = ::write(_fd, _tfmini_setup._com_mode, sizeof(_tfmini_setup._com_mode));
		break;

	case TFMINI_SETUP_STEP::STEP2_MODE_CONFIRMED:
		ret = ::write(_fd, _tfmini_setup._com_unit, sizeof(_tfmini_setup._com_unit));
		break;

	case TFMINI_SETUP_STEP::STEP3_UNIT_CONFIRMED:
		ret = ::write(_fd, _tfmini_setup._com_save, sizeof(_tfmini_setup._com_save));
		break;

	default:
		break;
	}

	if (ret < 0) {
		perf_count(_comms_errors);
	}
}

void
TFMINI::autosetup_tfminiplus_process()
{
	if (_hw_model != TFMINI_MODEL::MODEL_TFMINIPLUS) {
		return;
	}

	if (_tfminiplus_setup._setup_step == TFMINIPLUS_SETUP_STEP::STEP4_SAVE_CONFIRMED) {
		return;
	}

	_tfminiplus_setup._counter = 0;

	// informations about commands and responses to/from TFMini-plus has been found in TFmini-Plus A04-Product Mannual_EN.pdf
	switch (_tfminiplus_setup._setup_step) {
	case TFMINIPLUS_SETUP_STEP::STEP0_UNCONFIGURED:

		// we have written _tfminiplus_setup._com_version and we expect reading 5A 07 01 V1 V2 V3 SU
		if (_command_response[1] == 0x07 && _command_response[2] == 0x01) {
			_tfminiplus_setup._version[0] = _command_response[3];
			_tfminiplus_setup._version[1] = _command_response[4];
			_tfminiplus_setup._version[2] = _command_response[5];
			_tfminiplus_setup._setup_step = TFMINIPLUS_SETUP_STEP::STEP1_VERSION_CONFIRMED;
		}

		break;

	case TFMINIPLUS_SETUP_STEP::STEP1_VERSION_CONFIRMED:

		// we have written _tfminiplus_setup._com_mode and we expect reading 5A 05 05 01 65
		if (_command_response[1] == 0x05 && _command_response[2] == 0x05 && _command_response[3] == 0x01) {
			_tfminiplus_setup._setup_step = TFMINIPLUS_SETUP_STEP::STEP2_MODE_CONFIRMED;
		}

		break;

	case TFMINIPLUS_SETUP_STEP::STEP2_MODE_CONFIRMED:

		// we have written _tfminiplus_setup._com_enable and we expect reading 5A 05 07 01 67
		if (_command_response[1] == 0x05 && _command_response[2] == 0x07 && _command_response[3] == 0x01) {
			_tfminiplus_setup._setup_step = TFMINIPLUS_SETUP_STEP::STEP3_ENABLE_CONFIRMED;
		}

		break;

	case TFMINIPLUS_SETUP_STEP::STEP3_ENABLE_CONFIRMED:

		// we have written _tfminiplus_setup._com_save and we expect reading 5A 05 11 00 6F
		if (_command_response[1] == 0x05 && _command_response[2] == 0x11 && _command_response[3] == 0x00) {
			_tfminiplus_setup._setup_step = TFMINIPLUS_SETUP_STEP::STEP4_SAVE_CONFIRMED;
		}

		break;

	default:
		break;
	}
}

void
TFMINI::autosetup_tfminiplus()
{
	if (_hw_model != TFMINI_MODEL::MODEL_TFMINIPLUS) {
		return;
	}

	if (_tfminiplus_setup._setup_step == TFMINIPLUS_SETUP_STEP::STEP4_SAVE_CONFIRMED) {
		return;
	}

	if (_fd < 0) {
		return;
	}

	_tfminiplus_setup._counter++;

	if (_tfminiplus_setup._counter != 1) {
		return;
	}

	int ret = 0;

	// informations about commands and responses to/from TFMini-plus has been found in TFmini-Plus A04-Product Mannual_EN.pdf
	switch (_tfminiplus_setup._setup_step) {
	case TFMINIPLUS_SETUP_STEP::STEP0_UNCONFIGURED:
		ret = ::write(_fd, _tfminiplus_setup._com_version, sizeof(_tfminiplus_setup._com_version));
		break;

	case TFMINIPLUS_SETUP_STEP::STEP1_VERSION_CONFIRMED:
		ret = ::write(_fd, _tfminiplus_setup._com_mode, sizeof(_tfminiplus_setup._com_mode));
		break;

	case TFMINIPLUS_SETUP_STEP::STEP2_MODE_CONFIRMED:
		ret = ::write(_fd, _tfminiplus_setup._com_enable, sizeof(_tfminiplus_setup._com_enable));
		break;

	case TFMINIPLUS_SETUP_STEP::STEP3_ENABLE_CONFIRMED:
		ret = ::write(_fd, _tfminiplus_setup._com_save, sizeof(_tfminiplus_setup._com_save));
		break;

	default:
		break;
	}

	if (ret < 0) {
		perf_count(_comms_errors);
	}
}

int
TFMINI::write_command(uint8_t *command, uint8_t framelen)
{
	_command_result = false;

	if (_fd < 0) {
		PX4_ERR("tfmini fd is closed");
		return PX4_ERROR;
	}

	if (_hw_model != TFMINI_MODEL::MODEL_TFMINI && _hw_model != TFMINI_MODEL::MODEL_TFMINIPLUS) {
		PX4_ERR("tfmini model not defined");
		return PX4_ERROR;
	}

	if (framelen > 8) {
		PX4_ERR("invalid command - too long");
		return PX4_ERROR;
	}

	if (_hw_model == TFMINI_MODEL::MODEL_TFMINI && (command[0] != TFMINI_CMD_HEADER1 || command[1] != TFMINI_CMD_HEADER2
			|| framelen != TFMINI_CMD_SIZE)) {
		PX4_ERR("invalid command - not for tfmini model");
		return PX4_ERROR;
	}

	if (_hw_model == TFMINI_MODEL::MODEL_TFMINIPLUS && (command[0] != TFMINIPLUS_CMD_HEADER1 || command[1] != framelen)) {
		PX4_ERR("invalid command - not for tfmini-plus model");
		return PX4_ERROR;
	}

	memcpy(_command_buf, command, framelen);
	_command_size = framelen;
	_command_retry = 3;

	return PX4_OK;
}

bool
TFMINI::get_command_result()
{
	return _command_result;
}

uint8_t *
TFMINI::get_command_response(uint8_t *response_size)
{
	*response_size = _command_response_size;
	return _command_response;
}

int
TFMINI::collect()
{
	perf_begin(_sample_perf);

	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);
	int ret = 0;

	autosetup_tfmini();
	autosetup_tfminiplus();

	if (_command_result == false && _command_retry > 0) {
		_command_retry--;
		ret = ::write(_fd, _command_buf, _command_size);

		if (ret < 0) {
			PX4_ERR("write err: %d", ret);
			perf_count(_comms_errors);
		}
	}

	// the buffer for read chars is buflen minus null termination
	char readbuf[sizeof(_linebuf)] {};
	unsigned readlen = sizeof(readbuf) - 1;

	float distance_m = -1.0f;
	uint16_t signal_value = 0;
	float temperature = -273.0f;

	// Check the number of bytes available in the buffer
	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		perf_end(_sample_perf);
		return 0;
	}

	// parse entire buffer
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	do {
		// read from the sensor (uart buffer)
		ret = ::read(_fd, &readbuf[0], readlen);

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			// only throw an error if we time out
			if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}
		}

		_last_read = hrt_absolute_time();

		// parse buffer
		for (int i = 0; i < ret; i++) {
			tfmini_parse(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, _hw_model, &distance_m, &signal_value, &temperature,
				     _command_response, &_command_response_size);

			if (_parse_state == TFMINI_PARSE_STATE::STATE8_GOT_RESPONSE_CHECKSUM) {
				_command_result = true;
				_command_retry = 0;
				autosetup_tfmini_process();
				autosetup_tfminiplus_process();
			}

			if (_parse_state == TFMINI_PARSE_STATE::STATE0_UNSYNC) {
				perf_count(_comms_errors);
			}
		}

		// bytes left to parse
		bytes_available -= ret;

	} while (bytes_available > 0);

	// no valid measurement after parsing buffer
	if (distance_m < 0.0f) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	int8_t signal_quality = -1;

	switch (_hw_model) {
	case TFMINI_MODEL::MODEL_TFMINI: // TFMINI (0 to 3000)
		// to be implemented
		break;

	case TFMINI_MODEL::MODEL_TFMINIPLUS: // TFMINI-PLUS (0 to 0xFFFF)
		if (signal_value < 100 || signal_value == TFMINIPLUS_INVALID_MEASURE) {
			signal_quality = 0;
			break;
		}

		//according to the spec, signal is within [0-0xFFFF]
		//according to my test, signal is within [0-20000]
		signal_value = math::min(signal_value, static_cast<uint16_t>(20000));
		signal_quality = static_cast<int8_t>(static_cast<uint32_t>(signal_value) / 200);  //in percent, ie: x*100/20000 or x/200
		break;

	case TFMINI_MODEL::MODEL_UNKNOWN: // Other TFMINI models (unknown values)
	default:
		break;
	}

	// publish most recent valid measurement from buffer
	_px4_rangefinder.update(timestamp_sample, distance_m, signal_quality);

	perf_end(_sample_perf);

	return PX4_OK;
}

void
TFMINI::start()
{
	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(7_ms);
}

void
TFMINI::stop()
{
	ScheduleClear();
}

void
TFMINI::Run()
{
	// fds initialized?
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}

	// perform collection
	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(7_ms, 87 * 9);
		return;
	}
}

void
TFMINI::print_info()
{
	printf("Using port '%s'\n", _port);

	if (_hw_model == TFMINI_MODEL::MODEL_TFMINIPLUS
	    && _tfminiplus_setup._setup_step == TFMINIPLUS_SETUP_STEP::STEP4_SAVE_CONFIRMED) {
		PX4_INFO("TFMINI-Plus is configured - version = %u.%u.%u\n", _tfminiplus_setup._version[0],
			 _tfminiplus_setup._version[1],
			 _tfminiplus_setup._version[2]);

	} else if (_hw_model == TFMINI_MODEL::MODEL_TFMINIPLUS) {
		PX4_INFO("TFMINI-Plus is being configured(state = %u c = %u) - version = %u.%u.%u\n",
			 (uint8_t)_tfminiplus_setup._setup_step,
			 _tfminiplus_setup._counter, _tfminiplus_setup._version[0], _tfminiplus_setup._version[1],
			 _tfminiplus_setup._version[2]);

	} else if (_hw_model == TFMINI_MODEL::MODEL_TFMINI
		   && _tfmini_setup._setup_step == TFMINI_SETUP_STEP::STEP4_SAVE_CONFIRMED) {
		PX4_INFO("TFMINI is configured - version = unknown\n");

	} else if (_hw_model == TFMINI_MODEL::MODEL_TFMINI) {
		PX4_INFO("TFMINI is being configured(state = %u c = %u) - version = unknown\n", (uint8_t)_tfmini_setup._setup_step,
			 _tfmini_setup._counter);
	}

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
