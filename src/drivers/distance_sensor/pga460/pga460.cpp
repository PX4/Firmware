/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file pga460.cpp
 * @author Jacob Dahl <jacob.dahl@tealdrones.com>
 *
 * Driver for the TI PGA460 Ultrasonic Signal Processor and Transducer Driver
 */

#include "pga460.h"


PGA460::PGA460(const char *port)
{
	// Store port name.
	strncpy(_port, port, sizeof(_port) - 1);
	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';

	_device_id.devid_s.devtype = DRV_DIST_DEVTYPE_PGA460;
	_device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		_device_id.devid_s.bus = bus_num;
	}
}

PGA460::~PGA460()
{
	orb_unadvertise(_distance_sensor_topic);
}

uint8_t PGA460::calc_checksum(uint8_t *data, const uint8_t size)
{
	uint8_t checksum_input[size];

	for (size_t i = 0; i < size; i++) {
		checksum_input[i] = *data;
		data++;
	}

	uint16_t carry = 0;

	for (uint8_t j = 0; j < size; j++) {
		if ((checksum_input[j] + carry) < carry) {
			carry = carry + checksum_input[j] + 1;

		} else {
			carry = carry + checksum_input[j];
		}

		if (carry > 0xFF) {
			carry = carry - 255;
		}
	}

	carry = (~carry & 0x00FF);
	return carry;
}

int PGA460::close_serial()
{
	int ret = ::close(_fd);

	if (ret != 0) {
		PX4_WARN("Could not close serial port");
	}

	return ret;
}

int PGA460::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command.");
}

PGA460 *PGA460::instantiate(int argc, char *argv[])
{
	PGA460 *pga460 = new PGA460(PGA460_DEFAULT_PORT);
	return pga460;
}

int PGA460::initialize_device_settings()
{
	if (initialize_thresholds() != PX4_OK) {
		PX4_WARN("Thresholds not initialized");
		return PX4_ERROR;
	}

	px4_usleep(10000);

	// Read to see if eeprom saved data matches desired data, otherwise overwrite eeprom.
	if (read_eeprom() != PX4_OK) {
		write_eeprom();
		// Allow sufficient time for the device to complete writing to registers.
		px4_usleep(10000);
	}

	// Verify the device is alive.
	if (read_register(0x00) != USER_DATA1) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int PGA460::initialize_thresholds()
{
	const uint8_t array_size = 35;
	uint8_t settings_buf[array_size] = {SYNCBYTE, BC_THRBW,
					    P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5,
					    P1_THR_6, P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10,
					    P1_THR_11, P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
					    P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5,
					    P2_THR_6, P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10,
					    P2_THR_11, P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15, 0xFF
					   };

	uint8_t checksum = calc_checksum(&settings_buf[1], sizeof(settings_buf) - 2);
	settings_buf[array_size - 1] = checksum;

	int ret = ::write(_fd, &settings_buf[0], sizeof(settings_buf));

	if (!ret) {
		return PX4_ERROR;
	}

	// Must wait >50us per datasheet.
	px4_usleep(100);

	if (read_threshold_registers() == PX4_OK) {
		return PX4_OK;

	} else {
		print_device_status();
		return PX4_ERROR;
	}
}

uint32_t PGA460::collect_results()
{

	uint8_t buf_rx[6] = {};
	int bytes_available = sizeof(buf_rx);
	int total_bytes = 0;

	do {
		int ret = ::read(_fd, buf_rx + total_bytes, sizeof(buf_rx));

		total_bytes += ret;

		if (ret < 0) {
			tcflush(_fd, TCIFLUSH);
			PX4_ERR("read err3: %d", ret);
			return ret;
		}

		bytes_available -= ret;

		px4_usleep(1000);

	} while (bytes_available > 0);


	uint16_t time_of_flight = (buf_rx[1] << 8) + buf_rx[2];
	uint8_t Width = buf_rx[3];
	uint8_t Amplitude = buf_rx[4];

	float object_distance = calculate_object_distance(time_of_flight);

	uORB_publish_results(object_distance);

	// B1,2: time_of_flight  B3: Width  B4: Amplitude
	uint32_t results = (time_of_flight << 16) | (Width << 8) | (Amplitude << 0);

	return results;
}

float PGA460::calculate_object_distance(uint16_t time_of_flight)
{
	float temperature = get_temperature();

	// Default temperature if no temperature measurement can be obtained.
	if (temperature > MAX_DETECTABLE_TEMPERATURE ||
	    temperature < MIN_DETECTABLE_TEMPERATURE) {
		temperature = 20.0f;
	}

	// Formula for the speed of sound over temperature: v = 331m/s + 0.6m/s/C * T
	float speed_of_sound = 331.0f + 0.6f * temperature;
	float millseconds_to_meters = 0.000001f;

	// Calculate the object distance in meters.
	float object_distance = (float)time_of_flight * millseconds_to_meters * (speed_of_sound / 2.0f);

	return object_distance;
}

int PGA460::flash_eeprom()
{
	// Send same unlock code with prog bit set to 1.
	uint8_t eeprom_write_buf[5] = {SYNCBYTE, SRW, EE_CNTRL_ADDR, EE_UNLOCK_ST2, 0xFF};
	uint8_t checksum = calc_checksum(&eeprom_write_buf[1], sizeof(eeprom_write_buf) - 2);
	eeprom_write_buf[4] = checksum;
	int ret = ::write(_fd, &eeprom_write_buf[0], sizeof(eeprom_write_buf));

	if (!ret) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

float PGA460::get_temperature()
{
	uint8_t buf_tx[4] = {SYNCBYTE, TNLM, 0x00, 0xFF};
	uint8_t checksum = calc_checksum(&buf_tx[0], 3);
	buf_tx[3] = checksum;

	int ret = ::write(_fd, &buf_tx[0], sizeof(buf_tx));

	if (!ret) {
		return PX4_ERROR;
	}

	// The pga460 requires a 2ms delay per the datasheet.
	px4_usleep(5000);

	buf_tx[1] = TNLR;
	ret = ::write(_fd, &buf_tx[0], sizeof(buf_tx) - 2);

	if (!ret) {
		return PX4_ERROR;
	}

	px4_usleep(10000);

	uint8_t buf_rx[4] = {};
	int bytes_available = sizeof(buf_rx);
	int total_bytes = 0;

	do {
		ret = ::read(_fd, buf_rx + total_bytes, sizeof(buf_rx));

		total_bytes += ret;

		if (ret < 0) {
			tcflush(_fd, TCIFLUSH);
			PX4_ERR("read err1: %d", ret);
			return ret;
		}

		bytes_available -= ret;

		px4_usleep(1000);

	} while (bytes_available > 0);

	// These constants and equations are from the pga460 datasheet, page 50.
	float juntion_to_ambient_thermal_resistance = 96.1;
	float v_power = 16.5;
	float supply_current_listening = 0.012;
	float temperature = ((buf_rx[1] - 64) / 1.5f) -
			    (juntion_to_ambient_thermal_resistance * supply_current_listening * v_power);

	return temperature;
}

int PGA460::open_serial()
{
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_WARN("Failed to open serial port");
		return PX4_ERROR;
	}

	struct termios uart_config;

	int termios_state;

	// Fill the struct for the new configuration.
	tcgetattr(_fd, &uart_config);

	// Input flags - Turn off input processing:
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |  INLCR | IGNCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF);

	uart_config.c_iflag |= IGNPAR;

	// Output flags - Turn off output processing:
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// No parity, one stop bit, disable flow control.
	uart_config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);

	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	uart_config.c_cc[VMIN] = 1;

	uart_config.c_cc[VTIME] = 0;

	speed_t speed = 115200;

	// Set the baud rate.
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_WARN("ERR CFG: %d ISPD", termios_state);
		return PX4_ERROR;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_WARN("ERR CFG: %d OSPD\n", termios_state);
		return PX4_ERROR;
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR baud %d ATTR", termios_state);
		return PX4_ERROR;
	}

	return _fd;
}

void PGA460::print_device_status()
{
	uint8_t status_flags1 = read_register(0x4C);
	uint8_t status_flags2 = read_register(0x4D);

	if ((status_flags1 & 0x0F) || status_flags2) {
		if (status_flags1 & 0x0F & 1) {
			PX4_INFO("Trim EEPROM space data CRC error");
		}

		if (status_flags1 & 0x0F & 1 << 1) {
			PX4_INFO("User EEPROM space data CRC error");
		}

		if (status_flags1 & 0x0F & 1 << 2) {
			PX4_INFO("Threshold map configuration register data CRC error");
		}

		if (status_flags1 & 0x0F & 1 << 3) {
			PX4_INFO("Wakeup Error");
		}

		if (status_flags2 & 1) {
			PX4_INFO("VPWR pin under voltage");
		}

		if (status_flags2 & 1 << 1) {
			PX4_INFO("VPWR pin over voltage");
		}

		if (status_flags2 & 1 << 2) {
			PX4_INFO("AVDD pin under voltage");
		}

		if (status_flags2 & 1 << 3) {
			PX4_INFO("AVDD pin over voltage");
		}

		if (status_flags2 & 1 << 4) {
			PX4_INFO("IOREG pin under voltage");
		}

		if (status_flags2 & 1 << 5) {
			PX4_INFO("IOREG pin over voltage");
		}

		if (status_flags2 & 1 << 6) {
			PX4_INFO("Thermal shutdown has occured");
		}
	}
}

void PGA460::print_diagnostics(const uint8_t diagnostic_byte)
{
	// Check the diagnostics bit field.
	if (diagnostic_byte & 1 << 6) {
		if (diagnostic_byte & 1 << 0) {
			PX4_INFO("Device busy");
		}

		if (diagnostic_byte & 1 << 1) {
			PX4_INFO("Sync field bit rate too high/low");
		}

		if (diagnostic_byte & 1 << 2) {
			PX4_INFO("Consecutive sync bit fields do not match");
		}

		if (diagnostic_byte & 1 << 3) {
			PX4_INFO("Invalid checksum");
		}

		if (diagnostic_byte & 1 << 4) {
			PX4_INFO("Invalid command");
		}

		if (diagnostic_byte & 1 << 5) {
			PX4_INFO("General comm erorr");
		}

	} else if (diagnostic_byte & 1 << 7) {
		if (diagnostic_byte & 1 << 0) {
			PX4_INFO("Device busy");
		}

		if (diagnostic_byte & 1 << 1) {
			PX4_INFO("Threshold settings CRC error");
		}

		if (diagnostic_byte & 1 << 2) {
			PX4_INFO("Frequency diagnostics error");
		}

		if (diagnostic_byte & 1 << 3) {
			PX4_INFO("Voltage diagnostics error");
		}

		if (diagnostic_byte & 1 << 4) {
			PX4_INFO("Always zero....");
		}

		if (diagnostic_byte & 1 << 5) {
			PX4_INFO("EEPROM CRC or TRIM CRC error");
		}
	}
}

int PGA460::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Ultrasonic range finder driver that handles the communication with the device and publishes the distance via uORB.

### Implementation
This driver is implented as a NuttX task. This Implementation was chosen due to the need for polling on a message
via UART, which is not supported in the work_queue. This driver continuously takes range measurements while it is
running. A simple algorithm to detect false readings is implemented at the driver levelin an attemptto improve
the quality of data that is being published. The driver will not publish data at all if it deems the sensor data
to be invalid or unstable.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pga460", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("device_path", "The pga460 sensor device path, (e.g: /dev/ttyS6)", true);
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("help");

	return PX4_OK;
}

int PGA460::read_eeprom()
{
	unlock_eeprom();

	const int array_size = 43;
	 uint8_t user_settings[array_size] =
		{USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4,
		 USER_DATA5, USER_DATA6, USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10,
		 USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, USER_DATA15, USER_DATA16,
		 USER_DATA17, USER_DATA18, USER_DATA19, USER_DATA20,
		 TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, INIT_GAIN, FREQUENCY, DEADTIME,
		 PULSE_P1, PULSE_P2, CURR_LIM_P1, CURR_LIM_P2, REC_LENGTH, FREQ_DIAG, SAT_FDIAG_TH, FVOLT_DEC, DECPL_TEMP,
		 DSP_SCALE, TEMP_TRIM, P1_GAIN_CTRL, P2_GAIN_CTRL};


	int ret = -1;
	uint8_t cmd_buf[2] = {SYNCBYTE, EEBR};
	uint8_t buf_rx[array_size + 2] = {};

	// The pga460 responds to this write() call by reporting current eeprom values.
	ret = ::write(_fd, &cmd_buf[0], sizeof(cmd_buf));

	if (!ret) {
		return PX4_ERROR;
	}

	px4_usleep(10000);

	int bytes_available = sizeof(buf_rx);
	int total_bytes = 0;

	do {
		ret = ::read(_fd, buf_rx + total_bytes, sizeof(buf_rx));

		total_bytes += ret;

		if(ret < 0) {
			tcflush(_fd, TCIFLUSH);
			PX4_ERR("read err2: %d", ret);
			return ret;
		}

		bytes_available -= ret;

		px4_usleep(1000);

	} while (bytes_available > 0);

	// Check the buffers to ensure they match.
	int mismatched_bytes = memcmp(buf_rx + 1, user_settings, array_size);

	if (mismatched_bytes == 0) {
		PX4_INFO("EEPROM has settings.");
		return PX4_OK;
	} else {
		print_diagnostics(buf_rx[0]);
		PX4_INFO("EEPROM does not have settings.");
		return PX4_ERROR;
	}
}

uint8_t PGA460::read_register(const uint8_t reg)
{
	// must unlock the eeprom registers before you can read or write to them
	if (reg < 0x40) {
		unlock_eeprom();
	}

	uint8_t buf_tx[4] = {SYNCBYTE, SRR, reg, 0xFF};
	uint8_t checksum = calc_checksum(&buf_tx[1], 2);
	buf_tx[3] = checksum;

	int ret = ::write(_fd, &buf_tx[0], sizeof(buf_tx));

	if(!ret) {
		return PX4_ERROR;
	}

	px4_usleep(10000);

	uint8_t buf_rx[3] = {};
	int bytes_available = sizeof(buf_rx);
	int total_bytes = 0;

	do {
		ret = ::read(_fd, buf_rx + total_bytes, sizeof(buf_rx));

		total_bytes += ret;

		if(ret < 0) {
			tcflush(_fd, TCIFLUSH);
			PX4_ERR("read err3: %d", ret);
			return ret;
		}

		bytes_available -= ret;

		px4_usleep(1000);

	} while (bytes_available > 0);

	// Prints errors if there are any.
	print_diagnostics(buf_rx[0]);

	return buf_rx[1];
}

int PGA460::read_threshold_registers()
{
	const int array_size = 32;
	uint8_t user_settings[array_size] = {P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4,
					     P1_THR_5, P1_THR_6, P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11,
					     P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
					     P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6,
					     P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13,
					     P2_THR_14, P2_THR_15
					    };

	uint8_t buf_tx[2] =  {SYNCBYTE, THRBR};

	int ret = ::write(_fd, &buf_tx[0], sizeof(buf_tx));

	if(!ret) {
		return PX4_ERROR;
	}

	px4_usleep(10000);

	uint8_t buf_rx[array_size + 2] = {};
	int bytes_available = sizeof(buf_rx);
	int total_bytes = 0;

	do {
		ret = ::read(_fd, buf_rx + total_bytes, sizeof(buf_rx));

		total_bytes += ret;

		if(ret < 0) {
			tcflush(_fd, TCIFLUSH);
			PX4_ERR("read err3: %d", ret);
			return ret;
		}

		bytes_available -= ret;

		px4_usleep(1000);

	} while (bytes_available > 0);

	// Check to ensure the buffers match.
	int mismatch = memcmp(buf_rx + 1, user_settings, sizeof(buf_rx) - 2);

	if (mismatch == 0) {
		PX4_INFO("Threshold registers have program settings");
		return PX4_OK;

	} else {
		PX4_WARN("Threshold registers do not have program settings");
		print_diagnostics(buf_rx[0]);
		return PX4_ERROR;
	}
}

int PGA460::request_results()
{
	uint8_t buf_tx[2] = {SYNCBYTE, UMR};
	int ret = ::write(_fd, &buf_tx[0], sizeof(buf_tx));
	px4_usleep(10000);

	if(!ret) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

void PGA460::run()
{
	open_serial();
	int ret = initialize_device_settings();

	if(ret != PX4_OK) {
		close_serial();
		PX4_INFO("Could not initialize device settings. Exiting.");
		return;
	}

	struct distance_sensor_s report = {};
	_distance_sensor_topic = orb_advertise(ORB_ID(distance_sensor), &report);

	if (_distance_sensor_topic == nullptr) {
		PX4_WARN("Advertise failed.");
		return;
	}

	_start_loop = hrt_absolute_time();

	while (!should_exit()) {
		// Check last report to determine if we need to switch range modes.
		uint8_t mode = set_range_mode();
		take_measurement(mode);

		// Control rate.
		uint64_t loop_time = hrt_absolute_time() - _start_loop;
		uint32_t sleep_time = (loop_time > POLL_RATE_US) ? 0 : POLL_RATE_US - loop_time;
		px4_usleep(sleep_time);

		_start_loop = hrt_absolute_time();
		request_results();
		collect_results();
	}

	PX4_INFO("Exiting.");
	close_serial();
}

uint8_t PGA460::set_range_mode()
{
	// Set the ASICs settings depening on the distance read from our last report.
	if (_previous_valid_report_distance > (MODE_SET_THRESH + MODE_SET_HYST)) {
		_ranging_mode = MODE_LONG_RANGE;

	} else if (_previous_valid_report_distance < (MODE_SET_THRESH - MODE_SET_HYST)) {
		_ranging_mode = MODE_SHORT_RANGE;

	}

	return _ranging_mode;
}

int PGA460::take_measurement(const uint8_t mode)
{
	// Issue a measurement command to detect one object using Preset 1 Burst/Listen.
	uint8_t buf_tx[4] = {SYNCBYTE, mode, 0x01, 0xFF};
	uint8_t checksum = calc_checksum(&buf_tx[1], 2);
	buf_tx[3] = checksum;

	int ret = ::write(_fd, &buf_tx[0], sizeof(buf_tx));

	if(!ret) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int PGA460::task_spawn(int argc, char *argv[])
{
	px4_main_t entry_point = (px4_main_t)&run_trampoline;
	int stack_size = 1256;

	int task_id = px4_task_spawn_cmd("pga460", SCHED_DEFAULT,
					 SCHED_PRIORITY_SLOW_DRIVER, stack_size,
					 entry_point, (char *const *)argv);

	if (task_id < 0) {
		task_id = -1;
		return -errno;
	}

	_task_id = task_id;

	return PX4_OK;
}

void PGA460::uORB_publish_results(const float object_distance)
{
	struct distance_sensor_s report = {};
	report.timestamp = hrt_absolute_time();
	report.device_id = _device_id.devid;
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	report.current_distance = object_distance;
	report.min_distance = MIN_DETECTABLE_DISTANCE;
	report.max_distance = MAX_DETECTABLE_DISTANCE;
	report.signal_quality = 0;

	bool data_is_valid = false;
	static uint8_t good_data_counter = 0;

	// If we are within our MIN and MAX thresholds, continue.
	if (object_distance > MIN_DETECTABLE_DISTANCE && object_distance < MAX_DETECTABLE_DISTANCE) {

		// Height cannot change by more than MAX_SAMPLE_DEVIATION between measurements.
		bool sample_deviation_valid = (report.current_distance < _previous_valid_report_distance + MAX_SAMPLE_DEVIATION)
					      && (report.current_distance > _previous_valid_report_distance - MAX_SAMPLE_DEVIATION);

		// Must have NUM_SAMPLES_CONSISTENT valid samples to be publishing.
		if (sample_deviation_valid) {
			good_data_counter++;

			if (good_data_counter > NUM_SAMPLES_CONSISTENT - 1) {
				good_data_counter = NUM_SAMPLES_CONSISTENT;
				data_is_valid = true;

			} else {
				// Have not gotten NUM_SAMPLES_CONSISTENT consistently valid samples.
				data_is_valid = false;
			}

		} else if (good_data_counter > 0) {
			good_data_counter--;

		} else {
			// Reset our quality of data estimate after NUM_SAMPLES_CONSISTENT invalid samples.
			_previous_valid_report_distance = _previous_report_distance;
		}

		_previous_report_distance = report.current_distance;
	}

	if (data_is_valid) {
		report.signal_quality = 1;
		_previous_valid_report_distance = report.current_distance;
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}
}

int PGA460::unlock_eeprom()
{
	// Two step EEPROM unlock -- send unlock code w/ prog bit set to 0.
	// This might actually be wrapped into command 11 (ee bulk write) but I am not sure.
	uint8_t eeprom_write_buf[5] = {SYNCBYTE, SRW, EE_CNTRL_ADDR, EE_UNLOCK_ST1, 0xFF};
	uint8_t checksum = calc_checksum(&eeprom_write_buf[1], sizeof(eeprom_write_buf) - 2);
	eeprom_write_buf[4] = checksum;
	int ret = ::write(_fd, &eeprom_write_buf[0], sizeof(eeprom_write_buf));

	if(!ret) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int PGA460::write_eeprom()
{
	uint8_t settings_buf[46] = {SYNCBYTE, EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4,
				    USER_DATA5, USER_DATA6, USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10,
				    USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, USER_DATA15, USER_DATA16,
				    USER_DATA17, USER_DATA18, USER_DATA19, USER_DATA20,
				    TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, INIT_GAIN, FREQUENCY, DEADTIME,
				    PULSE_P1, PULSE_P2, CURR_LIM_P1, CURR_LIM_P2, REC_LENGTH, FREQ_DIAG, SAT_FDIAG_TH, FVOLT_DEC, DECPL_TEMP,
				    DSP_SCALE, TEMP_TRIM, P1_GAIN_CTRL, P2_GAIN_CTRL, 0xFF
				   };

	uint8_t checksum = calc_checksum(&settings_buf[1], sizeof(settings_buf) - 2);
	settings_buf[45] = checksum;

	int ret = ::write(_fd, &settings_buf[0], sizeof(settings_buf));

	if(!ret) {
		return PX4_ERROR;
	}

	// Needs time, see datasheet timing requirements.
	px4_usleep(5000);
	unlock_eeprom();
	flash_eeprom();
	px4_usleep(5000);

	uint8_t result = 0;

	// Give up to 100ms for ee_cntrl register to reflect a successful eeprom write.
	for (int i = 0; i < 100; i++) {
		result = read_register(EE_CNTRL_ADDR);
		px4_usleep(5000);

		if (result & 1 << 2) {
			PX4_INFO("EEPROM write successful");
			return PX4_OK;
		}
	}

	PX4_WARN("Failed to write to EEPROM");
	print_diagnostics(result);
	return PX4_ERROR;
}

int PGA460::write_register(const uint8_t reg, const uint8_t val)
{
	// Must unlock the eeprom registers before you can read or write to them.
	if (reg < 0x40) {
		unlock_eeprom();
	}

	uint8_t buf_tx[5] = {SYNCBYTE, SRW, reg, val, 0xFF};
	uint8_t checksum = calc_checksum(&buf_tx[1], sizeof(buf_tx) - 2);
	buf_tx[4] = checksum;

	uint8_t ret = ::write(_fd, &buf_tx[0], sizeof(buf_tx));

	if (ret != sizeof(buf_tx)) {
		return PX4_OK;
	} else {
		return PX4_ERROR;
	}
}

extern "C" __EXPORT int pga460_main(int argc, char *argv[])
{
	return PGA460::main(argc, argv);
}
