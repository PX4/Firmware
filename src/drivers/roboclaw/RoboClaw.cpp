/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file RoboClaw.cpp
 *
 * RoboClaw Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

#include "RoboClaw.hpp"
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>
#include <math.h>

// The RoboClaw has a serial communication timeout of 10ms.
// Add a little extra to account for timing inaccuracy
#define TIMEOUT_US 10500

// If a timeout occurs during serial communication, it will immediately try again this many times
#define TIMEOUT_RETRIES 1

// If a timeout occurs while disarmed, it will try again this many times. This should be a higher number,
// because stopping when disarmed is pretty important.
#define STOP_RETRIES 10

// Number of bytes returned by the Roboclaw when sending command 78, read both encoders
#define ENCODER_MESSAGE_SIZE 10

// Number of bytes for commands 18 and 19, read speeds.
#define ENCODER_SPEED_MESSAGE_SIZE 7

bool RoboClaw::taskShouldExit = false;

RoboClaw::RoboClaw(const char *deviceName, const char *baudRateParam):
	_uart(0),
	_uart_set(),
	_uart_timeout{.tv_sec = 0, .tv_usec = TIMEOUT_US},
	_actuatorsSub(-1),
	_lastEncoderCount{0, 0},
	_encoderCounts{0, 0},
	_motorSpeeds{0, 0}

{
	_param_handles.actuator_write_period_ms = 	param_find("RBCLW_WRITE_PER");
	_param_handles.encoder_read_period_ms = 	param_find("RBCLW_READ_PER");
	_param_handles.counts_per_rev = 			param_find("RBCLW_COUNTS_REV");
	_param_handles.serial_baud_rate = 			param_find(baudRateParam);
	_param_handles.address = 					param_find("RBCLW_ADDRESS");

	_parameters_update();

	// start serial port
	_uart = open(deviceName, O_RDWR | O_NOCTTY);

	if (_uart < 0) { err(1, "could not open %s", deviceName); }

	int ret = 0;
	struct termios uart_config {};
	ret = tcgetattr(_uart, &uart_config);

	if (ret < 0) { err(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, _parameters.serial_baud_rate);

	if (ret < 0) { err(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, _parameters.serial_baud_rate);

	if (ret < 0) { err(1, "failed to set output speed"); }

	ret = tcsetattr(_uart, TCSANOW, &uart_config);

	if (ret < 0) { err(1, "failed to set attr"); }

	FD_ZERO(&_uart_set);

	// setup default settings, reset encoders
	resetEncoders();
}

RoboClaw::~RoboClaw()
{
	setMotorDutyCycle(MOTOR_1, 0.0);
	setMotorDutyCycle(MOTOR_2, 0.0);
	close(_uart);
}

void RoboClaw::taskMain()
{
	// Make sure the Roboclaw is actually connected, so I don't just spam errors if it's not.
	uint8_t rbuff[4];
	int err_code = _transaction(CMD_READ_STATUS, nullptr, 0, &rbuff[0], sizeof(rbuff), false, true);

	if (err_code <= 0) {
		PX4_ERR("Unable to connect to Roboclaw. Shutting down Roboclaw driver.");
		return;
	}

	// This main loop performs two different tasks, asynchronously:
	// - Send actuator_controls_0 to the Roboclaw as soon as they are available
	// - Read the encoder values at a constant rate
	// To do this, the timeout on the poll() function is used.
	// waitTime is the amount of time left (int microseconds) until the next time I should read from the encoders.
	// It is updated at the end of every loop. Sometimes, if the actuator_controls_0 message came in right before
	// I should have read the encoders, waitTime will be 0. This is fine. When waitTime is 0, poll() will return
	// immediately with a timeout. (Or possibly with a message, if one happened to be available at that exact moment)
	uint64_t encoderTaskLastRun = 0;
	int waitTime = 0;

	uint64_t actuatorsLastWritten = 0;

	_actuatorsSub = orb_subscribe(ORB_ID(actuator_controls_0));
	orb_set_interval(_actuatorsSub, _parameters.actuator_write_period_ms);

	_armedSub = orb_subscribe(ORB_ID(actuator_armed));
	_paramSub = orb_subscribe(ORB_ID(parameter_update));

	pollfd fds[3];
	fds[0].fd = _paramSub;
	fds[0].events = POLLIN;
	fds[1].fd = _actuatorsSub;
	fds[1].events = POLLIN;
	fds[2].fd = _armedSub;
	fds[2].events = POLLIN;

	memset((void *) &_wheelEncoderMsg[0], 0, sizeof(_wheelEncoderMsg));
	_wheelEncoderMsg[0].pulses_per_rev = _parameters.counts_per_rev;
	_wheelEncoderMsg[1].pulses_per_rev = _parameters.counts_per_rev;

	while (!taskShouldExit) {

		int pret = poll(fds, sizeof(fds) / sizeof(pollfd), waitTime / 1000);

		bool actuators_timeout = int(hrt_absolute_time() - actuatorsLastWritten) > 2000 * _parameters.actuator_write_period_ms;

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(parameter_update), _paramSub, &_paramUpdate);
			_parameters_update();
		}

		// No timeout, update on either the actuator controls or the armed state
		if (pret > 0 && (fds[1].revents & POLLIN || fds[2].revents & POLLIN || actuators_timeout)) {
			orb_copy(ORB_ID(actuator_controls_0), _actuatorsSub, &_actuatorControls);
			orb_copy(ORB_ID(actuator_armed), _armedSub, &_actuatorArmed);

			int drive_ret = 0, turn_ret = 0;

			const bool disarmed = !_actuatorArmed.armed || _actuatorArmed.lockdown || _actuatorArmed.manual_lockdown
					      || _actuatorArmed.force_failsafe || actuators_timeout;

			if (disarmed) {
				// If disarmed, I want to be certain that the stop command gets through.
				int tries = 0;

				while (tries < STOP_RETRIES && ((drive_ret = drive(0.0)) <= 0 || (turn_ret = turn(0.0)) <= 0)) {
					PX4_ERR("Error trying to stop: Drive: %d, Turn: %d", drive_ret, turn_ret);
					tries++;
					px4_usleep(TIMEOUT_US);
				}

			} else {
				drive_ret = drive(_actuatorControls.control[actuator_controls_s::INDEX_THROTTLE]);
				turn_ret = turn(_actuatorControls.control[actuator_controls_s::INDEX_YAW]);

				if (drive_ret <= 0 || turn_ret <= 0) {
					PX4_ERR("Error controlling RoboClaw. Drive err: %d. Turn err: %d", drive_ret, turn_ret);
				}
			}

			actuatorsLastWritten = hrt_absolute_time();

		} else {
			// A timeout occurred, which means that it's time to update the encoders
			encoderTaskLastRun = hrt_absolute_time();

			if (readEncoder() > 0) {

				for (int i = 0; i < 2; i++) {
					_wheelEncoderMsg[i].timestamp = encoderTaskLastRun;

					_wheelEncoderMsg[i].encoder_position = _encoderCounts[i];
					_wheelEncoderMsg[i].speed = _motorSpeeds[i];

					_wheelEncodersAdv[i].publish(_wheelEncoderMsg[i]);
				}

			} else {
				PX4_ERR("Error reading encoders");
			}
		}

		waitTime = _parameters.encoder_read_period_ms * 1000 - (hrt_absolute_time() - encoderTaskLastRun);
		waitTime = waitTime < 0 ? 0 : waitTime;
	}

	orb_unsubscribe(_actuatorsSub);
	orb_unsubscribe(_armedSub);
	orb_unsubscribe(_paramSub);
}

int RoboClaw::readEncoder()
{

	uint8_t rbuff_pos[ENCODER_MESSAGE_SIZE];
	// I am saving space by overlapping the two separate motor speeds, so that the final buffer will look like:
	// [<speed 1> <speed 2> <status 2> <checksum 2>]
	// And I just ignore all of the statuses and checksums. (The _transaction() function internally handles the
	// checksum)
	uint8_t rbuff_speed[ENCODER_SPEED_MESSAGE_SIZE + 4];

	bool success = false;

	for (int retry = 0; retry < TIMEOUT_RETRIES && !success; retry++) {
		success = _transaction(CMD_READ_BOTH_ENCODERS, nullptr, 0, &rbuff_pos[0], ENCODER_MESSAGE_SIZE, false,
				       true) == ENCODER_MESSAGE_SIZE;
		success = success && _transaction(CMD_READ_SPEED_1, nullptr, 0, &rbuff_speed[0], ENCODER_SPEED_MESSAGE_SIZE, false,
						  true) == ENCODER_SPEED_MESSAGE_SIZE;
		success = success && _transaction(CMD_READ_SPEED_2, nullptr, 0, &rbuff_speed[4], ENCODER_SPEED_MESSAGE_SIZE, false,
						  true) == ENCODER_SPEED_MESSAGE_SIZE;
	}

	if (!success) {
		PX4_ERR("Error reading encoders");
		return -1;
	}

	uint32_t count;
	uint32_t speed;
	uint8_t *count_bytes;
	uint8_t *speed_bytes;

	for (int motor = 0; motor <= 1; motor++) {
		count = 0;
		speed = 0;
		count_bytes = &rbuff_pos[motor * 4];
		speed_bytes = &rbuff_speed[motor * 4];

		// Data from the roboclaw is big-endian. This converts the bytes to an integer, regardless of the
		// endianness of the system this code is running on.
		for (int byte = 0; byte < 4; byte++) {
			count = (count << 8) + count_bytes[byte];
			speed = (speed << 8) + speed_bytes[byte];
		}

		// The Roboclaw stores encoder counts as unsigned 32-bit ints. This can overflow, especially when starting
		// at 0 and moving backward. The Roboclaw has overflow flags for this, but in my testing, they don't seem
		// to work. This code detects overflow manually.
		// These diffs are the difference between the count I just read from the Roboclaw and the last
		// count that was read from the roboclaw for this motor. fwd_diff assumes that the wheel moved forward,
		// and rev_diff assumes it moved backward. If the motor actually moved forward, then rev_diff will be close
		// to 2^32 (UINT32_MAX). If the motor actually moved backward, then fwd_diff will be close to 2^32.
		// To detect and account for overflow, I just assume that the smaller diff is correct.
		// Strictly speaking, if the wheel rotated more than 2^31 encoder counts since the last time I checked, this
		// will be wrong. But that's 1.7 million revolutions, so it probably won't come up.
		uint32_t fwd_diff = count - _lastEncoderCount[motor];
		uint32_t rev_diff = _lastEncoderCount[motor] - count;
		// At this point, abs(diff) is always <= 2^31, so this cast from unsigned to signed is safe.
		int32_t diff = fwd_diff <= rev_diff ? fwd_diff : -int32_t(rev_diff);
		_encoderCounts[motor] += diff;
		_lastEncoderCount[motor] = count;

		_motorSpeeds[motor] = speed;
	}

	return 1;
}

void RoboClaw::printStatus(char *string, size_t n)
{
	snprintf(string, n, "pos1,spd1,pos2,spd2: %10.2f %10.2f %10.2f %10.2f\n",
		 double(getMotorPosition(MOTOR_1)),
		 double(getMotorSpeed(MOTOR_1)),
		 double(getMotorPosition(MOTOR_2)),
		 double(getMotorSpeed(MOTOR_2)));
}

float RoboClaw::getMotorPosition(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _encoderCounts[0];

	} else if (motor == MOTOR_2) {
		return _encoderCounts[1];

	} else {
		warnx("Unknown motor value passed to RoboClaw::getMotorPosition");
		return NAN;
	}
}

float RoboClaw::getMotorSpeed(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _motorSpeeds[0];

	} else if (motor == MOTOR_2) {
		return _motorSpeeds[1];

	} else {
		warnx("Unknown motor value passed to RoboClaw::getMotorPosition");
		return NAN;
	}
}

int RoboClaw::setMotorSpeed(e_motor motor, float value)
{
	e_command command;

	// send command
	if (motor == MOTOR_1) {
		if (value > 0) {
			command = CMD_DRIVE_FWD_1;

		} else {
			command = CMD_DRIVE_REV_1;
		}

	} else if (motor == MOTOR_2) {
		if (value > 0) {
			command = CMD_DRIVE_FWD_2;

		} else {
			command = CMD_DRIVE_REV_2;
		}

	} else {
		return -1;
	}

	return _sendUnsigned7Bit(command, value);
}

int RoboClaw::setMotorDutyCycle(e_motor motor, float value)
{

	e_command command;

	// send command
	if (motor == MOTOR_1) {
		command = CMD_SIGNED_DUTYCYCLE_1;

	} else if (motor == MOTOR_2) {
		command = CMD_SIGNED_DUTYCYCLE_2;

	} else {
		return -1;
	}

	return _sendSigned16Bit(command, value);
}

int RoboClaw::drive(float value)
{
	e_command command = value >= 0 ? CMD_DRIVE_FWD_MIX : CMD_DRIVE_REV_MIX;
	return _sendUnsigned7Bit(command, value);
}

int RoboClaw::turn(float value)
{
	e_command command = value >= 0 ? CMD_TURN_LEFT : CMD_TURN_RIGHT;
	return _sendUnsigned7Bit(command, value);
}

int RoboClaw::resetEncoders()
{
	return _sendNothing(CMD_RESET_ENCODERS);
}

int RoboClaw::_sendUnsigned7Bit(e_command command, float data)
{
	data = fabs(data);

	if (data > 1.0f) {
		data = 1.0f;
	}

	auto byte = (uint8_t)(data * INT8_MAX);
	uint8_t recv_byte;
	return _transaction(command, &byte, 1, &recv_byte, 1);
}

int RoboClaw::_sendSigned16Bit(e_command command, float data)
{
	if (data > 1.0f) {
		data = 1.0f;

	} else if (data < -1.0f) {
		data = -1.0f;
	}

	auto buff = (uint16_t)(data * INT16_MAX);
	uint8_t recv_buff;
	return _transaction(command, (uint8_t *) &buff, 2, &recv_buff, 1);
}

int RoboClaw::_sendNothing(e_command command)
{
	uint8_t recv_buff;
	return _transaction(command, nullptr, 0, &recv_buff, 1);
}

uint16_t RoboClaw::_calcCRC(const uint8_t *buf, size_t n, uint16_t init)
{
	uint16_t crc = init;

	for (size_t byte = 0; byte < n; byte++) {
		crc = crc ^ (((uint16_t) buf[byte]) << 8);

		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;

			} else {
				crc = crc << 1;
			}
		}
	}

	return crc;
}

int RoboClaw::_transaction(e_command cmd, uint8_t *wbuff, size_t wbytes,
			   uint8_t *rbuff, size_t rbytes, bool send_checksum, bool recv_checksum)
{
	int err_code = 0;

	// WRITE

	tcflush(_uart, TCIOFLUSH); // flush  buffers
	uint8_t buf[wbytes + 4];
	buf[0] = (uint8_t) _parameters.address;
	buf[1] = cmd;

	if (wbuff) {
		memcpy(&buf[2], wbuff, wbytes);
	}

	wbytes = wbytes + (send_checksum ? 4 : 2);

	if (send_checksum) {
		uint16_t sum = _calcCRC(buf, wbytes - 2);
		buf[wbytes - 2] = (sum >> 8) & 0xFF;
		buf[wbytes - 1] = sum & 0xFFu;
	}

	int count = write(_uart, buf, wbytes);

	if (count < (int) wbytes) { // Did not successfully send all bytes.
		PX4_ERR("Only wrote %d out of %d bytes", count, (int) wbytes);
		return -1;
	}

	// READ

	FD_ZERO(&_uart_set);
	FD_SET(_uart, &_uart_set);

	uint8_t *rbuff_curr = rbuff;
	size_t bytes_read = 0;

	// select(...) returns as soon as even 1 byte is available. read(...) returns immediately, no matter how many
	// bytes are available. I need to keep reading until I get the number of bytes I expect.
	while (bytes_read < rbytes) {
		// select(...) may change this timeout struct (because it is not const). So I reset it every time.
		_uart_timeout.tv_sec = 0;
		_uart_timeout.tv_usec = TIMEOUT_US;
		err_code = select(_uart + 1, &_uart_set, nullptr, nullptr, &_uart_timeout);

		// An error code of 0 means that select timed out, which is how the Roboclaw indicates an error.
		if (err_code <= 0) {
			return err_code;
		}

		err_code = read(_uart, rbuff_curr, rbytes - bytes_read);

		if (err_code <= 0) {
			return err_code;

		} else {
			bytes_read += err_code;
			rbuff_curr += err_code;
		}
	}

	//TODO: Clean up this mess of IFs and returns

	if (recv_checksum) {
		if (bytes_read < 2) {
			return -1;
		}

		// The checksum sent back by the roboclaw is calculated based on the address and command bytes as well
		// as the data returned.
		uint16_t checksum_calc = _calcCRC(buf, 2);
		checksum_calc = _calcCRC(rbuff, bytes_read - 2, checksum_calc);
		uint16_t checksum_recv = (rbuff[bytes_read - 2] << 8) + rbuff[bytes_read - 1];

		if (checksum_calc == checksum_recv) {
			return bytes_read;

		} else {
			return -10;
		}

	} else {
		if (bytes_read == 1 && rbuff[0] == 0xFF) {
			return 1;

		} else {
			return -11;
		}
	}
}

void RoboClaw::_parameters_update()
{
	param_get(_param_handles.counts_per_rev, &_parameters.counts_per_rev);
	param_get(_param_handles.encoder_read_period_ms, &_parameters.encoder_read_period_ms);
	param_get(_param_handles.actuator_write_period_ms, &_parameters.actuator_write_period_ms);
	param_get(_param_handles.address, &_parameters.address);

	if (_actuatorsSub > 0) {
		orb_set_interval(_actuatorsSub, _parameters.actuator_write_period_ms);
	}

	int baudRate;
	param_get(_param_handles.serial_baud_rate, &baudRate);

	switch (baudRate) {
	case 2400:
		_parameters.serial_baud_rate = B2400;
		break;

	case 9600:
		_parameters.serial_baud_rate = B9600;
		break;

	case 19200:
		_parameters.serial_baud_rate = B19200;
		break;

	case 38400:
		_parameters.serial_baud_rate = B38400;
		break;

	case 57600:
		_parameters.serial_baud_rate = B57600;
		break;

	case 115200:
		_parameters.serial_baud_rate = B115200;
		break;

	case 230400:
		_parameters.serial_baud_rate = B230400;
		break;

	case 460800:
		_parameters.serial_baud_rate = B460800;
		break;

	default:
		_parameters.serial_baud_rate = B2400;
	}
}
