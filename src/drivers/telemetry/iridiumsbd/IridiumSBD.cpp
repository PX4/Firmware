/****************************************************************************
 *
 *   Copyright (c) 2016-2018 PX4 Development Team. All rights reserved.
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

#include "IridiumSBD.h"

#include <px4_platform_common/tasks.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>

#include <systemlib/err.h>
#include <parameters/param.h>

static constexpr const char *satcom_state_string[4] = {"STANDBY", "SIGNAL CHECK", "SBD SESSION", "TEST"};

#define VERBOSE_INFO(...) if (_verbose) { PX4_INFO(__VA_ARGS__); }

#define IRIDIUMSBD_DEVICE_PATH	"/dev/iridium"

IridiumSBD *IridiumSBD::instance;
int IridiumSBD::task_handle;

IridiumSBD::IridiumSBD()
	: CDev(IRIDIUMSBD_DEVICE_PATH)
{
}

///////////////////////////////////////////////////////////////////////
// public functions                                                  //
///////////////////////////////////////////////////////////////////////

int IridiumSBD::start(int argc, char *argv[])
{
	PX4_INFO("starting");

	if (IridiumSBD::instance != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}

	IridiumSBD::instance = new IridiumSBD();

	IridiumSBD::task_handle = px4_task_spawn_cmd("iridiumsbd", SCHED_DEFAULT,
				  SCHED_PRIORITY_SLOW_DRIVER, 1350, (main_t)&IridiumSBD::main_loop_helper, argv);

	int counter = 0;
	IridiumSBD::instance->_start_completed = false;
	IridiumSBD::instance->_task_should_exit = false;

	// give the driver 6 seconds to start up
	while (!IridiumSBD::instance->_start_completed && (IridiumSBD::task_handle != -1) && counter < 60) {
		counter++;
		usleep(100000);
	}

	if (IridiumSBD::instance->_start_completed && (IridiumSBD::task_handle != -1)) {
		return PX4_OK;

	} else {
		// the driver failed to start so make sure it is shut down before exiting
		IridiumSBD::instance->_task_should_exit = true;

		for (int i = 0; (i < 10 + 1) && (IridiumSBD::task_handle != -1); i++) {
			sleep(1);
		}

		return PX4_ERROR;
	}
}

int IridiumSBD::stop()
{
	if (IridiumSBD::instance == nullptr) {
		PX4_WARN("not started");
		return PX4_ERROR;
	}

	if (IridiumSBD::instance->_cdev_used) {
		PX4_WARN("device is used. Stop all users (MavLink)");
		return PX4_ERROR;
	}

	PX4_WARN("stopping...");

	IridiumSBD::instance->_task_should_exit = true;

	// give it enough time to stop
	//param_timeout_s = 10;

	// TODO
	for (int i = 0; (i < 10 + 1) && (IridiumSBD::task_handle != -1); i++) {
		sleep(1);
	}

	// well, kill it anyway, though this may crash
	if (IridiumSBD::task_handle != -1) {
		PX4_WARN("killing task forcefully");

		::close(IridiumSBD::instance->uart_fd);
		task_delete(IridiumSBD::task_handle);
		IridiumSBD::task_handle = -1;
		delete IridiumSBD::instance;
		IridiumSBD::instance = nullptr;
	}

	return OK;
}

void IridiumSBD::status()
{
	if (IridiumSBD::instance == nullptr) {
		PX4_WARN("not started");
		return;
	}

	PX4_INFO("started");
	PX4_INFO("state: %s", satcom_state_string[instance->_state]);

	PX4_INFO("TX buf written:               %d", instance->_tx_buf_write_idx);
	PX4_INFO("Signal quality:               %d", instance->_signal_quality);
	PX4_INFO("TX session pending:           %d", instance->_tx_session_pending);
	PX4_INFO("RX session pending:           %d", instance->_rx_session_pending);
	PX4_INFO("RX read pending:              %d", instance->_rx_read_pending);
	PX4_INFO("Time since last signal check: %lld", hrt_absolute_time() - instance->_last_signal_check);
	PX4_INFO("Last heartbeat:               %lld", instance->_last_heartbeat);
}

void IridiumSBD::test(int argc, char *argv[])
{
	if (instance == nullptr) {
		PX4_WARN("not started");
		return;
	}

	if (instance->_state != SATCOM_STATE_STANDBY || instance->_test_pending) {
		PX4_WARN("MODEM BUSY!");
		return;
	}

	if (argc > 2) {
		strncpy(instance->_test_command, argv[2], sizeof(instance->_test_command));
		instance->_test_command[sizeof(instance->_test_command) - 1] = 0;

	} else {
		instance->_test_command[0] = 0;
	}

	instance->schedule_test();
}

int IridiumSBD::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case FIONREAD: {
			int count = _rx_msg_end_idx - _rx_msg_read_idx;
			*(int *)arg = count;

			return OK;
		}

	case FIONSPACE: {
			int count = SATCOM_TX_BUF_LEN - _tx_buf_write_idx + SATCOM_MAX_MESSAGE_LENGTH;
			*(int *)arg = count;

			return OK;
		}

	default: {

			/* see if the parent class can make any use of it */
			return CDev::ioctl(filp, cmd, arg);
		}
	}
}

///////////////////////////////////////////////////////////////////////
// private functions                                                 //
///////////////////////////////////////////////////////////////////////

int IridiumSBD::main_loop_helper(int argc, char *argv[])
{
	// start the main loop and stay in it
	IridiumSBD::instance->main_loop(argc, argv);

	// tear down everything after the main loop exits
	::close(IridiumSBD::instance->uart_fd);
	IridiumSBD::task_handle = -1;
	delete IridiumSBD::instance;
	IridiumSBD::instance = nullptr;

	PX4_WARN("stopped");
	return 0;
}

void IridiumSBD::main_loop(int argc, char *argv[])
{
	CDev::init();

	pthread_mutex_init(&_tx_buf_mutex, NULL);
	pthread_mutex_init(&_rx_buf_mutex, NULL);

	int arg_i = 3;
	int arg_uart_name = 0;

	while (arg_i < argc) {
		if (!strcmp(argv[arg_i], "-d")) {
			arg_i++;
			arg_uart_name = arg_i;

		} else if (!strcmp(argv[arg_i], "-v")) {
			PX4_WARN("verbose mode ON");
			_verbose = true;
		}

		arg_i++;
	}

	if (arg_uart_name == 0) {
		PX4_ERR("no Iridium SBD modem UART port provided!");
		_task_should_exit = true;
		return;
	}


	bool command_executed = false;

	for (int counter = 0; (counter < 20) && !command_executed; counter++) {
		if (open_uart(argv[arg_uart_name]) == SATCOM_UART_OK) {
			command_executed = true;

		} else {
			usleep(100000);
		}
	}

	if (!command_executed) {
		PX4_ERR("failed to open UART port!");
		_task_should_exit = true;
		return;
	}

	// disable flow control
	command_executed = false;

	for (int counter = 0; (counter < 20) && !command_executed; counter++) {
		write_at("AT&K0");

		if (read_at_command() != SATCOM_RESULT_OK) {
			usleep(100000);

		} else {
			command_executed = true;
		}
	}

	if (!command_executed) {
		PX4_ERR("modem not responding");
		_task_should_exit = true;
		return;
	}

	// disable command echo
	command_executed = false;

	for (int counter = 0; (counter < 10) && !command_executed; counter++) {
		write_at("ATE0");

		if (read_at_command() != SATCOM_RESULT_OK) {
			usleep(100000);

		} else {
			command_executed = true;
		}
	}

	if (!command_executed) {
		PX4_ERR("modem not responding");
		_task_should_exit = true;
		return;
	}

	param_t param_pointer;

	param_pointer = param_find("ISBD_READ_INT");
	param_get(param_pointer, &_param_read_interval_s);

	param_pointer = param_find("ISBD_SBD_TIMEOUT");
	param_get(param_pointer, &_param_session_timeout_s);

	if (_param_session_timeout_s < 0) {
		_param_session_timeout_s = 60;
	}

	param_pointer = param_find("ISBD_STACK_TIME");
	param_get(param_pointer, &_param_stacking_time_ms);

	if (_param_stacking_time_ms < 0) {
		_param_stacking_time_ms = 0;
	}

	VERBOSE_INFO("read interval: %d s", _param_read_interval_s);
	VERBOSE_INFO("SBD session timeout: %d s", _param_session_timeout_s);
	VERBOSE_INFO("SBD stack time: %d ms", _param_stacking_time_ms);

	_start_completed = true;

	while (!_task_should_exit) {
		switch (_state) {
		case SATCOM_STATE_STANDBY:
			standby_loop();
			break;

		case SATCOM_STATE_CSQ:
			csq_loop();
			break;

		case SATCOM_STATE_SBDSESSION:
			sbdsession_loop();
			break;

		case SATCOM_STATE_TEST:
			test_loop();
			break;
		}

		if (_new_state != _state) {
			VERBOSE_INFO("SWITCHING STATE FROM %s TO %s", satcom_state_string[_state], satcom_state_string[_new_state]);
			_state = _new_state;
			publish_iridium_status();

		} else {
			publish_iridium_status();
			usleep(100000);	// 100ms
		}
	}
}

void IridiumSBD::standby_loop(void)
{
	if (_test_pending) {
		_test_pending = false;

		if (!strcmp(_test_command, "s")) {
			write(0, "kreczmer", 8);

		} else if (!strcmp(_test_command, "read")) {
			_rx_session_pending = true;

		} else {
			_test_timer = hrt_absolute_time();
			start_test();
			return;
		}
	}

	// check for incoming SBDRING, handled inside read_at_command()
	read_at_command();

	if (_param_read_interval_s > 0
	    && ((hrt_absolute_time() - _last_read_time) > (uint64_t)_param_read_interval_s * 1000000)) {
		_rx_session_pending = true;
	}

	// write the MO buffer when the message stacking time expires
	if (_tx_buf_write_pending && ((hrt_absolute_time() - _last_write_time) > (uint64_t)_param_stacking_time_ms * 1000)) {
		write_tx_buf();
	}

	// do not start an SBD session if there is still data in the MT buffer, or it will be lost
	if ((_tx_session_pending || _rx_session_pending) && !_rx_read_pending) {
		if (_signal_quality > 0) {
			// clear the MO buffer if we only want to read a message
			if (_rx_session_pending && !_tx_session_pending) {
				if (clear_mo_buffer()) {
					start_sbd_session();
					return;
				}

			} else {
				start_sbd_session();
				return;
			}

		} else {
			start_csq();
			return;
		}
	}

	// start a signal check if requested and not a switch to another mode is scheduled
	if (((hrt_absolute_time() - _last_signal_check) > SATCOM_SIGNAL_REFRESH_DELAY)
	    && (_new_state == SATCOM_STATE_STANDBY)) {
		start_csq();
		return;
	}

	// only read the MT buffer if the higher layer (mavlink app) read the previous message
	if (_rx_read_pending && (_rx_msg_read_idx == _rx_msg_end_idx) && (_new_state == SATCOM_STATE_STANDBY)) {
		read_rx_buf();
		return;
	}
}

void IridiumSBD::csq_loop(void)
{
	int res = read_at_command();

	if (res == SATCOM_RESULT_NA) {
		return;
	}

	if (res != SATCOM_RESULT_OK) {
		VERBOSE_INFO("UPDATE SIGNAL QUALITY: ERROR");

		_new_state = SATCOM_STATE_STANDBY;
		return;
	}

	if (strncmp((const char *)_rx_command_buf, "+CSQ:", 5)) {
		VERBOSE_INFO("UPDATE SIGNAL QUALITY: WRONG ANSWER:");
		VERBOSE_INFO("%s", _rx_command_buf);

		_new_state = SATCOM_STATE_STANDBY;
		return;
	}

	_signal_quality = _rx_command_buf[5] - 48;

	VERBOSE_INFO("SIGNAL QUALITY: %d", _signal_quality);

	_new_state = SATCOM_STATE_STANDBY;
}

void IridiumSBD::sbdsession_loop(void)
{
	int res = read_at_command();

	if (res == SATCOM_RESULT_NA) {
		if ((_param_session_timeout_s > 0)
		    && (((hrt_absolute_time() - _session_start_time))
			> (uint64_t)_param_session_timeout_s * 1000000)) {

			PX4_WARN("SBD SESSION: TIMEOUT!");
			++_failed_sbd_sessions;
			_new_state = SATCOM_STATE_STANDBY;
			pthread_mutex_unlock(&_tx_buf_mutex);
		}

		return;
	}

	if (res != SATCOM_RESULT_OK) {
		VERBOSE_INFO("SBD SESSION: ERROR. RESULT: %d", res);

		++_failed_sbd_sessions;
		_new_state = SATCOM_STATE_STANDBY;
		pthread_mutex_unlock(&_tx_buf_mutex);
		return;
	}

	if (strncmp((const char *)_rx_command_buf, "+SBDIX:", 7)) {

		VERBOSE_INFO("SBD SESSION: WRONG ANSWER: %s", _rx_command_buf);

		_new_state = SATCOM_STATE_STANDBY;
		++_failed_sbd_sessions;
		pthread_mutex_unlock(&_tx_buf_mutex);
		return;
	}

	int mo_status, mt_status, mt_len, mt_queued;
	const char *p = (const char *)_rx_command_buf + 7;
	char **rx_buf_parse = (char **)&p;

	mo_status = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	strtol(*rx_buf_parse, rx_buf_parse, 10); // MOMSN, ignore it
	(*rx_buf_parse)++;
	mt_status = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	strtol(*rx_buf_parse, rx_buf_parse, 10); // MTMSN, ignore it
	(*rx_buf_parse)++;
	mt_len = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	mt_queued = strtol(*rx_buf_parse, rx_buf_parse, 10);

	VERBOSE_INFO("MO ST: %d, MT ST: %d, MT LEN: %d, MT QUEUED: %d", mo_status, mt_status, mt_len, mt_queued);
	VERBOSE_INFO("SBD session duration: %.2f", (hrt_absolute_time() - _session_start_time) / 1000000.0);

	switch (mo_status) {
	case 0:
	case 2:
	case 3:
	case 4:
		VERBOSE_INFO("SBD SESSION: SUCCESS (%d)", mo_status);

		_ring_pending = false;
		_tx_session_pending = false;
		_last_read_time = hrt_absolute_time();
		_last_heartbeat = _last_read_time;
		++_successful_sbd_sessions;

		if (mt_queued > 0) {
			_rx_session_pending = true;

		} else {
			_rx_session_pending = false;

		}

		if (mt_len > 0) {
			_rx_read_pending = true;
		}

		// after a successful session reset the tx buffer
		_tx_buf_write_idx = 0;
		break;

	case 1:
		VERBOSE_INFO("SBD SESSION: MO SUCCESS, MT FAIL");
		_last_heartbeat = hrt_absolute_time();

		// after a successful session reset the tx buffer
		_tx_buf_write_idx = 0;
		++_successful_sbd_sessions;

		_tx_session_pending = false;
		break;

	case 32:
		VERBOSE_INFO("SBD SESSION: NO NETWORK SIGNAL");

		++_failed_sbd_sessions;
		_signal_quality = 0;
		break;

	default:
		++_failed_sbd_sessions;
		VERBOSE_INFO("SBD SESSION: FAILED (%d)", mo_status);
	}

	_new_state = SATCOM_STATE_STANDBY;
	pthread_mutex_unlock(&_tx_buf_mutex);
}

void IridiumSBD::test_loop(void)
{
	int res = read_at_command();

	if (res != SATCOM_RESULT_NA) {
		PX4_INFO("TEST RESULT: %d, LENGTH %d\nDATA:\n%s", res, _rx_command_len, _rx_command_buf);
		PX4_INFO("TEST DONE, TOOK %lld MS", (hrt_absolute_time() - _test_timer) / 1000);
		_new_state = SATCOM_STATE_STANDBY;
	}

	// timeout after 60 s in the test state
	if ((hrt_absolute_time() - _test_timer) > 60000000) {
		PX4_WARN("TEST TIMEOUT AFTER %lld S", (hrt_absolute_time() - _test_timer) / 1000000);
		_new_state = SATCOM_STATE_STANDBY;
	}
}

void IridiumSBD::start_csq(void)
{
	if ((_state != SATCOM_STATE_STANDBY) || (_new_state != SATCOM_STATE_STANDBY)) {
		VERBOSE_INFO("CANNOT ENTER CSQ STATE");
		return;

	} else {
		VERBOSE_INFO("UPDATING SIGNAL QUALITY");
	}

	_last_signal_check = hrt_absolute_time();

	if (!is_modem_ready()) {
		VERBOSE_INFO("UPDATE SIGNAL QUALITY: MODEM NOT READY!");
		return;
	}

	write_at("AT+CSQ");
	_new_state = SATCOM_STATE_CSQ;
}

void IridiumSBD::start_sbd_session(void)
{
	if ((_state != SATCOM_STATE_STANDBY) || (_new_state != SATCOM_STATE_STANDBY)) {
		VERBOSE_INFO("CANNOT ENTER SBD SESSION STATE");
		return;

	} else {
		VERBOSE_INFO("STARTING SBD SESSION");
	}

	if (!is_modem_ready()) {
		VERBOSE_INFO("SBD SESSION: MODEM NOT READY!");
		return;
	}

	if (_ring_pending) {
		write_at("AT+SBDIXA");

	} else {
		write_at("AT+SBDIX");
	}

	_new_state = SATCOM_STATE_SBDSESSION;

	pthread_mutex_lock(&_tx_buf_mutex);

	_session_start_time = hrt_absolute_time();
}

void IridiumSBD::start_test(void)
{
	if ((_state != SATCOM_STATE_STANDBY) || (_new_state != SATCOM_STATE_STANDBY)) {
		PX4_INFO("CANNOT ENTER TEST STATE");
		return;
	}

	int res = read_at_command();

	if (res != SATCOM_RESULT_NA) {
		PX4_WARN("SOMETHING WAS IN BUFFER");
		printf("TEST RESULT: %d, LENGTH %d\nDATA:\n%s\nRAW DATA:\n", res, _rx_command_len, _rx_command_buf);

		for (int i = 0; i < _rx_command_len; i++) {
			printf("%d ", _rx_command_buf[i]);
		}

		printf("\n");
	}

	if (!is_modem_ready()) {
		PX4_WARN("MODEM NOT READY!");
		return;
	}

	if (strlen(_test_command) != 0) {
		if ((strstr(_test_command, "AT") != nullptr) || (strstr(_test_command, "at") != nullptr)) {
			PX4_INFO("TEST %s", _test_command);
			write_at(_test_command);
			_new_state = SATCOM_STATE_TEST;

		} else {
			PX4_WARN("The test command does not include AT or at: %s, ignoring it.", _test_command);
			_new_state = SATCOM_STATE_STANDBY;
		}

	} else {
		PX4_INFO("TEST DONE");
	}
}

ssize_t IridiumSBD::write(struct file *filp, const char *buffer, size_t buflen)
{
	// general check if the incoming message would be too large (the buffer should not reset in that case)
	if ((ssize_t)buflen > SATCOM_TX_BUF_LEN) {
		return PX4_ERROR;
	}

	pthread_mutex_lock(&_tx_buf_mutex);

	// parsing the size of the message to write
	if (!_writing_mavlink_packet) {
		if (buflen < 3) {
			_packet_length = buflen;

		} else if ((unsigned char)buffer[0] == 253 && (buflen == 10)) { // mavlink 2
			const uint8_t payload_len = buffer[1];
			const uint8_t incompat_flags = buffer[2];
			_packet_length = payload_len + 12;
			_writing_mavlink_packet = true;

			if (incompat_flags & 0x1) { //signing
				_packet_length += 13;
			}

		} else if ((unsigned char)buffer[0] == 254 && (buflen == 6)) { // mavlink 1
			const uint8_t payload_len = buffer[1];
			_packet_length = payload_len + 8;
			_writing_mavlink_packet = true;

		} else {
			_packet_length = buflen;
		}
	}

	// check if there is enough space to write the message
	if (SATCOM_TX_BUF_LEN - _tx_buf_write_idx - _packet_length < 0) {
		_tx_buf_write_idx = 0;
		++_num_tx_buf_reset;
	}

	// keep track of the remaining packet length and if the full message is written
	_packet_length -= buflen;

	if (_packet_length == 0) {
		_writing_mavlink_packet = false;
	}

	VERBOSE_INFO("WRITE: LEN %d, TX WRITTEN: %d", buflen, _tx_buf_write_idx);

	memcpy(_tx_buf + _tx_buf_write_idx, buffer, buflen);

	_tx_buf_write_idx += buflen;
	_last_write_time = hrt_absolute_time();
	_tx_buf_write_pending = true;

	pthread_mutex_unlock(&_tx_buf_mutex);

	return buflen;
}

ssize_t IridiumSBD::read(struct file *filp, char *buffer, size_t buflen)
{
	pthread_mutex_lock(&_rx_buf_mutex);
	VERBOSE_INFO("READ: LEN %d, RX: %d RX END: %d", buflen, _rx_msg_read_idx, _rx_msg_end_idx);

	if (_rx_msg_read_idx < _rx_msg_end_idx) {
		size_t bytes_to_copy = _rx_msg_end_idx - _rx_msg_read_idx;

		if (bytes_to_copy > buflen) {
			bytes_to_copy = buflen;
		}

		memcpy(buffer, &_rx_msg_buf[_rx_msg_read_idx], bytes_to_copy);

		_rx_msg_read_idx += bytes_to_copy;

		pthread_mutex_unlock(&_rx_buf_mutex);
		return bytes_to_copy;

	} else {
		pthread_mutex_unlock(&_rx_buf_mutex);
		return -EAGAIN;
	}
}

void IridiumSBD::write_tx_buf()
{
	if (!is_modem_ready()) {
		VERBOSE_INFO("WRITE SBD: MODEM NOT READY!");
		return;
	}

	pthread_mutex_lock(&_tx_buf_mutex);

	char command[13];
	sprintf(command, "AT+SBDWB=%d", _tx_buf_write_idx);
	write_at(command);

	if (read_at_command() != SATCOM_RESULT_READY) {
		VERBOSE_INFO("WRITE SBD: MODEM NOT RESPONDING!");
		pthread_mutex_unlock(&_tx_buf_mutex);
		return;
	}

	int sum = 0;

	int written = 0;

	while (written != _tx_buf_write_idx) {
		written += ::write(uart_fd, _tx_buf + written, _tx_buf_write_idx - written);
	}

	for (int i = 0; i < _tx_buf_write_idx; i++) {
		sum += _tx_buf[i];
	}

	uint8_t checksum[2] = {(uint8_t)(sum / 256), (uint8_t)(sum & 255)};
	::write(uart_fd, checksum, 2);


	VERBOSE_INFO("SEND SBD: CHECKSUM %d %d", checksum[0], checksum[1]);

	if (read_at_command(250) != SATCOM_RESULT_OK) {
		VERBOSE_INFO("WRITE SBD: ERROR WHILE WRITING DATA TO MODEM!");

		pthread_mutex_unlock(&_tx_buf_mutex);
		return;
	}

	if (_rx_command_buf[0] != '0') {

		VERBOSE_INFO("WRITE SBD: ERROR WHILE WRITING DATA TO MODEM! (%d)", _rx_command_buf[0] - '0');

		pthread_mutex_unlock(&_tx_buf_mutex);
		return;
	}

	VERBOSE_INFO("WRITE SBD: DATA WRITTEN TO MODEM");

	_tx_buf_write_pending = false;

	pthread_mutex_unlock(&_tx_buf_mutex);

	_tx_session_pending = true;
}

void IridiumSBD::read_rx_buf(void)
{
	if (!is_modem_ready()) {
		VERBOSE_INFO("READ SBD: MODEM NOT READY!");
		return;
	}

	pthread_mutex_lock(&_rx_buf_mutex);


	write_at("AT+SBDRB");

	if (read_at_msg() != SATCOM_RESULT_OK) {
		VERBOSE_INFO("READ SBD: MODEM NOT RESPONDING!");
		_rx_msg_read_idx = _rx_msg_end_idx;
		pthread_mutex_unlock(&_rx_buf_mutex);
		return;
	}

	int data_len = (_rx_msg_buf[0] << 8) + _rx_msg_buf[1];

	// rx_buf contains 2 byte length, data, 2 byte checksum and /r/n delimiter
	if (data_len != _rx_msg_end_idx - 6) {
		PX4_ERR("READ SBD: WRONG DATA LENGTH");
		_rx_msg_read_idx = _rx_msg_end_idx;
		pthread_mutex_unlock(&_rx_buf_mutex);
		return;
	}

	int checksum = 0;

	for (int i = 2; i < data_len + 2; i++) {
		checksum += _rx_msg_buf[i];
	}

	if ((checksum / 256 != _rx_msg_buf[_rx_msg_end_idx - 4]) || ((checksum & 255) != _rx_msg_buf[_rx_msg_end_idx - 3])) {
		PX4_ERR("READ SBD: WRONG DATA CHECKSUM");
		_rx_msg_read_idx = _rx_msg_end_idx;
		pthread_mutex_unlock(&_rx_buf_mutex);
		return;
	}

	_rx_msg_read_idx = 2;	// ignore the length
	_rx_msg_end_idx -= 4;	// ignore the checksum and delimiter
	_rx_read_pending = false;

	pthread_mutex_unlock(&_rx_buf_mutex);
	VERBOSE_INFO("READ SBD: SUCCESS, LEN: %d", data_len);
}

void IridiumSBD::write_at(const char *command)
{
	VERBOSE_INFO("WRITING AT COMMAND: %s", command);

	::write(uart_fd, command, strlen(command));
	::write(uart_fd, "\r", 1);
}

satcom_result_code IridiumSBD::read_at_command(int16_t timeout)
{
	return read_at(_rx_command_buf, &_rx_command_len, timeout);
}

satcom_result_code IridiumSBD::read_at_msg(int16_t timeout)
{
	return read_at(_rx_msg_buf, &_rx_msg_end_idx, timeout);
}

satcom_result_code IridiumSBD::read_at(uint8_t *rx_buf, int *rx_len, int16_t timeout)
{
	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	uint8_t buf = 0;
	int last_rn_idx = 0;
	int rx_buf_pos = 0;
	*rx_len = 0;

	while (1) {
		if (::poll(&fds[0], 1, timeout) > 0) {
			if (::read(uart_fd, &buf, 1) > 0) {
				if (rx_buf_pos == 0 && (buf == '\r' || buf == '\n')) {
					// ignore the leading \r\n
					continue;
				}

				rx_buf[rx_buf_pos++] = buf;

				if (rx_buf[rx_buf_pos - 1] == '\n' && rx_buf[rx_buf_pos - 2] == '\r') {
					// found the \r\n delimiter
					if (rx_buf_pos == last_rn_idx + 2) {
						//if (verbose) { PX4_INFO("second in a row, ignore it");}
						; // second in a row, ignore it

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "OK\r\n", 4)) {
						rx_buf[*rx_len] = 0; 	// null terminator after the information response for printing purposes
						return SATCOM_RESULT_OK;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "ERROR\r\n", 7)) {
						return SATCOM_RESULT_ERROR;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "SBDRING\r\n", 9)) {
						_ring_pending = true;
						_rx_session_pending = true;

						VERBOSE_INFO("GET SBDRING");

						return SATCOM_RESULT_SBDRING;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "READY\r\n", 7)) {
						return SATCOM_RESULT_READY;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "HARDWARE FAILURE", 16)) {
						PX4_WARN("HARDWARE FAILURE!");
						return SATCOM_RESULT_HWFAIL;

					} else {
						*rx_len = rx_buf_pos;	// that was the information response, result code incoming
					}

					last_rn_idx = rx_buf_pos;
				}
			}

		} else {
			break;
		}
	}

	return SATCOM_RESULT_NA;
}

void IridiumSBD::schedule_test(void)
{
	_test_pending = true;
}

bool IridiumSBD::clear_mo_buffer()
{
	write_at("AT+SBDD0");

	if (read_at_command() != SATCOM_RESULT_OK || _rx_command_buf[0] != '0') {
		VERBOSE_INFO("CLEAR MO BUFFER: ERROR");
		return false;
	}

	return true;
}

satcom_uart_status IridiumSBD::open_uart(char *uart_name)
{
	VERBOSE_INFO("opening Iridium SBD modem UART: %s", uart_name);

	uart_fd = ::open(uart_name, O_RDWR | O_BINARY);

	if (uart_fd < 0) {
		VERBOSE_INFO("UART open failed!");
		return SATCOM_UART_OPEN_FAIL;
	}

	// set the UART speed to 115200
	struct termios uart_config;
	tcgetattr(uart_fd, &uart_config);
	cfsetspeed(&uart_config, 115200);
	tcsetattr(uart_fd, TCSANOW, &uart_config);

	VERBOSE_INFO("UART opened");

	return SATCOM_UART_OK;
}

bool IridiumSBD::is_modem_ready(void)
{
	write_at("AT");

	if (read_at_command() == SATCOM_RESULT_OK) {
		return true;

	} else {
		return false;
	}
}

pollevent_t IridiumSBD::poll_state(struct file *filp)
{
	pollevent_t pollstate = 0;

	if (_rx_msg_read_idx < _rx_msg_end_idx) {
		pollstate |= POLLIN;
	}

	if (SATCOM_TX_BUF_LEN - _tx_buf_write_idx > 0) {
		pollstate |= POLLOUT;
	}

	return pollstate;
}

void IridiumSBD::publish_iridium_status()
{
	bool need_to_publish = false;

	if (_status.last_heartbeat != _last_heartbeat) {
		need_to_publish = true;
		_status.last_heartbeat = _last_heartbeat;
	}

	if (_status.tx_buf_write_index != _tx_buf_write_idx) {
		need_to_publish = true;
		_status.tx_buf_write_index = _tx_buf_write_idx;
	}

	if (_status.rx_buf_read_index != _rx_msg_read_idx) {
		need_to_publish = true;
		_status.rx_buf_read_index = _rx_msg_read_idx;
	}

	if (_status.rx_buf_end_index != _rx_msg_end_idx) {
		need_to_publish = true;
		_status.rx_buf_end_index = _rx_msg_end_idx;
	}

	if (_status.failed_sbd_sessions != _failed_sbd_sessions) {
		need_to_publish = true;
		_status.failed_sbd_sessions = _failed_sbd_sessions;
	}

	if (_status.successful_sbd_sessions != _successful_sbd_sessions) {
		need_to_publish = true;
		_status.successful_sbd_sessions = _successful_sbd_sessions;
	}

	if (_status.num_tx_buf_reset != _num_tx_buf_reset) {
		need_to_publish = true;
		_status.num_tx_buf_reset = _num_tx_buf_reset;
	}

	if (_status.signal_quality != _signal_quality) {
		need_to_publish = true;
		_status.signal_quality = _signal_quality;
	}

	if (_status.state != _state) {
		need_to_publish = true;
		_status.state = _state;
	}

	if (_status.ring_pending != _ring_pending) {
		need_to_publish = true;
		_status.ring_pending = _ring_pending;
	}

	if (_status.tx_buf_write_pending != _tx_buf_write_pending) {
		need_to_publish = true;
		_status.tx_buf_write_pending = _tx_buf_write_pending;
	}

	if (_status.tx_session_pending != _tx_session_pending) {
		need_to_publish = true;
		_status.tx_session_pending = _tx_session_pending;
	}

	if (_status.rx_read_pending != _rx_read_pending) {
		need_to_publish = true;
		_status.rx_read_pending = _rx_read_pending;
	}

	if (_status.rx_session_pending != _rx_session_pending) {
		need_to_publish = true;
		_status.rx_session_pending = _rx_session_pending;
	}

	_status.timestamp = hrt_absolute_time();

	// publish the status if it changed
	if (need_to_publish) {
		_iridiumsbd_status_pub.publish(_status);
	}
}

int IridiumSBD::open_first(struct file *filep)
{
	_cdev_used = true;
	return CDev::open_first(filep);
}

int IridiumSBD::close_last(struct file *filep)
{
	_cdev_used = false;
	return CDev::close_last(filep);
}

int iridiumsbd_main(int argc, char *argv[])
{
	if (argc < 2) {
		goto out_error;
	}

	if (!strcmp(argv[1], "start")) {
		return IridiumSBD::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		return IridiumSBD::stop();

	} else if (!strcmp(argv[1], "status")) {
		IridiumSBD::status();
		return OK;

	} else if (!strcmp(argv[1], "test")) {
		IridiumSBD::test(argc, argv);
		return OK;
	}

out_error:
	PX4_INFO("usage: iridiumsbd {start|stop|status|test} [-d uart_device]");

	return PX4_ERROR;
}
