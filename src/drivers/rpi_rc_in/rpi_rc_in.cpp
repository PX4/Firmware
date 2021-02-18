/****************************************************************************
 *
 *   Copyright (c) 2017 Fan.zhang. All rights reserved. 421395590@qq.com
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
#include "rpi_rc_in.h"

using namespace rpi_rc_in;

RcInput::~RcInput()
{
	if (_mem) {
		shmdt(_mem);
		_mem = nullptr;
	}

	ScheduleClear();
	_is_running = false;
}

int RcInput::rpi_rc_init()
{
	int i;

	// initialize shared memory
	if ((_shmid = shmget(_key, sizeof(int) * _channels, 0666)) < 0) {
		PX4_WARN("Faild to access shared memory");
		return -1;
	}

	if ((_mem = (int *) shmat(_shmid, NULL, 0)) == (void *) - 1) {
		PX4_WARN("Faild to map shared memory");
		return -1;
	}

	// publish data for all channels
	for (i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; ++i) {
		_data.values[i] = UINT16_MAX;
	}

	return 0;
}

int RcInput::start()
{
	int result = 0;

	result = rpi_rc_init();

	if (result != 0) {
		PX4_WARN("error: RC initialization failed");
		return -1;
	}

	_is_running = true;

	ScheduleNow();

	return result;
}

void RcInput::stop()
{
	_should_exit = true;
}

void RcInput::Run()
{
	_measure();

	if (!_should_exit) {
		ScheduleDelayed(RCINPUT_MEASURE_INTERVAL_US);
	}
}

void RcInput::_measure(void)
{
	uint64_t ts;
	// publish PWM data
	// read pwm value from shared memory
	int i = 0;

	for (i = 0; i < _channels; ++i) {
		int value = _mem[i]; // access the shared memory (with a single read)
		_data.values[i] = (value <= 0) ? UINT16_MAX : value;
	}

	ts = hrt_absolute_time();
	_data.timestamp = ts;
	_data.timestamp_last_signal = ts;
	_data.channel_count = _channels;
	_data.rssi = input_rc_s::RC_RSSI_MAX;
	_data.rc_lost_frame_count = 0;
	_data.rc_total_frame_count = 1;
	_data.rc_ppm_frame_length = 100;
	_data.rc_failsafe = false;
	_data.rc_lost = false;
	_data.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

	_rcinput_pub.publish(_data);
}

static void rpi_rc_in::usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PX4_INFO("rpi_rc_in {start|stop|status}");
}

int rpi_rc_in_main(int argc, char **argv)
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (rc_input != nullptr && rc_input->is_running()) {
			PX4_INFO("already running");
			// this is not an error
			return 0;
		}

		rc_input = new RcInput();

		// Check if alloc worked.
		if (nullptr == rc_input) {
			PX4_ERR("Rc input module initialization faild");
			return -1;
		}

		int ret = rc_input->start();

		if (ret != 0) {
			PX4_ERR("Rc input module failure");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {

		if (rc_input == nullptr || !rc_input->is_running()) {
			PX4_WARN("Not running");
			// this is not an error
			return 0;
		}

		rc_input->stop();

		// Wait for task to die
		int i = 0;

		do {
			// wait for 100ms
			usleep(100000);

		} while (rc_input->is_running() && ++i < 30);

		delete rc_input;
		rc_input = nullptr;

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (rc_input != nullptr && rc_input->is_running()) {
			PX4_INFO("running");

		} else {
			PX4_INFO("Not running");
		}

		return 0;
	}

	usage("rpi_rc_in start|stop|status");
	return 1;

}

