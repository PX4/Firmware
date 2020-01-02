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

/**
 * @file tfmini.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 * @author Christoph Tobler <christoph@px4.io>
 * @author Mohammed Kabir <mhkabir@mit.edu>
 *
 * Driver for the Benewake TFmini laser rangefinder series
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/topics/distance_sensor.h>

#include "tfmini_parser.h"

#define TFMINI_DEFAULT_PORT	"/dev/ttyS3"

using namespace time_literals;

enum class TFMINI_SETUP_STEP {
	STEP0_UNCONFIGURED = 0,
	STEP1_ENTER_CONFIRMED = 1,
	STEP2_MODE_CONFIRMED = 2,
	STEP3_UNIT_CONFIRMED = 3,
	STEP4_SAVE_CONFIRMED = 4
};

class TFMINI_SETUP
{
public:
	uint8_t _counter{0};
	TFMINI_SETUP_STEP _setup_step{TFMINI_SETUP_STEP::STEP0_UNCONFIGURED};
	uint8_t _com_enter[8] {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x02}; //Enter configuration mode
	uint8_t _com_mode[8] {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06}; //Configure "Standard format, as show in in Table 6"
	uint8_t _com_unit[8] {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x1A}; //Configure "Output unit of distance data is cm"
	uint8_t _com_save[8] {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02}; //Exit configuration mode
};

enum class TFMINIPLUS_SETUP_STEP {
	STEP0_UNCONFIGURED = 0,
	STEP1_VERSION_CONFIRMED = 1,
	STEP2_MODE_CONFIRMED = 2,
	STEP3_ENABLE_CONFIRMED = 3,
	STEP4_SAVE_CONFIRMED = 4
};

class TFMINIPLUS_SETUP
{
public:
	uint8_t _counter{0};
	TFMINIPLUS_SETUP_STEP _setup_step{TFMINIPLUS_SETUP_STEP::STEP0_UNCONFIGURED};
	uint8_t _com_version[4] {0x5A, 0x04, 0x01, 0x5F};
	uint8_t _version[3] {0, 0, 0};
	uint8_t _com_mode[5] {0x5A, 0x05, 0x05, 0x01, 0x65};
	uint8_t _com_enable[5] {0x5A, 0x05, 0x07, 0x01, 0x67};
	uint8_t _com_save[4] {0X5A, 0x04, 0x11, 0x6F};
};

class TFMINI : public px4::ScheduledWorkItem
{
public:
	TFMINI(const char *port, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~TFMINI();

	int init();

	int write_command(uint8_t *command, uint8_t framelen);
	bool get_command_result();
	uint8_t *get_command_response(uint8_t *response_size);

	void print_info();

private:

	int collect();

	void Run() override;

	void start();
	void stop();
	void autosetup_tfmini();
	void autosetup_tfmini_process();
	void autosetup_tfminiplus();
	void autosetup_tfminiplus_process();

	PX4Rangefinder	_px4_rangefinder;

	TFMINI_MODEL _hw_model {TFMINI_MODEL::MODEL_UNKNOWN};
	TFMINI_PARSE_STATE _parse_state {TFMINI_PARSE_STATE::STATE0_UNSYNC};
	uint8_t _linebuf[10] {};
	char _port[20] {};

	bool _command_result{false};
	uint8_t _command_buf[10] {};
	uint8_t _command_size{0};
	uint8_t _command_response[10] {};
	uint8_t _command_response_size{0};
	uint8_t _command_retry{0};

	TFMINI_SETUP _tfmini_setup;
	TFMINIPLUS_SETUP _tfminiplus_setup;

	static constexpr int kCONVERSIONINTERVAL{9_ms};

	int _fd{-1};

	unsigned int _linebuf_index{0};

	hrt_abstime _last_read{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

};
