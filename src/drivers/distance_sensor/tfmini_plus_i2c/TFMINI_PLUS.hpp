/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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
 * @file tfmini_plus.hpp
 * @author Eren Ipek <eren.ipek@maxwell-innovations.com>
 *
 * I2C Driver for the Benewake TFmini Plus laser rangefinder series
 * Default I2C address 0x10 is used.
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

#include "tfmini_plus_parser.h"

#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>

/* Configuration Constants */
#define TFMINI_PLUS_BASEADDR 				0x10 	// see product mannual 5.5 i2c data communucation
#define TFMINI_PLUS_BUS_CLOCK                           400000  // 400kHz bus clock speed

/* Device limits */
#define TFMINI_PLUS_MIN_DISTANCE 			(0.10f)
#define TFMINI_PLUS_MAX_DISTANCE 			(12.f)
#define	TFMINI_PLUS_FOV					(3.6f)
#define TFMINI_PLUS_INVALID_MEASURE 			0xFFFF

//according to the spec, signal is within [0-0xFFFF]
//according to test, signal is within [0-20000]
#define TFMINI_PLUS_MAX_SIGNAL_VALUE 			20000
#define TFMINI_PLUS_ACCEPTABLE_MIN_SIGNAL_VALUE 	100	// see product mannual 5.4 Descriptions of default Output Data

#define TFMINI_PLUS_COMMAND_WAITING_PERIOD		100000	// see product mannual 4.6 Timing sequence description of I^2^C mode
#define TFMINI_PLUS_OBTAIN_DATA_PERIOD			10000	// 10000 us = 100 Hz

#define TFMINI_I2C_INIT_INTERVAL			200000
#define TFMINI_START_INTERVAL				5

enum class TFMINIPLUS_SETUP_STEP {
	STEP0_UNCONFIGURED = 0,
	STEP1_VERSION_CONFIRMED = 1,
	STEP2_MODE_CONFIRMED = 2,
	STEP3_ENABLE_CONFIRMED = 3,
	STEP4_SAVE_CONFIRMED = 4,
	STEP_MEMBER_COUNT
};

enum class TFMINI_COMMAND_RESPONSE_SIZE {
	SYSTEM_RESET = 5,
	OUTPUT_FORMAT = 5,
	ENABLE_DISABLE_OUTPUT = 5,
	SET_SLAVE_ADDR = 5,
	IO_MODE = 5,
	RESTORE_FACTORY_SETTIGS = 5,
	SAVE_SETTINGS = 5,
	OBTAIN_FW_VERSION = 7,
	BAUDRATE = 8,
	OBTAIN_DATA_FRAME = 9,
	TRIGGER_DETECTION = 9,
};

class TFMINIPLUS_COMMAND
{
public:
	uint8_t _com_version[4] {0x5A, 0x04, 0x01, 0x5F}; //Obtain firmware version.
	uint8_t _com_mode[5] {0x5A, 0x05, 0x05, 0x01, 0x65}; // standart 9byte (cm)
	uint8_t _com_enable[5] {0x5A, 0x05, 0x07, 0x01, 0x67}; // enable data output
	uint8_t _com_save[4] {0X5A, 0x04, 0x11, 0x6F}; // Save settings
	uint8_t _com_obtain_data[5] {0x5A, 0x05, 0x00, 0x01, 0x60};
};

class TFMINIPLUS_SETUP
{
public:
	TFMINIPLUS_COMMAND _tfminiplus_command;
	TFMINIPLUS_SETUP_STEP _setup_step{TFMINIPLUS_SETUP_STEP::STEP0_UNCONFIGURED};
	uint8_t _version[3] {0, 0, 0};

};

class tfmini_plus : public device::I2C, public I2CSPIDriver<tfmini_plus>
{
public:
	tfmini_plus(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency,
		    int address = TFMINI_PLUS_BASEADDR);
	~tfmini_plus() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init() override;
	void print_status() override;

	void RunImpl();

private:

	void start();
	int collect();

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 * @param address The I2C bus address to probe.
	 * @return True if the device is present.
	 */
	int probe_address(uint8_t address);

	int autosetup_tfminiplus(void);
	int sensorInit();

	PX4Rangefinder _px4_rangefinder;

	TFMINI_PLUS_PARSE_STATE _parse_state {TFMINI_PLUS_PARSE_STATE::STATE0_UNSYNC};
	int _interval{TFMINI_PLUS_OBTAIN_DATA_PERIOD};
	uint8_t _command_response[10] {};

	TFMINIPLUS_SETUP _tfminiplus_setup;

	bool _collect_phase{false};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED,  MODULE_NAME": read")};
};
