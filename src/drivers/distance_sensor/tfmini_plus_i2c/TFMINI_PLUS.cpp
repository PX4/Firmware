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
 * @file tfmini_plus_i2c.cpp
 *
 * @author Eren Ipek <eren.ipek@maxwell-innovations.com>
 *
 * I2C Driver for the Benewake TFmini Plus laser rangefinder series
 * Default I2C address 0x10 is used.
 */

#include "TFMINI_PLUS.hpp"

#include <fcntl.h>

tfmini_plus::tfmini_plus(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency,
			 int address) :
	I2C(DRV_DIST_DEVTYPE_TFMINI_PLUS, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_rangefinder(get_device_id(), rotation)
{
	_px4_rangefinder.set_max_distance(TFMINI_PLUS_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(TFMINI_PLUS_MIN_DISTANCE);
	_px4_rangefinder.set_fov(math::radians(TFMINI_PLUS_FOV));

}

tfmini_plus::~tfmini_plus()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

void tfmini_plus::start()
{
	// Reset the report ring and state machine.
	_collect_phase = false;

	// Schedule a cycle to start things.
	ScheduleDelayed(TFMINI_START_INTERVAL);
}

int tfmini_plus::init()
{
	// I2C init (and probe) first.
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	px4_usleep(TFMINI_I2C_INIT_INTERVAL);

	if (sensorInit() != PX4_OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int tfmini_plus::collect()
{
	float distance_m = -1.0f;
	uint16_t signal_value = 0;
	float temperature = -273.0f;
	int ret = 0;

	ret = transfer(_tfminiplus_setup._tfminiplus_command._com_obtain_data,
		       sizeof(_tfminiplus_setup._tfminiplus_command._com_obtain_data), nullptr, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		return ret;
	}

	perf_begin(_sample_perf);
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	//read from the sensor
	ret = transfer(nullptr, 0, _command_response, static_cast<int>(TFMINI_COMMAND_RESPONSE_SIZE::OBTAIN_DATA_FRAME));

	if (ret < 0) {
		perf_count(_comms_errors);
		return ret;
	}

	//parse
	for (int i = 0; i < static_cast<int>(TFMINI_COMMAND_RESPONSE_SIZE::OBTAIN_DATA_FRAME); i++) {
		tfmini_parse(_command_response[i], &_parse_state, &distance_m, &signal_value, &temperature);

		if (_parse_state == TFMINI_PLUS_PARSE_STATE::STATE0_UNSYNC) {
			perf_count(_comms_errors);
		}
	}

	if (distance_m < 0.0f) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	int8_t signal_quality = -1;

	if (signal_value < TFMINI_PLUS_ACCEPTABLE_MIN_SIGNAL_VALUE || signal_value == TFMINI_PLUS_INVALID_MEASURE) {
		signal_quality = 0;
	}

	signal_value = math::min(signal_value, static_cast<uint16_t>(TFMINI_PLUS_MAX_SIGNAL_VALUE));
	signal_quality = static_cast<int8_t>(static_cast<uint32_t>(signal_value) / 200);  //in percent, ie: x*100/20000 or x/200

	// publish most recent valid measurement from buffer
	_px4_rangefinder.update(timestamp_sample, distance_m, signal_quality);

	perf_end(_sample_perf);

	return PX4_OK;
}

void tfmini_plus::RunImpl()
{
	if (_collect_phase) {
		// Perform collection.
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			// If error restart the measurement state machine.
			start();
			return;
		}

		_collect_phase = false;
	}

	_collect_phase = true;

	// Schedule delay for 100 Hz.
	ScheduleDelayed(_interval);
}

void tfmini_plus::print_status()
{
	I2CSPIDriverBase::print_status();

	if (_tfminiplus_setup._setup_step == TFMINIPLUS_SETUP_STEP::STEP4_SAVE_CONFIRMED) {
		PX4_INFO("TFMINI-Plus is configured - version = %u.%u.%u\n", _tfminiplus_setup._version[0],
			 _tfminiplus_setup._version[1],
			 _tfminiplus_setup._version[2]);

	} else {
		PX4_INFO("TFMINI-Plus is being configured(state = %u) - version = %u.%u.%u\n",
			 (uint8_t)_tfminiplus_setup._setup_step,
			 _tfminiplus_setup._version[0], _tfminiplus_setup._version[1],
			 _tfminiplus_setup._version[2]);
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);
	}
}

int tfmini_plus::autosetup_tfminiplus(void)
{
	int ret = 0;

	if (_tfminiplus_setup._setup_step == TFMINIPLUS_SETUP_STEP::STEP4_SAVE_CONFIRMED) {
		return ret;
	}

	// informations about commands and responses to/from TFMini-plus has been found in TFmini-Plus A04-Product Mannual_EN.pdf
	switch (_tfminiplus_setup._setup_step) {
	case TFMINIPLUS_SETUP_STEP::STEP0_UNCONFIGURED:
		ret = transfer(_tfminiplus_setup._tfminiplus_command._com_version,
			       sizeof(_tfminiplus_setup._tfminiplus_command._com_version), nullptr, 0);

		if (ret < 0) {
			perf_count(_comms_errors);
			return ret;
		}

		px4_usleep(TFMINI_PLUS_COMMAND_WAITING_PERIOD);

		ret = transfer(nullptr, 0, _command_response, static_cast<int>(TFMINI_COMMAND_RESPONSE_SIZE::OBTAIN_FW_VERSION));

		if (ret < 0) {
			perf_count(_comms_errors);
			return ret;
		}

		// we have written _tfminiplus_setup._tfminiplus_command._com_version and we expect reading 5A 07 01 V1 V2 V3
		if (_command_response[1] == 0x07 && _command_response[2] == 0x01) {
			_tfminiplus_setup._version[0] = _command_response[3];
			_tfminiplus_setup._version[1] = _command_response[4];
			_tfminiplus_setup._version[2] = _command_response[5];
			_tfminiplus_setup._setup_step = TFMINIPLUS_SETUP_STEP::STEP1_VERSION_CONFIRMED;
		}

		break;

	case TFMINIPLUS_SETUP_STEP::STEP1_VERSION_CONFIRMED:
		ret = transfer(_tfminiplus_setup._tfminiplus_command._com_mode, sizeof(_tfminiplus_setup._tfminiplus_command._com_mode),
			       nullptr, 0);

		if (ret < 0) {
			perf_count(_comms_errors);
			return ret;
		}

		px4_usleep(TFMINI_PLUS_COMMAND_WAITING_PERIOD);

		ret = transfer(nullptr, 0, _command_response, static_cast<int>(TFMINI_COMMAND_RESPONSE_SIZE::IO_MODE));

		if (ret < 0) {
			perf_count(_comms_errors);
			return ret;
		}

		// we have written _tfminiplus_setup._tfminiplus_command._com_mode and we expect reading 5A 05 05 01 65
		if (_command_response[1] == 0x05 && _command_response[2] == 0x05 && _command_response[3] == 0x01) {
			_tfminiplus_setup._setup_step = TFMINIPLUS_SETUP_STEP::STEP2_MODE_CONFIRMED;
		}

		break;

	case TFMINIPLUS_SETUP_STEP::STEP2_MODE_CONFIRMED:
		ret = transfer(_tfminiplus_setup._tfminiplus_command._com_enable,
			       sizeof(_tfminiplus_setup._tfminiplus_command._com_enable), nullptr, 0);

		if (ret < 0) {
			perf_count(_comms_errors);
			return ret;
		}

		px4_usleep(TFMINI_PLUS_COMMAND_WAITING_PERIOD);

		ret = transfer(nullptr, 0, _command_response, static_cast<int>(TFMINI_COMMAND_RESPONSE_SIZE::ENABLE_DISABLE_OUTPUT));

		if (ret < 0) {
			perf_count(_comms_errors);
			return ret;
		}

		// we have written _tfminiplus_setup._tfminiplus_command._com_enable and we expect reading 5A 05 07 01 67
		if (_command_response[1] == 0x05 && _command_response[2] == 0x07 && _command_response[3] == 0x01) {
			_tfminiplus_setup._setup_step = TFMINIPLUS_SETUP_STEP::STEP3_ENABLE_CONFIRMED;
		}

		break;

	case TFMINIPLUS_SETUP_STEP::STEP3_ENABLE_CONFIRMED:
		ret = transfer(_tfminiplus_setup._tfminiplus_command._com_save, sizeof(_tfminiplus_setup._tfminiplus_command._com_save),
			       nullptr, 0);

		if (ret < 0) {
			perf_count(_comms_errors);
			return ret;
		}

		px4_usleep(TFMINI_PLUS_COMMAND_WAITING_PERIOD);

		ret = transfer(nullptr, 0, _command_response, static_cast<int>(TFMINI_COMMAND_RESPONSE_SIZE::SAVE_SETTINGS));

		if (ret < 0) {
			perf_count(_comms_errors);
			return ret;
		}

		// we have written _tfminiplus_setup._tfminiplus_command._com_save and we expect reading 5A 05 11 00 6F
		if (_command_response[1] == 0x05 && _command_response[2] == 0x11 && _command_response[3] == 0x00) {
			_tfminiplus_setup._setup_step = TFMINIPLUS_SETUP_STEP::STEP4_SAVE_CONFIRMED;
		}

		break;

	default:
		break;
	}

	if (ret < 0) {
		perf_count(_comms_errors);
		return ret;
	}

	return ret;
}

int tfmini_plus::sensorInit()
{
	int ret = 0;

	for (int i = 0 ; i < static_cast<int>(TFMINIPLUS_SETUP_STEP::STEP_MEMBER_COUNT); ++i) {

		ret = autosetup_tfminiplus();

		if (ret < 0) {
			PX4_ERR("sensor setup err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}
	}

	return PX4_OK;
}
