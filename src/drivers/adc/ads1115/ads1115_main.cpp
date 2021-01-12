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
 * @file ads1115_main.cpp
 * @author SalimTerryLi
 *
 * Driver for the ADS1115 connected via I2C.
 */

#include "ADS1115.h"
#include <px4_platform_common/module.h>
#include <drivers/drv_adc.h>

ADS1115::ADS1115(I2CSPIBusOption bus_option, int bus, int addr, int bus_frequency) :
	I2C(DRV_ADC_DEVTYPE_ADS1115, nullptr, bus, addr, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, addr),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample"))
{
	_adc_report.device_id = this->get_device_id();
	_adc_report.resolution = 32768;
	_adc_report.v_ref = 6.144f;

	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
		_adc_report.channel_id[i] = -1;
	}

}

ADS1115::~ADS1115()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

int ADS1115::Begin()
{
	int ret = init();

	if (ret != PX4_OK) {
		PX4_ERR("ADS1115 init failed");
		return ret;
	}

	setChannel(ADS1115::A0);  // prepare for the first measure.

	ScheduleOnInterval(SAMPLE_INTERVAL / 4, SAMPLE_INTERVAL / 4);

	return PX4_OK;
}

void ADS1115::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();	// nothing to do
}

void ADS1115::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("stopping");
		return;	// stop and return immediately to avoid unexpected schedule from stopping procedure
	}

	perf_begin(_cycle_perf);

	_adc_report.timestamp = hrt_absolute_time();

	if (isSampleReady()) { // whether ADS1115 is ready to be read or not
		int16_t buf;
		ADS1115::ChannelSelection ch = cycleMeasure(&buf);
		++_channel_cycle_count;

		switch (ch) {
		case ADS1115::A0:
			_adc_report.channel_id[0] = 0;
			_adc_report.raw_data[0] = buf;
			break;

		case ADS1115::A1:
			_adc_report.channel_id[1] = 1;
			_adc_report.raw_data[1] = buf;
			break;

		case ADS1115::A2:
			_adc_report.channel_id[2] = 2;
			_adc_report.raw_data[2] = buf;
			break;

		case ADS1115::A3:
			_adc_report.channel_id[3] = 3;
			_adc_report.raw_data[3] = buf;
			break;

		default:
			PX4_DEBUG("ADS1115: undefined behaviour");
			setChannel(ADS1115::A0);
			--_channel_cycle_count;
			break;
		}

		if (_channel_cycle_count == 4) { // ADS1115 has 4 channels
			_channel_cycle_count = 0;
			_to_adc_report.publish(_adc_report);
		}

	} else {
		PX4_WARN("ADS1115 not ready!");
	}

	perf_end(_cycle_perf);
}

I2CSPIDriverBase *ADS1115::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				       int runtime_instance)
{
	ADS1115 *instance = new ADS1115(iterator.configuredBusOption(), iterator.bus(), cli.i2c_address,
					cli.bus_frequency);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->Begin()) {
		delete instance;
		return nullptr;
	}

	return instance;
}

void ADS1115::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ads1115", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x48);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void ADS1115::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}

extern "C" int ads1115_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = ADS1115;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = 0x48;

	while ((ch = cli.getopt(argc, argv, "")) != EOF) {

	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ADC_DEVTYPE_ADS1115);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}