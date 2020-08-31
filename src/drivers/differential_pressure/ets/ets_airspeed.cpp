/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file ets_airspeed.cpp
 * @author Simon Wilks
 *
 * Driver for the Eagle Tree Airspeed V3 connected via I2C.
 */

#include <float.h>

#include <drivers/airspeed/airspeed.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>

/* I2C bus address */
#define I2C_ADDRESS	0x75	/* 7-bit address. 8-bit address is 0xEA */

/* Register address */
#define READ_CMD	0x07	/* Read the data */

/**
 * The Eagle Tree Airspeed V3 cannot provide accurate reading below speeds of 15km/h.
 * You can set this value to 12 if you want a zero reading below 15km/h.
 */
#define MIN_ACCURATE_DIFF_PRES_PA 0

/* Measurement rate is 100Hz */
#define CONVERSION_INTERVAL	(1000000 / 100)	/* microseconds */

class ETSAirspeed : public Airspeed, public I2CSPIDriver<ETSAirspeed>
{
public:
	ETSAirspeed(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = I2C_ADDRESS);

	virtual ~ETSAirspeed() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void	RunImpl();
protected:
	int	measure() override;
	int	collect() override;
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ets_airspeed_main(int argc, char *argv[]);

ETSAirspeed::ETSAirspeed(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address)
	: Airspeed(bus, bus_frequency, address, CONVERSION_INTERVAL),
	  I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
{
	_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_MS4525;
}

int
ETSAirspeed::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = READ_CMD;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
ETSAirspeed::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};

	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		perf_count(_comms_errors);
		return ret;
	}

	float diff_pres_pa_raw = (float)(val[1] << 8 | val[0]);

	differential_pressure_s report{};
	report.timestamp = hrt_absolute_time();

	if (diff_pres_pa_raw < FLT_EPSILON) {
		// a zero value indicates no measurement
		// since the noise floor has been arbitrarily killed
		// it defeats our stuck sensor detection - the best we
		// can do is to output some numerical noise to show
		// that we are still correctly sampling.
		diff_pres_pa_raw = 0.001f * (report.timestamp & 0x01);
	}

	// The raw value still should be compensated for the known offset
	diff_pres_pa_raw -= _diff_pres_offset;

	report.error_count = perf_event_count(_comms_errors);

	// XXX we may want to smooth out the readings to remove noise.
	report.differential_pressure_filtered_pa = diff_pres_pa_raw;
	report.differential_pressure_raw_pa = diff_pres_pa_raw;
	report.temperature = -1000.0f;
	report.device_id = _device_id.devid;

	_airspeed_pub.publish(report);

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

void
ETSAirspeed::RunImpl()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (OK != ret) {
			perf_count(_comms_errors);
			/* restart the measurement state machine */
			_collect_phase = false;
			_sensor_ok = false;
			ScheduleNow();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_interval > CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	_sensor_ok = (ret == OK);

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(CONVERSION_INTERVAL);
}

I2CSPIDriverBase *ETSAirspeed::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	ETSAirspeed *instance = new ETSAirspeed(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->ScheduleNow();
	return instance;
}


void
ETSAirspeed::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ets_airspeed", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("airspeed_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int
ets_airspeed_main(int argc, char *argv[])
{
	using ThisDriver = ETSAirspeed;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIFF_PRESS_DEVTYPE_ETS3);

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
