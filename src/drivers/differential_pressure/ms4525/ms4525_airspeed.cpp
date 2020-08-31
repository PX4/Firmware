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
 * @file meas_airspeed.cpp
 * @author Lorenz Meier
 * @author Sarthak Kaingade
 * @author Simon Wilks
 * @author Thomas Gubler
 *
 * Driver for the MEAS Spec series connected via I2C.
 *
 * Supported sensors:
 *
 *    - MS4525DO (http://www.meas-spec.com/downloads/MS4525DO.pdf)
 *
 * Interface application notes:
 *
 *    - Interfacing to MEAS Digital Pressure Modules (http://www.meas-spec.com/downloads/Interfacing_to_MEAS_Digital_Pressure_Modules.pdf)
 */

#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/system_power.h>

#include <drivers/airspeed/airspeed.h>

enum MS_DEVICE_TYPE {
	DEVICE_TYPE_MS4515	= 4515,
	DEVICE_TYPE_MS4525	= 4525
};

/* I2C bus address is 1010001x */
#define I2C_ADDRESS_MS4515DO	0x46
#define I2C_ADDRESS_MS4525DO	0x28	/**< 7-bit address. Depends on the order code (this is for code "I") */

/* Register address */
#define ADDR_READ_MR			0x00	/* write to this address to start conversion */

/* Measurement rate is 100Hz */
#define MEAS_RATE 100
#define MEAS_DRIVER_FILTER_FREQ 1.2f
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */


class MEASAirspeed : public Airspeed, public I2CSPIDriver<MEASAirspeed>
{
public:
	MEASAirspeed(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = I2C_ADDRESS_MS4525DO);

	virtual ~MEASAirspeed() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void	RunImpl();

protected:

	int	measure() override;
	int	collect() override;

	math::LowPassFilter2p	_filter{MEAS_RATE, MEAS_DRIVER_FILTER_FREQ};

	/**
	 * Correct for 5V rail voltage variations
	 */
	void voltage_correction(float &diff_pres_pa, float &temperature);

	int _t_system_power{-1};
	system_power_s system_power{};
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ms4525_airspeed_main(int argc, char *argv[]);

MEASAirspeed::MEASAirspeed(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address)
	: Airspeed(bus, bus_frequency, address, CONVERSION_INTERVAL),
	  I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
{
	_device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_MS4525;
}

int
MEASAirspeed::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
MEASAirspeed::collect()
{
	/* read from the sensor */
	uint8_t val[4] = {0, 0, 0, 0};

	perf_begin(_sample_perf);

	int ret = transfer(nullptr, 0, &val[0], 4);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint8_t status = (val[0] & 0xC0) >> 6;

	switch (status) {
	case 0:
		// Normal Operation. Good Data Packet
		break;

	case 1:
		// Reserved
		return -EAGAIN;

	case 2:
		// Stale Data. Data has been fetched since last measurement cycle.
		return -EAGAIN;

	case 3:
		// Fault Detected
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	int16_t dp_raw = 0, dT_raw = 0;
	dp_raw = (val[0] << 8) + val[1];
	/* mask the used bits */
	dp_raw = 0x3FFF & dp_raw;
	dT_raw = (val[2] << 8) + val[3];
	dT_raw = (0xFFE0 & dT_raw) >> 5;

	// dT max is almost certainly an invalid reading
	if (dT_raw == 2047) {
		perf_count(_comms_errors);
		return -EAGAIN;
	}

	float temperature = ((200.0f * dT_raw) / 2047) - 50;

	// Calculate differential pressure. As its centered around 8000
	// and can go positive or negative
	const float P_min = -1.0f;
	const float P_max = 1.0f;
	const float PSI_to_Pa = 6894.757f;
	/*
	  this equation is an inversion of the equation in the
	  pressure transfer function figure on page 4 of the datasheet

	  We negate the result so that positive differential pressures
	  are generated when the bottom port is used as the static
	  port on the pitot and top port is used as the dynamic port
	 */
	float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
	float diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;

	// correct for 5V rail voltage if possible
	voltage_correction(diff_press_pa_raw, temperature);

	/*
	  With the above calculation the MS4525 sensor will produce a
	  positive number when the top port is used as a dynamic port
	  and bottom port is used as the static port
	 */

	differential_pressure_s report{};

	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	report.temperature = temperature;
	report.differential_pressure_filtered_pa =  _filter.apply(diff_press_pa_raw) - _diff_pres_offset;
	report.differential_pressure_raw_pa = diff_press_pa_raw - _diff_pres_offset;
	report.device_id = _device_id.devid;

	_airspeed_pub.publish(report);

	ret = OK;

	perf_end(_sample_perf);

	return ret;
}

void
MEASAirspeed::RunImpl()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (OK != ret) {
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

/**
   correct for 5V rail voltage if the system_power ORB topic is
   available

   See http://uav.tridgell.net/MS4525/MS4525-offset.png for a graph of
   offset versus voltage for 3 sensors
 */
void
MEASAirspeed::voltage_correction(float &diff_press_pa, float &temperature)
{
#if defined(ADC_SCALED_V5_SENSE)

	if (_t_system_power == -1) {
		_t_system_power = orb_subscribe(ORB_ID(system_power));
	}

	if (_t_system_power == -1) {
		// not available
		return;
	}

	bool updated = false;
	orb_check(_t_system_power, &updated);

	if (updated) {
		orb_copy(ORB_ID(system_power), _t_system_power, &system_power);
	}

	if (system_power.voltage5v_v < 3.0f || system_power.voltage5v_v > 6.0f) {
		// not valid, skip correction
		return;
	}

	const float slope = 65.0f;
	/*
	  apply a piecewise linear correction, flattening at 0.5V from 5V
	 */
	float voltage_diff = system_power.voltage5v_v - 5.0f;

	if (voltage_diff > 0.5f) {
		voltage_diff = 0.5f;
	}

	if (voltage_diff < -0.5f) {
		voltage_diff = -0.5f;
	}

	diff_press_pa -= voltage_diff * slope;

	/*
	  the temperature masurement varies as well
	 */
	const float temp_slope = 0.887f;
	voltage_diff = system_power.voltage5v_v - 5.0f;

	if (voltage_diff > 0.5f) {
		voltage_diff = 0.5f;
	}

	if (voltage_diff < -1.0f) {
		voltage_diff = -1.0f;
	}

	temperature -= voltage_diff * temp_slope;
#endif // defined(ADC_SCALED_V5_SENSE)
}

I2CSPIDriverBase *MEASAirspeed::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	MEASAirspeed *instance = new MEASAirspeed(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency,
			cli.i2c_address);

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
MEASAirspeed::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ms4525_airspeed", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("airspeed_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_STRING('T', "4525", "4525|4515", "Device type", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int
ms4525_airspeed_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = MEASAirspeed;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	int device_type = DEVICE_TYPE_MS4525;

	while ((ch = cli.getopt(argc, argv, "T:")) != EOF) {
		switch (ch) {
		case 'T':
			device_type = atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	if (device_type == DEVICE_TYPE_MS4525) {
		cli.i2c_address = I2C_ADDRESS_MS4525DO;

	} else {
		cli.i2c_address = I2C_ADDRESS_MS4515DO;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli,
				     DRV_DIFF_PRESS_DEVTYPE_MS4525);

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
