/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file sf1xx.cpp
 *
 * @author ecmnet <ecm@gmx.de>
 * @author Vasily Evseenko <svpcom@gmail.com>
 *
 * Driver for the Lightware SF1xx lidar range finder series.
 * Default I2C address 0x66 is used.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <lib/parameters/param.h>
#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <board_config.h>

/* Configuration Constants */
#define SF1XX_BASEADDR		0x66
#define SF1XX_DEVICE_PATH	"/dev/sf1xx"


class SF1XX : public device::I2C, public I2CSPIDriver<SF1XX>
{
public:
	SF1XX(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency,
	      int address = SF1XX_BASEADDR);

	virtual ~SF1XX() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init() override;

	ssize_t read(device::file_t *filp, char *buffer, size_t buflen) override;
	int ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_status() override;

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void RunImpl();

protected:
	int probe() override;

private:
	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address The I2C bus address to probe.
	* @return True if the device is present.
	*/
	int probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults SF1XX_MIN_DISTANCE
	* and SF1XX_MAX_DISTANCE
	*/
	void set_minimum_distance(float min);
	void set_maximum_distance(float max);
	float get_minimum_distance();
	float get_maximum_distance();

	int measure();
	int collect();

	bool _sensor_ok{false};

	int _class_instance{-1};
	int _conversion_interval{-1};
	int _measure_interval{0};
	int _orb_class_instance{-1};

	float _max_distance{-1.0f};
	float _min_distance{-1.0f};

	uint8_t _rotation{0};

	ringbuffer::RingBuffer  *_reports{nullptr};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "sf1xx_read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "sf1xx_com_err")};
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int sf1xx_main(int argc, char *argv[]);

SF1XX::SF1XX(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency, int address) :
	I2C("SF1XX", SF1XX_DEVICE_PATH, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_rotation(rotation)
{
}

SF1XX::~SF1XX()
{
	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
SF1XX::init()
{
	int ret = PX4_ERROR;
	int hw_model;
	param_get(param_find("SENS_EN_SF1XX"), &hw_model);

	switch (hw_model) {
	case 0:
		PX4_WARN("disabled.");
		return ret;

	case 1:  /* SF10/a (25m 32Hz) */
		_min_distance = 0.01f;
		_max_distance = 25.0f;
		_conversion_interval = 31250;
		break;

	case 2:  /* SF10/b (50m 32Hz) */
		_min_distance = 0.01f;
		_max_distance = 50.0f;
		_conversion_interval = 31250;
		break;

	case 3:  /* SF10/c (100m 16Hz) */
		_min_distance = 0.01f;
		_max_distance = 100.0f;
		_conversion_interval = 62500;
		break;

	case 4:
		/* SF11/c (120m 20Hz) */
		_min_distance = 0.01f;
		_max_distance = 120.0f;
		_conversion_interval = 50000;
		break;

	case 5:
		/* SF/LW20/b (50m 48-388Hz) */
		_min_distance = 0.001f;
		_max_distance = 50.0f;
		_conversion_interval = 20834;
		break;

	case 6:
		/* SF/LW20/c (100m 48-388Hz) */
		_min_distance = 0.001f;
		_max_distance = 100.0f;
		_conversion_interval = 20834;
		break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return ret;
	}

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	set_device_address(SF1XX_BASEADDR);

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_HIGH);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	// Select altitude register
	int ret2 = measure();

	if (ret2 == 0) {
		ret = OK;
		_sensor_ok = true;
		PX4_INFO("(%dm %dHz) with address %d found", (int)_max_distance,
			 (int)(1e6f / _conversion_interval), SF1XX_BASEADDR);
	}

	return ret;
}

int
SF1XX::probe()
{
	return measure();
}

void
SF1XX::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
SF1XX::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
SF1XX::get_minimum_distance()
{
	return _min_distance;
}

float
SF1XX::get_maximum_distance()
{
	return _max_distance;
}

int
SF1XX::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_interval = (_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* convert hz to tick interval via microseconds */
					int interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < _conversion_interval) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_interval = interval;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
SF1XX::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		px4_usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
SF1XX::measure()
{
	int ret;

	/*
	 * Send the command '0' -- read altitude
	 */

	uint8_t cmd = 0;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
SF1XX::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};
	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = float(distance_cm) * 1e-2f;

	struct distance_sensor_s report;
	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = _rotation;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.variance = 0.0f;
	report.signal_quality = -1;
	/* TODO: set proper ID */
	report.id = 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
SF1XX::start()
{
	if (_measure_interval == 0) {
		_measure_interval = _conversion_interval;
	}

	/* reset the report ring and state machine */
	_reports->flush();

	/* set register to '0' */
	measure();

	/* schedule a cycle to start things */
	ScheduleDelayed(_conversion_interval);
}

void
SF1XX::RunImpl()
{
	/* Collect results */
	if (OK != collect()) {
		PX4_DEBUG("collection error");
		/* if error restart the measurement state machine */
		start();
		return;
	}

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_conversion_interval);
}

void
SF1XX::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u\n", _measure_interval);
	_reports->print_info("report queue");
}

void
SF1XX::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for Lightware SFxx series LIDAR rangefinders: SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20.

Setup/usage information: https://docs.px4.io/en/sensor/sfxx_lidar.html
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sf1xx", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *SF1XX::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	SF1XX* instance = new SF1XX(iterator.configuredBusOption(), iterator.bus(), cli.orientation, cli.bus_frequency);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->start();
	return instance;
}


int
sf1xx_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = SF1XX;
	BusCLIArguments cli{true, false};
	cli.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.default_i2c_frequency = 400000;

	while ((ch = cli.getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.orientation = atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_SF1XX);

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
