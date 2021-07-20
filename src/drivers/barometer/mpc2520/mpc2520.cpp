/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file mpc2520.cpp
 * Driver for the MPC2520 barometric pressure sensor connected via I2C or SPI.
 */

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>

#include <lib/perf/perf_counter.h>

#include "mpc2520.h"

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>

enum MPC2520_BUS {
	MPC2520_BUS_ALL = 0,
	MPC2520_BUS_I2C_INTERNAL,
	MPC2520_BUS_I2C_EXTERNAL,
	MPC2520_BUS_SPI_INTERNAL,
	MPC2520_BUS_SPI_EXTERNAL
};

enum class MPC2520_MEAS_MODE {
	CONTINUOUS_PRESSURE = 0,
	CONTINUOUS_TEMPERATURE,
	CONTINUOUS_P_AND_T
};


/* internal conversion time is 9.17 ms, so sensor should not be polled at rates higher than 100 Hz */
#define MPC2520_CONVERSION_INTERVAL		25000				/* microseconds */

class MPC2520 : public device::CDev
{
public:
	MPC2520(device::Device *interface, mpc2520::prom_s &prom_buf, const char *path);
	~MPC2520();

	virtual int		init();
	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	static int print_usage(const char *reason = nullptr);

private:

	PX4Barometer		_px4_baro;

	Device			*_interface;

	mpc2520::prom_s		_prom;

	struct work_s		_work {};
	unsigned		_measure_ticks{USEC2TICK(MPC2520_CONVERSION_INTERVAL)};

	bool			_collect_phase = false;
	unsigned		_measure_phase;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	int32_t _kP = 0;
	int32_t _kT = 0;

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @param delay_ticks the number of queue ticks before executing the next cycle
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start_cycle(unsigned delay_ticks = 1);

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop_cycle();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);


	int 		set_sampling_rate(uint8_t iSensor,
					  MPC2520_SAMPLING_RATE u8SmplRate,
					  MPC2520_OVERSAMPLING_RATE u8OverSmpl);

	int 		set_measure_mode(MPC2520_MEAS_MODE mode);

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int mpc2520_main(int argc, char *argv[]);

MPC2520::MPC2520(device::Device *interface, mpc2520::prom_s &prom_buf, const char *path) :
	CDev("MPC2520", path),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_prom(prom_buf),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpc2520_read")),
	_comms_errors(perf_alloc(PC_COUNT, "mpc2520_com_err")),
	_buffer_overflows(perf_alloc(PC_COUNT, "mpc2520_buf_of"))
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_BARO_DEVTYPE_MPC2520;

	_px4_baro.set_device_type(DRV_BARO_DEVTYPE_TCBP001TA);
}

MPC2520::~MPC2520()
{
	/* make sure we are truly inactive */
	stop_cycle();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);

	if (_interface != nullptr) {
		delete _interface;
	}
}

int MPC2520::init()
{
	int ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		return ret;
	}

	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;

	/* this [do..while] is a goto without the actual goto */
	do {
		// sampling rate = 1Hz; Pressure oversample = 2;
		set_sampling_rate(PRESSURE_SENSOR,
				  MPC2520_SAMPLING_RATE::RATE_32_HZ,
				  MPC2520_OVERSAMPLING_RATE::RATE_8_HZ);

		// sampling rate = 1Hz; Temperature oversample = 1;
		set_sampling_rate(TEMPERATURE_SENSOR,
				  MPC2520_SAMPLING_RATE::RATE_32_HZ,
				  MPC2520_OVERSAMPLING_RATE::RATE_8_HZ);

		set_measure_mode(MPC2520_MEAS_MODE::CONTINUOUS_P_AND_T);

		usleep(MPC2520_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		ret = OK;

	} while (0);

	return ret;
}

int MPC2520::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int MPC2520::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

int MPC2520::set_sampling_rate(uint8_t iSensor,
			       MPC2520_SAMPLING_RATE u8SmplRate,
			       MPC2520_OVERSAMPLING_RATE u8OverSmpl)
{
	uint8_t reg = 0;  		// Register for sensor configuration
	int32_t i32kPkT = 0;  	// Scale Factor (kP or kT), depending on oversampling rate

	// Change sampling rate
	switch (u8SmplRate) {
	case MPC2520_SAMPLING_RATE::RATE_2_HZ:
		reg |= (1 << 4);
		break;

	case MPC2520_SAMPLING_RATE::RATE_4_HZ:
		reg |= (2 << 4);
		break;

	case MPC2520_SAMPLING_RATE::RATE_8_HZ:
		reg |= (3 << 4);
		break;

	case MPC2520_SAMPLING_RATE::RATE_16_HZ:
		reg |= (4 << 4);
		break;

	case MPC2520_SAMPLING_RATE::RATE_32_HZ:
		reg |= (5 << 4);
		break;

	case MPC2520_SAMPLING_RATE::RATE_64_HZ:
		reg |= (6 << 4);
		break;

	case MPC2520_SAMPLING_RATE::RATE_128_HZ:
		reg |= (7 << 4);
		break;

	case MPC2520_SAMPLING_RATE::RATE_1_HZ:
	default:
		break;
	}

	// Compensation scale factor for the different oversampling rates
	switch (u8OverSmpl) {
	case MPC2520_OVERSAMPLING_RATE::RATE_2_HZ:
		reg |= 1;
		i32kPkT = MPC2520_2_HZ_SCALE_FACTOR;
		break;

	case MPC2520_OVERSAMPLING_RATE::RATE_4_HZ:
		reg |= 2;
		i32kPkT = MPC2520_4_HZ_SCALE_FACTOR;
		break;

	case MPC2520_OVERSAMPLING_RATE::RATE_8_HZ:
		reg |= 3;
		i32kPkT = MPC2520_8_HZ_SCALE_FACTOR;
		break;

	case MPC2520_OVERSAMPLING_RATE::RATE_16_HZ:
		i32kPkT = MPC2520_16_HZ_SCALE_FACTOR;
		reg |= 4;
		break;

	case MPC2520_OVERSAMPLING_RATE::RATE_32_HZ:
		i32kPkT = MPC2520_32_HZ_SCALE_FACTOR;
		reg |= 5;
		break;

	case MPC2520_OVERSAMPLING_RATE::RATE_64_HZ:
		i32kPkT = MPC2520_64_HZ_SCALE_FACTOR;
		reg |= 6;
		break;

	case MPC2520_OVERSAMPLING_RATE::RATE_128_HZ:
		i32kPkT = MPC2520_128_HZ_SCALE_FACTOR;
		reg |= 7;
		break;

	case MPC2520_OVERSAMPLING_RATE::RATE_1_HZ:
	default:
		i32kPkT = MPC2520_1_HZ_SCALE_FACTOR;
		break;
	}

	if (iSensor == PRESSURE_SENSOR) {
		_kP = i32kPkT;

		write_reg(MPC2520_PRS_CFG, reg);

		if (u8OverSmpl > MPC2520_OVERSAMPLING_RATE::RATE_8_HZ) {
			read_reg(MPC2520_CFG_REG, reg);

			reg = reg | 0x04;
			write_reg(MPC2520_CFG_REG, reg);

		} else {
			read_reg(MPC2520_CFG_REG, reg);

			reg = reg & (~0x04);
			write_reg(MPC2520_CFG_REG, reg);
		}
	}

	if (iSensor == TEMPERATURE_SENSOR) {
		_kT = i32kPkT;

		reg = reg | 0x80;
		write_reg(MPC2520_TMP_CFG, reg);

		if (u8OverSmpl > MPC2520_OVERSAMPLING_RATE::RATE_8_HZ) {
			read_reg(MPC2520_CFG_REG, reg);

			reg = reg | 0x80;
			write_reg(MPC2520_CFG_REG, reg);

		} else {
			read_reg(MPC2520_CFG_REG, reg);

			reg = reg & (~0x80);
			write_reg(MPC2520_CFG_REG, reg);
		}
	}

	return PX4_OK;
}

int MPC2520::set_measure_mode(MPC2520_MEAS_MODE mode)
{
	uint8_t measure_mode;

	switch (mode) {
	case MPC2520_MEAS_MODE::CONTINUOUS_PRESSURE:
		measure_mode = 0x01;
		break;

	case MPC2520_MEAS_MODE::CONTINUOUS_TEMPERATURE:
		measure_mode = 0x02;
		break;

	case MPC2520_MEAS_MODE::CONTINUOUS_P_AND_T:
		measure_mode = 0x07;
		break;

	default:
		measure_mode = 0x00;
		break;
	}

	_interface->write(MPC2520_MEAS_CFG, (void *)&measure_mode, 1);

	return PX4_OK;
}

void MPC2520::start_cycle(unsigned delay_ticks)
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MPC2520::cycle_trampoline, this, delay_ticks);
}

void MPC2520::stop_cycle()
{
	work_cancel(HPWORK, &_work);
}

void MPC2520::cycle_trampoline(void *arg)
{
	MPC2520 *dev = reinterpret_cast<MPC2520 *>(arg);

	dev->cycle();
}

void MPC2520::cycle()
{
	int ret;
	unsigned dummy;

	/* perform collection */
	ret = collect();

	if (ret != OK) {
		/* issue a reset command to the sensor */
		_interface->ioctl(IOCTL_RESET, dummy);

		/* reset the collection state machine and try again - we need
		 * to wait 2.8 ms after issuing the sensor reset command
		 */
		start_cycle(USEC2TICK(2800));
		return;
	}

	/* next phase is measurement */
	_collect_phase = false;

	/*
	 * Is there a collect->measure gap?
	 * Don't inject one after temperature measurements, so we can keep
	 * doing pressure measurements at something close to the desired rate.
	 */
	if ((_measure_phase != 0) &&
	    (_measure_ticks > USEC2TICK(MPC2520_CONVERSION_INTERVAL))) {

		/* schedule a fresh cycle call when we are ready to measure again */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&MPC2520::cycle_trampoline,
			   this,
			   _measure_ticks - USEC2TICK(MPC2520_CONVERSION_INTERVAL));

		return;
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MPC2520::cycle_trampoline,
		   this,
		   USEC2TICK(MPC2520_CONVERSION_INTERVAL));
}

int MPC2520::collect()
{
	uint8_t buf[3] {};
	int32_t raw;

	double fTsc, fPsc;
	double qua2, qua3;
	double fTCompensate, fPCompensate;

	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	hrt_abstime timestamp_sample = hrt_absolute_time();
	//report.error_count = perf_event_count(_comms_errors);

	/* read temperature */
	int ret = _interface->read(MPC2520_TMP_B2, (void *)&buf[0], 3);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	raw = (int32_t)(buf[0] << 16) | (int32_t)(buf[1] << 8) | (int32_t)buf[2];
	raw = (raw & 0x800000) ? (0xFF000000 | raw) : raw;
	fTsc = raw / (double)_kT;
	fTCompensate =  _prom.c0 * 0.5 + _prom.c1 * fTsc;

	/* read pressure  */
	ret = _interface->read(MPC2520_PSR_B2, (void *)&buf[0], 3);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	raw = (int32_t)(buf[0] << 16) | (int32_t)(buf[1] << 8) | (int32_t)buf[2];
	raw = (raw & 0x800000) ? (0xFF000000 | raw) : raw;
	fPsc = raw / (double)_kP;
	qua2 = _prom.c10 + fPsc * (_prom.c20 + fPsc * _prom.c30);
	qua3 = fTsc * fPsc * (_prom.c11 + fPsc * _prom.c21);
	fPCompensate = _prom.c00 + fPsc * qua2 + fTsc * _prom.c01 + qua3;

	/* generate a new report */
	_px4_baro.set_temperature(fTCompensate);
	float pressure = (int32_t)fPCompensate / 100.0f;		/* convert to millibar */

	/* publish it */
	_px4_baro.update(timestamp_sample, pressure);

	perf_end(_sample_perf);

	return OK;
}

int MPC2520::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for mpc2520 barometer sensor

### Implementation
TODO

### Example
TODO

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mpc2520", "driver");

	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task. Requries exactly one of the following flags:");
	PRINT_MODULE_USAGE_PARAM_FLAG('X', "external I2C bus", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('I', "internal I2C bus", true);
	// PRINT_MODULE_USAGE_PARAM_FLAG('S', "external SPI bus", true);  // Not implemented
	// PRINT_MODULE_USAGE_PARAM_FLAG('s', "internal SPI bus", true);  // Not implemented

	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Print sensor values");


	return PX4_OK;
}

/**
 * Local functions in support of the shell command.
 */
namespace mpc2520
{
/*
  list of supported bus configurations
 */
struct mpc2520_bus_option {
	enum MPC2520_BUS busid;
	const char *devpath;
	MPC2520_constructor interface_constructor;
	uint8_t busnum;
	MPC2520 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ MPC2520_BUS_SPI_EXTERNAL, "/dev/mpc2520_spi_ext", &MPC2520_spi_interface, PX4_SPI_BUS_EXT, NULL },
#endif
#ifdef PX4_SPIDEV_MPC_2520
	{ MPC2520_BUS_SPI_INTERNAL, "/dev/mpc2520_spi_int", &MPC2520_spi_interface, PX4_SPIDEV_MPC_2520, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ MPC2520_BUS_I2C_INTERNAL, "/dev/mpc2520_int", &MPC2520_i2c_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION
	{ MPC2520_BUS_I2C_EXTERNAL, "/dev/mpc2520_ext", &MPC2520_i2c_interface, PX4_I2C_BUS_EXPANSION, NULL },
#endif
};

#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct mpc2520_bus_option &bus);
struct  mpc2520_bus_option &find_bus(enum MPC2520_BUS busid);
void	start(enum MPC2520_BUS busid);
void	test(enum MPC2520_BUS busid);


/**
 * Start the driver.
 */
bool start_bus(struct mpc2520_bus_option &bus)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	prom_s prom_buf;
	device::Device *interface = bus.interface_constructor(prom_buf, bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new MPC2520(interface, prom_buf, bus.devpath);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void start(enum MPC2520_BUS busid)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MPC2520_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MPC2520_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started = started | start_bus(bus_options[i]);
	}

	if (!started) {
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}

/**
 * find a bus structure for a busid
 */
struct mpc2520_bus_option &find_bus(enum MPC2520_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MPC2520_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}
}

int mpc2520_main(int argc, char *argv[])
{
	enum MPC2520_BUS busid = MPC2520_BUS_ALL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "XISs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MPC2520_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = MPC2520_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = MPC2520_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = MPC2520_BUS_SPI_INTERNAL;
			break;

		//no break
		default:
			MPC2520::print_usage();
			exit(0);
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		mpc2520::start(busid);
	}

	MPC2520::print_usage("unrecognised command");

	return PX4_OK;
}
