/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file ist8310.cpp
 *
 * Driver for the IST8310 magnetometer connected via I2C.
 *
 * @author David Sidrane
 * @author Maelok Dong
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_device.h>

/*
 * IST8310 internal constants and data structures.
 */

/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150
 * The datasheet gives 200Hz maximum measurement rate, but it's not true according to tech support from iSentek*/
#define IST8310_CONVERSION_INTERVAL	(1000000 / 100) /* microseconds */

#define IST8310_BUS_I2C_ADDR		0xE
#define IST8310_DEFAULT_BUS_SPEED	400000

/*
 * FSR:
 *   x, y: +- 1600 µT
 *   z:    +- 2500 µT
 *
 * Resolution according to datasheet is 0.3µT/LSB
 */
#define IST8310_RESOLUTION	0.3

static const int16_t IST8310_MAX_VAL_XY	= (1600 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_XY	= -IST8310_MAX_VAL_XY;
static const int16_t IST8310_MAX_VAL_Z  = (2500 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_Z  = -IST8310_MAX_VAL_Z;

/* Hardware definitions */

#define ADDR_WAI                0		/* WAI means 'Who Am I'*/
# define WAI_EXPECTED_VALUE     0x10

#define ADDR_STAT1              0x02
# define STAT1_DRDY_SHFITS      0x0
# define STAT1_DRDY             (1 << STAT1_DRDY_SHFITS)
# define STAT1_DRO_SHFITS       0x1
# define STAT1_DRO              (1 << STAT1_DRO_SHFITS)

#define ADDR_DATA_OUT_X_LSB     0x03
#define ADDR_DATA_OUT_X_MSB     0x04
#define ADDR_DATA_OUT_Y_LSB     0x05
#define ADDR_DATA_OUT_Y_MSB     0x06
#define ADDR_DATA_OUT_Z_LSB     0x07
#define ADDR_DATA_OUT_Z_MSB     0x08

#define ADDR_STAT2              0x09
# define STAT2_INT_SHFITS       3
# define STAT2_INT              (1 << STAT2_INT_SHFITS)

#define ADDR_CTRL1              0x0a
# define CTRL1_MODE_SHFITS      0
# define CTRL1_MODE_STDBY       (0 << CTRL1_MODE_SHFITS)
# define CTRL1_MODE_SINGLE      (1 << CTRL1_MODE_SHFITS)

#define ADDR_CTRL2              0x0b
# define CTRL2_SRST_SHFITS      0   /* Begin POR (auto cleared) */
# define CTRL2_SRST             (1 << CTRL2_SRST_SHFITS)
# define CTRL2_DRP_SHIFTS       2
# define CTRL2_DRP              (1 << CTRL2_DRP_SHIFTS)
# define CTRL2_DREN_SHIFTS      3
# define CTRL2_DREN             (1 << CTRL2_DREN_SHIFTS)

#define ADDR_CTRL3				0x41
# define CTRL3_SAMPLEAVG_16		0x24	/* Sample Averaging 16 */
# define CTRL3_SAMPLEAVG_8		0x1b	/* Sample Averaging 8 */
# define CTRL3_SAMPLEAVG_4		0x12	/* Sample Averaging 4 */
# define CTRL3_SAMPLEAVG_2		0x09	/* Sample Averaging 2 */

#define ADDR_CTRL4				0x42
# define CTRL4_SRPD				0xC0	/* Set Reset Pulse Duration */

#define ADDR_STR                0x0c
# define STR_SELF_TEST_SHFITS   6
# define STR_SELF_TEST_ON       (1 << STR_SELF_TEST_SHFITS)
# define STR_SELF_TEST_OFF      (0 << STR_SELF_TEST_SHFITS)

#define ADDR_Y11_Low			0x9c
#define ADDR_Y11_High			0x9d
#define ADDR_Y12_Low			0x9e
#define ADDR_Y12_High			0x9f
#define ADDR_Y13_Low			0xa0
#define ADDR_Y13_High			0xa1
#define ADDR_Y21_Low			0xa2
#define ADDR_Y21_High			0xa3
#define ADDR_Y22_Low			0xa4
#define ADDR_Y22_High			0xa5
#define ADDR_Y23_Low			0xa6
#define ADDR_Y23_High			0xa7
#define ADDR_Y31_Low			0xa8
#define ADDR_Y31_High			0xa9
#define ADDR_Y32_Low			0xaa
#define ADDR_Y32_High			0xab
#define ADDR_Y33_Low			0xac
#define ADDR_Y33_High			0xad

#define ADDR_TEMPL              0x1c
#define ADDR_TEMPH              0x1d

class IST8310 : public device::I2C, public I2CSPIDriver<IST8310>
{
public:
	IST8310(I2CSPIBusOption bus_option, int bus_number, int address, enum Rotation rotation, int bus_frequency);
	virtual ~IST8310();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int     init();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void		start();

	/**
	 * Reset the device
	 */
	int         reset();

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
	void            RunImpl();

private:
	int probe() override;

	void            print_status() override;

	PX4Magnetometer _px4_mag;

	unsigned        _measure_interval{IST8310_CONVERSION_INTERVAL};

	bool        _collect_phase{false};

	perf_counter_t      _sample_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _range_errors;
	perf_counter_t      _conf_errors;

	uint8_t 		_ctl3_reg{0};
	uint8_t			_ctl4_reg{0};

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void            check_conf();

	/**
	 * Write a register.
	 *
	 * @param reg       The register to write.
	 * @param val       The value to write.
	 * @return      OK on write success.
	 */
	int         write_reg(uint8_t reg, uint8_t val);

	/**
	 * Write to a register block.
	 *
	 * @param address   The register address to write to.
	 * @param data      The buffer to write from.
	 * @param count     The number of bytes to write.
	 * @return      OK on write success.
	 */
	int     write(unsigned address, void *data, unsigned count);

	/**
	 * Read a register.
	 *
	 * @param reg       The register to read.
	 * @param val       The value read.
	 * @return      OK on read success.
	 */
	int         read_reg(uint8_t reg, uint8_t &val);

	/**
	 * read register block.
	 *
	 * @param address   The register address to read from.
	 * @param data      The buffer to read into.
	 * @param count     The number of bytes to read.
	 * @return      OK on write success.
	 */
	int read(unsigned address, void *data, unsigned count);

	/**
	 * Issue a measurement command.
	 *
	 * @return      OK if the measurement command was successful.
	 */
	int         measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int         collect();
};

IST8310::IST8310(I2CSPIBusOption bus_option, int bus_number, int address, enum Rotation rotation, int bus_frequency) :
	I2C(DRV_MAG_DEVTYPE_IST8310, MODULE_NAME, bus_number, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus_number, address),
	_px4_mag(get_device_id(), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_range_errors(perf_alloc(PC_COUNT, MODULE_NAME": rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, MODULE_NAME": conf_err"))
{
	_px4_mag.set_external(external());

	// default range scale from counts to gauss
	_px4_mag.set_scale(0.003f);
}

IST8310::~IST8310()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int IST8310::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		DEVICE_DEBUG("I2C init failed");
		goto out;
	}

	/* reset the device configuration */
	reset();

	ret = OK;
out:
	return ret;
}

int IST8310::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int IST8310::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void IST8310::check_conf()
{
	int ret;

	uint8_t ctrl_reg_in = 0;
	ret = read_reg(ADDR_CTRL3, ctrl_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (ctrl_reg_in != _ctl3_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CTRL3, _ctl3_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}

	ret = read_reg(ADDR_CTRL4, ctrl_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (ctrl_reg_in != _ctl4_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CTRL4, _ctl4_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

void IST8310::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

int IST8310::reset()
{
	/* software reset */
	write_reg(ADDR_CTRL2, CTRL2_SRST);

	/* configure control register 3 */
	_ctl3_reg = CTRL3_SAMPLEAVG_16;
	write_reg(ADDR_CTRL3, _ctl3_reg);

	/* configure control register 4 */
	_ctl4_reg = CTRL4_SRPD;
	write_reg(ADDR_CTRL4, _ctl4_reg);

	return OK;
}

int IST8310::probe()
{
	uint8_t data[1] = {0};

	_retries = 10;

	if (read(ADDR_WAI, &data[0], 1)) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	_retries = 2;

	if ((data[0] != WAI_EXPECTED_VALUE)) {
		DEVICE_DEBUG("ID byte mismatch (%02x) expected %02x", data[0], WAI_EXPECTED_VALUE);
		return -EIO;
	}

	return OK;
}

void IST8310::RunImpl()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_interval > IST8310_CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - IST8310_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(IST8310_CONVERSION_INTERVAL);
}

int IST8310::measure()
{
	/*
	 * Send the command to begin a measurement.
	 */
	int ret = write_reg(ADDR_CTRL1, CTRL1_MODE_SINGLE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int IST8310::collect()
{
	struct { /* status register and data as read back from the device */
		uint8_t     x[2];
		uint8_t     y[2];
		uint8_t     z[2];
	} report_buffer{};

	struct {
		int16_t     x, y, z;
	} report{};

	int ret;
	uint8_t check_counter;

	perf_begin(_sample_perf);

	float xraw_f;
	float yraw_f;
	float zraw_f;

	_px4_mag.set_error_count(perf_event_count(_comms_errors));

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	ret = read(ADDR_DATA_OUT_X_LSB, (uint8_t *)&report_buffer, sizeof(report_buffer));

	if (ret != OK) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("I2C read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int16_t)report_buffer.x[1]) << 8) | (int16_t)report_buffer.x[0];
	report.y = (((int16_t)report_buffer.y[1]) << 8) | (int16_t)report_buffer.y[0];
	report.z = (((int16_t)report_buffer.z[1]) << 8) | (int16_t)report_buffer.z[0];

	/*
	 * Check if value makes sense according to the FSR and Resolution of
	 * this sensor, discarding outliers
	 */
	if (report.x > IST8310_MAX_VAL_XY || report.x < IST8310_MIN_VAL_XY ||
	    report.y > IST8310_MAX_VAL_XY || report.y < IST8310_MIN_VAL_XY ||
	    report.z > IST8310_MAX_VAL_Z  || report.z < IST8310_MIN_VAL_Z) {
		perf_count(_range_errors);
		DEVICE_DEBUG("data/status read error");
		goto out;
	}

	/*
	 * raw outputs
	 *
	 * Sensor doesn't follow right hand rule, swap x and y to make it obey
	 * it.
	 */
	xraw_f = report.y;
	yraw_f = report.x;
	zraw_f = report.z;

	_px4_mag.update(timestamp_sample, xraw_f, yraw_f, zraw_f);

	/*
	  periodically check the range register and configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;

	if (check_counter == 128) {
		check_conf();
	}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int IST8310::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return write(reg, &buf, 1);
}

int IST8310::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = 0;
	int ret = read(reg, &buf, 1);
	val = buf;
	return ret;
}

void IST8310::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u interval\n", _measure_interval);
}

I2CSPIDriverBase *
IST8310::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator, int runtime_instance)
{
	IST8310 *interface = new IST8310(iterator.configuredBusOption(), iterator.bus(), cli.i2c_address, cli.rotation,
						 cli.bus_frequency);

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	interface->start();

	return interface;
}

void IST8310::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ist8310", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0xE);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int ist8310_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = IST8310;
	BusCLIArguments cli{true, false};
	cli.i2c_address = IST8310_BUS_I2C_ADDR;
	cli.default_i2c_frequency = IST8310_DEFAULT_BUS_SPEED;

	while ((ch = cli.getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_MAG_DEVTYPE_IST8310);

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
	return 1;
}
