/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file bma180.cpp
 * Driver for the Bosch BMA 180 MEMS accelerometer connected via SPI.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>

static constexpr uint8_t DIR_READ{1 << 7};

#define ADDR_CHIP_ID			0x00
#define CHIP_ID				0x03

#define ADDR_ACC_X_LSB			0x02
#define ADDR_ACC_Y_LSB			0x04
#define ADDR_ACC_Z_LSB			0x06
#define ADDR_TEMPERATURE		0x08

#define ADDR_CTRL_REG0			0x0D
#define REG0_WRITE_ENABLE		0x10

#define ADDR_RESET			0x10
#define SOFT_RESET				0xB6

#define ADDR_BW_TCS			0x20
#define BW_TCS_BW_MASK				(0xf<<4)
#define BW_TCS_BW_10HZ				(0<<4)
#define BW_TCS_BW_20HZ				(1<<4)
#define BW_TCS_BW_40HZ				(2<<4)
#define BW_TCS_BW_75HZ				(3<<4)
#define BW_TCS_BW_150HZ				(4<<4)
#define BW_TCS_BW_300HZ				(5<<4)
#define BW_TCS_BW_600HZ				(6<<4)
#define BW_TCS_BW_1200HZ			(7<<4)

#define ADDR_HIGH_DUR			0x27
#define HIGH_DUR_DIS_I2C			(1<<0)

#define ADDR_TCO_Z			0x30
#define TCO_Z_MODE_MASK				0x3

#define ADDR_GAIN_Y			0x33
#define GAIN_Y_SHADOW_DIS			(1<<0)

#define ADDR_OFFSET_LSB1		0x35
#define OFFSET_LSB1_RANGE_MASK			(7<<1)
#define OFFSET_LSB1_RANGE_1G			(0<<1)
#define OFFSET_LSB1_RANGE_2G			(2<<1)
#define OFFSET_LSB1_RANGE_3G			(3<<1)
#define OFFSET_LSB1_RANGE_4G			(4<<1)
#define OFFSET_LSB1_RANGE_8G			(5<<1)
#define OFFSET_LSB1_RANGE_16G			(6<<1)

#define ADDR_OFFSET_T			0x37
#define OFFSET_T_READOUT_12BIT			(1<<0)

class BMA180 : public device::SPI, public I2CSPIDriver<BMA180>
{
public:
	BMA180(I2CSPIBusOption bus_option, int bus, int32_t device, enum Rotation rotation, int bus_frequency,
	       spi_mode_e spi_mode);
	virtual ~BMA180();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void     print_usage();
	int		init() override;
	void		print_status() override;
	void		RunImpl();

private:
	int		probe() override;

	PX4Accelerometer	_px4_accel;

	unsigned		_current_lowpass;
	unsigned		_current_range;

	perf_counter_t		_sample_perf;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Read a register from the BMA180
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(uint8_t reg);

	/**
	 * Write a register in the BMA180
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(uint8_t reg, uint8_t value);

	/**
	 * Modify a register in the BMA180
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(uint8_t reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Set the BMA180 measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_g);

	/**
	 * Set the BMA180 internal lowpass filter frequency.
	 *
	 * @param frequency	The internal lowpass filter frequency is set to a value
	 *			equal or greater to this.
	 *			Zero selects the highest frequency supported.
	 * @return		OK if the value can be supported.
	 */
	int			set_lowpass(unsigned frequency);
};

BMA180::BMA180(I2CSPIBusOption bus_option, int bus, int32_t device, enum Rotation rotation, int bus_frequency,
	       spi_mode_e spi_mode) :
	SPI(DRV_ACC_DEVTYPE_BMA180, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_accel(get_device_id(), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read"))
{
}

BMA180::~BMA180()
{
	/* delete the perf counter */
	perf_free(_sample_perf);
}

int BMA180::init()
{
	int ret = PX4_ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		goto out;
	}

	/* perform soft reset (p48) */
	write_reg(ADDR_RESET, SOFT_RESET);

	/* wait 10 ms (datasheet incorrectly lists 10 us on page 49) */
	usleep(10000);

	/* enable writing to chip config */
	modify_reg(ADDR_CTRL_REG0, 0, REG0_WRITE_ENABLE);

	/* disable I2C interface */
	modify_reg(ADDR_HIGH_DUR, HIGH_DUR_DIS_I2C, 0);

	/* switch to low-noise mode */
	modify_reg(ADDR_TCO_Z, TCO_Z_MODE_MASK, 0);

	/* disable 12-bit mode */
	modify_reg(ADDR_OFFSET_T, OFFSET_T_READOUT_12BIT, 0);

	/* disable shadow-disable mode */
	modify_reg(ADDR_GAIN_Y, GAIN_Y_SHADOW_DIS, 0);

	/* disable writing to chip config */
	modify_reg(ADDR_CTRL_REG0, REG0_WRITE_ENABLE, 0);

	if (set_range(4)) {
		PX4_ERR("Failed setting range");
	}

	if (set_lowpass(75)) {
		PX4_ERR("Failed setting lowpass");
	}

	if (read_reg(ADDR_CHIP_ID) == CHIP_ID) {
		ret = OK;

	} else {
		ret = PX4_ERROR;
	}

	/* advertise sensor topic, measure manually to initialize valid report */
	measure();

	start();

out:
	return ret;
}

int BMA180::probe()
{
	/* dummy read to ensure SPI state machine is sane */
	read_reg(ADDR_CHIP_ID);

	if (read_reg(ADDR_CHIP_ID) == CHIP_ID) {
		return OK;
	}

	return -EIO;
}

uint8_t BMA180::read_reg(uint8_t reg)
{
	uint8_t cmd[2] {};
	cmd[0] = reg | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void BMA180::write_reg(uint8_t reg, uint8_t value)
{
	uint8_t	cmd[2] {reg, value};
	transfer(cmd, nullptr, sizeof(cmd));
}

void BMA180::modify_reg(uint8_t reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

int BMA180::set_range(unsigned max_g)
{
	uint8_t rangebits;

	if (max_g == 0) {
		max_g = 16;
	}

	if (max_g > 16) {
		return -ERANGE;
	}

	if (max_g <= 2) {
		_current_range = 2;
		rangebits = OFFSET_LSB1_RANGE_2G;

	} else if (max_g <= 3) {
		_current_range = 3;
		rangebits = OFFSET_LSB1_RANGE_3G;

	} else if (max_g <= 4) {
		_current_range = 4;
		rangebits = OFFSET_LSB1_RANGE_4G;

	} else if (max_g <= 8) {
		_current_range = 8;
		rangebits = OFFSET_LSB1_RANGE_8G;

	} else if (max_g <= 16) {
		_current_range = 16;
		rangebits = OFFSET_LSB1_RANGE_16G;

	} else {
		return -EINVAL;
	}

	/* set new range scaling factor */
	float accel_range_m_s2 = _current_range * CONSTANTS_ONE_G;
	_px4_accel.set_scale(accel_range_m_s2 / 8192.f);

	/* enable writing to chip config */
	modify_reg(ADDR_CTRL_REG0, 0, REG0_WRITE_ENABLE);

	/* adjust sensor configuration */
	modify_reg(ADDR_OFFSET_LSB1, OFFSET_LSB1_RANGE_MASK, rangebits);

	/* block writing to chip config */
	modify_reg(ADDR_CTRL_REG0, REG0_WRITE_ENABLE, 0);

	/* check if wanted value is now in register */
	return !((read_reg(ADDR_OFFSET_LSB1) & OFFSET_LSB1_RANGE_MASK) ==
		 (OFFSET_LSB1_RANGE_MASK & rangebits));
}

int BMA180::set_lowpass(unsigned frequency)
{
	uint8_t	bwbits;

	if (frequency > 1200) {
		return -ERANGE;

	} else if (frequency > 600) {
		bwbits = BW_TCS_BW_1200HZ;

	} else if (frequency > 300) {
		bwbits = BW_TCS_BW_600HZ;

	} else if (frequency > 150) {
		bwbits = BW_TCS_BW_300HZ;

	} else if (frequency > 75) {
		bwbits = BW_TCS_BW_150HZ;

	} else if (frequency > 40) {
		bwbits = BW_TCS_BW_75HZ;

	} else if (frequency > 20) {
		bwbits = BW_TCS_BW_40HZ;

	} else if (frequency > 10) {
		bwbits = BW_TCS_BW_20HZ;

	} else {
		bwbits = BW_TCS_BW_10HZ;
	}

	/* enable writing to chip config */
	modify_reg(ADDR_CTRL_REG0, 0, REG0_WRITE_ENABLE);

	/* adjust sensor configuration */
	modify_reg(ADDR_BW_TCS, BW_TCS_BW_MASK, bwbits);

	/* block writing to chip config */
	modify_reg(ADDR_CTRL_REG0, REG0_WRITE_ENABLE, 0);

	/* check if wanted value is now in register */
	return !((read_reg(ADDR_BW_TCS) & BW_TCS_BW_MASK) == (BW_TCS_BW_MASK & bwbits));
}

void BMA180::start()
{
	/* start polling at the specified rate */
	ScheduleOnInterval(1000, 10000);
}

void BMA180::RunImpl()
{
	/* make another measurement */
	measure();
}

void BMA180::measure()
{
	/* start the performance counter */
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	/*
	 * y of board is x of sensor and x of board is -y of sensor
	 * perform only the axis assignment here.
	 * Two non-value bits are discarded directly
	 */
	int16_t y_raw  = read_reg(ADDR_ACC_X_LSB + 0);
	y_raw |= read_reg(ADDR_ACC_X_LSB + 1) << 8;

	int16_t x_raw  = read_reg(ADDR_ACC_X_LSB + 2);
	x_raw |= read_reg(ADDR_ACC_X_LSB + 3) << 8;

	int16_t z_raw  = read_reg(ADDR_ACC_X_LSB + 4);
	z_raw |= read_reg(ADDR_ACC_X_LSB + 5) << 8;

	/* discard two non-value bits in the 16 bit measurement */
	x_raw = (x_raw / 4);
	y_raw = (y_raw / 4);
	z_raw = (z_raw / 4);

	/* invert y axis, due to 14 bit data no overflow can occur in the negation */
	y_raw = -y_raw;

	_px4_accel.update(timestamp_sample, x_raw, y_raw, z_raw);

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void BMA180::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
}

void BMA180::print_usage()
{
	PRINT_MODULE_USAGE_NAME("bma180", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *BMA180::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	BMA180 *instance = new BMA180(iterator.configuredBusOption(), iterator.bus(), iterator.devid(), cli.rotation,
				      cli.bus_frequency, cli.spi_mode);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	return instance;
}

extern "C" __EXPORT int bma180_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = BMA180;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = 8000000;

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

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ACC_DEVTYPE_BMA180);

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
