/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include "L3GD20.hpp"

constexpr uint8_t L3GD20::_checked_registers[];

L3GD20::L3GD20(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
	       spi_mode_e spi_mode) :
	SPI(DRV_GYR_DEVTYPE_L3GD20, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_gyro(get_device_id(), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_errors(perf_alloc(PC_COUNT, MODULE_NAME": err")),
	_bad_registers(perf_alloc(PC_COUNT, MODULE_NAME": bad_reg")),
	_duplicates(perf_alloc(PC_COUNT, MODULE_NAME": dupe"))
{
}

L3GD20::~L3GD20()
{
	perf_free(_sample_perf);
	perf_free(_errors);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
L3GD20::init()
{
	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		return PX4_ERROR;
	}

	reset();

	start();

	return PX4_OK;
}

int
L3GD20::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	read_reg(ADDR_WHO_AM_I);

	bool success = false;
	uint8_t v = 0;

	/* verify that the device is attached and functioning, accept
	 * L3GD20, L3GD20H and L3G4200D */
	if ((v = read_reg(ADDR_WHO_AM_I)) == WHO_I_AM) {
		_orientation = SENSOR_BOARD_ROTATION_DEFAULT;
		success = true;

	} else if ((v = read_reg(ADDR_WHO_AM_I)) == WHO_I_AM_H) {
		_orientation = SENSOR_BOARD_ROTATION_180_DEG;
		success = true;

	} else if ((v = read_reg(ADDR_WHO_AM_I)) == WHO_I_AM_L3G4200D) {
		/* Detect the L3G4200D used on AeroCore */
		_is_l3g4200d = true;
		_orientation = SENSOR_BOARD_ROTATION_DEFAULT;
		success = true;
	}

	if (success) {
		_checked_values[0] = v;
		return OK;
	}

	return -EIO;
}

uint8_t
L3GD20::read_reg(unsigned reg)
{
	uint8_t cmd[2] {};

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

int
L3GD20::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2] {};

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	return transfer(cmd, nullptr, sizeof(cmd));
}

void
L3GD20::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < L3GD20_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

void
L3GD20::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
L3GD20::set_range(unsigned max_dps)
{
	uint8_t bits = REG4_BDU;
	float new_range_scale_dps_digit;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 250) {
		//new_range = 250;
		bits |= RANGE_250DPS;
		new_range_scale_dps_digit = 8.75e-3f;

	} else if (max_dps <= 500) {
		//new_range = 500;
		bits |= RANGE_500DPS;
		new_range_scale_dps_digit = 17.5e-3f;

	} else if (max_dps <= 2000) {
		//new_range = 2000;
		bits |= RANGE_2000DPS;
		new_range_scale_dps_digit = 70e-3f;

	} else {
		return -EINVAL;
	}

	_px4_gyro.set_scale(new_range_scale_dps_digit / 180.0f * M_PI_F);

	write_checked_reg(ADDR_CTRL_REG4, bits);

	return OK;
}

int
L3GD20::set_samplerate(unsigned frequency)
{
	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;

	if (frequency == 0) {
		frequency = _is_l3g4200d ? 800 : 760;
	}

	/*
	 * Use limits good for H or non-H models. Rates are slightly different
	 * for L3G4200D part but register settings are the same.
	 */
	if (frequency <= 100) {
		bits |= RATE_95HZ_LP_25HZ;

	} else if (frequency <= 200) {
		bits |= RATE_190HZ_LP_50HZ;

	} else if (frequency <= 400) {
		bits |= RATE_380HZ_LP_50HZ;

	} else if (frequency <= 800) {
		bits |= RATE_760HZ_LP_50HZ;

	} else {
		return -EINVAL;
	}

	write_checked_reg(ADDR_CTRL_REG1, bits);

	return OK;
}

void
L3GD20::start()
{
	/* start polling at the specified rate */
	uint64_t interval = 1000000 / L3GD20_DEFAULT_RATE;
	ScheduleOnInterval(interval - L3GD20_TIMER_REDUCTION, 10000);
}

void
L3GD20::disable_i2c()
{
	uint8_t retries = 10;

	while (retries--) {
		// add retries
		uint8_t a = read_reg(0x05);
		write_reg(0x05, (0x20 | a));

		if (read_reg(0x05) == (a | 0x20)) {
			// this sets the I2C_DIS bit on the
			// L3GD20H. The l3gd20 datasheet doesn't
			// mention this register, but it does seem to
			// accept it.
			write_checked_reg(ADDR_LOW_ODR, 0x08);
			return;
		}
	}

	DEVICE_DEBUG("FAILED TO DISABLE I2C");
}

void
L3GD20::reset()
{
	// ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c();

	/* set default configuration */
	write_checked_reg(ADDR_CTRL_REG1, REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
	write_checked_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_checked_reg(ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
	write_checked_reg(ADDR_CTRL_REG4, REG4_BDU);
	write_checked_reg(ADDR_CTRL_REG5, 0);
	write_checked_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */

	/* disable FIFO. This makes things simpler and ensures we
	 * aren't getting stale data. It means we must run the hrt
	 * callback fast enough to not miss data. */
	write_checked_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);

	set_samplerate(0); // 760Hz or 800Hz
	set_range(L3GD20_DEFAULT_RANGE_DPS);

	_read = 0;
}

void
L3GD20::check_registers()
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {
		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus. We skip zero as that is the WHO_AM_I, which
		  is not writeable
		 */
		if (_checked_next != 0) {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % L3GD20_NUM_CHECKED_REGISTERS;
}

void
L3GD20::RunImpl()
{
	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		int8_t		temp;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_report{};
#pragma pack(pop)

	/* start the performance counter */
	perf_begin(_sample_perf);

	check_registers();

	/* fetch data from the sensor */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

	if (!(raw_report.status & STATUS_ZYXDA)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		return;
	}

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */
	_px4_gyro.set_error_count(perf_event_count(_bad_registers));

	_px4_gyro.set_temperature(L3GD20_TEMP_OFFSET_CELSIUS - raw_report.temp);

	switch (_orientation) {
	case SENSOR_BOARD_ROTATION_090_DEG:
		/* swap x and y */
		_px4_gyro.update(timestamp_sample, raw_report.y, raw_report.x, raw_report.z);
		break;

	case SENSOR_BOARD_ROTATION_180_DEG: {
			/* swap x and y and negate both */
			int16_t x = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
			int16_t y = ((raw_report.y == -32768) ? 32767 : -raw_report.y);
			_px4_gyro.update(timestamp_sample, x, y, raw_report.z);
		}

		break;

	case SENSOR_BOARD_ROTATION_270_DEG: {
			/* swap x and y and negate y */
			int16_t x = raw_report.y;
			int16_t y = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
			_px4_gyro.update(timestamp_sample, x, y, raw_report.z);
		}
		break;

	case SENSOR_BOARD_ROTATION_000_DEG:

	// FALLTHROUGH
	default:
		// keep axes in place
		_px4_gyro.update(timestamp_sample, raw_report.x, raw_report.y, raw_report.z);
	}

	_read++;

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void
L3GD20::print_status()
{
	I2CSPIDriverBase::print_status();
	printf("gyro reads:          %u\n", _read);
	perf_print_counter(_sample_perf);
	perf_print_counter(_errors);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);

	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < L3GD20_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}
	}

}

void
L3GD20::print_registers()
{
	printf("L3GD20 registers\n");

	for (uint8_t reg = 0; reg <= 0x40; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if ((reg + 1) % 16 == 0) {
			printf("\n");
		}
	}

	printf("\n");
}

void
L3GD20::test_error()
{
	// trigger a deliberate error
	write_reg(ADDR_CTRL_REG3, 0);
}
