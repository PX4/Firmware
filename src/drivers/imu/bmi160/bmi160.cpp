/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "bmi160.hpp"

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t BMI160::_checked_registers[BMI160_NUM_CHECKED_REGISTERS] = {    BMIREG_CHIP_ID,
									      BMIREG_ACC_CONF,
									      BMIREG_ACC_RANGE,
									      BMIREG_GYR_CONF,
									      BMIREG_GYR_RANGE,
									      BMIREG_INT_EN_1,
									      BMIREG_INT_OUT_CTRL,
									      BMIREG_INT_MAP_1,
									      BMIREG_IF_CONF,
									      BMIREG_NV_CONF
									 };

BMI160::BMI160(I2CSPIBusOption bus_option, int bus, int32_t device, enum Rotation rotation, int bus_frequency,
	       spi_mode_e spi_mode) :
	SPI(DRV_IMU_DEVTYPE_BMI160, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_accel(get_device_id(), rotation),
	_px4_gyro(get_device_id(), rotation),
	_accel_reads(perf_alloc(PC_COUNT, MODULE_NAME": accel read")),
	_gyro_reads(perf_alloc(PC_COUNT, MODULE_NAME": gyro read")),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_bad_transfers(perf_alloc(PC_COUNT, MODULE_NAME": bad transfers")),
	_bad_registers(perf_alloc(PC_COUNT, MODULE_NAME": bad registers")),
	_good_transfers(perf_alloc(PC_COUNT, MODULE_NAME": good transfers")),
	_reset_retries(perf_alloc(PC_COUNT, MODULE_NAME": reset retries")),
	_duplicates(perf_alloc(PC_COUNT, MODULE_NAME": duplicates"))
{
}

BMI160::~BMI160()
{
	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_good_transfers);
	perf_free(_reset_retries);
	perf_free(_duplicates);
}

int BMI160::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	ret = reset();

	if (ret != PX4_OK) {
		return ret;
	}

	start();

	return ret;
}

int BMI160::reset()
{
	write_reg(BMIREG_CONF, (1 << 1)); //Enable NVM programming

	write_checked_reg(BMIREG_ACC_CONF,      BMI_ACCEL_US | BMI_ACCEL_BWP_NORMAL); //Normal operation, no decimation
	write_checked_reg(BMIREG_ACC_RANGE,     0);
	write_checked_reg(BMIREG_GYR_CONF,      BMI_GYRO_BWP_NORMAL);   //Normal operation, no decimation
	write_checked_reg(BMIREG_GYR_RANGE,     0);
	write_checked_reg(BMIREG_INT_EN_1,      BMI_DRDY_INT_EN); //Enable DRDY interrupt
	write_checked_reg(BMIREG_INT_OUT_CTRL,  BMI_INT1_EN);   //Enable interrupts on pin INT1
	write_checked_reg(BMIREG_INT_MAP_1,     BMI_DRDY_INT1); //DRDY interrupt on pin INT1
	write_checked_reg(BMIREG_IF_CONF,       BMI_SPI_4_WIRE |
			  BMI_AUTO_DIS_SEC); //Disable secondary interface; Work in SPI 4-wire mode
	write_checked_reg(BMIREG_NV_CONF,       BMI_SPI); //Disable I2C interface

	set_accel_range(BMI160_ACCEL_DEFAULT_RANGE_G);
	accel_set_sample_rate(BMI160_ACCEL_DEFAULT_RATE);

	set_gyro_range(BMI160_GYRO_DEFAULT_RANGE_DPS);
	gyro_set_sample_rate(BMI160_GYRO_DEFAULT_RATE);

	// Enable Accelerometer in normal mode
	write_reg(BMIREG_CMD, BMI_ACCEL_NORMAL_MODE);
	px4_usleep(4100);

	//Enable Gyroscope in normal mode
	write_reg(BMIREG_CMD, BMI_GYRO_NORMAL_MODE);
	px4_usleep(80300);

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI160_NUM_CHECKED_REGISTERS; i++) {
			if (read_reg(_checked_registers[i]) != _checked_values[i]) {
				write_reg(_checked_registers[i], _checked_values[i]);
				all_ok = false;
			}
		}

		if (all_ok) {
			break;
		}
	}

	_accel_reads = 0;
	_gyro_reads = 0;

	return OK;
}

int BMI160::probe()
{
	/* look for device ID */
	_whoami = read_reg(BMIREG_CHIP_ID);

	// verify product revision
	switch (_whoami) {
	case BMI160_WHO_AM_I:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = _whoami;
		_checked_bad[0] = _whoami;
		return OK;
	}

	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

int BMI160::accel_set_sample_rate(float frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = (BMI_ACCEL_RATE_25_8 | BMI_ACCEL_RATE_1600);

	if ((int)frequency == 0) {
		frequency = 1600;
	}

	if (frequency <= 25 / 32) {
		setbits |= BMI_ACCEL_RATE_25_32;
		_accel_sample_rate = 25 / 32;

	} else if (frequency <= 25 / 16) {
		setbits |= BMI_ACCEL_RATE_25_16;
		_accel_sample_rate = 25 / 16;

	} else if (frequency <= 25 / 8) {
		setbits |= BMI_ACCEL_RATE_25_8;
		_accel_sample_rate = 25 / 8;

	} else if (frequency <= 25 / 4) {
		setbits |= BMI_ACCEL_RATE_25_4;
		_accel_sample_rate = 25 / 4;

	} else if (frequency <= 25 / 2) {
		setbits |= BMI_ACCEL_RATE_25_2;
		_accel_sample_rate = 25 / 2;

	} else if (frequency <= 25) {
		setbits |= BMI_ACCEL_RATE_25;
		_accel_sample_rate = 25;

	} else if (frequency <= 50) {
		setbits |= BMI_ACCEL_RATE_50;
		_accel_sample_rate = 50;

	} else if (frequency <= 100) {
		setbits |= BMI_ACCEL_RATE_100;
		_accel_sample_rate = 100;

	} else if (frequency <= 200) {
		setbits |= BMI_ACCEL_RATE_200;
		_accel_sample_rate = 200;

	} else if (frequency <= 400) {
		setbits |= BMI_ACCEL_RATE_400;
		_accel_sample_rate = 400;

	} else if (frequency <= 800) {
		setbits |= BMI_ACCEL_RATE_800;
		_accel_sample_rate = 800;

	} else if (frequency > 800) {
		setbits |= BMI_ACCEL_RATE_1600;
		_accel_sample_rate = 1600;

	} else {
		return -EINVAL;
	}

	modify_reg(BMIREG_ACC_CONF, clearbits, setbits);

	return OK;
}

int BMI160::gyro_set_sample_rate(float frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = (BMI_GYRO_RATE_200 | BMI_GYRO_RATE_25);

	if ((int)frequency == 0) {
		frequency = 3200;
	}

	if (frequency <= 25) {
		setbits |= BMI_GYRO_RATE_25;
		_gyro_sample_rate = 25;

	} else if (frequency <= 50) {
		setbits |= BMI_GYRO_RATE_50;
		_gyro_sample_rate = 50;

	} else if (frequency <= 100) {
		setbits |= BMI_GYRO_RATE_100;
		_gyro_sample_rate = 100;

	} else if (frequency <= 200) {
		setbits |= BMI_GYRO_RATE_200;
		_gyro_sample_rate = 200;

	} else if (frequency <= 400) {
		setbits |= BMI_GYRO_RATE_400;
		_gyro_sample_rate = 400;

	} else if (frequency <= 800) {
		setbits |= BMI_GYRO_RATE_800;
		_gyro_sample_rate = 800;

	} else if (frequency <= 1600) {
		setbits |= BMI_GYRO_RATE_1600;
		_gyro_sample_rate = 1600;

	} else if (frequency > 1600) {
		setbits |= BMI_GYRO_RATE_3200;
		_gyro_sample_rate = 3200;

	} else {
		return -EINVAL;
	}

	modify_reg(BMIREG_GYR_CONF, clearbits, setbits);

	return OK;
}

uint8_t BMI160::read_reg(uint8_t reg)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

int BMI160::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];
	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;
	return transfer(cmd, nullptr, sizeof(cmd));
}

void BMI160::modify_reg(uint8_t reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void BMI160::write_checked_reg(uint8_t reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < BMI160_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

int BMI160::set_accel_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI_ACCEL_RANGE_2_G | BMI_ACCEL_RANGE_16_G;
	float lsb_per_g;

	if (max_g == 0) {
		max_g = 16;
	}

	if (max_g <= 2) {
		//max_accel_g = 2;
		setbits |= BMI_ACCEL_RANGE_2_G;
		lsb_per_g = 16384;

	} else if (max_g <= 4) {
		//max_accel_g = 4;
		setbits |= BMI_ACCEL_RANGE_4_G;
		lsb_per_g = 8192;

	} else if (max_g <= 8) {
		//max_accel_g = 8;
		setbits |= BMI_ACCEL_RANGE_8_G;
		lsb_per_g = 4096;

	} else if (max_g <= 16) {
		//max_accel_g = 16;
		setbits |= BMI_ACCEL_RANGE_16_G;
		lsb_per_g = 2048;

	} else {
		return -EINVAL;
	}

	_px4_accel.set_scale(CONSTANTS_ONE_G / lsb_per_g);

	modify_reg(BMIREG_ACC_RANGE, clearbits, setbits);

	return OK;
}

int BMI160::set_gyro_range(unsigned max_dps)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI_GYRO_RANGE_125_DPS | BMI_GYRO_RANGE_250_DPS;
	float lsb_per_dps;
	//float max_gyro_dps;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 125) {
		//max_gyro_dps = 125;
		lsb_per_dps = 262.4;
		setbits |= BMI_GYRO_RANGE_125_DPS;

	} else if (max_dps <= 250) {
		//max_gyro_dps = 250;
		lsb_per_dps = 131.2;
		setbits |= BMI_GYRO_RANGE_250_DPS;

	} else if (max_dps <= 500) {
		//max_gyro_dps = 500;
		lsb_per_dps = 65.6;
		setbits |= BMI_GYRO_RANGE_500_DPS;

	} else if (max_dps <= 1000) {
		//max_gyro_dps = 1000;
		lsb_per_dps = 32.8;
		setbits |= BMI_GYRO_RANGE_1000_DPS;

	} else if (max_dps <= 2000) {
		//max_gyro_dps = 2000;
		lsb_per_dps = 16.4;
		setbits |= BMI_GYRO_RANGE_2000_DPS;

	} else {
		return -EINVAL;
	}

	_px4_gyro.set_scale(M_PI_F / (180.0f * lsb_per_dps));

	modify_reg(BMIREG_GYR_RANGE, clearbits, setbits);

	return OK;
}

void BMI160::start()
{
	/* start polling at the specified rate */
	ScheduleOnInterval((1_s / BMI160_GYRO_DEFAULT_RATE) - BMI160_TIMER_REDUCTION, 1000);

	reset();
}

void BMI160::check_registers()
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) !=
	    _checked_values[_checked_next]) {
		_checked_bad[_checked_next] = v;

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
		  bus.
		 */
		if (_register_wait == 0 || _checked_next == 0) {
			// if the product_id is wrong then reset the
			// sensor completely
			write_reg(BMIREG_CMD, BMI160_SOFT_RESET);
			_reset_wait = hrt_absolute_time() + 10000;
			_checked_next = 0;

		} else {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
			// waiting 3ms between register writes seems
			// to raise the chance of the sensor
			// recovering considerably
			_reset_wait = hrt_absolute_time() + 3000;
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % BMI160_NUM_CHECKED_REGISTERS;
}

void BMI160::RunImpl()
{
	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct BMIReport bmi_report {};

	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} report{};

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the BMI160 in one pass.
	 */
	bmi_report.cmd = BMIREG_GYR_X_L | DIR_READ;

	uint8_t status = read_reg(BMIREG_STATUS);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (OK != transfer((uint8_t *)&bmi_report, ((uint8_t *)&bmi_report), sizeof(bmi_report))) {
		return;
	}

	check_registers();

	if ((!(status & (0x80))) && (!(status & (0x04)))) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return;
	}

	_last_accel[0] = bmi_report.accel_x;
	_last_accel[1] = bmi_report.accel_y;
	_last_accel[2] = bmi_report.accel_z;
	_got_duplicate = false;

	uint8_t temp_l = read_reg(BMIREG_TEMP_0);
	uint8_t temp_h = read_reg(BMIREG_TEMP_1);

	report.temp = ((temp_h << 8) + temp_l);

	report.accel_x = bmi_report.accel_x;
	report.accel_y = bmi_report.accel_y;
	report.accel_z = bmi_report.accel_z;

	report.gyro_x = bmi_report.gyro_x;
	report.gyro_y = bmi_report.gyro_y;
	report.gyro_z = bmi_report.gyro_z;

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0 &&
	    report.gyro_x == 0 &&
	    report.gyro_y == 0 &&
	    report.gyro_z == 0) {

		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the bmi160 does go bad it would cause a FMU failure,
		// regardless of whether another sensor is available,
		return;
	}

	perf_count(_good_transfers);

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again. We still increment
		// _good_transfers, but don't return any data yet
		_register_wait--;
		return;
	}

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	const uint64_t error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);
	_px4_accel.set_error_count(error_count);
	_px4_gyro.set_error_count(error_count);

	const float temperature = 23.0f + report.temp * 1.0f / 512.0f;
	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

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


	/* NOTE: Axes have been swapped to match the board a few lines above. */

	_px4_accel.update(timestamp_sample, bmi_report.accel_x, bmi_report.accel_y, bmi_report.accel_z);
	_px4_gyro.update(timestamp_sample, bmi_report.gyro_x, bmi_report.gyro_y, bmi_report.gyro_z);

	/* stop measuring */
	perf_end(_sample_perf);
}

void BMI160::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
	perf_print_counter(_duplicates);
}
