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

#include "BMI088_accel.hpp"

/*
 * Global variable of the accelerometer temperature reading, to read it in the bmi055_gyro driver. The variable is changed in bmi055_accel.cpp.
 * This is a HACK! The driver should be rewritten with the gyro as subdriver.
 */
extern float _accel_last_temperature_copy;

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t BMI088_accel::_checked_registers[BMI088_ACCEL_NUM_CHECKED_REGISTERS] = {BMI088_ACC_CHIP_ID,
										      BMI088_ACC_CONF,
										      BMI088_ACC_RANGE,
										      BMI088_ACC_INT1_IO_CONF,
										      BMI088_ACC_INT1_INT2_MAP_DATA,
										      BMI088_ACC_PWR_CONF,
										      BMI088_ACC_PWR_CTRL,
										     };

BMI088_accel::BMI088_accel(int bus, const char *path_accel, uint32_t device, enum Rotation rotation) :
	BMI088("BMI088_ACCEL", path_accel, bus, device, SPIDEV_MODE3, BMI088_BUS_SPEED, rotation),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), (external() ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi088_accel_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi088_accel_bad_transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi088_accel_bad_registers")),
	_duplicates(perf_alloc(PC_COUNT, "bmi088_accel_duplicates")),
	_got_duplicate(false)
{
	_px4_accel.set_device_type(DRV_ACC_DEVTYPE_BMI088);
}

BMI088_accel::~BMI088_accel()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
BMI088_accel::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	return reset();
}

uint8_t
BMI088_accel::read_reg(unsigned reg)
{
	// For the BMI088, you need to read out a dummy byte before you can read out the normal data (see section "SPI interface of accelerometer part" of the BMI088 datasheet)
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0};

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[2]; // Skip dummy byte in cmd[1] and read out actual data
}

uint16_t
BMI088_accel::read_reg16(unsigned reg)
{
	// For the BMI088, you need to read out the dummy byte before you can read out the normal data (see section "SPI interface of accelerometer part" of the BMI088 datasheet)
	uint8_t cmd[4] = { (uint8_t)(reg | DIR_READ), 0, 0, 0 };

	transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[2] << 8) | cmd[3]; // Skip dummy byte in cmd[1]
}

int BMI088_accel::reset()
{
	write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET); // Soft-reset
	/* After a POR or soft-reset, the sensor needs up to 1ms boot time
	 * (see section "Power Modes: Accelerometer" in the BMI088 datasheet).
	 * Based off of testing it seems this value needs to be increased from 1ms.
	 * Setting it to 5ms.
	 */

	up_udelay(5000);

	// Perform a dummy read here to put the accelerometer part of the BMI088 back into SPI mode after the reset
	// The dummy read basically pulls the chip select line low and then high
	// See section "Serial Peripheral Interface (SPI)" of the BMI088 datasheet for more details.
	read_reg(BMI088_ACC_CHIP_ID);

	// Enable Accelerometer
	// The accelerometer needs to be enabled first, before writing to its registers
	write_checked_reg(BMI088_ACC_PWR_CTRL, BMI088_ACC_PWR_CTRL_EN);
	/* After changing power modes, the sensor requires up to 5ms to settle.
	 * Any communication with the sensor during this time should be avoided
	 * (see section "Power Modes: Acceleromter" in the BMI datasheet) */

	up_udelay(5000);

	// Set the PWR CONF to be active
	write_checked_reg(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_CONF_ACTIVE); // Sets the accelerometer to active mode

	// Write accel bandwidth and output data rate
	// ToDo set the bandwidth
	accel_set_sample_rate(BMI088_ACCEL_DEFAULT_RATE); //set accel ODR
	set_accel_range(BMI088_ACCEL_DEFAULT_RANGE_G); //set accel range

	// Configure the accel INT1
	write_checked_reg(BMI088_ACC_INT1_IO_CONF,
			  BMI088_ACC_INT1_IO_CONF_INT1_OUT | BMI088_ACC_INT1_IO_CONF_PP |
			  BMI088_ACC_INT1_IO_CONF_ACTIVE_HIGH); // Configure INT1 pin as output, push-pull, active high
	write_checked_reg(BMI088_ACC_INT1_INT2_MAP_DATA,
			  BMI088_ACC_INT1_INT2_MAP_DATA_INT1_DRDY); // Map DRDY interrupt on pin INT1

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
			if (read_reg(_checked_registers[i]) != _checked_values[i]) {
				write_reg(_checked_registers[i], _checked_values[i]);
				all_ok = false;
			}
		}

		if (all_ok) {
			break;
		}
	}

	return OK;
}

int
BMI088_accel::probe()
{
	// Perform a dummy read here to put the accelerometer part of the BMI088 back into SPI mode after the reset
	// The dummy read basically pulls the chip select line low and then high
	// See section "Serial Peripheral Interface (SPI)" of the BMI088 datasheet for more details.
	read_reg(BMI088_ACC_CHIP_ID);

	/* look for device ID */
	_whoami = read_reg(BMI088_ACC_CHIP_ID);

	// verify product revision
	switch (_whoami) {
	case BMI088_ACC_WHO_AM_I:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = _whoami;
		_checked_bad[0] = _whoami;
		return OK;
	}

	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

int
BMI088_accel::accel_set_sample_rate(float frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = 0x0F;

	if (frequency < 25) {
		setbits |= BMI088_ACC_CONF_ODR_12_5;
		//_accel_sample_rate = 12.5f;

	} else if (frequency < 50) {
		setbits |= BMI088_ACC_CONF_ODR_25;
		//_accel_sample_rate = 25.f;

	} else if (frequency < 100) {
		setbits |= BMI088_ACC_CONF_ODR_50;
		//_accel_sample_rate = 50.f;

	} else if (frequency < 200) {
		setbits |= BMI088_ACC_CONF_ODR_100;
		//_accel_sample_rate = 100.f;

	} else if (frequency < 400) {
		setbits |= BMI088_ACC_CONF_ODR_200;
		//_accel_sample_rate = 200.f;

	} else if (frequency < 800) {
		setbits |= BMI088_ACC_CONF_ODR_400;
		//_accel_sample_rate = 400.f;

	} else if (frequency < 1600) {
		setbits |= BMI088_ACC_CONF_ODR_800;
		//_accel_sample_rate = 800.f;

	} else if (frequency >= 1600) {
		setbits |= BMI088_ACC_CONF_ODR_1600;
		//_accel_sample_rate = 1600.f;

	} else {
		printf("Set sample rate error \n");
		return -EINVAL;
	}

	/* Write accel ODR */
	modify_reg(BMI088_ACC_CONF, clearbits, setbits);

	return OK;
}

/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
BMI088_accel::test_error()
{
	write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET);
	::printf("error triggered\n");
	print_registers();
}

void
BMI088_accel::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
BMI088_accel::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

int
BMI088_accel::set_accel_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI088_ACCEL_RANGE_24_G;
	float lsb_per_g;

	if (max_g == 0) {
		max_g = 24;
	}

	if (max_g <= 3) {
		//max_accel_g = 3;
		setbits |= BMI088_ACCEL_RANGE_3_G;
		lsb_per_g = 10922.67;

	} else if (max_g <= 6) {
		//max_accel_g = 6;
		setbits |= BMI088_ACCEL_RANGE_6_G;
		lsb_per_g = 5461.33;

	} else if (max_g <= 12) {
		//max_accel_g = 12;
		setbits |= BMI088_ACCEL_RANGE_12_G;
		lsb_per_g = 2730.67;

	} else if (max_g <= 24) {
		//max_accel_g = 24;
		setbits |= BMI088_ACCEL_RANGE_24_G;
		lsb_per_g = 1365.33;

	} else {
		return -EINVAL;
	}

	_px4_accel.set_scale(CONSTANTS_ONE_G / lsb_per_g);

	modify_reg(BMI088_ACC_RANGE, clearbits, setbits);

	return OK;
}

void
BMI088_accel::start()
{
	/* make sure we are stopped first */
	stop();

	// Reset the accelerometer
	reset();

	/* start polling at the specified rate */
	ScheduleOnInterval(BMI088_ACCEL_DEFAULT_RATE - BMI088_TIMER_REDUCTION, 1000);

}

void
BMI088_accel::stop()
{
	ScheduleClear();
}

void
BMI088_accel::Run()
{
	/* make another measurement */
	measure();
}

void
BMI088_accel::check_registers(void)
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
			write_reg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET);
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

	_checked_next = (_checked_next + 1) % BMI088_ACCEL_NUM_CHECKED_REGISTERS;
}

void
BMI088_accel::measure()
{
	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct Report {
		int16_t     accel_x;
		int16_t     accel_y;
		int16_t     accel_z;
		int16_t     temp;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	// Checking the status of new data
	uint8_t status;
	status = read_reg(BMI088_ACC_STATUS);

	if (!(status & BMI088_ACC_STATUS_DRDY)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return;
	}

	_got_duplicate = false;

	/*
	     * Fetch the full set of measurements from the BMI088 in one pass.
	     */
	uint8_t index = 0;
	uint8_t accel_data[8]; // Need an extra byte for the command, and an an extra dummy byte for the read (see section "SPI interface of accelerometer part" of the BMI088 datasheet)
	accel_data[index] = BMI088_ACC_X_L | DIR_READ;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (OK != transfer(accel_data, accel_data, sizeof(accel_data))) {
		return;
	}

	check_registers();

	/* Extracting accel data from the read data */
	index = 2; // Skip the dummy byte at index=1
	uint16_t lsb, msb, msblsb;

	lsb = (uint16_t)accel_data[index++];
	msb = (uint16_t)accel_data[index++];
	msblsb = (msb << 8) | lsb;
	report.accel_x = (int16_t)msblsb; /* Data in X axis */

	lsb = (uint16_t)accel_data[index++];
	msb = (uint16_t)accel_data[index++];
	msblsb = (msb << 8) | lsb;
	report.accel_y = (int16_t)msblsb; /* Data in Y axis */

	lsb = (uint16_t)accel_data[index++];
	msb = (uint16_t)accel_data[index++];
	msblsb = (msb << 8) | lsb;
	report.accel_z = (int16_t)msblsb; /* Data in Z axis */

	// Extract the temperature data
	// Note: the temp sensor data is only updated every 1.28s (see "Register 0x22-0x23 Temperature Sensor Data" section in BMI088 Datasheet)
	index = 0;
	accel_data[index] = BMI088_ACC_TEMP_H | DIR_READ;

	// Need to perform a dummy read, hence the num bytes to read is 3 (plus 1 send byte)
	if (OK != transfer(accel_data, accel_data, 4)) {
		return;
	}

	index = 2;
	msb = (uint16_t)accel_data[index++];
	lsb = (uint16_t)accel_data[index++];
	uint16_t temp = msb * 8 + lsb / 32;

	if (temp > 1023) {
		report.temp = temp - 2048;

	} else {
		report.temp = temp;
	}


	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0) {
		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the bmi088 accel does go bad it would cause a FMU failure,
		// regardless of whether another sensor is available,
		return;
	}

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again, but don't return any data yet
		_register_wait--;
		return;
	}

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	const uint64_t error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);
	_px4_accel.set_error_count(error_count);

	// Convert the bit-wise representation of temperature to degrees C
	_accel_last_temperature_copy = (report.temp * 0.125f) + 23.0f;
	_px4_accel.set_temperature(_accel_last_temperature_copy);

	/*
	     * 1) Scale raw value to SI units using scaling from datasheet.
	     * 2) Subtract static offset (in SI units)
	     * 3) Scale the statically calibrated values with a linear
	     *    dynamically obtained factor
	     *
	     * Note: the static sensor offset is the number the sensor outputs
	     *   at a nominally 'zero' input. Therefore the offset has to
	     *   be subtracted.
	     *
	     */
	_px4_accel.update(timestamp_sample, report.accel_x, report.accel_y, report.accel_z);

	/* stop measuring */
	perf_end(_sample_perf);
}

void
BMI088_accel::print_info()
{
	PX4_INFO("Accel");

	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);

	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}

		if (v != _checked_bad[i]) {
			::printf("reg %02x:%02x was bad %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_bad[i]);
		}
	}

	_px4_accel.print_status();
}

void
BMI088_accel::print_registers()
{
	uint8_t index = 0;
	printf("BMI088 accel registers\n");

	uint8_t reg = _checked_registers[index++];
	uint8_t v = read_reg(reg);
	printf("Accel Chip Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Conf: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Range: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Int1 Conf: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Int1-Int2_Map-Data: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Pwr Conf: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Accel Pwr Ctrl: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");



	printf("\n");
}
