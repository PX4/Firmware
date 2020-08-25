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

#include "BMI088_gyro.hpp"
#include "BMI088_accel.hpp"


/*
 * Global variable of the accelerometer temperature reading, to read it in the bmi055_gyro driver.
 * This is a ugly HACK! The driver should potentially be rewritten with the gyro as subdriver.
 */
__EXPORT float _accel_last_temperature_copy = 0;

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */

const uint8_t BMI088_gyro::_checked_registers[BMI088_GYRO_NUM_CHECKED_REGISTERS] = {    BMI088_GYR_CHIP_ID,
											BMI088_GYR_LPM1,
											BMI088_GYR_BW,
											BMI088_GYR_RANGE,
											BMI088_GYR_INT_EN_0,
											BMI088_GYR_INT_EN_1,
											BMI088_GYR_INT_MAP_1
										   };

BMI088_gyro::BMI088_gyro(int bus, const char *path_gyro, uint32_t device, enum Rotation rotation) :
	BMI088("BMI088_GYRO", path_gyro, bus, device, SPIDEV_MODE3, BMI088_BUS_SPEED, rotation),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_gyro(get_device_id(), (external() ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi088_gyro_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi088_gyro_bad_transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi088_gyro_bad_registers"))
{
	_px4_gyro.set_device_type(DRV_GYR_DEVTYPE_BMI088);
}

BMI088_gyro::~BMI088_gyro()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
}

int
BMI088_gyro::init()
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

int BMI088_gyro::reset()
{
	write_reg(BMI088_GYR_SOFTRESET, BMI088_SOFT_RESET);//Soft-reset
	usleep(5000);
	write_checked_reg(BMI088_GYR_BW,     0); // Write Gyro Bandwidth (will be overwritten in gyro_set_sample_rate())
	write_checked_reg(BMI088_GYR_RANGE,     0);// Write Gyro range
	write_checked_reg(BMI088_GYR_INT_EN_0,  BMI088_GYR_DRDY_INT_EN); //Enable DRDY interrupt
	write_checked_reg(BMI088_GYR_INT_MAP_1, BMI088_GYR_DRDY_INT1); //Map DRDY interrupt on pin INT1

	set_gyro_range(BMI088_GYRO_DEFAULT_RANGE_DPS);// set Gyro range
	gyro_set_sample_rate(BMI088_GYRO_DEFAULT_RATE);// set Gyro ODR & Filter Bandwidth

	//Enable Gyroscope in normal mode
	write_reg(BMI088_GYR_LPM1, BMI088_GYRO_NORMAL);
	up_udelay(1000);

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI088_GYRO_NUM_CHECKED_REGISTERS; i++) {
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
BMI088_gyro::probe()
{
	/* look for device ID */
	_whoami = read_reg(BMI088_GYR_CHIP_ID);

	// verify product revision
	switch (_whoami) {
	case BMI088_GYR_WHO_AM_I:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = _whoami;
		_checked_bad[0] = _whoami;
		return OK;
	}

	printf("unexpected whoami 0x%02x\n", _whoami);
	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

int
BMI088_gyro::gyro_set_sample_rate(float frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI088_GYRO_BW_MASK;

	if (frequency <= 100) {
		setbits |= BMI088_GYRO_RATE_100; /* 32 Hz cutoff */
		//_gyro_sample_rate = 100;

	} else if (frequency <= 250) {
		setbits |= BMI088_GYRO_RATE_400; /* 47 Hz cutoff */
		//_gyro_sample_rate = 400;

	} else if (frequency <= 1000) {
		setbits |= BMI088_GYRO_RATE_1000; /* 116 Hz cutoff */
		//_gyro_sample_rate = 1000;

	} else if (frequency > 1000) {
		setbits |= BMI088_GYRO_RATE_2000; /* 230 Hz cutoff */
		//_gyro_sample_rate = 2000;

	} else {
		return -EINVAL;
	}

	modify_reg(BMI088_GYR_BW, clearbits, setbits);

	return OK;
}

/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
BMI088_gyro::test_error()
{
	write_reg(BMI088_GYR_SOFTRESET, BMI088_SOFT_RESET);
	::printf("error triggered\n");
	print_registers();
}

void
BMI088_gyro::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
BMI088_gyro::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < BMI088_GYRO_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

int
BMI088_gyro::set_gyro_range(unsigned max_dps)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI088_GYRO_RANGE_125_DPS | BMI088_GYRO_RANGE_250_DPS;
	float lsb_per_dps;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 125) {
		//max_gyro_dps = 125;
		lsb_per_dps = 262.4;
		setbits |= BMI088_GYRO_RANGE_125_DPS;

	} else if (max_dps <= 250) {
		//max_gyro_dps = 250;
		lsb_per_dps = 131.2;
		setbits |= BMI088_GYRO_RANGE_250_DPS;

	} else if (max_dps <= 500) {
		//max_gyro_dps = 500;
		lsb_per_dps = 65.6;
		setbits |= BMI088_GYRO_RANGE_500_DPS;

	} else if (max_dps <= 1000) {
		//max_gyro_dps = 1000;
		lsb_per_dps = 32.8;
		setbits |= BMI088_GYRO_RANGE_1000_DPS;

	} else if (max_dps <= 2000) {
		//max_gyro_dps = 2000;
		lsb_per_dps = 16.4;
		setbits |= BMI088_GYRO_RANGE_2000_DPS;

	} else {
		return -EINVAL;
	}

	_px4_gyro.set_scale(M_PI_F / (180.0f * lsb_per_dps));

	modify_reg(BMI088_GYR_RANGE, clearbits, setbits);

	return OK;
}

void
BMI088_gyro::start()
{
	/* make sure we are stopped first */
	stop();

	/* start polling at the specified rate */
	ScheduleOnInterval(BMI088_GYRO_DEFAULT_RATE - BMI088_TIMER_REDUCTION, 1000);
}

void
BMI088_gyro::stop()
{
	ScheduleClear();
}

void
BMI088_gyro::Run()
{
	/* make another measurement */
	measure();
}

void
BMI088_gyro::measure_trampoline(void *arg)
{
	BMI088_gyro *dev = static_cast<BMI088_gyro *>(arg);

	/* make another measurement */
	dev->measure();
}

void
BMI088_gyro::check_registers(void)
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
			write_reg(BMI088_GYR_SOFTRESET, BMI088_SOFT_RESET);
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

	_checked_next = (_checked_next + 1) % BMI088_GYRO_NUM_CHECKED_REGISTERS;
}

void
BMI088_gyro::measure()
{
	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct BMI_GyroReport bmi_gyroreport;

	struct Report {
		int16_t     gyro_x;
		int16_t     gyro_y;
		int16_t     gyro_z;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	* Fetch the full set of measurements from the BMI088 gyro in one pass.
	*/
	bmi_gyroreport.cmd = BMI088_GYR_CHIP_ID | DIR_READ;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (OK != transfer((uint8_t *)&bmi_gyroreport, ((uint8_t *)&bmi_gyroreport), sizeof(bmi_gyroreport))) {
		return;
	}

	check_registers();

	// Get the last temperature from the accelerometer (the Gyro does not have its own temperature measurement)
	_last_temperature = _accel_last_temperature_copy;


	report.gyro_x = bmi_gyroreport.gyro_x;
	report.gyro_y = bmi_gyroreport.gyro_y;
	report.gyro_z = bmi_gyroreport.gyro_z;

	if ((bmi_gyroreport.chip_id) != BMI088_GYR_WHO_AM_I) {
		// If chip id is not right, assume a bus transfer error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the bmi088 does go bad it would cause a FMU failure,
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
	const uint64_t error_count  = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);
	_px4_gyro.set_error_count(error_count);

	// Get the temperature from the accelerometer part of the BMI088, because the gyro part does not have a temperature register
	_px4_gyro.set_temperature(_accel_last_temperature_copy);

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
	*   Example: A gyro outputs a value of 74 at zero angular rate
	*        the offset is 74 from the origin and subtracting
	*        74 from all measurements centers them around zero.
	*/
	_px4_gyro.update(timestamp_sample, report.gyro_x, report.gyro_y, report.gyro_z);

	/* stop measuring */
	perf_end(_sample_perf);
}

void
BMI088_gyro::print_info()
{
	PX4_INFO("Gyro");

	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);

	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < BMI088_GYRO_NUM_CHECKED_REGISTERS; i++) {
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

	_px4_gyro.print_status();
}

void
BMI088_gyro::print_registers()
{
	uint8_t index = 0;
	printf("BMI088 gyro registers\n");

	uint8_t reg = _checked_registers[index++];
	uint8_t v = read_reg(reg);
	printf("Gyro Chip Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Power: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Bw: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Range: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Int-en-0: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Int-en-1: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = read_reg(reg);
	printf("Gyro Int-Map-1: %02x:%02x ", (unsigned)reg, (unsigned)v);

	printf("\n");
}
