/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @file fxos8701cq.cpp
 * Driver for the NXP FXOS8701CQ 6-axis sensor with integrated linear accelerometer and
 * magnetometer connected via SPI.
 */

#include "FXOS8701CQ.hpp"

using namespace time_literals;

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t FXOS8701CQ::_checked_registers[FXOS8701C_NUM_CHECKED_REGISTERS] = {
	FXOS8701CQ_WHOAMI,
	FXOS8701CQ_XYZ_DATA_CFG,
	FXOS8701CQ_CTRL_REG1,
	FXOS8701CQ_M_CTRL_REG1,
	FXOS8701CQ_M_CTRL_REG2,
};

FXOS8701CQ::FXOS8701CQ(device::Device *interface, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_interface(interface),
	_px4_accel(interface->get_device_id(), config.rotation),
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	_px4_mag(interface->get_device_id(), config.rotation),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": mag read")),
#endif
	_accel_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": acc read")),
	_bad_registers(perf_alloc(PC_COUNT, MODULE_NAME": bad reg")),
	_accel_duplicates(perf_alloc(PC_COUNT, MODULE_NAME": acc dupe"))
{
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	_px4_mag.set_scale(0.001f);
#endif
}

FXOS8701CQ::~FXOS8701CQ()
{
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	perf_free(_mag_sample_perf);
#endif

	// delete the perf counter
	perf_free(_accel_sample_perf);
	perf_free(_bad_registers);
	perf_free(_accel_duplicates);
}

int
FXOS8701CQ::init()
{
	// do SPI/I2C init (and probe) first
	int ret = _interface->init();

	if (ret != OK) {
		PX4_ERR("SPI/I2C interface init failed");
		return ret;
	}

	// There are 2 possible WHOAMI return values,
	// so probe here again to set proper _checked_values[0]
	ret = probe();

	if (ret != OK) {
		PX4_ERR("FXOS8701CQ::probe() failed");
		return ret;
	}

	reset();

	start();

	return PX4_OK;
}

void
FXOS8701CQ::reset()
{
	// enable accel set it To Standby
	write_checked_reg(FXOS8701CQ_CTRL_REG1, 0);
	write_checked_reg(FXOS8701CQ_XYZ_DATA_CFG, 0);

	// Use hybird mode to read Accel and Mag
	write_checked_reg(FXOS8701CQ_M_CTRL_REG1, M_CTRL_REG1_HMS_AM | M_CTRL_REG1_OS(7));

	// Use the hybird auto increment mode  to read all the data at the same time
	write_checked_reg(FXOS8701CQ_M_CTRL_REG2, CTRL_REG2_AUTO_INC);

	accel_set_range(FXOS8701C_ACCEL_DEFAULT_RANGE_G);
	accel_set_samplerate(FXOS8701C_ACCEL_DEFAULT_RATE);

	// enable  set it To Standby mode at 800 Hz which becomes 400 Hz due to hybird mode
	write_checked_reg(FXOS8701CQ_CTRL_REG1, CTRL_REG1_DR(0) | CTRL_REG1_ACTIVE);
}

int
FXOS8701CQ::probe()
{
	// verify that the device is attached and functioning
	uint8_t whoami = read_reg(FXOS8701CQ_WHOAMI);
	bool success = (whoami == FXOS8700CQ_WHOAMI_VAL) || (whoami == FXOS8701CQ_WHOAMI_VAL);

	if (success) {
		_checked_values[0] = whoami;
		return OK;
	}

	return -EIO;
}

void
FXOS8701CQ::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < FXOS8701C_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

void
FXOS8701CQ::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
FXOS8701CQ::accel_set_range(unsigned max_g)
{
	uint8_t setbits = 0;
	float lsb_per_g;

	if (max_g == 0 || max_g > 8) {
		max_g = 8;
	}

	if (max_g > 4) { //  8g
		setbits = XYZ_DATA_CFG_FS_8G;
		lsb_per_g = 1024;
		//max_accel_g = 8;

	} else if (max_g > 2) { //  4g
		setbits = XYZ_DATA_CFG_FS_4G;
		lsb_per_g = 2048;
		//max_accel_g = 4;

	} else {                //  2g
		setbits = XYZ_DATA_CFG_FS_2G;
		lsb_per_g = 4096;
		//max_accel_g = 2;
	}

	float accel_range_scale = (CONSTANTS_ONE_G / lsb_per_g);

	modify_reg(FXOS8701CQ_XYZ_DATA_CFG, XYZ_DATA_CFG_FS_MASK, setbits);

	_px4_accel.set_scale(accel_range_scale);

	return OK;
}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
int
FXOS8701CQ::mag_set_range(unsigned max_ga)
{
	// mag_range_ga = 12;
	float mag_range_scale = 0.001f;

	_px4_mag.set_scale(mag_range_scale);

	return OK;
}
#endif

int
FXOS8701CQ::accel_set_samplerate(unsigned frequency)
{
	uint8_t setbits = 0;

	// The selected ODR is reduced by a factor of two when the device is operated in hybrid mode.
	uint8_t active = read_reg(FXOS8701CQ_CTRL_REG1) & CTRL_REG1_ACTIVE;

	if (frequency == 0) {
		frequency = FXOS8701C_ACCEL_DEFAULT_RATE;
	}

	if (frequency <= 25) {
		setbits = CTRL_REG1_DR(4); // Use 50 as it is 50 / 2
		_accel_samplerate = 25;

	} else if (frequency <= 50) {
		setbits = CTRL_REG1_DR(3); // Use 100 as it is 100 / 2
		_accel_samplerate = 50;

	} else if (frequency <= 100) {
		setbits = CTRL_REG1_DR(2); // Use 200 as it is 200 / 2
		_accel_samplerate = 100;

	} else if (frequency <= 200) {
		setbits = CTRL_REG1_DR(1); // Use 400 as it is 400 / 2;
		_accel_samplerate = 200;

	} else if (frequency <= 400) {
		setbits = CTRL_REG1_DR(0); // Use 800 as it is 800 / 2;
		_accel_samplerate = 400;

	} else {
		return -EINVAL;
	}

	modify_reg(FXOS8701CQ_CTRL_REG1, CTRL_REG1_ACTIVE, 0);
	modify_reg(FXOS8701CQ_CTRL_REG1, CTRL_REG1_DR_MASK, setbits);
	modify_reg(FXOS8701CQ_CTRL_REG1, 0, active);

	return OK;
}

void FXOS8701CQ::start()
{
	// start polling at the specified rate
	ScheduleOnInterval((1_s / FXOS8701C_ACCEL_DEFAULT_RATE) / 2);
}

void FXOS8701CQ::check_registers()
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

	_checked_next = (_checked_next + 1) % FXOS8701C_NUM_CHECKED_REGISTERS;
}

void FXOS8701CQ::RunImpl()
{
	// start the performance counter
	perf_begin(_accel_sample_perf);

	check_registers();

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again.
		_register_wait--;
		perf_end(_accel_sample_perf);
		return;
	}

	/* fetch data from the sensor */
	RawAccelMagReport raw_accel_mag_report{};
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	_interface->read(FXOS8701CQ_DR_STATUS, (uint8_t *)&raw_accel_mag_report, sizeof(raw_accel_mag_report));

	if (!(raw_accel_mag_report.status & DR_STATUS_ZYXDR)) {
		perf_end(_accel_sample_perf);
		perf_count(_accel_duplicates);
		return;
	}

	int16_t x = swap16RightJustify14(raw_accel_mag_report.x);
	int16_t y = swap16RightJustify14(raw_accel_mag_report.y);
	int16_t z = swap16RightJustify14(raw_accel_mag_report.z);

	// don't publish duplicated reads
	if ((x == _accel_prev[0]) && (y == _accel_prev[1]) && (z == _accel_prev[2])) {
		perf_count(_accel_duplicates);
		perf_end(_accel_sample_perf);
		return;

	} else {
		_accel_prev[0] = x;
		_accel_prev[1] = y;
		_accel_prev[2] = z;
	}

	// report the error count as the sum of the number of bad register reads and bad values.
	_px4_accel.set_error_count(perf_event_count(_bad_registers));
	_px4_accel.update(timestamp_sample, x, y, z);

	if ((timestamp_sample - _last_temperature_update) > 100_ms) {
		/*
		 * Eight-bit 2’s complement sensor temperature value with 0.96 °C/LSB sensitivity.
		 * Temperature data is only valid between –40 °C and 125 °C. The temperature sensor
		 * output is only valid when M_CTRL_REG1[m_hms] > 0b00. Please note that the
		 * temperature sensor is uncalibrated and its output for a given temperature will vary from
		 * one device to the next
		 */
		_last_temperature_update = timestamp_sample;
		float temperature = (read_reg(FXOS8701CQ_TEMP)) * 0.96f;

		_px4_accel.set_temperature(temperature);
	}


#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)

	if ((timestamp_sample - _mag_last_measure) >= 10_ms) {
		int16_t mag_x = swap16(raw_accel_mag_report.mx);
		int16_t mag_y = swap16(raw_accel_mag_report.my);
		int16_t mag_z = swap16(raw_accel_mag_report.mz);
		_px4_mag.update(timestamp_sample, mag_x, mag_y, mag_z);

		_mag_last_measure = timestamp_sample;
	}

#endif

	// stop the perf counter
	perf_end(_accel_sample_perf);
}

void
FXOS8701CQ::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_accel_sample_perf);

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	perf_print_counter(_mag_sample_perf);
#endif
	perf_print_counter(_bad_registers);
	perf_print_counter(_accel_duplicates);

	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < FXOS8701C_NUM_CHECKED_REGISTERS; i++) {
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
FXOS8701CQ::print_registers()
{
	const struct {
		uint8_t reg;
		const char *name;
	} regmap[] = {
		DEF_REG(FXOS8701CQ_DR_STATUS),
		DEF_REG(FXOS8701CQ_OUT_X_MSB),
		DEF_REG(FXOS8701CQ_XYZ_DATA_CFG),
		DEF_REG(FXOS8701CQ_WHOAMI),
		DEF_REG(FXOS8701CQ_CTRL_REG1),
		DEF_REG(FXOS8701CQ_CTRL_REG2),
		DEF_REG(FXOS8701CQ_M_DR_STATUS),
		DEF_REG(FXOS8701CQ_M_OUT_X_MSB),
		DEF_REG(FXOS8701CQ_M_CTRL_REG1),
		DEF_REG(FXOS8701CQ_M_CTRL_REG2),
		DEF_REG(FXOS8701CQ_M_CTRL_REG3),
	};

	for (uint8_t i = 0; i < sizeof(regmap) / sizeof(regmap[0]); i++) {
		printf("0x%02x %s\n", read_reg(regmap[i].reg), regmap[i].name);
	}
}

void
FXOS8701CQ::test_error()
{
	// trigger an error
	write_reg(FXOS8701CQ_CTRL_REG1, 0);
}
