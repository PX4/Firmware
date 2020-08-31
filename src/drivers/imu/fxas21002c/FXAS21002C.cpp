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

#include "FXAS21002C.hpp"

#define DEF_REG(r)   {r, #r}

/* default values for this device */
#define FXAS21002C_MAX_RATE              800
#define FXAS21002C_DEFAULT_RATE          FXAS21002C_MAX_RATE
#define FXAS21002C_DEFAULT_RANGE_DPS     2000
#define FXAS21002C_DEFAULT_ONCHIP_FILTER_FREQ 	64 // ODR dependant

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
  Typical reductions for the MPU6000 is 20% so 20% of 1/800 is 250 us
 */
#define FXAS21002C_TIMER_REDUCTION				250

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
static constexpr uint8_t _checked_registers[] {
	FXAS21002C_WHO_AM_I,
	FXAS21002C_F_SETUP,
	FXAS21002C_CTRL_REG0,
	FXAS21002C_CTRL_REG1,
	FXAS21002C_CTRL_REG2,
	FXAS21002C_CTRL_REG3,
};

using namespace time_literals;

FXAS21002C::FXAS21002C(device::Device *interface, I2CSPIBusOption bus_option, int bus, enum Rotation rotation,
		       int i2c_address) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus, i2c_address),
	_interface(interface),
	_px4_gyro(_interface->get_device_id(), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_errors(perf_alloc(PC_COUNT, MODULE_NAME": err")),
	_bad_registers(perf_alloc(PC_COUNT, MODULE_NAME": bad register")),
	_duplicates(perf_alloc(PC_COUNT, MODULE_NAME": duplicate reading"))
{
}

FXAS21002C::~FXAS21002C()
{
	perf_free(_sample_perf);
	perf_free(_errors);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
FXAS21002C::init()
{
	/* do SPI/I2C init (and probe) first */
	if (_interface->init() != OK) {
		PX4_ERR("SPI/I2C interface init failed");
		return PX4_ERROR;
	}

	// passed SPI::probe or I2C::probe, which checked WHO_AM_I
	// measurements will not start before registers are checked OK
	_checked_values[0] = WHO_AM_I;

	reset();

	start();

	return PX4_OK;
}

void FXAS21002C::reset()
{
	/* write 0 0 0 000 00 = 0x00 to CTRL_REG1 to place FXOS21002 in Standby
	 * [6]: RST=0
	 * [5]: ST=0 self test disabled
	 * [4-2]: DR[2-0]=000 for 200Hz ODR
	 * [1-0]: Active=0, Ready=0 for Standby mode
	 */
	write_reg(FXAS21002C_CTRL_REG1, 0);

	/* write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
	 * [7-6]: BW[1-0]=00, LPF 64 @ 800Hz ODR
	 *  [5]: SPIW=0 4 wire SPI
	 * [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
	 *  [2]: HPF_EN=0 disable HPF
	 * [1-0]: FS[1-0]=00 for 1600dps (TBD CHANGE TO 2000dps when final trimmed parts available)
	 */
	write_checked_reg(FXAS21002C_CTRL_REG0, CTRL_REG0_BW_LOW | CTRL_REG0_FS_2000_DPS);

	/* write CTRL_REG1 to configure 800Hz ODR and enter Active mode */
	write_checked_reg(FXAS21002C_CTRL_REG1, CTRL_REG1_DR_800HZ | CTRL_REG1_ACTIVE);

	/* Set the default */
	set_samplerate(FXAS21002C_DEFAULT_RATE);
	set_range(FXAS21002C_DEFAULT_RANGE_DPS);
	set_onchip_lowpass_filter(FXAS21002C_DEFAULT_ONCHIP_FILTER_FREQ);
}

void FXAS21002C::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < FXAS21002C_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

void FXAS21002C::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int FXAS21002C::set_range(unsigned max_dps)
{
	uint8_t bits = CTRL_REG0_FS_250_DPS;
	float new_range_scale_dps_digit;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 250) {
		//new_range = 250;
		new_range_scale_dps_digit = 7.8125e-3f;
		bits = CTRL_REG0_FS_250_DPS;

	} else if (max_dps <= 500) {
		//new_range = 500;
		new_range_scale_dps_digit = 15.625e-3f;
		bits = CTRL_REG0_FS_500_DPS;

	} else if (max_dps <= 1000) {
		//new_range = 1000;
		new_range_scale_dps_digit = 31.25e-3f;
		bits = CTRL_REG0_FS_1000_DPS;

	} else if (max_dps <= 2000) {
		//new_range = 2000;
		new_range_scale_dps_digit = 62.5e-3f;
		bits = CTRL_REG0_FS_2000_DPS;

	} else {
		return -EINVAL;
	}

	set_standby(_current_rate, true);

	_px4_gyro.set_scale(new_range_scale_dps_digit / 180.0f * M_PI_F);

	modify_reg(FXAS21002C_CTRL_REG0, CTRL_REG0_FS_MASK, bits);
	set_standby(_current_rate, false);

	return OK;
}

void FXAS21002C::set_standby(int rate, bool standby_true)
{
	uint8_t c = 0;
	uint8_t s = 0;

	if (standby_true) {
		c = CTRL_REG1_ACTIVE | CTRL_REG1_READY;

	} else {
		s = CTRL_REG1_ACTIVE | CTRL_REG1_READY;
	}

	modify_reg(FXAS21002C_CTRL_REG1, c, s);

	// From the data sheet

	int wait_ms = (1000 / rate) + 60 + 1;

	usleep(wait_ms * 1000);
}

int FXAS21002C::set_samplerate(unsigned frequency)
{
	uint8_t bits = 0;

	unsigned last_rate = _current_rate;

	if (frequency == 0) {
		frequency = FXAS21002C_DEFAULT_RATE;
	}

	if (frequency <= 13) {
		_current_rate = 13;
		bits = CTRL_REG1_DR_12_5;

	} else if (frequency <= 25) {
		_current_rate = 25;
		bits = CTRL_REG1_DR_25HZ;

	} else if (frequency <= 50) {
		_current_rate = 50;
		bits = CTRL_REG1_DR_50HZ;

	} else if (frequency <= 100) {
		_current_rate = 100;
		bits = CTRL_REG1_DR_100HZ;

	} else if (frequency <= 200) {
		_current_rate = 200;
		bits = CTRL_REG1_DR_200HZ;

	} else if (frequency <= 400) {
		_current_rate = 400;
		bits = CTRL_REG1_DR_400HZ;

	} else if (frequency <= 800) {
		_current_rate = 800;
		bits = CTRL_REG1_DR_800HZ;

	} else {
		return -EINVAL;
	}

	set_standby(last_rate, true);
	modify_reg(FXAS21002C_CTRL_REG1, CTRL_REG1_DR_MASK, bits);
	set_standby(_current_rate, false);

	return OK;
}

void FXAS21002C::set_onchip_lowpass_filter(int frequency_hz)
{
	int high = 256 / (800 / _current_rate);
	int med = high / 2 ;
	int low = med / 2;

	if (_current_rate <= 25) {
		low = -1;
	}

	if (_current_rate == 13) {
		med = -1;
		low = -1;
	}

	uint8_t filter = CTRL_REG0_BW_HIGH;

	if (frequency_hz == 0) {
		filter = CTRL_REG0_BW_HIGH;

	} else if (frequency_hz <= low) {
		filter = CTRL_REG0_BW_LOW;

	} else if (frequency_hz <= med) {
		filter = CTRL_REG0_BW_MED;

	} else if (frequency_hz <= high) {
		filter = CTRL_REG0_BW_HIGH;
	}

	set_standby(_current_rate, true);
	modify_reg(FXAS21002C_CTRL_REG1, CTRL_REG0_BW_MASK, filter);
	set_standby(_current_rate, false);
}

void FXAS21002C::start()
{
	/* start polling at the specified rate */
	ScheduleOnInterval((1_s / FXAS21002C_DEFAULT_RATE) - FXAS21002C_TIMER_REDUCTION);
}

void FXAS21002C::check_registers()
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

	_checked_next = (_checked_next + 1) % FXAS21002C_NUM_CHECKED_REGISTERS;
}

void FXAS21002C::RunImpl()
{
	// start the performance counter
	perf_begin(_sample_perf);

	check_registers();

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again.
		_register_wait--;
		perf_end(_sample_perf);
		return;
	}

	/* fetch data from the sensor */
	RawGyroReport raw_gyro_report{};
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	_interface->read(FXAS21002C_STATUS, (uint8_t *)&raw_gyro_report, sizeof(raw_gyro_report));

	if (!(raw_gyro_report.status & DR_STATUS_ZYXDR)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		return;
	}

	int16_t x_raw = swap16(raw_gyro_report.x);
	int16_t y_raw = swap16(raw_gyro_report.y);
	int16_t z_raw = swap16(raw_gyro_report.z);

	// don't publish duplicated reads
	if ((x_raw == _gyro_prev[0]) && (y_raw == _gyro_prev[1]) && (z_raw == _gyro_prev[2])) {
		perf_count(_duplicates);
		perf_end(_sample_perf);
		return;

	} else {
		_gyro_prev[0] = x_raw;
		_gyro_prev[1] = y_raw;
		_gyro_prev[2] = z_raw;
	}

	// report the error count as the number of bad register reads. This allows the higher level
	_px4_gyro.set_error_count(perf_event_count(_bad_registers));
	_px4_gyro.update(timestamp_sample, x_raw, y_raw, z_raw);

	if (hrt_elapsed_time(&_last_temperature_update) > 100_ms) {
		/*
		 * The TEMP register contains an 8-bit 2's complement temperature value with a range
		 * of –128 °C to +127 °C and a scaling of 1 °C/LSB. The temperature data is only
		 * compensated (factory trim values applied) when the device is operating in the Active
		 * mode and actively measuring the angular rate.
		 */
		const float temperature = read_reg(FXAS21002C_TEMP) * 1.0f;
		_px4_gyro.set_temperature(temperature);
		_last_temperature_update = timestamp_sample;
	}

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void FXAS21002C::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_errors);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < FXAS21002C_NUM_CHECKED_REGISTERS; i++) {
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
FXAS21002C::print_registers()
{
	const struct {
		uint8_t reg;
		const char *name;
	} regmap[] = {
		DEF_REG(FXAS21002C_STATUS),
		DEF_REG(FXAS21002C_OUT_X_MSB),
		DEF_REG(FXAS21002C_OUT_X_LSB),
		DEF_REG(FXAS21002C_OUT_Y_MSB),
		DEF_REG(FXAS21002C_OUT_Y_LSB),
		DEF_REG(FXAS21002C_OUT_Z_MSB),
		DEF_REG(FXAS21002C_OUT_Z_LSB),
		DEF_REG(FXAS21002C_DR_STATUS),
		DEF_REG(FXAS21002C_F_STATUS),
		DEF_REG(FXAS21002C_F_SETUP),
		DEF_REG(FXAS21002C_F_EVENT),
		DEF_REG(FXAS21002C_INT_SRC_FLAG),
		DEF_REG(FXAS21002C_WHO_AM_I),
		DEF_REG(FXAS21002C_CTRL_REG0),
		DEF_REG(FXAS21002C_RT_CFG),
		DEF_REG(FXAS21002C_RT_SRC),
		DEF_REG(FXAS21002C_RT_THS),
		DEF_REG(FXAS21002C_RT_COUNT),
		DEF_REG(FXAS21002C_TEMP),
		DEF_REG(FXAS21002C_CTRL_REG1),
		DEF_REG(FXAS21002C_CTRL_REG2),
		DEF_REG(FXAS21002C_CTRL_REG3),
	};

	for (uint8_t i = 0; i < sizeof(regmap) / sizeof(regmap[0]); i++) {
		printf("0x%02x %d:%s\n", read_reg(regmap[i].reg), regmap[i].reg, regmap[i].name);
	}
}

void
FXAS21002C::test_error()
{
	// trigger an error
	write_reg(FXAS21002C_CTRL_REG1, 0);
}
