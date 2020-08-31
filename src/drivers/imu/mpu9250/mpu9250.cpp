/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file mpu9250.cpp
 *
 * Driver for the Invensense MPU9250 connected via I2C or SPI.
 *
 * @author Andrew Tridgell
 *
 * based on the mpu6000 driver
 */

#include "mpu9250.h"

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates by comparing
  accelerometer values. This time reduction is enough to cope with
  worst case timing jitter due to other timers
 */
#define MPU9250_TIMER_REDUCTION				200

/* Set accel range used */
#define ACCEL_RANGE_G  16
/*
  list of registers that will be checked in check_registers(). Note
  that MPUREG_PRODUCT_ID must be first in the list.
 */
const uint16_t MPU9250::_mpu9250_checked_registers[MPU9250_NUM_CHECKED_REGISTERS] = { MPUREG_WHOAMI,
										      MPUREG_PWR_MGMT_1,
										      MPUREG_PWR_MGMT_2,
										      MPUREG_USER_CTRL,
										      MPUREG_SMPLRT_DIV,
										      MPUREG_CONFIG,
										      MPUREG_GYRO_CONFIG,
										      MPUREG_ACCEL_CONFIG,
										      MPUREG_ACCEL_CONFIG2,
										      MPUREG_INT_ENABLE,
										      MPUREG_INT_PIN_CFG
										    };

MPU9250::MPU9250(device::Device *interface, device::Device *mag_interface, enum Rotation rotation,
		 I2CSPIBusOption bus_option, int bus) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_interface(interface),
	_px4_accel(_interface->get_device_id(), rotation),
	_px4_gyro(_interface->get_device_id(), rotation),
	_mag(this, mag_interface, rotation),
	_dlpf_freq(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_bad_registers(perf_alloc(PC_COUNT, MODULE_NAME": bad_reg")),
	_duplicates(perf_alloc(PC_COUNT, MODULE_NAME": dupe"))
{
}

MPU9250::~MPU9250()
{
	// delete the perf counter
	perf_free(_sample_perf);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
MPU9250::init()
{
	/*
	 * If the MPU is using I2C we should reduce the sample rate to 200Hz and
	 * make the integration autoreset faster so that we integrate just one
	 * sample since the sampling rate is already low.
	*/
	const bool is_i2c = (_interface->get_device_bus_type() == device::Device::DeviceBusType_I2C);

	if (is_i2c) {
		_sample_rate = 200;
	}

	int ret = probe();

	if (ret != OK) {
		PX4_DEBUG("probe failed");
		return ret;
	}

	_reset_wait = hrt_absolute_time() + 100000;

	if (reset_mpu() != OK) {
		PX4_ERR("Exiting! Device failed to take initialization");
		return PX4_ERROR;
	}

	/* Magnetometer setup */
	if (_whoami == MPU_WHOAMI_9250) {

#ifdef USE_I2C
		px4_usleep(100);

		if (!_mag.is_passthrough() && _mag._interface->init() != PX4_OK) {
			PX4_ERR("failed to setup ak8963 interface");
		}

#endif /* USE_I2C */

		ret = _mag.ak8963_reset();

		if (ret != OK) {
			PX4_DEBUG("mag reset failed");
			return ret;
		}
	}

	start();

	return ret;
}

int
MPU9250::reset()
{
	/* When the mpu9250 starts from 0V the internal power on circuit
	 * per the data sheet will require:
	 *
	 * Start-up time for register read/write From power-up Typ:11 max:100 ms
	 *
	 */
	px4_usleep(110000);

	// Hold off sampling until done (100 MS will be shortened)
	_reset_wait = hrt_absolute_time() + 100000;

	int ret = reset_mpu();

	if (ret == OK && (_whoami == MPU_WHOAMI_9250)) {
		ret = _mag.ak8963_reset();
	}

	_reset_wait = hrt_absolute_time() + 10;

	return ret;
}

int
MPU9250::reset_mpu()
{
	switch (_whoami) {
	case MPU_WHOAMI_9250:
	case MPU_WHOAMI_6500:
		write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
		write_checked_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_AUTO);
		write_checked_reg(MPUREG_PWR_MGMT_2, 0);
		px4_usleep(1000);
		break;
	}

	// Enable I2C bus or Disable I2C bus (recommended on data sheet)
	const bool is_i2c = (_interface->get_device_bus_type() == device::Device::DeviceBusType_I2C);
	write_checked_reg(MPUREG_USER_CTRL, is_i2c ? 0 : BIT_I2C_IF_DIS);

	// SAMPLE RATE
	_set_sample_rate(_sample_rate);

	_set_dlpf_filter(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ);

	// Gyro scale 2000 deg/s ()
	switch (_whoami) {
	case MPU_WHOAMI_9250:
	case MPU_WHOAMI_6500:
		write_checked_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
		break;
	}

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_px4_gyro.set_scale(0.0174532 / 16.4); //1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);

	set_accel_range(ACCEL_RANGE_G);

	// INT CFG => Interrupt on Data Ready
	write_checked_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready

#ifdef USE_I2C
	bool bypass = !_mag.is_passthrough();
#else
	bool bypass = false;
#endif

	/* INT: Clear on any read.
	 * If this instance is for a device is on I2C bus the Mag will have an i2c interface
	 * that it will use to access the either: a) the internal mag device on the internal I2C bus
	 * or b) it could be used to access a downstream I2C devices connected to the chip on
	 * it's AUX_{ASD|SCL} pins. In either case we need to disconnect (bypass) the internal master
	 * controller that chip provides as a SPI to I2C bridge.
	 * so bypass is true if the mag has an i2c non null interfaces.
	 */

	write_checked_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR | (bypass ? BIT_INT_BYPASS_EN : 0));

	write_checked_reg(MPUREG_ACCEL_CONFIG2, BITS_ACCEL_CONFIG2_41HZ);

	uint8_t retries = 3;
	bool all_ok = false;

	while (!all_ok && retries--) {

		// Assume all checked values are as expected
		all_ok = true;
		uint8_t reg = 0;

		for (uint8_t i = 0; i < _num_checked_registers; i++) {
			if ((reg = read_reg(_checked_registers[i])) != _checked_values[i]) {

				write_reg(_checked_registers[i], _checked_values[i]);
				PX4_ERR("Reg %d is:%d s/b:%d Tries:%d", _checked_registers[i], reg, _checked_values[i], retries);
				all_ok = false;
			}
		}
	}

	return all_ok ? OK : -EIO;
}

int
MPU9250::probe()
{
	int ret = PX4_ERROR;

	// Try first for mpu9250/6500
	_whoami = read_reg(MPUREG_WHOAMI);

	if (_whoami == MPU_WHOAMI_9250 || _whoami == MPU_WHOAMI_6500) {

		_num_checked_registers = MPU9250_NUM_CHECKED_REGISTERS;
		_checked_registers = _mpu9250_checked_registers;
		memset(_checked_values, 0, MPU9250_NUM_CHECKED_REGISTERS);
		ret = PX4_OK;
	}

	_checked_values[0] = _whoami;

	if (ret != PX4_OK) {
		PX4_DEBUG("unexpected whoami 0x%02x", _whoami);
	}

	return ret;
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
MPU9250::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	uint8_t div = 1;

	if (desired_sample_rate_hz == 0) {
		desired_sample_rate_hz = MPU9250_GYRO_DEFAULT_RATE;
	}

	switch (_whoami) {
	case MPU_WHOAMI_9250:
	case MPU_WHOAMI_6500:
		div = 1000 / desired_sample_rate_hz;
		break;
	}

	if (div > 200) { div = 200; }

	if (div < 1) { div = 1; }


	switch (_whoami) {
	case MPU_WHOAMI_9250:
	case MPU_WHOAMI_6500:
		write_checked_reg(MPUREG_SMPLRT_DIV, div - 1);
		_sample_rate = 1000 / div;
		break;
	}
}

/*
  set the DLPF filter frequency. This affects both accel and gyro.
 */
void
MPU9250::_set_dlpf_filter(uint16_t frequency_hz)
{
	uint8_t filter;

	switch (_whoami) {
	case MPU_WHOAMI_9250:
	case MPU_WHOAMI_6500:

		/*
		   choose next highest filter frequency available
		 */
		if (frequency_hz == 0) {
			_dlpf_freq = 0;
			filter = BITS_DLPF_CFG_3600HZ;

		} else if (frequency_hz <= 5) {
			_dlpf_freq = 5;
			filter = BITS_DLPF_CFG_5HZ;

		} else if (frequency_hz <= 10) {
			_dlpf_freq = 10;
			filter = BITS_DLPF_CFG_10HZ;

		} else if (frequency_hz <= 20) {
			_dlpf_freq = 20;
			filter = BITS_DLPF_CFG_20HZ;

		} else if (frequency_hz <= 41) {
			_dlpf_freq = 41;
			filter = BITS_DLPF_CFG_41HZ;

		} else if (frequency_hz <= 92) {
			_dlpf_freq = 92;
			filter = BITS_DLPF_CFG_92HZ;

		} else if (frequency_hz <= 184) {
			_dlpf_freq = 184;
			filter = BITS_DLPF_CFG_184HZ;

		} else if (frequency_hz <= 250) {
			_dlpf_freq = 250;
			filter = BITS_DLPF_CFG_250HZ;

		} else {
			_dlpf_freq = 0;
			filter = BITS_DLPF_CFG_3600HZ;
		}

		write_checked_reg(MPUREG_CONFIG, filter);
		break;
	}
}

uint8_t
MPU9250::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t buf{};

	_interface->read(MPU9250_SET_SPEED(reg, speed), &buf, 1);

	return buf;
}

uint8_t
MPU9250::read_reg_range(unsigned start_reg, uint32_t speed, uint8_t *buf, uint16_t count)
{
	if (buf == NULL) {
		return PX4_ERROR;
	}

	return _interface->read(MPU9250_SET_SPEED(start_reg, speed), buf, count);
}

void
MPU9250::write_reg(unsigned reg, uint8_t value)
{
	// general register transfer at low clock speed
	_interface->write(MPU9250_LOW_SPEED_OP(reg), &value, 1);
}

void
MPU9250::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
MPU9250::modify_checked_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
MPU9250::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < _num_checked_registers; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			break;
		}
	}
}

int
MPU9250::set_accel_range(unsigned max_g_in)
{
	uint8_t afs_sel;
	float lsb_per_g;
	//float max_accel_g;

	if (max_g_in > 8) { // 16g - AFS_SEL = 3
		afs_sel = 3;
		lsb_per_g = 2048;
		//max_accel_g = 16;

	} else if (max_g_in > 4) { //  8g - AFS_SEL = 2
		afs_sel = 2;
		lsb_per_g = 4096;
		//max_accel_g = 8;

	} else if (max_g_in > 2) { //  4g - AFS_SEL = 1
		afs_sel = 1;
		lsb_per_g = 8192;
		//max_accel_g = 4;

	} else {                //  2g - AFS_SEL = 0
		afs_sel = 0;
		lsb_per_g = 16384;
		//max_accel_g = 2;
	}

	switch (_whoami) {
	case MPU_WHOAMI_9250:
	case MPU_WHOAMI_6500:
		write_checked_reg(MPUREG_ACCEL_CONFIG, afs_sel << 3);
		break;
	}

	_px4_accel.set_scale(CONSTANTS_ONE_G / lsb_per_g);

	return OK;
}

void
MPU9250::start()
{
	ScheduleOnInterval(_call_interval - MPU9250_TIMER_REDUCTION, 1000);
}

void
MPU9250::RunImpl()
{
	/* make another measurement */
	measure();
}

void
MPU9250::check_registers()
{
	/*
	  we read the register at full speed, even though it isn't
	  listed as a high speed register. The low speed requirement
	  for some registers seems to be a propagation delay
	  requirement for changing sensor configuration, which should
	  not apply to reading a single register. It is also a better
	  test of SPI bus health to read at the same speed as we read
	  the data registers.
	*/
	uint8_t v = 0;

	if ((v = read_reg(_checked_registers[_checked_next], MPU9250_HIGH_BUS_SPEED)) != _checked_values[_checked_next]) {

		PX4_DEBUG("reg: %d = %d (should be %d) _reset_wait: %lu", _checked_registers[_checked_next], v,
			  _checked_values[_checked_next], _reset_wait);

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
			// if the product_id is wrong then reset the sensor completely
			write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
			write_reg(MPUREG_PWR_MGMT_2, MPU_CLK_SEL_AUTO);

			// after doing a reset we need to wait a long
			// time before we do any other register writes
			// or we will end up with the mpu9250 in a
			// bizarre state where it has all correct
			// register values but large offsets on the
			// accel axes
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

	_checked_next = (_checked_next + 1) % _num_checked_registers;
}

bool
MPU9250::check_duplicate(uint8_t *accel_data)
{
	/*
	   see if this is duplicate accelerometer data. Note that we
	   can't use the data ready interrupt status bit in the status
	   register as that also goes high on new gyro data, and when
	   we run with BITS_DLPF_CFG_256HZ_NOLPF2 the gyro is being
	   sampled at 8kHz, so we would incorrectly think we have new
	   data when we are in fact getting duplicate accelerometer data.
	*/
	if (!_got_duplicate && memcmp(accel_data, &_last_accel_data, sizeof(_last_accel_data)) == 0) {
		// it isn't new data - wait for next timer
		_got_duplicate = true;

	} else {
		memcpy(&_last_accel_data, accel_data, sizeof(_last_accel_data));
		_got_duplicate = false;
	}

	return _got_duplicate;
}

void
MPU9250::measure()
{
	perf_begin(_sample_perf);

	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		perf_end(_sample_perf);
		return;
	}

	MPUReport mpu_report{};

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// Fetch the full set of measurements from the ICM20948 in one pass
	if (_mag.is_passthrough() && _register_wait == 0) {
		if (OK != read_reg_range(MPUREG_ACCEL_XOUT_H, MPU9250_HIGH_BUS_SPEED, (uint8_t *)&mpu_report, sizeof(mpu_report))) {
			perf_end(_sample_perf);
			return;
		}

		check_registers();

		if (check_duplicate(&mpu_report.ACCEL_XOUT_H)) {
			perf_end(_sample_perf);
			perf_count(_duplicates);
			return;
		}
	}

	/*
	 * In case of a mag passthrough read, hand the magnetometer data over to _mag. Else,
	 * try to read a magnetometer report.
	 */

#   ifdef USE_I2C

	if (_mag.is_passthrough()) {
#   endif

		if (_register_wait == 0) {
			_mag._measure(timestamp_sample, mpu_report.mag);
		}

#   ifdef USE_I2C

	} else {
		_mag.measure();
	}

#   endif

	if (_register_wait != 0) {
		// We are waiting for some good transfers before using the sensor again
		_register_wait--;

		perf_end(_sample_perf);
		return;
	}

	// Convert from big to little endian
	int16_t accel_x = combine(mpu_report.ACCEL_XOUT_H, mpu_report.ACCEL_XOUT_L);
	int16_t accel_y = combine(mpu_report.ACCEL_YOUT_H, mpu_report.ACCEL_YOUT_L);
	int16_t accel_z = combine(mpu_report.ACCEL_ZOUT_H, mpu_report.ACCEL_ZOUT_L);
	int16_t temp    = combine(mpu_report.TEMP_OUT_H, mpu_report.TEMP_OUT_L);
	int16_t gyro_x  = combine(mpu_report.GYRO_XOUT_H, mpu_report.GYRO_XOUT_L);
	int16_t gyro_y  = combine(mpu_report.GYRO_YOUT_H, mpu_report.GYRO_YOUT_L);
	int16_t gyro_z  = combine(mpu_report.GYRO_ZOUT_H, mpu_report.GYRO_ZOUT_L);

	// Get sensor temperature
	_last_temperature = temp / 333.87f + 21.0f;

	_px4_accel.set_temperature(_last_temperature);
	_px4_gyro.set_temperature(_last_temperature);

	// Swap axes and negate y
	int16_t accel_xt = accel_y;
	int16_t accel_yt = ((accel_x == -32768) ? 32767 : -accel_x);

	int16_t gyro_xt = gyro_y;
	int16_t gyro_yt = ((gyro_x == -32768) ? 32767 : -gyro_x);

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	const uint64_t error_count = perf_event_count(_bad_registers);
	_px4_accel.set_error_count(error_count);
	_px4_gyro.set_error_count(error_count);

	/* NOTE: Axes have been swapped to match the board a few lines above. */
	_px4_accel.update(timestamp_sample, accel_xt, accel_yt, accel_z);
	_px4_gyro.update(timestamp_sample, gyro_xt, gyro_yt, gyro_z);

	/* stop measuring */
	perf_end(_sample_perf);
}

void
MPU9250::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);
}
