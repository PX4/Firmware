/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include "MPU6000.hpp"

/*
  list of registers that will be checked in check_registers(). Note
  that MPUREG_PRODUCT_ID must be first in the list.
 */
constexpr uint8_t MPU6000::_checked_registers[MPU6000_NUM_CHECKED_REGISTERS];

MPU6000::MPU6000(device::Device *interface, enum Rotation rotation, int device_type, I2CSPIBusOption bus_option,
		 int bus) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus, 0, device_type),
	_interface(interface),
	_device_type(device_type),
	_px4_accel(_interface->get_device_id(), rotation),
	_px4_gyro(_interface->get_device_id(), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpu6k_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "mpu6k_bad_trans")),
	_bad_registers(perf_alloc(PC_COUNT, "mpu6k_bad_reg")),
	_reset_retries(perf_alloc(PC_COUNT, "mpu6k_reset")),
	_duplicates(perf_alloc(PC_COUNT, "mpu6k_duplicates"))
{
	switch (_device_type) {
	default:
	case MPU_DEVICE_TYPE_MPU6000:
		_interface->set_device_type(DRV_IMU_DEVTYPE_MPU6000);
		_px4_accel.set_device_type(DRV_IMU_DEVTYPE_MPU6000);
		_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_MPU6000);
		break;

	case MPU_DEVICE_TYPE_ICM20602:
		_interface->set_device_type(DRV_IMU_DEVTYPE_ICM20602);
		_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ICM20602);
		_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ICM20602);
		break;

	case MPU_DEVICE_TYPE_ICM20608:
		_interface->set_device_type(DRV_IMU_DEVTYPE_ICM20608G);
		_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ICM20608G);
		_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ICM20608G);
		break;

	case MPU_DEVICE_TYPE_ICM20689:
		_interface->set_device_type(DRV_IMU_DEVTYPE_ICM20689);
		_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ICM20689);
		_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ICM20689);
		break;
	}
}

MPU6000::~MPU6000()
{
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_reset_retries);
	perf_free(_duplicates);
}

int
MPU6000::init()
{
	/* probe again to get our settings that are based on the device type */
	int ret = probe();

	/* if probe failed, bail now */
	if (ret != OK) {
		PX4_DEBUG("probe init failed");
		return ret;
	}

	if (reset() != OK) {
		return ret;
	}

	return ret;
}

int MPU6000::reset()
{
	// if the mpu6000 is initialized after the l3gd20 and lsm303d
	// then if we don't do an irqsave/irqrestore here the mpu6000
	// frequently comes up in a bad state where all transfers
	// come as zero
	uint8_t tries = 5;
	irqstate_t state;

	while (--tries != 0) {
		state = px4_enter_critical_section();

		// Hold off sampling for 60 ms
		_reset_wait = hrt_absolute_time() + 60000;

		write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
		up_udelay(10000);

		// Wake up device and select GyroZ clock. Note that the
		// MPU6000 starts up in sleep mode, and it can take some time
		// for it to come out of sleep
		write_checked_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
		up_udelay(1000);

		// Enable I2C bus or Disable I2C bus (recommended on data sheet)
		const bool is_i2c = (_interface->get_device_bus_type() == device::Device::DeviceBusType_I2C);
		write_checked_reg(MPUREG_USER_CTRL, is_i2c ? 0 : BIT_I2C_IF_DIS);

		px4_leave_critical_section(state);

		if (read_reg(MPUREG_PWR_MGMT_1) == MPU_CLK_SEL_PLLGYROZ) {
			break;
		}

		perf_count(_reset_retries);
		px4_usleep(2000);
	}

	// Hold off sampling for 30 ms

	state = px4_enter_critical_section();
	_reset_wait = hrt_absolute_time() + 30000;
	px4_leave_critical_section(state);

	if (read_reg(MPUREG_PWR_MGMT_1) != MPU_CLK_SEL_PLLGYROZ) {
		return -EIO;
	}

	px4_usleep(1000);

	// SAMPLE RATE
	_set_sample_rate(1000);
	px4_usleep(1000);

	_set_dlpf_filter(MPU6000_DEFAULT_ONCHIP_FILTER_FREQ);

	if (is_icm_device()) {
		_set_icm_acc_dlpf_filter(MPU6000_DEFAULT_ONCHIP_FILTER_FREQ);
	}

	px4_usleep(1000);
	// Gyro scale 2000 deg/s ()
	write_checked_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	px4_usleep(1000);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_px4_gyro.set_scale(0.0174532f / 16.4f);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);

	set_accel_range(MPU6000_ACCEL_DEFAULT_RANGE_G);

	px4_usleep(1000);

	// INT CFG => Interrupt on Data Ready
	write_checked_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready
	px4_usleep(1000);
	write_checked_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read
	px4_usleep(1000);

	if (is_icm_device()) {
		write_checked_reg(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE);
	}

	// Oscillator set
	// write_reg(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
	px4_usleep(1000);
	return OK;
}

int
MPU6000::probe()
{
	uint8_t whoami = read_reg(MPUREG_WHOAMI);
	uint8_t expected = 0;
	bool unknown_product_id = true;

	switch (_device_type) {

	default:
	case MPU_DEVICE_TYPE_MPU6000:
		expected = MPU_WHOAMI_6000;
		break;

	case MPU_DEVICE_TYPE_ICM20602:
		expected = ICM_WHOAMI_20602;
		break;

	case MPU_DEVICE_TYPE_ICM20608:
		expected = ICM_WHOAMI_20608;
		break;

	case MPU_DEVICE_TYPE_ICM20689:
		expected = ICM_WHOAMI_20689;
		break;
	}

	if (whoami != expected) {
		PX4_DEBUG("unexpected WHOAMI 0x%02x", whoami);
		return -EIO;
	}

	/* look for a product ID we recognize */
	_product = read_reg(MPUREG_PRODUCT_ID);

	// verify product revision
	switch (_product) {
	case MPU6000ES_REV_C4:
	case MPU6000ES_REV_C5:
	case MPU6000_REV_C4:
	case MPU6000_REV_C5:
	case MPU6000ES_REV_D6:
	case MPU6000ES_REV_D7:
	case MPU6000ES_REV_D8:
	case MPU6000_REV_D6:
	case MPU6000_REV_D7:
	case MPU6000_REV_D8:
	case MPU6000_REV_D9:
	case MPU6000_REV_D10:
	case ICM20608_REV_FF:
	case ICM20689_REV_FE:
	case ICM20689_REV_03:
	case ICM20689_REV_04:
	case ICM20602_REV_01:
	case ICM20602_REV_02:
	case MPU6050_REV_D8:
		unknown_product_id = false;
	}

	_checked_values[MPU6000_CHECKED_PRODUCT_ID_INDEX] = _product;

	PX4_DEBUG("ID 0x%02x", _product);

	if (unknown_product_id) {

		PX4_WARN("unexpected ID 0x%02x %s", _product, is_icm_device() ? "accepted" : "exiting!");

		if (is_mpu_device()) {
			return -EIO;
		}
	}

	return OK;
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
MPU6000::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	if (desired_sample_rate_hz == 0) {
		desired_sample_rate_hz = MPU6000_GYRO_DEFAULT_RATE;
	}

	uint8_t div = 1000 / desired_sample_rate_hz;

	if (div > 200) {
		div = 200;
	}

	if (div < 1) {
		div = 1;
	}

	write_checked_reg(MPUREG_SMPLRT_DIV, div - 1);
	_sample_rate = 1000 / div;
}

/*
  set the DLPF filter frequency. This affects both accel and gyro.
 */
void
MPU6000::_set_dlpf_filter(uint16_t frequency_hz)
{
	uint8_t filter = MPU_GYRO_DLPF_CFG_2100HZ_NOLPF;

	/*
	   choose next highest filter frequency available
	 */
	if (frequency_hz == 0) {
		filter = MPU_GYRO_DLPF_CFG_2100HZ_NOLPF;

	} else if (frequency_hz <= 5) {
		filter = MPU_GYRO_DLPF_CFG_5HZ;

	} else if (frequency_hz <= 10) {
		filter = MPU_GYRO_DLPF_CFG_10HZ;

	} else if (frequency_hz <= 20) {
		filter = MPU_GYRO_DLPF_CFG_20HZ;

	} else if (frequency_hz <= 42) {
		filter = MPU_GYRO_DLPF_CFG_42HZ;

	} else if (frequency_hz <= 98) {
		filter = MPU_GYRO_DLPF_CFG_98HZ;

	} else if (frequency_hz <= 188) {
		filter = MPU_GYRO_DLPF_CFG_188HZ;

	} else if (frequency_hz <= 256) {
		filter = MPU_GYRO_DLPF_CFG_256HZ_NOLPF2;
	}

	write_checked_reg(MPUREG_CONFIG, filter);
}

void
MPU6000::_set_icm_acc_dlpf_filter(uint16_t frequency_hz)
{
	uint8_t filter = ICM_ACC_DLPF_CFG_1046HZ_NOLPF;

	/*
	   choose next highest filter frequency available
	 */
	if (frequency_hz == 0) {
		filter = ICM_ACC_DLPF_CFG_1046HZ_NOLPF;

	} else if (frequency_hz <= 5) {
		filter = ICM_ACC_DLPF_CFG_5HZ;

	} else if (frequency_hz <= 10) {
		filter = ICM_ACC_DLPF_CFG_10HZ;

	} else if (frequency_hz <= 21) {
		filter = ICM_ACC_DLPF_CFG_21HZ;

	} else if (frequency_hz <= 44) {
		filter = ICM_ACC_DLPF_CFG_44HZ;

	} else if (frequency_hz <= 99) {
		filter = ICM_ACC_DLPF_CFG_99HZ;

	} else if (frequency_hz <= 218) {
		filter = ICM_ACC_DLPF_CFG_218HZ;

	} else if (frequency_hz <= 420) {
		filter = ICM_ACC_DLPF_CFG_420HZ;
	}

	write_checked_reg(ICMREG_ACCEL_CONFIG2, filter);
}

#ifndef CONSTRAINED_FLASH
/*
  perform a self-test comparison to factory trim values. This takes
  about 200ms and will return OK if the current values are within 14%
  of the expected values (as per datasheet)
 */
void
MPU6000::factory_self_test()
{
	_in_factory_test = true;
	uint8_t saved_gyro_config = read_reg(MPUREG_GYRO_CONFIG);
	uint8_t saved_accel_config = read_reg(MPUREG_ACCEL_CONFIG);
	const uint16_t repeats = 100;
	int ret = OK;

	// gyro self test has to be done at 250DPS
	write_reg(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);

	struct MPUReport mpu_report {};
	float accel_baseline[3] {};
	float gyro_baseline[3] {};
	float accel[3] {};
	float gyro[3] {};
	float accel_ftrim[3] {};
	float gyro_ftrim[3] {};

	// get baseline values without self-test enabled
	for (uint8_t i = 0; i < repeats; i++) {
		up_udelay(1000);
		_interface->read(MPU6000_SET_SPEED(MPUREG_INT_STATUS, MPU6000_HIGH_BUS_SPEED), (uint8_t *)&mpu_report,
				 sizeof(mpu_report));

		accel_baseline[0] += int16_t_from_bytes(mpu_report.accel_x);
		accel_baseline[1] += int16_t_from_bytes(mpu_report.accel_y);
		accel_baseline[2] += int16_t_from_bytes(mpu_report.accel_z);
		gyro_baseline[0] += int16_t_from_bytes(mpu_report.gyro_x);
		gyro_baseline[1] += int16_t_from_bytes(mpu_report.gyro_y);
		gyro_baseline[2] += int16_t_from_bytes(mpu_report.gyro_z);
	}

#if 1
	write_reg(MPUREG_GYRO_CONFIG,
		  BITS_FS_250DPS |
		  BITS_GYRO_ST_X |
		  BITS_GYRO_ST_Y |
		  BITS_GYRO_ST_Z);

	// accel 8g, self-test enabled all axes
	write_reg(MPUREG_ACCEL_CONFIG, saved_accel_config | 0xE0);
#endif

	up_udelay(20000);

	// get values with self-test enabled
	for (uint8_t i = 0; i < repeats; i++) {
		up_udelay(1000);
		_interface->read(MPU6000_SET_SPEED(MPUREG_INT_STATUS, MPU6000_HIGH_BUS_SPEED), (uint8_t *)&mpu_report,
				 sizeof(mpu_report));

		accel[0] += int16_t_from_bytes(mpu_report.accel_x);
		accel[1] += int16_t_from_bytes(mpu_report.accel_y);
		accel[2] += int16_t_from_bytes(mpu_report.accel_z);
		gyro[0] += int16_t_from_bytes(mpu_report.gyro_x);
		gyro[1] += int16_t_from_bytes(mpu_report.gyro_y);
		gyro[2] += int16_t_from_bytes(mpu_report.gyro_z);
	}

	for (uint8_t i = 0; i < 3; i++) {
		accel_baseline[i] /= repeats;
		gyro_baseline[i] /= repeats;
		accel[i] /= repeats;
		gyro[i] /= repeats;
	}

	// extract factory trim values
	uint8_t trims[4];
	trims[0] = read_reg(MPUREG_TRIM1);
	trims[1] = read_reg(MPUREG_TRIM2);
	trims[2] = read_reg(MPUREG_TRIM3);
	trims[3] = read_reg(MPUREG_TRIM4);
	uint8_t atrim[3];
	uint8_t gtrim[3];

	atrim[0] = ((trims[0] >> 3) & 0x1C) | ((trims[3] >> 4) & 0x03);
	atrim[1] = ((trims[1] >> 3) & 0x1C) | ((trims[3] >> 2) & 0x03);
	atrim[2] = ((trims[2] >> 3) & 0x1C) | ((trims[3] >> 0) & 0x03);
	gtrim[0] = trims[0] & 0x1F;
	gtrim[1] = trims[1] & 0x1F;
	gtrim[2] = trims[2] & 0x1F;

	// convert factory trims to right units
	for (uint8_t i = 0; i < 3; i++) {
		accel_ftrim[i] = 4096 * 0.34f * powf(0.92f / 0.34f, (atrim[i] - 1) / 30.0f);
		gyro_ftrim[i] = 25 * 131.0f * powf(1.046f, gtrim[i] - 1);
	}

	// Y gyro trim is negative
	gyro_ftrim[1] *= -1;

	for (uint8_t i = 0; i < 3; i++) {
		float diff = accel[i] - accel_baseline[i];
		float err = 100 * (diff - accel_ftrim[i]) / accel_ftrim[i];
		::printf("ACCEL[%u] baseline=%d accel=%d diff=%d ftrim=%d err=%d\n",
			 (unsigned)i,
			 (int)(1000 * accel_baseline[i]),
			 (int)(1000 * accel[i]),
			 (int)(1000 * diff),
			 (int)(1000 * accel_ftrim[i]),
			 (int)err);

		if (fabsf(err) > 14) {
			::printf("FAIL\n");
			ret = -EIO;
		}
	}

	for (uint8_t i = 0; i < 3; i++) {
		float diff = gyro[i] - gyro_baseline[i];
		float err = 100 * (diff - gyro_ftrim[i]) / gyro_ftrim[i];
		::printf("GYRO[%u] baseline=%d gyro=%d diff=%d ftrim=%d err=%d\n",
			 (unsigned)i,
			 (int)(1000 * gyro_baseline[i]),
			 (int)(1000 * gyro[i]),
			 (int)(1000 * (gyro[i] - gyro_baseline[i])),
			 (int)(1000 * gyro_ftrim[i]),
			 (int)err);

		if (fabsf(err) > 14) {
			::printf("FAIL\n");
			ret = -EIO;
		}
	}

	write_reg(MPUREG_GYRO_CONFIG, saved_gyro_config);
	write_reg(MPUREG_ACCEL_CONFIG, saved_accel_config);

	_in_factory_test = false;

	if (ret == OK) {
		::printf("PASSED\n");
	}

}
#endif

/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
MPU6000::test_error()
{
	_in_factory_test = true;
	// deliberately trigger an error. This was noticed during
	// development as a handy way to test the reset logic
	uint8_t data[sizeof(MPUReport)] {};
	_interface->read(MPU6000_SET_SPEED(MPUREG_INT_STATUS, MPU6000_LOW_BUS_SPEED), data, sizeof(data));
	PX4_WARN("error triggered");
	print_registers();
	_in_factory_test = false;
}

uint8_t
MPU6000::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t buf{};
	_interface->read(MPU6000_SET_SPEED(reg, speed), &buf, 1);
	return buf;
}

uint16_t
MPU6000::read_reg16(unsigned reg)
{
	uint8_t buf[2] {};

	// general register transfer at low clock speed
	_interface->read(MPU6000_LOW_SPEED_OP(reg), &buf, arraySize(buf));
	return (uint16_t)(buf[0] << 8) | buf[1];
}

int
MPU6000::write_reg(unsigned reg, uint8_t value)
{
	// general register transfer at low clock speed
	return _interface->write(MPU6000_LOW_SPEED_OP(reg), &value, 1);
}

void
MPU6000::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
MPU6000::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < MPU6000_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

int
MPU6000::set_accel_range(unsigned max_g_in)
{
	// workaround for bugged versions of MPU6k (rev C)
	if (is_mpu_device()) {
		switch (_product) {
		case MPU6000ES_REV_C4:
		case MPU6000ES_REV_C5:
		case MPU6000_REV_C4:
		case MPU6000_REV_C5:
			write_checked_reg(MPUREG_ACCEL_CONFIG, 1 << 3);
			_px4_accel.set_scale(CONSTANTS_ONE_G / 4096.0f);
			return OK;
		}
	}

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

	write_checked_reg(MPUREG_ACCEL_CONFIG, afs_sel << 3);

	_px4_accel.set_scale(CONSTANTS_ONE_G / lsb_per_g);

	return OK;
}

void
MPU6000::start()
{
	/* make sure we are stopped first */
	uint32_t last_call_interval = _call_interval;
	stop();
	_call_interval = last_call_interval;

	ScheduleOnInterval(_call_interval - MPU6000_TIMER_REDUCTION, 1000);
}

void
MPU6000::stop()
{
	ScheduleClear();

	/* reset internal states */
	memset(_last_accel, 0, sizeof(_last_accel));
}

void
MPU6000::check_registers(void)
{
	/*
	  we read the register at full speed, even though it isn't
	  listed as a high speed register. The low speed requirement
	  for some registers seems to be a propgation delay
	  requirement for changing sensor configuration, which should
	  not apply to reading a single register. It is also a better
	  test of SPI bus health to read at the same speed as we read
	  the data registers.
	*/
	uint8_t v;

	// the MPUREG_ICM_UNDOC1 is specific to the ICM20608 (and undocumented)
	if ((_checked_registers[_checked_next] == MPUREG_ICM_UNDOC1 && !is_icm_device())) {
		_checked_next = (_checked_next + 1) % MPU6000_NUM_CHECKED_REGISTERS;
	}

	if ((v = read_reg(_checked_registers[_checked_next], MPU6000_HIGH_BUS_SPEED)) !=
	    _checked_values[_checked_next]) {
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
		if (_register_wait == 0 || _checked_next == MPU6000_CHECKED_PRODUCT_ID_INDEX) {
			// if the product_id is wrong then reset the
			// sensor completely
			write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
			// after doing a reset we need to wait a long
			// time before we do any other register writes
			// or we will end up with the mpu6000 in a
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

	_checked_next = (_checked_next + 1) % MPU6000_NUM_CHECKED_REGISTERS;
}

void MPU6000::RunImpl()
{
	if (_in_factory_test) {
		// don't publish any data while in factory test mode
		return;
	}

	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct MPUReport mpu_report;

	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the MPU6000 in one pass.
	 */

	// sensor transfer at high clock speed

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (sizeof(mpu_report) != _interface->read(MPU6000_SET_SPEED(MPUREG_INT_STATUS, MPU6000_HIGH_BUS_SPEED),
			(uint8_t *)&mpu_report, sizeof(mpu_report))) {

		perf_end(_sample_perf);
		return;
	}

	check_registers();

	/*
	   see if this is duplicate accelerometer data. Note that we
	   can't use the data ready interrupt status bit in the status
	   register as that also goes high on new gyro data, and when
	   we run with BITS_DLPF_CFG_256HZ_NOLPF2 the gyro is being
	   sampled at 8kHz, so we would incorrectly think we have new
	   data when we are in fact getting duplicate accelerometer data.
	*/
	if (!_got_duplicate && memcmp(&mpu_report.accel_x[0], &_last_accel[0], 6) == 0) {
		// it isn't new data - wait for next timer
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return;
	}

	perf_end(_sample_perf);

	memcpy(&_last_accel[0], &mpu_report.accel_x[0], 6);
	_got_duplicate = false;

	/*
	 * Convert from big to little endian
	 */

	report.accel_x = int16_t_from_bytes(mpu_report.accel_x);
	report.accel_y = int16_t_from_bytes(mpu_report.accel_y);
	report.accel_z = int16_t_from_bytes(mpu_report.accel_z);

	report.temp = int16_t_from_bytes(mpu_report.temp);

	report.gyro_x = int16_t_from_bytes(mpu_report.gyro_x);
	report.gyro_y = int16_t_from_bytes(mpu_report.gyro_y);
	report.gyro_z = int16_t_from_bytes(mpu_report.gyro_z);

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0 &&
	    report.gyro_x == 0 &&
	    report.gyro_y == 0 &&
	    report.gyro_z == 0) {

		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);

		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the mpu6k does go bad it would cause a FMU failure,
		// regardless of whether another sensor is available,
		return;
	}

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again, don't return any data yet
		_register_wait--;
		return;
	}


	/*
	 * Swap axes and negate y
	 */
	int16_t accel_xt = report.accel_y;
	int16_t accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);

	int16_t gyro_xt = report.gyro_y;
	int16_t gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);

	/*
	 * Apply the swap
	 */
	report.accel_x = accel_xt;
	report.accel_y = accel_yt;
	report.gyro_x = gyro_xt;
	report.gyro_y = gyro_yt;

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures

	const uint64_t error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);
	_px4_accel.set_error_count(error_count);
	_px4_gyro.set_error_count(error_count);

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

	float temperature = 0.0f;

	if (is_icm_device()) { // if it is an ICM20608
		temperature = (report.temp / 326.8f + 25.0f);

	} else { // If it is an MPU6000
		temperature = (report.temp / 340.0f + 35.0f);
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	_px4_accel.update(timestamp_sample, report.accel_x, report.accel_y, report.accel_z);
	_px4_gyro.update(timestamp_sample, report.gyro_x, report.gyro_y, report.gyro_z);
}

void
MPU6000::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("Device type: %i", _device_type);
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_reset_retries);
	perf_print_counter(_duplicates);

}

void
MPU6000::print_registers()
{
	PX4_INFO("registers");

	for (uint8_t reg = MPUREG_PRODUCT_ID; reg <= 108; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if ((reg - (MPUREG_PRODUCT_ID - 1)) % 13 == 0) {
			printf("\n");
		}
	}

	printf("\n");
}
