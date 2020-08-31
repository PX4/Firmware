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
 * @file mag.cpp
 *
 * Driver for the ak8963 magnetometer within the Invensense mpu9250
 *
 * @author Robert Dickenson
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include "MPU9250_mag.h"
#include "mpu9250.h"

// If interface is non-null, then it will used for interacting with the device.
// Otherwise, it will passthrough the parent MPU9250
MPU9250_mag::MPU9250_mag(MPU9250 *parent, device::Device *interface, enum Rotation rotation) :
	_interface(interface),
	_px4_mag(parent->_interface->get_device_id(), rotation),
	_parent(parent),
	_mag_overruns(perf_alloc(PC_COUNT, MODULE_NAME": mag overruns")),
	_mag_overflows(perf_alloc(PC_COUNT, MODULE_NAME": mag overflows")),
	_mag_errors(perf_alloc(PC_COUNT, MODULE_NAME": mag errors"))
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_AK8963);
	_px4_mag.set_external(_parent->is_external());
	_px4_mag.set_scale(MPU9250_MAG_RANGE_GA);
}

MPU9250_mag::~MPU9250_mag()
{
	perf_free(_mag_overruns);
	perf_free(_mag_overflows);
	perf_free(_mag_errors);
}

void
MPU9250_mag::measure()
{
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	uint8_t st1 = 0;
	int ret = _interface->read(AK8963REG_ST1, &st1, sizeof(st1));

	if (ret != OK) {
		perf_count(_mag_errors);
		_px4_mag.set_error_count(perf_event_count(_mag_errors));
		return;
	}

	/* Check if data ready is set.
	 * This is not described to be set in continuous mode according to the
	 * MPU9250 datasheet. However, the datasheet of the 8963 recommends to
	 * check data ready before doing the read and before triggering the
	 * next measurement by reading ST2. */
	if (!(st1 & AK09916_ST1_DRDY)) {
		return;
	}

	/* Monitor if data overrun flag is ever set. */
	if (st1 & 0x02) {
		perf_count(_mag_overruns);
	}

	ak8963_regs data{};
	ret = _interface->read(AK8963REG_ST1, &data, sizeof(data));

	if (ret != OK) {
		_px4_mag.set_error_count(perf_event_count(_mag_errors));
		return;
	}

	/* Monitor magnetic sensor overflow flag. */
	if (data.ST2 & 0x08) {
		perf_count(_mag_overflows);
	}

	_measure(timestamp_sample, data);
}

bool MPU9250_mag::_measure(const hrt_abstime &timestamp_sample, const ak8963_regs &data)
{
	/* Check if data ready is set.
	 * This is not described to be set in continuous mode according to the
	 * MPU9250 datasheet. However, the datasheet of the 8963 recommends to
	 * check data ready before doing the read and before triggering the
	 * next measurement by reading ST2.
	 *
	 * If _measure is used in passthrough mode, all the data is already
	 * fetched, however, we should still not use the data if the data ready
	 * is not set. This has lead to intermittent spikes when the data was
	 * being updated while getting read.
	 */
	if (!(data.ST1 & AK09916_ST1_DRDY)) {
		return false;
	}

	/* Monitor magnetic sensor overflow flag. */
	if (data.ST2 & 0x08) {
		perf_count(_mag_overflows);
		return false;
	}

	_px4_mag.set_temperature(_parent->_last_temperature);

	/*
	 * Align axes - note the accel & gyro are also re-aligned so this
	 *              doesn't look obvious with the datasheet
	 */
	int16_t x = combine(data.HXH, data.HXL);
	int16_t y = -combine(data.HYH, data.HYL);
	int16_t z = -combine(data.HZH, data.HZL);
	_px4_mag.update(timestamp_sample, x * _ak8963_ASA[0], y * _ak8963_ASA[1], z * _ak8963_ASA[2]);

	return true;
}

void
MPU9250_mag::set_passthrough(uint8_t reg, uint8_t size, uint8_t *out)
{
	uint8_t addr;

	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // ensure slave r/w is disabled before changing the registers

	if (out) {
		_parent->write_reg(MPUREG_I2C_SLV0_D0, *out);
		addr = AK8963_I2C_ADDR;

	} else {
		addr = AK8963_I2C_ADDR | BIT_I2C_READ_FLAG;
	}

	_parent->write_reg(MPUREG_I2C_SLV0_ADDR, addr);
	_parent->write_reg(MPUREG_I2C_SLV0_REG, reg);
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, size | BIT_I2C_SLV0_EN);
}

uint8_t
MPU9250_mag::read_reg(unsigned int reg)
{
	uint8_t buf{};

	if (_interface == nullptr) {
		read_reg_through_mpu9250(reg, &buf);

	} else {
		_interface->read(reg, &buf, 1);
	}

	return buf;
}

bool
MPU9250_mag::ak8963_check_id(uint8_t &deviceid)
{
	deviceid = read_reg(AK8963REG_WIA);

	return (AK8963_DEVICE_ID == deviceid);
}

void
MPU9250_mag::write_reg(unsigned reg, uint8_t value)
{
	// general register transfer at low clock speed
	if (_interface == nullptr) {
		write_reg_through_mpu9250(reg, value);

	} else {
		_interface->write(MPU9250_LOW_SPEED_OP(reg), &value, 1);
	}
}

int
MPU9250_mag::ak8963_reset()
{
	// First initialize it to use the bus
	int rv = ak8963_setup();

	if (rv == OK) {
		// Now reset the mag
		write_reg(AK8963REG_CNTL2, AK8963_RESET);

		// Then re-initialize the bus/mag
		rv = ak8963_setup();
	}

	return rv;
}

bool
MPU9250_mag::ak8963_read_adjustments()
{
	uint8_t response[3] {};

	write_reg_through_mpu9250(AK8963REG_CNTL1, AK8963_FUZE_MODE | AK8963_16BIT_ADC);
	px4_usleep(200);

	if (_interface != nullptr) {
		_interface->read(AK8963REG_ASAX, response, 3);

	} else {
		for (int i = 0; i < 3; ++i) {
			read_reg_through_mpu9250(AK8963REG_ASAX + i, response + i);
		}
	}

	write_reg_through_mpu9250(AK8963REG_CNTL1, AK8963_POWERDOWN_MODE);



	for (int i = 0; i < 3; i++) {
		if (0 != response[i] && 0xff != response[i]) {
			_ak8963_ASA[i] = ((float)(response[i] - 128) / 256.0f) + 1.0f;

		} else {
			return false;
		}
	}

	return true;
}

int
MPU9250_mag::ak8963_setup_master_i2c()
{
	/* When _interface is null we are using SPI and must
	 * use the parent interface to configure the device to act
	 * in master mode (SPI to I2C bridge)
	 */
	if (_interface == nullptr) {
		_parent->modify_checked_reg(MPUREG_USER_CTRL, 0, BIT_I2C_MST_EN);
		_parent->write_reg(MPUREG_I2C_MST_CTRL, BIT_I2C_MST_P_NSR | BIT_I2C_MST_WAIT_FOR_ES | BITS_I2C_MST_CLOCK_400HZ);

	} else {
		_parent->modify_checked_reg(MPUREG_USER_CTRL, BIT_I2C_MST_EN, 0);
	}

	return OK;
}

int
MPU9250_mag::ak8963_setup()
{
	int retries = 10;

	do {
		ak8963_setup_master_i2c();
		write_reg(AK8963REG_CNTL2, AK8963_RESET);
		px4_usleep(100);

		uint8_t id = 0;

		if (ak8963_check_id(id)) {
			break;
		}

		retries--;
		PX4_WARN("AK8963: bad id %d retries %d", id, retries);
		_parent->modify_reg(MPUREG_USER_CTRL, 0, BIT_I2C_MST_RST);
		px4_usleep(100);
	} while (retries > 0);

	if (retries > 0) {
		retries = 10;

		while (!ak8963_read_adjustments() && retries) {
			retries--;
			PX4_ERR("AK8963: failed to read adjustment data. Retries %d", retries);

			_parent->modify_reg(MPUREG_USER_CTRL, 0, BIT_I2C_MST_RST);
			px4_usleep(100);
			ak8963_setup_master_i2c();
			write_reg(AK8963REG_CNTL2, AK8963_RESET);
		}
	}

	if (retries == 0) {
		PX4_ERR("AK8963: failed to initialize, disabled!");
		_parent->modify_checked_reg(MPUREG_USER_CTRL, BIT_I2C_MST_EN, 0);
		_parent->write_reg(MPUREG_I2C_MST_CTRL, 0);

		return -EIO;
	}

	write_reg(AK8963REG_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC);

	if (_interface == nullptr) {
		// Configure mpu' I2C Master interface to read ak8963 data into to fifo
		set_passthrough(AK8963REG_ST1, sizeof(ak8963_regs));
	}

	return OK;
}

void MPU9250_mag::write_imu_reg_verified(int reg, uint8_t val, uint8_t mask)
{
	uint8_t b;
	int retry = 5;

	while (retry) { // should not reach any retries in normal condition
		--retry;
		_parent->write_reg(reg, val);

		b = _parent->read_reg(reg);

		if ((b & mask) != val) {
			PX4_DEBUG("MPU9250_mag::write_imu_reg_verified failed. retrying...");
			continue;

		} else {
			return;
		}
	}
}

void MPU9250_mag::read_reg_through_mpu9250(uint8_t reg, uint8_t *val)
{
	// Read operation on the mag using the slave 4 registers.
	write_imu_reg_verified(MPUREG_I2C_SLV4_ADDR, AK8963_I2C_ADDR | BIT_I2C_READ_FLAG, 0xff);

	// Set the mag register to read from.
	write_imu_reg_verified(MPUREG_I2C_SLV4_REG, reg, 0xff);

	// Read the existing value of the SLV4 control register.
	uint8_t b = _parent->read_reg(MPUREG_I2C_SLV4_CTRL);

	// Set the I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits. Enable data transfer, a read transfer as configured above.
	b |= 0x80;
	// Trigger the data transfer
	_parent->write_reg(MPUREG_I2C_SLV4_CTRL, b);

	// Continuously check I2C_MST_STATUS register value for the completion
	// of I2C transfer until timeout

	int loop_ctrl = 1000; // wait up to 1000 * 1ms for completion

	do {
		px4_usleep(1000);
		b = _parent->read_reg(MPUREG_I2C_MST_STATUS);
	} while (((b & 0x40) == 0x00) && (--loop_ctrl));

	if (loop_ctrl == 0) {
		PX4_ERR("I2C transfer timed out");

	} else {
		PX4_DEBUG("mpu9250 SPI2IIC read delay: %dms", loop_ctrl);
	}

	// Read the value received from the mag, and copy to the caller's out parameter.
	*val = _parent->read_reg(MPUREG_I2C_SLV4_DI);
}

void MPU9250_mag::write_reg_through_mpu9250(uint8_t reg, uint8_t val)
{
	// Configure a write operation to the mag using Slave 4.
	write_imu_reg_verified(MPUREG_I2C_SLV4_ADDR, AK8963_I2C_ADDR, 0xff);

	// Set the mag register address to write to using Slave 4.
	write_imu_reg_verified(MPUREG_I2C_SLV4_REG, reg, 0xff);

	// Set the value to write in the I2C_SLV4_DO register.
	write_imu_reg_verified(MPUREG_I2C_SLV4_DO, val, 0xff);

	// Read the current value of the Slave 4 control register.
	uint8_t b = _parent->read_reg(MPUREG_I2C_SLV4_CTRL);

	// Set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits.
	b |= 0x80;
	// Trigger the data transfer from the byte now stored in the SLV4_DO register.
	_parent->write_reg(MPUREG_I2C_SLV4_CTRL, b);

	// Continuously check I2C_MST_STATUS regsiter value for the completion
	// of I2C transfer until timeout.

	int loop_ctrl = 1000; // wait up to 1000 * 1ms for completion

	do {
		px4_usleep(1000);
		b = _parent->read_reg(MPUREG_I2C_MST_STATUS);

	} while (((b & 0x40) == 0x00) && (--loop_ctrl));

	if (loop_ctrl == 0) {
		PX4_ERR("I2C transfer to mag timed out");

	} else {
		PX4_DEBUG("mpu9250 SPI2IIC write delay: %dms", loop_ctrl);
	}
}
