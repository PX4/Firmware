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

#pragma once

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>

#include "BMI055.hpp"

#define BMI055_DEVICE_PATH_ACCEL	"/dev/bmi055_accel"
#define BMI055_DEVICE_PATH_ACCEL_EXT	"/dev/bmi055_accel_ext"

// BMI055 Accel registers
#define BMI055_ACC_CHIP_ID          0x00
#define BMI055_ACC_X_L              0x02
#define BMI055_ACC_X_H              0x03
#define BMI055_ACC_Y_L              0x04
#define BMI055_ACC_Y_H              0x05
#define BMI055_ACC_Z_L              0x06
#define BMI055_ACC_Z_H              0x07
#define BMI055_ACC_TEMP             0x08
#define BMI055_ACC_INT_STATUS_0     0x09
#define BMI055_ACC_INT_STATUS_1     0x0A
#define BMI055_ACC_INT_STATUS_2     0x0B
#define BMI055_ACC_INT_STATUS_3     0x0C
#define BMI055_ACC_FIFO_STATUS      0x0E
#define BMI055_ACC_RANGE            0x0F
#define BMI055_ACC_BW               0x10
#define BMI055_ACC_PMU_LPW          0x11
#define BMI055_ACC_PMU_LOW_POWER    0x12
#define BMI055_ACC_DATA_CTRL        0x13
#define BMI055_ACC_SOFTRESET        0x14
#define BMI055_ACC_INT_EN_0         0x16
#define BMI055_ACC_INT_EN_1         0x17
#define BMI055_ACC_INT_EN_2         0x18
#define BMI055_ACC_INT_MAP_0        0x19
#define BMI055_ACC_INT_MAP_1        0x1A
#define BMI055_ACC_INT_MAP_2        0x1B
#define BMI055_ACC_INT_SRC          0x1E
#define BMI055_ACC_INT_OUT_CTRL     0x20
#define BMI055_ACC_INT_LATCH        0x21
#define BMI055_ACC_INT_LH_0         0x22
#define BMI055_ACC_INT_LH_1         0x23
#define BMI055_ACC_INT_LH_2         0x24
#define BMI055_ACC_INT_LH_3         0x25
#define BMI055_ACC_INT_LH_4         0x26
#define BMI055_ACC_INT_MOT_0        0x27
#define BMI055_ACC_INT_MOT_1        0x28
#define BMI055_ACC_INT_MOT_2        0x29
#define BMI055_ACC_INT_TAP_0        0x2A
#define BMI055_ACC_INT_TAP_1        0x2B
#define BMI055_ACC_INT_ORIE_0       0x2C
#define BMI055_ACC_INT_ORIE_1       0x2D
#define BMI055_ACC_INT_FLAT_0       0x2E
#define BMI055_ACC_INT_FLAT_1       0x2F
#define BMI055_ACC_FIFO_CONFIG_0    0x30
#define BMI055_ACC_SELF_TEST        0x32
#define BMI055_ACC_EEPROM_CTRL      0x33
#define BMI055_ACC_SERIAL_CTRL      0x34
#define BMI055_ACC_OFFSET_CTRL      0x36
#define BMI055_ACC_OFC_SETTING      0x37
#define BMI055_ACC_OFFSET_X         0x38
#define BMI055_ACC_OFFSET_Y         0x39
#define BMI055_ACC_OFFSET_Z         0x3A
#define BMI055_ACC_TRIM_GPO         0x3B
#define BMI055_ACC_TRIM_GP1         0x3C
#define BMI055_ACC_FIFO_CONFIG_1    0x3E
#define BMI055_ACC_FIFO_DATA        0x3F

// BMI055 Accelerometer Chip-Id
#define BMI055_ACC_WHO_AM_I         0xFA

// DLPF filter bandwidth settings
#define BMI055_ACCEL_BW_7_81      (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI055_ACCEL_BW_15_63     (1<<3) | (0<<2) | (0<<1) | (1<<0)
#define BMI055_ACCEL_BW_31_25     (1<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI055_ACCEL_BW_62_5      (1<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI055_ACCEL_BW_125       (1<<3) | (1<<2) | (0<<1) | (0<<0)
#define BMI055_ACCEL_BW_250       (1<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI055_ACCEL_BW_500       (1<<3) | (1<<2) | (1<<1) | (0<<0)
#define BMI055_ACCEL_BW_1000      (1<<3) | (1<<2) | (1<<1) | (1<<0)

//BMI055_ACC_PMU_LPW     0x11
#define BMI055_ACCEL_NORMAL         (0<<7) | (0<<6) | (0<<5)
#define BMI055_ACCEL_DEEP_SUSPEND   (0<<7) | (0<<6) | (1<<5)
#define BMI055_ACCEL_LOW_POWER      (0<<7) | (1<<6) | (0<<5)
#define BMI055_ACCEL_SUSPEND        (1<<7) | (0<<6) | (0<<5)

//BMI055_ACC_RANGE        0x0F
#define BMI055_ACCEL_RANGE_2_G      (0<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI055_ACCEL_RANGE_4_G      (0<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI055_ACCEL_RANGE_8_G      (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI055_ACCEL_RANGE_16_G     (1<<3) | (1<<2) | (0<<1) | (0<<0)

//BMI055_ACC_INT_EN_1      0x17
#define BMI055_ACC_DRDY_INT_EN      (1<<4)

//BMI055_ACC_INT_MAP_1         0x1A
#define BMI055_ACC_DRDY_INT1        (1<<0)

// Default and Max values
#define BMI055_ACCEL_DEFAULT_RANGE_G		16
#define BMI055_ACCEL_DEFAULT_RATE           1000
#define BMI055_ACCEL_MAX_RATE               1000
#define BMI055_ACCEL_MAX_PUBLISH_RATE       280

#define BMI055_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 50

/* Mask definitions for ACCD_X_LSB, ACCD_Y_LSB and ACCD_Z_LSB Register */
#define BMI055_NEW_DATA_MASK                 0x01

class BMI055_accel : public BMI055
{
public:
	BMI055_accel(I2CSPIBusOption bus_option, int bus, const char *path_accel, uint32_t device, enum Rotation rotation,
		     int bus_frequency, spi_mode_e spi_mode);
	virtual ~BMI055_accel();

	int init() override;

	/// Start automatic measurement.
	void start() override;

	void print_status() override;

	void print_registers() override;

	/// deliberately cause a sensor error
	void test_error() override;

	void RunImpl() override;
protected:

	int probe() override;
private:

	PX4Accelerometer	_px4_accel;

	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _bad_registers;
	perf_counter_t      _duplicates;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define BMI055_ACCEL_NUM_CHECKED_REGISTERS 5
	static const uint8_t    _checked_registers[BMI055_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_values[BMI055_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_bad[BMI055_ACCEL_NUM_CHECKED_REGISTERS];

	bool            _got_duplicate;

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int         reset();

	/**
	 * Modify a register in the BMI055_accel
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg       The register to modify.
	 * @param clearbits Bits in the register to clear.
	 * @param setbits   Bits in the register to set.
	 */
	void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the BMI055_accel, updating _checked_values
	 *
	 * @param reg       The register to write.
	 * @param value     The new value to write.
	 */
	void            write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the BMI055_accel measurement range.
	 *
	 * @param max_g     The maximum G value the range must support.
	 * @return      OK if the value can be supported, -EINVAL otherwise.
	 */
	int         set_accel_range(unsigned max_g);

	/*
	 * check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	BMI055_accel(const BMI055_accel &);
	BMI055_accel operator=(const BMI055_accel &);

};
