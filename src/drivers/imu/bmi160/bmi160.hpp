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

#include <drivers/device/spi.h>
#include <ecl/geo/geo.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <systemlib/conversions.h>

#define DIR_READ                0x80
#define DIR_WRITE               0x00

// BMI 160 registers

#define BMIREG_CHIP_ID          0x00
#define BMIREG_ERR_REG          0x02
#define BMIREG_PMU_STATUS       0x03
#define BMIREG_DATA_0           0x04
#define BMIREG_DATA_1           0x05
#define BMIREG_DATA_2           0x06
#define BMIREG_DATA_3           0x07
#define BMIREG_DATA_4           0x08
#define BMIREG_DATA_5           0x09
#define BMIREG_DATA_6           0x0A
#define BMIREG_DATA_7           0x0B
#define BMIREG_GYR_X_L          0x0C
#define BMIREG_GYR_X_H          0x0D
#define BMIREG_GYR_Y_L          0x0E
#define BMIREG_GYR_Y_H          0x0F
#define BMIREG_GYR_Z_L          0x10
#define BMIREG_GYR_Z_H          0x11
#define BMIREG_ACC_X_L          0x12
#define BMIREG_ACC_X_H          0x13
#define BMIREG_ACC_Y_L          0x14
#define BMIREG_ACC_Y_H          0x15
#define BMIREG_ACC_Z_L          0x16
#define BMIREG_ACC_Z_H          0x17
#define BMIREG_SENSORTIME0      0x18
#define BMIREG_SENSORTIME1      0x19
#define BMIREG_SENSORTIME2      0x1A
#define BMIREG_STATUS           0x1B
#define BMIREG_INT_STATUS_0     0x1C
#define BMIREG_INT_STATUS_1     0x1D
#define BMIREG_INT_STATUS_2     0x1E
#define BMIREG_INT_STATUS_3     0x1F
#define BMIREG_TEMP_0           0x20
#define BMIREG_TEMP_1           0x21
#define BMIREG_FIFO_LEN_0       0x22
#define BMIREG_FIFO_LEN_1       0x23
#define BMIREG_FIFO_DATA        0x24
#define BMIREG_ACC_CONF         0x40
#define BMIREG_ACC_RANGE        0x41
#define BMIREG_GYR_CONF         0x42
#define BMIREG_GYR_RANGE        0x43
#define BMIREG_MAG_CONF         0x44
#define BMIREG_FIFO_DOWNS       0x45
#define BMIREG_FIFO_CONFIG_0    0x46
#define BMIREG_FIFO_CONFIG_1    0x47
#define BMIREG_MAG_IF_0         0x4B
#define BMIREG_MAG_IF_1         0x4C
#define BMIREG_MAG_IF_2         0x4D
#define BMIREG_MAG_IF_3         0x4E
#define BMIREG_MAG_IF_4         0x4F
#define BMIREG_INT_EN_0         0x50
#define BMIREG_INT_EN_1         0x51
#define BMIREG_INT_EN_2         0x52
#define BMIREG_INT_OUT_CTRL     0x53
#define BMIREG_INT_LANTCH       0x54
#define BMIREG_INT_MAP_0        0x55
#define BMIREG_INT_MAP_1        0x56
#define BMIREG_INT_MAP_2        0x57
#define BMIREG_INT_DATA_0       0x58
#define BMIREG_INT_DATA_1       0x59
#define BMIREG_INT_LH_0         0x5A
#define BMIREG_INT_LH_1         0x5B
#define BMIREG_INT_LH_2         0x5C
#define BMIREG_INT_LH_3         0x5D
#define BMIREG_INT_LH_4         0x5E
#define BMIREG_INT_MOT_0        0x5F
#define BMIREG_INT_MOT_1        0x60
#define BMIREG_INT_MOT_2        0x61
#define BMIREG_INT_MOT_3        0x62
#define BMIREG_INT_TAP_0        0x63
#define BMIREG_INT_TAP_1        0x64
#define BMIREG_INT_ORIE_0       0x65
#define BMIREG_INT_ORIE_1       0x66
#define BMIREG_INT_FLAT_0       0x67
#define BMIREG_INT_FLAT_1       0x68
#define BMIREG_FOC_CONF         0x69
#define BMIREG_CONF             0x6A
#define BMIREG_IF_CONF          0x6B
#define BMIREG_PMU_TRIGGER      0x6C
#define BMIREG_SELF_TEST        0x6D
#define BMIREG_NV_CONF          0x70
#define BMIREG_OFFSET_ACC_X     0x71
#define BMIREG_OFFSET_ACC_Y     0x72
#define BMIREG_OFFSET_ACC_Z     0x73
#define BMIREG_OFFSET_GYR_X     0x74
#define BMIREG_OFFSET_GYR_Y     0x75
#define BMIREG_OFFSET_GYR_Z     0x76
#define BMIREG_OFFSET_EN        0x77
#define BMIREG_STEP_CONT_0      0x78
#define BMIREG_STEP_CONT_1      0x79
#define BMIREG_STEP_CONF_0      0x7A
#define BMIREG_STEP_CONF_1      0x7B
#define BMIREG_CMD              0x7E


// Configuration bits BMI 160
#define BMI160_WHO_AM_I         0xD1

//BMIREG_STATUS           0x1B
#define BMI_DRDY_ACCEL          (1<<7)
#define BMI_DRDY_GYRO           (1<<6)
#define BMI_DRDY_MAG            (1<<5)
#define BMI_GYRO_SELF_TEST_OK   (1<<1)

//BMIREG_INT_STATUS_1     0x1D
#define BMI_DRDY_INT            (1<<4)

//BMIREG_ACC_CONF         0x40
#define BMI_ACCEL_RATE_25_32    (0<<3) | (0<<2) | (0<<1) | (1<<0)
#define BMI_ACCEL_RATE_25_16    (0<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI_ACCEL_RATE_25_8     (0<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI_ACCEL_RATE_25_4     (0<<3) | (1<<2) | (0<<1) | (0<<0)
#define BMI_ACCEL_RATE_25_2     (0<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI_ACCEL_RATE_25       (0<<3) | (1<<2) | (1<<1) | (0<<0)
#define BMI_ACCEL_RATE_50       (0<<3) | (1<<2) | (1<<1) | (1<<0)
#define BMI_ACCEL_RATE_100      (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI_ACCEL_RATE_200      (1<<3) | (0<<2) | (0<<1) | (1<<0)
#define BMI_ACCEL_RATE_400      (1<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI_ACCEL_RATE_800      (1<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI_ACCEL_RATE_1600     (1<<3) | (1<<2) | (0<<1) | (0<<0)
#define BMI_ACCEL_US            (0<<7)
#define BMI_ACCEL_BWP_NORMAL    (0<<6) | (1<<5) | (0<<4)
#define BMI_ACCEL_BWP_OSR2      (0<<6) | (0<<5) | (1<<4)
#define BMI_ACCEL_BWP_OSR4      (0<<6) | (0<<5) | (0<<4)

//BMIREG_ACC_RANGE        0x41
#define BMI_ACCEL_RANGE_2_G     (0<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI_ACCEL_RANGE_4_G     (0<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI_ACCEL_RANGE_8_G     (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI_ACCEL_RANGE_16_G    (1<<3) | (1<<2) | (0<<1) | (0<<0)

//BMIREG_GYR_CONF         0x42
#define BMI_GYRO_RATE_25        (0<<3) | (1<<2) | (1<<1) | (0<<0)
#define BMI_GYRO_RATE_50        (0<<3) | (1<<2) | (1<<1) | (1<<0)
#define BMI_GYRO_RATE_100       (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI_GYRO_RATE_200       (1<<3) | (0<<2) | (0<<1) | (1<<0)
#define BMI_GYRO_RATE_400       (1<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI_GYRO_RATE_800       (1<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI_GYRO_RATE_1600      (1<<3) | (1<<2) | (0<<1) | (0<<0)
#define BMI_GYRO_RATE_3200      (1<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI_GYRO_BWP_NORMAL     (1<<5) | (0<<4)
#define BMI_GYRO_BWP_OSR2       (0<<5) | (1<<4)
#define BMI_GYRO_BWP_OSR4       (0<<5) | (0<<4)

//BMIREG_GYR_RANGE        0x43
#define BMI_GYRO_RANGE_2000_DPS (0<<2) | (0<<1) | (0<<0)
#define BMI_GYRO_RANGE_1000_DPS (0<<2) | (0<<1) | (1<<0)
#define BMI_GYRO_RANGE_500_DPS  (0<<2) | (1<<1) | (0<<0)
#define BMI_GYRO_RANGE_250_DPS  (0<<2) | (1<<1) | (1<<0)
#define BMI_GYRO_RANGE_125_DPS  (1<<2) | (0<<1) | (0<<0)

//BMIREG_INT_EN_1         0x51
#define BMI_DRDY_INT_EN         (1<<4)

//BMIREG_INT_OUT_CTRL     0x53
#define BMI_INT1_EN             (1<<3) | (0<<2) | (1<<1)    //Data Ready on INT1 High

//BMIREG_INT_MAP_1        0x56
#define BMI_DRDY_INT1           (1<<7)

//BMIREG_IF_CONF          0x6B
#define BMI_SPI_3_WIRE          (1<<0)
#define BMI_SPI_4_WIRE          (0<<0)
#define BMI_AUTO_DIS_SEC        (0<<5) | (0<<4)
#define BMI_I2C_OIS_SEC         (0<<5) | (1<<4)
#define BMI_AUTO_MAG_SEC        (1<<5) | (0<<4)

//BMIREG_NV_CONF          0x70
#define BMI_SPI                 (1<<0)

//BMIREG_CMD              0x7E
#define BMI_ACCEL_NORMAL_MODE   0x11 //Wait at least 3.8 ms before another CMD
#define BMI_GYRO_NORMAL_MODE    0x15 //Wait at least 80 ms before another CMD
#define BMI160_SOFT_RESET       0xB6

#define BMI160_ACCEL_DEFAULT_RANGE_G		16
#define BMI160_GYRO_DEFAULT_RANGE_DPS		2000
#define BMI160_ACCEL_DEFAULT_RATE           800
#define BMI160_ACCEL_MAX_RATE               1600
#define BMI160_ACCEL_MAX_PUBLISH_RATE       280
#define BMI160_GYRO_DEFAULT_RATE            800
#define BMI160_GYRO_MAX_RATE                3200
#define BMI160_GYRO_MAX_PUBLISH_RATE        BMI160_ACCEL_MAX_PUBLISH_RATE

#define BMI160_BUS_SPEED				10*1000*1000

#define BMI160_TIMER_REDUCTION				200

using namespace time_literals;

class BMI160 : public device::SPI, public I2CSPIDriver<BMI160>
{
public:
	BMI160(I2CSPIBusOption bus_option, int bus, int32_t device, enum Rotation rotation, int bus_frequency,
	       spi_mode_e spi_mode);
	virtual ~BMI160();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();
	int		init() override;
	void		print_status() override;
	void		RunImpl();

private:
	int		probe() override;

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	uint8_t			_whoami;	///< whoami result

	unsigned		_dlpf_freq;

	float			_accel_sample_rate{BMI160_ACCEL_DEFAULT_RATE};
	float			_gyro_sample_rate{BMI160_GYRO_DEFAULT_RATE};

	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_good_transfers;
	perf_counter_t		_reset_retries;
	perf_counter_t		_duplicates;

	uint8_t			_register_wait{0};
	uint64_t		_reset_wait{0};

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
	static constexpr int BMI160_NUM_CHECKED_REGISTERS{10};
	static const uint8_t	_checked_registers[BMI160_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[BMI160_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_bad[BMI160_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next{0};

	// keep last accel reading for duplicate detection
	uint16_t		_last_accel[3] {};
	bool			_got_duplicate{false};

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/**
	 * Read a register from the BMI160
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(uint8_t reg);

	/**
	 * Write a register in the BMI160
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 * @return	   	OK on success, negative errno otherwise.
	 */
	int			write_reg(uint8_t reg, uint8_t value);

	/**
	 * Modify a register in the BMI160
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(uint8_t reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the BMI160, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(uint8_t reg, uint8_t value);

	/**
	 * Set the BMI160 measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @param max_dps	The maximum DPS value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);
	int			set_gyro_range(unsigned max_dps);

	/*
	  set sample rate (approximate) - 10 - 952 Hz
	*/
	int accel_set_sample_rate(float desired_sample_rate_hz);
	int gyro_set_sample_rate(float desired_sample_rate_hz);
	/*
	  check that key registers still have the right value
	 */
	void check_registers();

#pragma pack(push, 1)
	/**
	 * Report conversation within the BMI160, including command byte and
	 * interrupt status.
	 */
	struct BMIReport {
		uint8_t		cmd;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
	};
#pragma pack(pop)
};
