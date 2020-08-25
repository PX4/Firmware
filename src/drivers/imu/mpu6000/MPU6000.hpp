/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file mpu6000.cpp
 *
 * Driver for the Invensense MPU6000, MPU6050, ICM20608, and ICM20602 connected via
 * SPI or I2C.
 *
 * When the device is on the SPI bus the hrt is used to provide thread of
 * execution to the driver.
 *
 * When the device is on the I2C bus a work queue is used provide thread of
 * execution to the driver.
 *
 * The I2C code is only included in the build if USE_I2C is defined by the
 * existance of any of PX4_I2C_MPU6050_ADDR, PX4_I2C_MPU6000_ADDR
 * PX4_I2C_ICM_20608_G_ADDR in the board_config.h file.
 *
 * The command line option -T 6000|20608|20602 (default 6000) selects between
 * MPU60x0, ICM20608G, or ICM20602G;
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author David Sidrane
 */

#include <lib/conversion/rotation.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/device/spi.h>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>


/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates by comparing
  accelerometer values. This time reduction is enough to cope with
  worst case timing jitter due to other timers

  I2C bus is running at 100 kHz Transaction time is 2.163ms
  I2C bus is running at 400 kHz (304 kHz actual) Transaction time
  is 583 us

 */

#pragma once

#if defined(PX4_I2C_MPU6050_ADDR) || \
	defined(PX4_I2C_MPU6000_ADDR) || \
	defined(PX4_I2C_ICM_20608_G_ADDR)
#  define USE_I2C
#endif

enum MPU_DEVICE_TYPE {
	MPU_DEVICE_TYPE_MPU6000	= 6000,
	MPU_DEVICE_TYPE_ICM20602 = 20602,
	MPU_DEVICE_TYPE_ICM20608 = 20608,
	MPU_DEVICE_TYPE_ICM20689 = 20689
};

#define DIR_READ			0x80
#define DIR_WRITE			0x00

// MPU 6000 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
#define MPUREG_PRODUCT_ID		0x0C
#define MPUREG_TRIM1			0x0D
#define MPUREG_TRIM2			0x0E
#define MPUREG_TRIM3			0x0F
#define MPUREG_TRIM4			0x10
#define MPU_GYRO_DLPF_CFG_256HZ_NOLPF2	0x00  // delay: 0.98ms
#define MPU_GYRO_DLPF_CFG_188HZ	0x01  // delay: 1.9ms
#define MPU_GYRO_DLPF_CFG_98HZ		0x02  // delay: 2.8ms
#define MPU_GYRO_DLPF_CFG_42HZ		0x03  // delay: 4.8ms
#define MPU_GYRO_DLPF_CFG_20HZ		0x04  // delay: 8.3ms
#define MPU_GYRO_DLPF_CFG_10HZ		0x05  // delay: 13.4ms
#define MPU_GYRO_DLPF_CFG_5HZ		0x06  // delay: 18.6ms
#define MPU_GYRO_DLPF_CFG_2100HZ_NOLPF	0x07
#define MPU_DLPF_CFG_MASK		0x07

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP				0x40
#define BIT_H_RESET				0x80
#define BITS_CLKSEL				0x07
#define MPU_CLK_SEL_PLLGYROX	0x01
#define MPU_CLK_SEL_PLLGYROZ	0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18
#define BIT_INT_ANYRD_2CLEAR	0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS			0x10
#define BIT_INT_STATUS_DATA		0x01

#define MPU_WHOAMI_6000			0x68
#define ICM_WHOAMI_20602		0x12
#define ICM_WHOAMI_20608		0xaf
#define ICM_WHOAMI_20689		0x98

// ICM2608 specific registers

#define ICMREG_ACCEL_CONFIG2		0x1D
#define ICM_ACC_DLPF_CFG_1046HZ_NOLPF	0x00
#define ICM_ACC_DLPF_CFG_218HZ		0x01
#define ICM_ACC_DLPF_CFG_99HZ		0x02
#define ICM_ACC_DLPF_CFG_44HZ		0x03
#define ICM_ACC_DLPF_CFG_21HZ		0x04
#define ICM_ACC_DLPF_CFG_10HZ		0x05
#define ICM_ACC_DLPF_CFG_5HZ		0x06
#define ICM_ACC_DLPF_CFG_420HZ		0x07
/* this is an undocumented register which
   if set incorrectly results in getting a 2.7m/s/s offset
   on the Y axis of the accelerometer
*/
#define MPUREG_ICM_UNDOC1		0x11
#define MPUREG_ICM_UNDOC1_VALUE	0xc9

// Product ID Description for ICM20602
// Read From device

#define ICM20602_REV_01		1
#define ICM20602_REV_02		2

// Product ID Description for ICM20608

#define ICM20608_REV_FF		0xff // In the past, was thought to be not returning a value. But seem repeatable.

// Product ID Description for ICM20689

#define ICM20689_REV_FE		0xfe
#define ICM20689_REV_03   0x03
#define ICM20689_REV_04   0x04

// Product ID Description for MPU6000
// high 4 bits 	low 4 bits
// Product Name	Product Revision
#define MPU6000ES_REV_C4		0x14
#define MPU6000ES_REV_C5		0x15
#define MPU6000ES_REV_D6		0x16
#define MPU6000ES_REV_D7		0x17
#define MPU6000ES_REV_D8		0x18
#define MPU6000_REV_C4			0x54
#define MPU6000_REV_C5			0x55
#define MPU6000_REV_D6			0x56
#define MPU6000_REV_D7			0x57
#define MPU6000_REV_D8			0x58
#define MPU6000_REV_D9			0x59
#define MPU6000_REV_D10			0x5A
#define MPU6050_REV_D8			0x28	// TODO:Need verification

#define MPU6000_ACCEL_DEFAULT_RANGE_G				16

#define MPU6000_GYRO_DEFAULT_RANGE_G				8
#define MPU6000_GYRO_DEFAULT_RATE					1000


#define MPU6000_DEFAULT_ONCHIP_FILTER_FREQ			98

#pragma pack(push, 1)
/**
 * Report conversation within the MPU6000, including command byte and
 * interrupt status.
 */
struct MPUReport {
	uint8_t		cmd;
	uint8_t		status;
	uint8_t		accel_x[2];
	uint8_t		accel_y[2];
	uint8_t		accel_z[2];
	uint8_t		temp[2];
	uint8_t		gyro_x[2];
	uint8_t		gyro_y[2];
	uint8_t		gyro_z[2];
};
#pragma pack(pop)

#define MPU_MAX_READ_BUFFER_SIZE (sizeof(MPUReport) + 1)
#define MPU_MAX_WRITE_BUFFER_SIZE (2)
/*
  The MPU6000 can only handle high bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  Communication with all registers of the device is performed using either
  I2C at 400kHz or SPI at 1MHz. For applications requiring faster communications,
  the sensor and interrupt registers may be read using SPI at 20MHz
 */
#define MPU6000_LOW_BUS_SPEED				0
#define MPU6000_HIGH_BUS_SPEED				0x8000
#  define MPU6000_IS_HIGH_SPEED(r) 			((r) & MPU6000_HIGH_BUS_SPEED)
#  define MPU6000_REG(r) 					((r) &~MPU6000_HIGH_BUS_SPEED)
#  define MPU6000_SET_SPEED(r, s) 			((r)|(s))
#  define MPU6000_HIGH_SPEED_OP(r) 			MPU6000_SET_SPEED((r), MPU6000_HIGH_BUS_SPEED)
#  define MPU6000_LOW_SPEED_OP(r)			MPU6000_REG((r))

/* interface factories */
extern device::Device *MPU6000_SPI_interface(int bus, uint32_t devid, int device_type, bool external_bus,
		int bus_frequency, spi_mode_e spi_mode);
extern device::Device *MPU6000_I2C_interface(int bus, uint32_t devid, int device_type, bool external_bus,
		int bus_frequency);
extern int MPU6000_probe(device::Device *dev, int device_type);


#define MPU6000_TIMER_REDUCTION				200


class MPU6000 : public I2CSPIDriver<MPU6000>
{
public:
	MPU6000(device::Device *interface, enum Rotation rotation, int device_type, I2CSPIBusOption bus_option, int bus);

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual ~MPU6000();

	int		init();

	void			print_registers();

#ifndef CONSTRAINED_FLASH
	/**
	 * Test behaviour against factory offsets
	 */
	void 			factory_self_test();
#endif

	// deliberately cause a sensor error
	void 			test_error();

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

	void RunImpl();

protected:
	device::Device			*_interface;

	int probe();

	void print_status() override;

	void custom_method(const BusCLIArguments &cli) override;

private:

	int 			_device_type;
	uint8_t			_product{0};	/** product code */

	unsigned		_call_interval{1000};

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	unsigned		_sample_rate{1000};

	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_reset_retries;
	perf_counter_t		_duplicates;

	uint8_t			_register_wait{0};
	uint64_t		_reset_wait{0};

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
	static constexpr int MPU6000_CHECKED_PRODUCT_ID_INDEX = 0;
	static constexpr int MPU6000_NUM_CHECKED_REGISTERS = 10;

	static constexpr uint8_t _checked_registers[MPU6000_NUM_CHECKED_REGISTERS] {
		MPUREG_PRODUCT_ID,
		MPUREG_PWR_MGMT_1,
		MPUREG_USER_CTRL,
		MPUREG_SMPLRT_DIV,
		MPUREG_CONFIG,
		MPUREG_GYRO_CONFIG,
		MPUREG_ACCEL_CONFIG,
		MPUREG_INT_ENABLE,
		MPUREG_INT_PIN_CFG,
		MPUREG_ICM_UNDOC1
	};

	uint8_t			_checked_values[MPU6000_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next{0};

	// use this to avoid processing measurements when in factory
	// self test
	volatile bool		_in_factory_test{false};

	// keep last accel reading for duplicate detection
	uint16_t		_last_accel[3] {};
	bool			_got_duplicate{false};

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * is_icm_device
	 */
	bool 		is_icm_device() { return !is_mpu_device(); }
	/**
	 * is_mpu_device
	 */
	bool 		is_mpu_device() { return _device_type == MPU_DEVICE_TYPE_MPU6000; }

	/**
	 * Read a register from the MPU6000
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg, uint32_t speed = MPU6000_LOW_BUS_SPEED);
	uint16_t		read_reg16(unsigned reg);


	/**
	 * Write a register in the MPU6000
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	int				write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the MPU6000
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the MPU6000, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the MPU6000 measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);

	/*
	  set low pass filter frequency
	 */
	void 			_set_dlpf_filter(uint16_t frequency_hz);
	void 			_set_icm_acc_dlpf_filter(uint16_t frequency_hz);

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void			_set_sample_rate(unsigned desired_sample_rate_hz);

	/*
	  check that key registers still have the right value
	 */
	void			check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	MPU6000(const MPU6000 &);
	MPU6000 operator=(const MPU6000 &);

};
