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
 * @file FXAS21002C.hpp
 * Driver for the NXP FXAS21002C 3-Axis Digital Angular Rate Gyroscope
 * connected via SPI or I2C
 */

#pragma once

#include <drivers/device/Device.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>

#if defined(PX4_I2C_FXAS21002C_ADDR) && defined(PX4_I2C_BUS_EXPANSION)
#  define FXAS21002C_USE_I2C
#endif

/* SPI protocol address bits */
#define DIR_READ(a)                     ((a) | (1 << 7))
#define DIR_WRITE(a)                    ((a) & 0x7f)
#define swap16(w)                       __builtin_bswap16((w))

#define FXAS21002C_REG_MASK					0x00FF
#define FXAS21002C_REG(r) 					((r) & FXAS21002C_REG_MASK)

#define FXAS21002C_STATUS                0x00
#define FXAS21002C_OUT_X_MSB             0x01
#define FXAS21002C_OUT_X_LSB             0x02
#define FXAS21002C_OUT_Y_MSB             0x03
#define FXAS21002C_OUT_Y_LSB             0x04
#define FXAS21002C_OUT_Z_MSB             0x05
#define FXAS21002C_OUT_Z_LSB             0x06

#define FXAS21002C_DR_STATUS             0x07
#  define DR_STATUS_ZYXOW                (1 << 7)
#  define DR_STATUS_ZOW                  (1 << 6)
#  define DR_STATUS_YOW                  (1 << 5)
#  define DR_STATUS_XOW                  (1 << 4)
#  define DR_STATUS_ZYXDR                (1 << 3)
#  define DR_STATUS_ZDR                  (1 << 2)
#  define DR_STATUS_YDR                  (1 << 1)
#  define DR_STATUS_XDR                  (1 << 0)

#define FXAS21002C_F_STATUS              0x08
#  define F_STATUS_F_OVF                 (1 << 7)
#  define F_STATUS_F_WMKF                (1 << 6)
#  define F_STATUS_F_CNT_SHIFTS          0
#  define F_STATUS_F_CNT_MASK            (0x3f << F_STATUS_F_CNT_SHIFTS)

#define FXAS21002C_F_SETUP               0x09
#  define F_SETUP_F_MODE_SHIFTS          6
#  define F_SETUP_F_MODE_MASK            (0x3 << F_SETUP_F_MODE_SHIFTS)
#  define F_SETUP_F_WMRK_SHIFTS          0
#  define F_SETUP_F_WMRK_MASK            (0x3f << F_SETUP_F_WMRK_SHIFTS)

#define FXAS21002C_F_EVENT               0x0a
#  define F_EVENT_F_EVENT                (1 << 5)
#  define F_EVENT_FE_TIME_SHIFTS         0
#  define F_EVENT_FE_TIME_MASK           (0x1f << F_EVENT_FE_TIME_SHIFTS)

#define FXAS21002C_INT_SRC_FLAG          0x0b
#  define INT_SRC_FLAG_BOOTEND           (1 << 3)
#  define INT_SRC_FLAG_SRC_FIFO          (1 << 2)
#  define INT_SRC_FLAG_SRC_RT            (1 << 1)
#  define INT_SRC_FLAG_SRC_DRDY          (1 << 0)

#define FXAS21002C_WHO_AM_I              0x0c
#define   WHO_AM_I                       0xd7

#define FXAS21002C_CTRL_REG0             0x0d
#  define CTRL_REG0_BW_SHIFTS            6
#  define CTRL_REG0_BW_MASK              (0x3 << CTRL_REG0_BW_SHIFTS)
#  define CTRL_REG0_BW(n)                (((n) & 0x3) << CTRL_REG0_BW_SHIFTS)
#    define CTRL_REG0_BW_HIGH             CTRL_REG0_BW(0)
#    define CTRL_REG0_BW_MED              CTRL_REG0_BW(1)
#    define CTRL_REG0_BW_LOW              CTRL_REG0_BW(2)
#  define CTRL_REG0_SPIW                 (1 << 6)
#  define CTRL_REG0_SEL_SHIFTS           3
#  define CTRL_REG0_SEL_MASK             (0x2 << CTRL_REG0_SEL_SHIFTS)
#  define CTRL_REG0_HPF_EN               (1 << 2)
#  define CTRL_REG0_FS_SHIFTS            0
#  define CTRL_REG0_FS_MASK              (0x3 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_2000_DPS          (0 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_1000_DPS          (1 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_500_DPS           (2 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_250_DPS           (3 << CTRL_REG0_FS_SHIFTS)

#define FXAS21002C_RT_CFG                0x0e
#  define RT_CFG_ELE                     (1 << 3)
#  define RT_CFG_ZTEFE                   (1 << 2)
#  define RT_CFG_YTEFE                   (1 << 1)
#  define RT_CFG_XTEFE                   (1 << 0)

#define FXAS21002C_RT_SRC                0x0f
#  define RT_SRC_EA                      (1 << 6)
#  define RT_SRC_ZRT                     (1 << 5)
#  define RT_SRC_Z_RT_POL                (1 << 4)
#  define RT_SRC_YRT                     (1 << 3)
#  define RT_SRC_Y_RT_POL                (1 << 2)
#  define RT_SRC_XRT                     (1 << 1)
#  define RT_SRC_X_RT_POL                (1 << 0)

#define FXAS21002C_RT_THS                0x10
#  define RT_THS_DBCNTM                  (1 << 7)
#  define RT_THS_THS_SHIFTS              0
#  define RT_THS_THS_MASK                (0x7f << RT_THS_THS_SHIFTS)

#define FXAS21002C_RT_COUNT              0x11
#define FXAS21002C_TEMP                  0x12

#define FXAS21002C_CTRL_REG1             0x13
#  define CTRL_REG1_RST                  (1 << 6)
#  define CTRL_REG1_ST                   (1 << 5)
#  define CTRL_REG1_DR_SHIFTS             2
#  define CTRL_REG1_DR_MASK               (0x07 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_12_5               (7 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_12_5_1             (6 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_25HZ               (5 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_50HZ               (4 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_100HZ              (3 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_200HZ              (2 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_400HZ              (1 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_800HZ              (0 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_ACTIVE               (1 << 1)
#  define CTRL_REG1_READY                (1 << 0)

#define FXAS21002C_CTRL_REG2             0x14
#  define CTRL_REG2_INT_CFG_FIFO         (1 << 7)
#  define CTRL_REG2_INT_EN_FIFO          (1 << 6)
#  define CTRL_REG2_INT_CFG_RT           (1 << 5)
#  define CTRL_REG2_INT_EN_RT            (1 << 4)
#  define CTRL_REG2_INT_CFG_DRDY         (1 << 3)
#  define CTRL_REG2_INT_EN_DRDY          (1 << 2)
#  define CTRL_REG2_IPOL                 (1 << 1)
#  define CTRL_REG2_PP_OD                (1 << 0)

#define FXAS21002C_CTRL_REG3             0x15
#  define CTRL_REG3_WRAPTOONE            (1 << 3)
#  define CTRL_REG3_EXTCTRLEN            (1 << 2)
#  define CTRL_REG3_FS_DOUBLE            (1 << 0)

device::Device *FXAS21002C_SPI_interface(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode);
device::Device *FXAS21002C_I2C_interface(int bus, int bus_frequency, int i2c_address);

/* status register and data as read back from the device */
#pragma pack(push, 1)
struct RawGyroReport {
	uint8_t		cmd;
	uint8_t		status;
	int16_t		x;
	int16_t		y;
	int16_t		z;
};
#pragma pack(pop)


class FXAS21002C : public I2CSPIDriver<FXAS21002C>
{
public:
	FXAS21002C(device::Device *interface, I2CSPIBusOption bus_option, int bus, enum Rotation rotation, int i2c_address);
	virtual ~FXAS21002C();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int		init();

	void			print_status() override;

	void			RunImpl();

	void			print_registers();
	void			test_error();
protected:
	void custom_method(const BusCLIArguments &cli);
	int probe();

private:
	device::Device *_interface;
	PX4Gyroscope _px4_gyro;

	unsigned _current_rate{800};

	int16_t _gyro_prev[3] {};

	perf_counter_t _sample_perf;
	perf_counter_t _errors;
	perf_counter_t _bad_registers;
	perf_counter_t _duplicates;

	hrt_abstime _last_temperature_update{0};

	uint8_t _register_wait{0};

	/* this is used to support runtime checking of key
	 *configuration registers to detect SPI bus errors and sensor
	 * reset
	 */
	static constexpr int FXAS21002C_NUM_CHECKED_REGISTERS{6};

	uint8_t _checked_values[FXAS21002C_NUM_CHECKED_REGISTERS] {};
	uint8_t _checked_next{0};

	/**
	 * Start automatic measurement.
	 */
	void start();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	void reset();

	/**
	 * Put the chip In stand by
	 */
	void set_standby(int rate, bool standby_true);

	/**
	 * check key registers for correct values
	 */
	void check_registers(void);

	/**
	 * Read a register from the FXAS21002C
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	inline uint8_t read_reg(unsigned reg)
	{
		return _interface->read_reg(reg);
	}

	/**
	 * Write a register in the FXAS21002C
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	inline int write_reg(unsigned reg, uint8_t value)
	{
		return _interface->write_reg(reg, value);
	}

	/**
	 * Modify a register in the FXAS21002C
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the FXAS21002C, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the FXAS21002C measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int set_range(unsigned max_dps);

	/**
	 * Set the FXAS21002C internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int set_samplerate(unsigned frequency);

	/*
	  set onchip low pass filter frequency
	 */
	void set_onchip_lowpass_filter(int frequency_hz);
};
