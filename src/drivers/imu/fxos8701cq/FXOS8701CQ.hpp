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
 * @file FXOS8701CQ.hpp
 * Driver for the NXP FXOS8701CQ 6-axis sensor with integrated linear accelerometer and
 * magnetometer connected via SPI or I2C.
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/device/spi.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

#if defined(PX4_I2C_FXOS8701CQ_ADDR) && defined(PX4_I2C_BUS_EXPANSION)
#  define FXOS8701CQ_USE_I2C
#endif

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#endif

/* SPI protocol address bits */
#define DIR_READ(a)                     ((a) & 0x7f)
#define DIR_WRITE(a)                    ((a) | (1 << 7))
#define ADDR_7(a)                       ((a) & (1 << 7))
#define swap16(w)                       __builtin_bswap16((w))
#define swap16RightJustify14(w)         (((int16_t)swap16(w)) >> 2)

// I2C address
#define FXOS8701CQ_REG_MASK					0x00FF
#define FXOS8701CQ_REG(r) 					((r) & FXOS8701CQ_REG_MASK)

#define FXOS8701CQ_DR_STATUS       0x00
#  define DR_STATUS_ZYXDR          (1 << 3)

#define FXOS8701CQ_OUT_X_MSB       0x01

#define FXOS8701CQ_XYZ_DATA_CFG    0x0e
#  define XYZ_DATA_CFG_FS_SHIFTS   0
#  define XYZ_DATA_CFG_FS_MASK     (3 << XYZ_DATA_CFG_FS_SHIFTS)
#  define XYZ_DATA_CFG_FS_2G       (0 << XYZ_DATA_CFG_FS_SHIFTS)
#  define XYZ_DATA_CFG_FS_4G       (1 << XYZ_DATA_CFG_FS_SHIFTS)
#  define XYZ_DATA_CFG_FS_8G       (2 << XYZ_DATA_CFG_FS_SHIFTS)

#define FXOS8701CQ_WHOAMI          0x0d
#  define FXOS8700CQ_WHOAMI_VAL    0xC7
#  define FXOS8701CQ_WHOAMI_VAL    0xCA

#define FXOS8701CQ_CTRL_REG1       0x2a
#  define CTRL_REG1_ACTIVE         (1 << 0)
#  define CTRL_REG1_DR_SHIFTS      3
#  define CTRL_REG1_DR_MASK        (7 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR(n)          (((n) & 7) << CTRL_REG1_DR_SHIFTS)
#define FXOS8701CQ_CTRL_REG2       0x2b
#  define CTRL_REG2_AUTO_INC       (1 << 5)

#define FXOS8701CQ_M_DR_STATUS     0x32
#  define M_DR_STATUS_ZYXDR         (1 << 3)
#define FXOS8701CQ_M_OUT_X_MSB     0x33
#define FXOS8701CQ_TEMP            0x51
#define FXOS8701CQ_M_CTRL_REG1     0x5b
#  define M_CTRL_REG1_HMS_SHIFTS   0
#  define M_CTRL_REG1_HMS_MASK     (3 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_HMS_A        (0 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_HMS_M        (1 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_HMS_AM       (3 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_OS_SHIFTS    2
#  define M_CTRL_REG1_OS_MASK      (7 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_OS(n)        (((n) & 7) << M_CTRL_REG1_OS_SHIFTS)

#define FXOS8701CQ_M_CTRL_REG2     0x5c
#define FXOS8701CQ_M_CTRL_REG3     0x5d

#define DEF_REG(r)   {r, #r}

/* default values for this device */
#define FXOS8701C_ACCEL_DEFAULT_RANGE_G              8
#define FXOS8701C_ACCEL_DEFAULT_RATE                 400 /* ODR is 400 in Hybird mode (accel + mag) */

#pragma pack(push, 1)
struct RawAccelMagReport {
#       ifdef FXOS8701CQ_USE_I2C
	uint8_t  cmd;
#       else
	uint8_t	 cmd[2];
#       endif

	uint8_t		status;
	int16_t		x;
	int16_t		y;
	int16_t		z;
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	int16_t		mx;
	int16_t		my;
	int16_t		mz;
#endif // !BOARD_HAS_NOISY_FXOS8700_MAG
};
#pragma pack(pop)

extern device::Device *FXOS8701CQ_SPI_interface(int bus, uint32_t chip_select, int bus_frequency, spi_mode_e spi_mode);
extern device::Device *FXOS8701CQ_I2C_interface(int bus, int bus_frequency, int i2c_address);

class FXOS8701CQ : public I2CSPIDriver<FXOS8701CQ>
{
public:
	FXOS8701CQ(device::Device *interface, I2CSPIBusOption bus_option, int bus, enum Rotation rotation, int i2c_address);
	virtual ~FXOS8701CQ();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int		init();

	void			print_status() override;

	void			RunImpl();

	void			print_registers();
	void			test_error();
protected:
	int		probe();
	void custom_method(const BusCLIArguments &cli) override;

private:
	device::Device *_interface;

	void			start();
	void			reset();

	/**
	 * check key registers for correct values
	 */
	void			check_registers();

	/**
	 * Read a register from the FXOS8701C
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	inline uint8_t read_reg(unsigned reg) { return _interface->read_reg(reg); }

	/**
	 * Write a register in the FXOS8701C
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 * @return		OK on success, negative errno otherwise.
	 */
	inline int write_reg(unsigned reg, uint8_t value) { return _interface->write_reg(reg, value); }

	/**
	 * Modify a register in the FXOS8701C
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the FXOS8701C, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the FXOS8701C accel measurement range.
	 *
	 * @param max_g	The measurement range of the accel is in g (9.81m/s^2)
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_range(unsigned max_g);

	/**
	 * Set the FXOS8701C mag measurement range.
	 *
	 * @param max_ga	The measurement range of the mag is in Ga
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			mag_set_range(unsigned max_g);

	/**
	 * Set the FXOS8701C internal accel and mag sampling frequency.
	 *
	 * @param frequency	The internal accel and mag sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			accel_set_samplerate(unsigned frequency);


	PX4Accelerometer	_px4_accel;

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	PX4Magnetometer		_px4_mag;
	hrt_abstime		_mag_last_measure{0};
	perf_counter_t		_mag_sample_perf;
#endif

	hrt_abstime		_last_temperature_update{0};

	unsigned		_accel_samplerate{FXOS8701C_ACCEL_DEFAULT_RATE};

	perf_counter_t		_accel_sample_perf;
	perf_counter_t		_bad_registers;
	perf_counter_t		_accel_duplicates;

	int16_t _accel_prev[3] {};

	uint8_t			_register_wait{0};

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
	static constexpr int FXOS8701C_NUM_CHECKED_REGISTERS{5};
	static const uint8_t	_checked_registers[FXOS8701C_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[FXOS8701C_NUM_CHECKED_REGISTERS] {};
	uint8_t			_checked_next{0};

};
