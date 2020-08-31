/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file lis2mdl.h
 *
 * Shared defines for the LIS2MDL driver.
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

/**
 * LIS2MDL internal constants and data structures.
 */

/* Max measurement rate is 50Hz */
#define LIS2MDL_CONVERSION_INTERVAL     (1000000 / 50)

#define ADDR_WHO_AM_I                   0x4f
#define ID_WHO_AM_I                     0x40

#define ADDR_CFG_REG_A                  0x60
#define ADDR_CFG_REG_B                  0x61
#define ADDR_CFG_REG_C                  0x62
#define ADDR_INT_CTRL_REG               0x63

#define ADDR_STATUS_REG                 0x67
#define ADDR_OUT_X_L                    0x68
#define ADDR_OUT_X_H                    0x69
#define ADDR_OUT_Y_L                    0x6a
#define ADDR_OUT_Y_H                    0x6b
#define ADDR_OUT_Z_L                    0x6c
#define ADDR_OUT_Z_H                    0x6d
#define ADDR_OUT_T_L                    0x6e
#define ADDR_OUT_T_H                    0x6f

#define CFG_REG_A_TEMP_COMP_EN          (1 << 7)
#define CFG_REG_A_ODR                   (2 << 2) /* 50Hz (100Hz creates spikes randomly) */
#define CFG_REG_A_MD                    (0 << 0) /* continuous mode */

#define CFG_REG_B_LPF                   (1 << 0) /* LPF */

#define CFG_REG_C_BDU                   (1 << 4) /* avoids reading of incorrect data due to async reads */

/* interface factories */
extern device::Device *LIS2MDL_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);
extern device::Device *LIS2MDL_I2C_interface(int bus, int bus_frequency);


class LIS2MDL : public I2CSPIDriver<LIS2MDL>
{
public:
	LIS2MDL(device::Device *interface, enum Rotation rotation, I2CSPIBusOption bus_option, int bus);
	virtual ~LIS2MDL();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int init();

	void print_status() override;

	/**
	 * Configures the device with default register values.
	 */
	int set_default_register_values();

	void RunImpl();

private:
	PX4Magnetometer _px4_mag;

	device::Device *_interface;

	perf_counter_t _comms_errors;
	perf_counter_t _conf_errors;
	perf_counter_t _range_errors;
	perf_counter_t _sample_perf;

	unsigned int _measure_interval;

	/**
	 * Issue a measurement command & publish data.
	 *
	 * @return              OK if the measurement command was successful.
	 */
	int measure();

	/**
	 * @brief Initialises the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * @brief Reads a register.
	 *
	 * @param reg           The register to read.
	 * @param val           The value read.
	 * @return              OK on read success.
	 */
	int read_reg(uint8_t reg, uint8_t &val);

	/**
	 * @brief  Writes a register.
	 *
	 * @param reg           The register to write.
	 * @param val           The value to write.
	 * @return              OK on write success.
	 */
	int write_reg(uint8_t reg, uint8_t val);

};
