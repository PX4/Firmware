/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file qmc5883.cpp
 *
 * Driver for the QMC5883 magnetometer connected via I2C.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_device.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#include "qmc5883.h"

/*
 * QMC5883 internal constants and data structures.
 */

/* Max measurement rate is 200Hz */
#define QMC5883_CONVERSION_INTERVAL	(1000000 / 150)	/* microseconds */
#define QMC5883_MAX_COUNT 			32767

#define QMC5883_ADDR_DATA_OUT_X_LSB		0x00
#define QMC5883_ADDR_DATA_OUT_X_MSB		0x01
#define QMC5883_ADDR_DATA_OUT_Y_LSB		0x02
#define QMC5883_ADDR_DATA_OUT_Y_MSB		0x03
#define QMC5883_ADDR_DATA_OUT_Z_LSB		0x04
#define QMC5883_ADDR_DATA_OUT_Z_MSB		0x05
#define QMC5883_ADDR_STATUS			0x06
#define QMC5883_ADDR_TEMP_OUT_LSB 		0x07
#define QMC5883_ADDR_TEMP_OUT_MSB 		0x08
#define QMC5883_ADDR_CONTROL_1 			0x09
#define QMC5883_ADDR_CONTROL_2 			0x0A
#define QMC5883_ADDR_SET_RESET 			0x0B


#define QMC5883_STATUS_REG_DRDY 		(1 << 0)  /* Data Ready: "0": no new data, "1": new data is ready */
#define QMC5883_STATUS_REG_OVL 			(1 << 1)  /* Overflow Flag: "0": normal, "1": data overflow */
#define QMC5883_STATUS_REG_DOR			(1 << 2)  /* Data Skip: "0": normal, "1": data skipped for reading */

/* Control Register 1 */
#define QMC5883_MODE_REG_STANDBY 		(0 << 0)
#define QMC5883_MODE_REG_CONTINOUS_MODE         (1 << 0)
#define QMC5883_OUTPUT_DATA_RATE_10 		(0 << 2) /* Hz */
#define QMC5883_OUTPUT_DATA_RATE_50 		(1 << 2)
#define QMC5883_OUTPUT_DATA_RATE_100 		(2 << 2)
#define QMC5883_OUTPUT_DATA_RATE_200 		(3 << 2)
#define QMC5883_OUTPUT_RANGE_2G 		(0 << 4)  /* +/- 2 gauss */
#define QMC5883_OUTPUT_RANGE_8G 		(1 << 4)  /* +/- 8 gauss */
#define QMC5883_OVERSAMPLE_512 			(0 << 6)  /* controls digital filter bw - larger OSR -> smaller bw */
#define QMC5883_OVERSAMPLE_256 			(1 << 6)
#define QMC5883_OVERSAMPLE_128 			(2 << 6)
#define QMC5883_OVERSAMPLE_64 			(3 << 6)

/* Control Register 2 */
#define QMC5883_INT_ENB 			(1 << 0)
#define QMC5883_ROL_PNT 			(1 << 6)
#define QMC5883_SOFT_RESET 			(1 << 7)

/* Set Register */
#define QMC5883_SET_DEFAULT 			(1 << 0)

#define QMC5883_TEMP_OFFSET 50 /* deg celsius */

class QMC5883 : public I2CSPIDriver<QMC5883>
{
public:
	QMC5883(device::Device *interface, enum Rotation rotation, I2CSPIBusOption bus_option, int bus, int i2c_address);
	virtual ~QMC5883();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void			RunImpl();

	int		init();

protected:
	void print_status() override;

private:
	PX4Magnetometer		_px4_mag;
	device::Device		*_interface;
	unsigned		_measure_interval{0};

	bool			_collect_phase;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_range_errors;
	perf_counter_t		_conf_errors;

	uint8_t			_conf_reg;
	uint8_t			_temperature_counter;
	uint8_t			_temperature_error_count;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void 			check_conf();

	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();
};
