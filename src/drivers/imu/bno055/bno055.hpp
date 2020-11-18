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

#pragma once

#include <functional>
#include <drivers/device/i2c.h>
#include <ecl/geo/geo.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <systemlib/conversions.h>

#include "BNO055_driver/bno055.h"

using namespace time_literals;


/**
 * Small PX4 cpp wrapper around the Bosch library
 */
class BNO055 : public device::I2C, public I2CSPIDriver<BNO055>
{
public:
	BNO055(I2CSPIBusOption bus_option, int bus, int bus_frequency);
	virtual ~BNO055();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);


	int init() override;

	/**
	 * Completely resets chip
	 *
	 * @note init() must have been called before, and since the chip is now reset, init() should be called again
	 *
	 * @todo not used
	 */
	int reset();

	/**
	 * Starts polling the sensor
	 *
	 * @todo why is this a separate function?
	 */
	void start();

	/**
	 * The function that polls the sensors and updates their values
	 */
	void RunImpl();

	static void print_usage();

private:

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;
	PX4Magnetometer _px4_mag;

	// most recent data
	struct bno055_accel_double_t accel_xyz;
	struct bno055_gyro_double_t gyro_xyz;
	struct bno055_mag_double_t mag_xyz;

	// cutoff freqs for the internal low pass filters
	const uint8_t ACCEL_BW_REGVAL = BNO055_ACCEL_BW_125HZ;
	const uint8_t GYRO_BW_REGVAL = BNO055_GYRO_BW_116HZ;
	// update freq for the magnetometer
	const uint8_t MAG_RATE_REGVAL = BNO055_MAG_DATA_OUTRATE_30HZ;
	// poll at 250 Hz to match other sensors, this should probably be about 2x the low pass bandwidth
	const uint32_t POLLING_INTERVAL_US = uint32_t(1000000 / 250);
	// the magnetometer is slower than the other sensors, so don't fetch it every time
	const uint32_t MAG_INTERVAL_US = uint32_t(1000000 / 30);
	// save the last time the magnetometer was read
	uint32_t mag_last_read = 0;

	////////////////////////////////////////////////////////////////////////////////
	// Support code for the Bosch library
	////////////////////////////////////////////////////////////////////////////////


	/**
	 * Read a register from the BNO055, this is used by the Bosch lib
	 *
	 * @param dev_addr this is <b>not used</b>, it is only there because the library needs it in its signature
	 * @param reg_addr the (starting) register to from
	 * @param reg_data returned data
	 * @param count amount of bytes to read
	 * @return 0 on success
	 */
	int8_t read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
	static int8_t read_reg_trampoline(void * obj, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
		return ((BNO055*)obj)->read_reg(dev_addr, reg_addr, reg_data, cnt);
	}

	/**
	 * Read a register from the BNO055, this is used by the Bosch lib
	 *
	 * @param dev_addr this is <b>not used</b>, it is only there because the library needs it in its signature
	 * @param reg_addr the (starting) register to write to
	 * @param reg_data the data to send
	 * @param count amount of bytes to send
	 * @return 0 on success
	 */
	int8_t write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
	static int8_t write_reg_trampoline(void * obj, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
		return ((BNO055*)obj)->write_reg(dev_addr, reg_addr, reg_data, cnt);
	}

	/** Bosch lib main data struct */
	struct bno055_t bno055_struct;

	/**
	 * This exists because the lib needs a pointer to a sleep(ms) function
	 *
	 * @param ms sleep time in millis
	 *
	 * @todo does this work with Nuttx?
	 * @todo replace with workqueue sleep
	 */
	inline static void bno_msleep(uint32_t ms) { usleep(ms * 1000); }
};
