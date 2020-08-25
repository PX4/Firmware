/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file teraranger.cpp
 * @author Luis Rodrigues
 *
 * Driver for the TeraRanger One range finders connected via I2C.
 */

#pragma once

#include <drivers/device/i2c.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>

using namespace time_literals;

/* Configuration Constants */
#define TERARANGER_ONE_BASEADDR                 0x30 // 7-bit address.
#define TERARANGER_EVO_BASEADDR                 0x31 // 7-bit address.

/* TERARANGER Registers addresses */
#define TERARANGER_MEASURE_REG                  0x00 // Measure range register.
#define TERARANGER_WHO_AM_I_REG                 0x01 // Who am I test register.
#define TERARANGER_WHO_AM_I_REG_VAL             0xA1

/* Device limits */
#define TERARANGER_ONE_MAX_DISTANCE             (14.00f)
#define TERARANGER_ONE_MIN_DISTANCE             (0.20f)

#define TERARANGER_EVO_3M_MAX_DISTANCE          (3.0f)
#define TERARANGER_EVO_3M_MIN_DISTANCE          (0.10f)

#define TERARANGER_EVO_60M_MAX_DISTANCE         (60.0f)
#define TERARANGER_EVO_60M_MIN_DISTANCE         (0.50f)

#define TERARANGER_EVO_600HZ_MAX_DISTANCE       (8.0f)
#define TERARANGER_EVO_600HZ_MIN_DISTANCE       (0.75f)

#define TERARANGER_MEASUREMENT_INTERVAL         10_ms

class TERARANGER : public device::I2C, public I2CSPIDriver<TERARANGER>
{
public:
	TERARANGER(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency);
	~TERARANGER() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int init() override;
	void print_status() override;

	void RunImpl();
protected:

	virtual int probe() override;

private:
	void start();
	int collect();
	int measure();

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address The I2C bus address to probe.
	* @return True if the device is present.
	*/
	int probe_address(const uint8_t address);


	PX4Rangefinder _px4_rangefinder;

	bool _collect_phase{false};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comm_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
