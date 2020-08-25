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
 * @file srf02.cpp
 *
 * Driver for the SRF02 sonar range finder adapted from the Maxbotix sonar range finder driver (srf02).
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

/* Configuration Constants */
#define SRF02_BASEADDR 				0x70 	// 7-bit address. 8-bit address is 0xE0.

/* SRF02 Registers addresses */
#define SRF02_TAKE_RANGE_REG			0x51	// Measure range Register.
#define SRF02_SET_ADDRESS_0			0xA0	// Change address 0 Register.
#define SRF02_SET_ADDRESS_1			0xAA	// Change address 1 Register.
#define SRF02_SET_ADDRESS_2			0xA5	// Change address 2 Register.

/* Device limits */
#define SRF02_MIN_DISTANCE 			(0.20f)
#define SRF02_MAX_DISTANCE 			(7.65f)

#define SRF02_CONVERSION_INTERVAL 		100000	// 60ms for one sonar.
#define SRF02_INTERVAL_BETWEEN_SUCCESIVE_FIRES 	100000	// 30ms between each sonar measurement (watch out for interference!).

class SRF02 : public device::I2C, public I2CSPIDriver<SRF02>
{
public:
	SRF02(I2CSPIBusOption bus_option, const int bus, const uint8_t rotation, int bus_frequency,
	      int address = SRF02_BASEADDR);
	~SRF02() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init() override;
	void print_status() override;

	void RunImpl();

private:

	void start();
	int collect();
	int measure();

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 * @param address The I2C bus address to probe.
	 * @return True if the device is present.
	 */
	int probe_address(uint8_t address);

	PX4Rangefinder _px4_rangefinder;

	int _interval{SRF02_CONVERSION_INTERVAL};

	bool _collect_phase{false};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED,  MODULE_NAME": read")};
};
