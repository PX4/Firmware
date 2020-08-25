/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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


#include <px4_platform_common/px4_config.h>
#include <lib/perf/perf_counter.h>
#include <systemlib/conversions.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

// in 16-bit sampling mode the mag resolution is 1.5 milli Gauss per bit.
static constexpr float AK09916_MAG_RANGE_GA = 1.5e-3f;

static constexpr uint8_t AK09916_I2C_ADDR = 0x0C;

static constexpr uint8_t AK09916_DEVICE_ID_A = 0x48;
static constexpr uint8_t AK09916REG_WIA = 0x00;

static constexpr uint8_t AK09916REG_ST1 = 0x10;
static constexpr uint8_t AK09916REG_HXL = 0x11;
static constexpr uint8_t AK09916REG_CNTL2 = 0x31;
static constexpr uint8_t AK09916REG_CNTL3 = 0x32;

static constexpr uint8_t AK09916_RESET = 0x01;
static constexpr uint8_t AK09916_CNTL2_CONTINOUS_MODE_100HZ = 0x08;

static constexpr uint8_t AK09916_ST1_DRDY = 0x01;
static constexpr uint8_t AK09916_ST1_DOR = 0x02;

static constexpr uint8_t AK09916_ST2_HOFL = 0x08;

// Run at 100 Hz.
static constexpr unsigned AK09916_CONVERSION_INTERVAL_us = 1000000 / 100;

#pragma pack(push, 1)
struct ak09916_regs {
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t tmps;
	uint8_t st2;
};
#pragma pack(pop)


class AK09916 : public device::I2C, public I2CSPIDriver<AK09916>
{
public:
	AK09916(I2CSPIBusOption bus_option, const int bus, int bus_frequency, enum Rotation rotation);
	virtual ~AK09916();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init() override;
	void start();
	void print_status() override;
	int probe() override;

	void RunImpl();

protected:
	int setup();
	int setup_master_i2c();
	bool check_id();
	void try_measure();
	bool is_ready();
	void measure();
	int reset();

	uint8_t read_reg(uint8_t reg);
	void read_block(uint8_t reg, uint8_t *val, uint8_t count);
	void write_reg(uint8_t reg, uint8_t value);

private:

	PX4Magnetometer _px4_mag;

	static constexpr uint32_t _cycle_interval{AK09916_CONVERSION_INTERVAL_us};

	perf_counter_t _mag_reads;
	perf_counter_t _mag_errors;
	perf_counter_t _mag_overruns;
	perf_counter_t _mag_overflows;

	AK09916(const AK09916 &) = delete;
	AK09916 operator=(const AK09916 &) = delete;
};
