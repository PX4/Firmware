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
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#define DIR_READ                0x80
#define DIR_WRITE               0x00

//Soft-reset command Value
#define BMI055_SOFT_RESET       0xB6

#define BMI055_BUS_SPEED				10*1000*1000

#define BMI055_TIMER_REDUCTION				200

class BMI055 : public device::SPI, public I2CSPIDriver<BMI055>
{
public:
	BMI055(const char *name, const char *devname, I2CSPIBusOption bus_option, int bus, int type, uint32_t device,
	       enum spi_mode_e mode, uint32_t frequency, enum Rotation rotation);
	virtual ~BMI055() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual void start() = 0;

	virtual void RunImpl() = 0;
protected:

	virtual void print_registers() = 0;
	virtual void test_error() = 0;

	void custom_method(const BusCLIArguments &cli) override;

	uint8_t         _whoami;    ///< whoami result

	uint8_t         _register_wait;
	uint64_t        _reset_wait;

	enum Rotation       _rotation;

	uint8_t         _checked_next;

	/**
	* Read a register from the BMI055
	*
	* @param       The register to read.
	* @return      The value that was read.
	*/
	uint8_t         read_reg(unsigned reg);
	uint16_t        read_reg16(unsigned reg);

	/**
	* Write a register in the BMI055
	*
	* @param reg       The register to write.
	* @param value     The new value to write.
	*/
	void            write_reg(unsigned reg, uint8_t value);

};
