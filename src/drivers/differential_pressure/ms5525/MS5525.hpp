/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include <drivers/airspeed/airspeed.h>
#include <math.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>

/* The MS5525DSO address is 111011Cx, where C is the complementary value of the pin CSB */
static constexpr uint8_t I2C_ADDRESS_1_MS5525DSO = 0x76;

/* Measurement rate is 100Hz */
static constexpr unsigned MEAS_RATE = 100;
static constexpr float MEAS_DRIVER_FILTER_FREQ = 1.2f;
static constexpr int64_t CONVERSION_INTERVAL = (1000000 / MEAS_RATE); /* microseconds */

class MS5525 : public Airspeed, public I2CSPIDriver<MS5525>
{
public:
	MS5525(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = I2C_ADDRESS_1_MS5525DSO) :
		Airspeed(bus, bus_frequency, address, CONVERSION_INTERVAL),
		I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
	{
	}

	virtual ~MS5525() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void	RunImpl();

private:

	int measure() override;
	int collect() override;

	// temperature is read once every 10 cycles
	math::LowPassFilter2p _filter{MEAS_RATE * 0.9, MEAS_DRIVER_FILTER_FREQ};

	static constexpr uint8_t CMD_RESET = 0x1E; // ADC reset command
	static constexpr uint8_t CMD_ADC_READ = 0x00; // ADC read command

	static constexpr uint8_t CMD_PROM_START = 0xA0; // Prom read command (first)

	// D1 - pressure convert commands
	// Convert D1 (OSR=256)  0x40
	// Convert D1 (OSR=512)  0x42
	// Convert D1 (OSR=1024) 0x44
	// Convert D1 (OSR=2048) 0x46
	// Convert D1 (OSR=4096) 0x48
	static constexpr uint8_t CMD_CONVERT_PRES = 0x44;

	// D2 - temperature convert commands
	// Convert D2 (OSR=256)  0x50
	// Convert D2 (OSR=512)  0x52
	// Convert D2 (OSR=1024) 0x54
	// Convert D2 (OSR=2048) 0x56
	// Convert D2 (OSR=4096) 0x58
	static constexpr uint8_t CMD_CONVERT_TEMP = 0x54;

	uint8_t _current_cmd{CMD_CONVERT_PRES};

	unsigned _pressure_count{0};

	// Qx Coefficients Matrix by Pressure Range
	//  5525DSO-pp001DS (Pmin = -1, Pmax = 1)
	static constexpr uint8_t Q1 = 15;
	static constexpr uint8_t Q2 = 17;
	static constexpr uint8_t Q3 = 7;
	static constexpr uint8_t Q4 = 5;
	static constexpr uint8_t Q5 = 7;
	static constexpr uint8_t Q6 = 21;

	// calibration coefficients from prom
	uint16_t C1{0};
	uint16_t C2{0};
	uint16_t C3{0};
	uint16_t C4{0};
	uint16_t C5{0};
	uint16_t C6{0};

	int64_t Tref{0};

	// last readings for D1 (uncompensated pressure) and D2 (uncompensated temperature)
	uint32_t D1{0};
	uint32_t D2{0};

	bool init_ms5525();
	bool _inited{false};

	uint8_t prom_crc4(uint16_t n_prom[]) const;

};
