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

/**
 * @file SDP3X.hpp
 *
 * Driver for Sensirion SDP3X Differential Pressure Sensor
 *
 * Datasheet: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/8_Differential_Pressure/Sensirion_Differential_Pressure_Sensors_SDP3x_Digital_Datasheet_V0.8.pdf
 */

#pragma once

#include <drivers/airspeed/airspeed.h>
#include <math.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>

#define I2C_ADDRESS_1_SDP3X		0x21
#define I2C_ADDRESS_2_SDP3X		0x22
#define I2C_ADDRESS_3_SDP3X		0x23

#define SDP3X_SCALE_TEMPERATURE		200.0f
#define SDP3X_RESET_ADDR		0x00
#define SDP3X_RESET_CMD			0x06
#define SDP3X_CONT_MEAS_AVG_MODE	0x3615

#define SDP3X_SCALE_PRESSURE_SDP31	60
#define SDP3X_SCALE_PRESSURE_SDP32	240
#define SDP3X_SCALE_PRESSURE_SDP33	20

// Measurement rate is 20Hz
#define SPD3X_MEAS_RATE 100
#define SDP3X_MEAS_DRIVER_FILTER_FREQ 3.0f
#define CONVERSION_INTERVAL	(1000000 / SPD3X_MEAS_RATE)	/* microseconds */

class SDP3X : public Airspeed, public I2CSPIDriver<SDP3X>
{
public:
	SDP3X(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = I2C_ADDRESS_1_SDP3X) :
		Airspeed(bus, bus_frequency, address, CONVERSION_INTERVAL),
		I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address)
	{
	}

	virtual ~SDP3X() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void	RunImpl();

private:

	int	measure() override { return 0; }
	int	collect() override;
	int	probe() override;

	math::LowPassFilter2p _filter{SPD3X_MEAS_RATE, SDP3X_MEAS_DRIVER_FILTER_FREQ};

	bool init_sdp3x();

	/**
	 * Calculate the CRC8 for the sensor payload data
	 */
	bool crc(const uint8_t data[], unsigned size, uint8_t checksum);

	/**
	 * Write a command in Sensirion specific logic
	 */
	int write_command(uint16_t command);

	uint16_t _scale{0};
};
