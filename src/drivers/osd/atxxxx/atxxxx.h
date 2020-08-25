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

/**
 * @file atxxxx.h
 * @author Daniele Pettenuzzo
 *
 * Driver for the ATXXXX chip on the omnibus fcu connected via SPI.
 */
#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#define OSD_SPI_BUS_SPEED (2000000L) /*  2 MHz  */

#define DIR_READ(a) ((a) | (1 << 7))
#define DIR_WRITE(a) ((a) & 0x7f)

#define OSD_CHARS_PER_ROW	30
#define OSD_NUM_ROWS_PAL	16
#define OSD_NUM_ROWS_NTSC	13
#define OSD_ZERO_BYTE 0x00
#define OSD_PAL_TX_MODE 0x40

extern "C" __EXPORT int atxxxx_main(int argc, char *argv[]);

class OSDatxxxx : public device::SPI, public ModuleParams, public I2CSPIDriver<OSDatxxxx>
{
public:
	OSDatxxxx(I2CSPIBusOption bus_option, int bus, int devid, int bus_frequency, spi_mode_e spi_mode);
	virtual ~OSDatxxxx() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init() override;

	void RunImpl();

protected:
	int probe() override;

private:
	int start();

	int reset();

	int init_osd();

	int readRegister(unsigned reg, uint8_t *data, unsigned count);
	int writeRegister(unsigned reg, uint8_t data);

	int add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y);
	void add_string_to_screen_centered(const char *str, uint8_t pos_y, int max_length);
	void clear_line(uint8_t pos_x, uint8_t pos_y, int length);

	int add_battery_info(uint8_t pos_x, uint8_t pos_y);
	int add_altitude(uint8_t pos_x, uint8_t pos_y);
	int add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y);

	static const char *get_flight_mode(uint8_t nav_state);

	int enable_screen();
	int disable_screen();

	int update_topics();
	int update_screen();

	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// battery
	float _battery_voltage_filtered_v{0.f};
	float _battery_discharge_mah{0.f};
	bool _battery_valid{false};

	// altitude
	float _local_position_z{0.f};
	bool _local_position_valid{false};

	// flight time
	uint8_t _arming_state{0};
	uint64_t _arming_timestamp{0};

	// flight mode
	uint8_t _nav_state{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_ATXXXX_CFG>) _param_osd_atxxxx_cfg
	)
};
