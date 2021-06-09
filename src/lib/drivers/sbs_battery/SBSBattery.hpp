/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file SBSBattery.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for SMBUS SBS v1.1-compatible Smart Batteries
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 * @author Eohan George <eg@.skymul.com>
 * @author Alex Mikhalev <alex@corvus-robotics.com>
 */

#pragma once

#include <ecl/geo/geo.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/smbus/SMBus.hpp>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/battery_status.h>

#include <board_config.h>

using namespace time_literals;

class SBSBatteryBase : public ModuleParams
{
protected:
	static constexpr hrt_abstime MEASUREMENT_INTERVAL_US =	100_ms;	///< time in microseconds, measure at 10Hz

	static constexpr uint8_t BATT_ADDR = 			0x0B;	///< Default 7 bit address I2C address. 8 bit = 0x16

	enum SBS_Register {
		SBS_REG_TEMP =                                  0x08,   ///< temperature register
		SBS_REG_VOLTAGE =                               0x09,   ///< voltage register
		SBS_REG_CURRENT =                               0x0A,   ///< current register
		SBS_REG_AVERAGE_CURRENT =                       0x0B,   ///< average current register
		SBS_REG_MAX_ERROR =                             0x0C,   ///< max error
		SBS_REG_RELATIVE_SOC =                          0x0D,   ///< Relative State Of Charge
		SBS_REG_ABSOLUTE_SOC =                          0x0E,   ///< Absolute State of charge
		SBS_REG_REMAINING_CAPACITY =                    0x0F,   ///< predicted remaining battery capacity as a percentage
		SBS_REG_FULL_CHARGE_CAPACITY =                  0x10,   ///< capacity when fully charged
		SBS_REG_RUN_TIME_TO_EMPTY =                     0x11,   ///< predicted remaining battery capacity based on the present rate of discharge in min
		SBS_REG_AVERAGE_TIME_TO_EMPTY =                 0x12,   ///< predicted remaining battery capacity based on the present rate of discharge in min
		SBS_REG_CYCLE_COUNT =                           0x17,   ///< number of cycles the battery has experienced
		SBS_REG_DESIGN_CAPACITY =                       0x18,   ///< design capacity register
		SBS_REG_DESIGN_VOLTAGE =                        0x19,   ///< design voltage register
		SBS_REG_MANUFACTURER_NAME =                     0x20,   ///< manufacturer name
		SBS_REG_MANUFACTURE_DATE =                      0x1B,   ///< manufacture date register
		SBS_REG_SERIAL_NUMBER =                         0x1C,   ///< serial number register
		SBS_REG_MANUFACTURER_ACCESS =                   0x00,
		SBS_REG_MANUFACTURER_DATA =                     0x23,
	};

	static constexpr size_t MANUFACTURER_NAME_SIZE =        21;     ///< manufacturer name data size

public:
	SBSBatteryBase(SMBus *interface);

	~SBSBatteryBase();

	virtual int populate_startup_data();

	virtual int populate_runtime_data(battery_status_s &msg);

	virtual void run();

protected:
	SMBus *_interface;

	perf_counter_t _cycle{nullptr};

	uORB::PublicationMulti<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};

	uint8_t _cell_count{0};
	float _design_voltage{0};
	uint16_t _design_capacity{0};
	uint16_t _actual_capacity{0};
	uint16_t _cycle_count{0};
	uint16_t _serial_number{0};
	char _manufacturer_name[MANUFACTURER_NAME_SIZE + 1] {};	// Plus one for terminator
	uint16_t _manufacture_date{0};

	SBSBatteryBase(const SBSBatteryBase &) = delete;
	SBSBatteryBase operator=(const SBSBatteryBase &) = delete;

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		ModuleParams,
		(ParamFloat<px4::params::BAT_CRIT_THR>)		_param_bat_crit_thr,
		(ParamFloat<px4::params::BAT_LOW_THR>)  	_param_bat_low_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>)	_param_bat_emergen_thr,
		(ParamInt<px4::params::SBS_BAT_N_CELLS>)	_param_sbs_bat_n_cells,
		(ParamFloat<px4::params::SBS_BAT_C_MULT>)	_param_sbs_bat_c_mult
	)
};


template<typename ThisDriver>
class SBSBattery : public SBSBatteryBase, public I2CSPIDriver<ThisDriver>
{
public:
	static constexpr int NO_SUCH_COMMAND = 0xABCD;

	SBSBattery(I2CSPIBusOption bus_option, const int bus, SMBus *interface);
	~SBSBattery();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);

	void print_status() override;

	void RunImpl();

	/**
	 * Handle a command verb.
	 *
	 * Returns NO_SUCH_COMMAND if no such command verb exists.
	 */
	static int handle_command(const char *verb, BusCLIArguments &cli, BusInstanceIterator &iterator,
				  int argc, char *argv[]);

	static void print_module_description();
	static void print_usage_commands();
	static void print_usage();

	static int main(int argc, char *argv[]);
};

template<typename ThisDriver>
SBSBattery<ThisDriver>::SBSBattery(I2CSPIBusOption bus_option, const int bus, SMBus *interface) :
	SBSBatteryBase(interface),
	I2CSPIDriver<ThisDriver>(ThisDriver::MOD_NAME, px4::device_bus_to_wq(interface->get_device_id()),
				 bus_option, bus, interface->get_device_address())
{
}

template<typename ThisDriver>
SBSBattery<ThisDriver>::~SBSBattery()
{
}

template<typename ThisDriver>
void SBSBattery<ThisDriver>::RunImpl()
{
	run();
}

template<typename ThisDriver>
I2CSPIDriverBase *SBSBattery<ThisDriver>::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	SMBus *interface = new SMBus(iterator.bus(), cli.i2c_address);

	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	SBSBattery<ThisDriver> *instance = new ThisDriver(iterator.configuredBusOption(), iterator.bus(), interface);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	// TODO: need a better probe mechanism here?
	int ret = instance->populate_startup_data();

	if (ret != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->ScheduleOnInterval(ThisDriver::MEASUREMENT_INTERVAL_US);

	return instance;
}

template<typename ThisDriver>
int SBSBattery<ThisDriver>::handle_command(const char *verb, BusCLIArguments &cli, BusInstanceIterator &iterator,
		int argc, char *argv[])
{
	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return NO_SUCH_COMMAND;
}

template<typename ThisDriver>
void SBSBattery<ThisDriver>::print_status()
{
	I2CSPIDriver<ThisDriver>::print_status();

	// TODO: print static pack info here?

	perf_print_counter(_cycle);
}

template<typename ThisDriver>
void SBSBattery<ThisDriver>::print_module_description()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for generic SMBUS Compatible smart-batteries.

### Examples
To read manufacturer information
$ SBSBattery -X man_info

)DESCR_STR");
}

template<typename ThisDriver>
void SBSBattery<ThisDriver>::print_usage_commands()
{
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(ThisDriver::BATT_ADDR);

	// TODO: implement
	PRINT_MODULE_USAGE_COMMAND_DESCR("man_info", "Prints manufacturer info.");
}

template<typename ThisDriver>
void SBSBattery<ThisDriver>::print_usage()
{
	ThisDriver::print_module_description();
	PRINT_MODULE_USAGE_NAME(ThisDriver::MOD_NAME, "driver");

	ThisDriver::print_usage_commands();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

template<typename ThisDriver>
int SBSBattery<ThisDriver>::main(int argc, char *argv[])
{
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = ThisDriver::BATT_ADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(ThisDriver::MOD_NAME, cli, DRV_BAT_DEVTYPE_SMBUS);

	int res = ThisDriver::handle_command(verb, cli, iterator, argc, argv);

	if (res == NO_SUCH_COMMAND) {
		ThisDriver::print_usage();
		return -1;
	} else {
		return res;
	}
}
