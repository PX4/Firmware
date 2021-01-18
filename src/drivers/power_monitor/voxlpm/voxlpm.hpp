/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file voxlpm.hpp
 *
 * Shared defines for the voxlpm driver.
 *
 * This is roughly what's goin on:
 *
 * - VOXLPM v2 (QTY2 LTC2946) -
 *
 *             +~~~~~~~~~~~~~~+
 *  VBATT -----| RSENSE_VBATT | ----------+---------------------> VBATT TO ESCS
 *     |       +~~~~~~~~~~~~~~+           |
 *     |              |          +--------+------+
 *     +----+    +----+          | 5V REGULATOR  |
 *          |    |               +--------+------+
 *          |    |                        |   +~~~~~~~~~~~~~~+
 *          |    |                        +---| RSENSE_5VOUT |---> 5VDC TO COMPUTE
 *          |    |                        |   +~~~~~~~~~~~~~~+
 *          |    |                        |         |
 *         V|    |A                      V|         |A
 *     #################              #################
 *     # LTC2946, 0x6a #              # LTC2946, 0x6b #
 *     #################              #################
 *
  * - VOXLPM v3 (QTY2 INA231) -
 *
 *             +~~~~~~~~~~~~~~+
 *  VBATT -----| RSENSE_VBATT | ----------+---------------------> VBATT TO ESCS
 *     |       +~~~~~~~~~~~~~~+           |
 *     |              |          +--------+------+
 *     +----+    +----+          | 5/12V REGULATOR  |
 *          |    |               +--------+------+
 *          |    |                        |   +~~~~~~~~~~~~~~+
 *          |    |                        +---| RSENSE_5VOUT |---> 5/12VDC TO COMPUTE/PERIPHERAL
 *          |    |                        |   +~~~~~~~~~~~~~~+
 *          |    |                        |         |
 *         V|    |A                      V|         |A
 *     #################              #################
 *     # INA231, 0x44  #              # INA231, 0x45  #
 *     #################              #################
 *
 *
 *     Publishes:                     Publishes:
 *     - ORB_ID(battery_status)
 *     - ORB_ID(power_monitor)        - ORB_ID(power_monitor)
 *
 */
#pragma once

#include <drivers/device/i2c.h>
#include <perf/perf_counter.h>

#include <px4_platform_common/i2c_spi_buses.h>

#include <battery/battery.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

/*
 * VOXLPM v2 - Note that these are unshifted addresses.
 */
#define VOXLPM_LTC2946_ADDR_VBATT		0x6a // 0x6a  = 0xd4 >> 1
#define VOXLPM_LTC2946_ADDR_P5VD		0x6b // 0x6b  = 0xd6 >> 1

#define VOXLPM_LTC2946_CTRLA_REG		0x00
#define VOXLPM_LTC2946_CTRLB_REG		0x01

#define VOXLPM_LTC2946_POWER_MSB2_REG		0x05
#define VOXLPM_LTC2946_CTRLB_MSG1_REG		0x06
#define VOXLPM_LTC2946_CTRLB_LSB_REG		0x07
#define VOXLPM_LTC2946_STATUS_REG		0x80

#define VOXLPM_LTC2946_DELTA_SENSE_MSB_REG	0x14
#define VOXLPM_LTC2946_DELTA_SENSE_LSB_REG	0x15

#define VOXLPM_LTC2946_VIN_MSB_REG		0x1E
#define VOXLPM_LTC2946_VIN_LSB_REG 		0x1F

/*
 * CTRLA (Address 0x00 - LTC2946_CTRLA_REG)
 *
 *   7 - [ADIN Configuration]
 *       0  --> ADIN Measured with Respect to GND
 * 6:5 - [Offset Calibratoin Configuration]
 *       00 --> Every Conversion
 * 4:3 - [Voltage Selection]
 *       11 --> SENSE+
 * 2:0 - [Channel Configuration]
 *       000 --> Alternate Voltage, Current Measurement
 */
#define DEFAULT_LTC2946_CTRLA_REG_VAL		0x18

/*
 * CTRLB (Address 0x01 - LTC2946_CTRLA_REG)
 *
 *   7 - [!ALERT Clear Enable ]
 *       0  --> Disable
 *   6 - [Shutdown]
 *       0  --> Power-Up
 *   5 - [Cleared on Read Control]
 *       0  --> Registers Not Affected by Reading
 *   4 - [Stuck Bus Timeout Auto Wake-Up]
 *       0 --> Disable
 * 3:2 - [Enable Accumulation]
 *       00 --> Accumulate
 * 1:0 - [Auto-Reset Mode/Reset]
 *       01 --> Enable Auto-Reset
 */
#define DEFAULT_LTC2946_CTRLB_REG_VAL		0x01

/* 12 bits */
#define VOXLPM_LTC2946_RESOLUTION 		4095.0f

/* VFS Full-Scale Voltage, SENSE+ */
#define VOXLPM_LTC2946_VFS_SENSE 		102.4f

/* VFS Full-Scale Voltage, delta sense */
#define VOXLPM_LTC2946_VFS_DELTA_SENSE 		0.1024f

/* Power sense resistor for battery current */
#define VOXLPM_LTC2946_VBAT_SHUNT		0.0005f

/* Power sense resistor for 5VDC output current */
#define VOXLPM_LTC2946_VREG_SHUNT		0.005f

/*
 * VOXLPM v3 - Coniguration from SBOS644C –FEBRUARY 2013–REVISED MARCH 2018
 *             http://www.ti.com/lit/ds/symlink/ina231.pdf
 */
#define VOXLPM_INA231_ADDR_VBATT		0x44
#define VOXLPM_INA231_ADDR_P5_12VDC		0x45

/* INA231 Registers addresses */
#define INA231_REG_CONFIG			0x00
#define INA231_REG_SHUNTVOLTAGE			0x01
#define INA231_REG_BUSVOLTAGE			0x02
#define INA231_REG_POWER			0x03
#define INA231_REG_CURRENT			0x04
#define INA231_REG_CALIBRATION			0x05
#define INA231_REG_MASKENABLE			0x06
#define INA231_REG_ALERTLIMIT			0x07

/* [0:2] Mode - Shunt and bus, 111, continuous (INA231A default) */
#define INA231_CONFIG_MODE			(0x07 << 0)
/* [5:3] Shunt Voltage Conversion Time, 100, 1.1ms (INA231A default) */
#define INA231_CONFIG_SHUNT_CT			(0x04 << 3)
/* [8:6] Shunt Voltage Conversion Time, 100, 1.1ms (INA231A default) */
#define INA231_CONFIG_BUS_CT			(0x04 << 6)
/* [11:9] Averaging Mode, 010, 16 */
#define INA231_CONFIG_AVG			(0x02 << 9)
/* [1] Reset bit */
#define INA231_RST_BIT				(0x01 << 15)
/* Configuration register settings */
#define INA231_CONFIG				(INA231_CONFIG_MODE+INA231_CONFIG_SHUNT_CT+INA231_CONFIG_BUS_CT+INA231_CONFIG_AVG)

#define INA231_CONST				0.00512f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define INA231_VBUSSCALE			0.00125f  /* LSB of bus voltage is 1.25 mV  */
#define INA231_VSHUNTSCALE			0.0000025f /* LSB of shunt voltage is 2.5 uV  */

/* From SCH-M00041 REVB */
#define VOXLPM_INA231_VBAT_SHUNT		0.0005f   /* VBAT shunt is 500 micro-ohm */
#define VOXLPM_INA231_VREG_SHUNT		0.005f    /* VREG output shunt is 5 milli-ohm */
#define VOXLPM_INA231_VBAT_MAX_AMPS		90.0f     /* 90.0 Amps max through VBAT sense resistor */
#define VOXLPM_INA231_VREG_MAX_AMPS		6.0f      /* 6.0 Amps max through VREG sense resistor */

/* ina231.pdf section 8.5 */
#define VOXLPM_INA231_VBAT_I_LSB		(VOXLPM_INA231_VBAT_MAX_AMPS/32768.0f)
#define VOXLPM_INA231_VREG_I_LSB		(VOXLPM_INA231_VREG_MAX_AMPS/32768.0f)

#define swap16(w)				__builtin_bswap16((w))

enum VOXLPM_TYPE {
	VOXLPM_UNKOWN,
	VOXLPM_TYPE_V2_LTC,
	VOXLPM_TYPE_V3_INA
};

enum VOXLPM_CH_TYPE {
	VOXLPM_CH_TYPE_VBATT = 0,
	VOXLPM_CH_TYPE_P5VDC,
	VOXLPM_CH_TYPE_P12VDC
};

class VOXLPM : public device::I2C, public ModuleParams, public I2CSPIDriver<VOXLPM>
{
public:
	VOXLPM(I2CSPIBusOption bus_option, const int bus, int bus_frequency, VOXLPM_CH_TYPE ch_type);
	virtual ~VOXLPM();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int		init();
	int			force_init();
	void			print_status() override;

	void 			RunImpl();

private:
	int			probe() override;
	void 			start();
	int 			measure();
	int 			load_params(VOXLPM_TYPE pm_type, VOXLPM_CH_TYPE ch_type);
	int 			init_ltc2946();
	int 			init_ina231();
	int 			measure_ltc2946();
	int 			measure_ina231();

	bool			_initialized;
	static constexpr unsigned _meas_interval_us{100_ms};
	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uORB::PublicationMulti<power_monitor_s>		_pm_pub_topic{ORB_ID(power_monitor)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	power_monitor_s 	_pm_status{};

	VOXLPM_TYPE		_pm_type{VOXLPM_UNKOWN};
	const VOXLPM_CH_TYPE	_ch_type;
	float			_voltage{0.0f};
	float			_amperage{0.0f};
	float			_rshunt{0.0005f};
	float			_vshunt{0.0f};
	float			_vshuntamps{0.0f};
	int16_t			_cal{0};

	Battery 		_battery;

	uint8_t 		read_reg(uint8_t addr);
	int 			read_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len);
	int 			write_reg(uint8_t addr, uint8_t value);
	int 			write_word_swapped(uint8_t addr, uint16_t value);
};
