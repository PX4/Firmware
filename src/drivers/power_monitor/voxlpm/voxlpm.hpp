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
 * Shared defines for the voxlpm (QTY2 LTC2946) driver.
 *
 * This is roughly what's goin on:
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
#include <uORB/topics/battery_status.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/parameter_update.h>

/*
 * Note that these are unshifted addresses.
 */
#define VOXLPM_LTC2946_ADDR_VBATT		0x6a // 0x6a  = 0xd4 >> 1
#define VOXLPM_LTC2946_ADDR_P5VD		0x6b // 0x6b  = 0xd6 >> 1

#define VOXLPM_LTC2946_CTRLA_REG		0x00
#define VOXLPM_LTC2946_CTRLB_REG		0x01

#define VOXLPM_LTC2946_POWER_MSB2_REG		0x05
#define VOXLPM_LTC2946_CTRLB_MSG1_REG		0x06
#define VOXLPM_LTC2946_CTRLB_LSB_REG		0x07

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
#define DEFAULT_CTRLA_REG_VAL			0x18

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
#define DEFAULT_CTRLB_REG_VAL			0x01

/* 12 bits */
#define VOXLPM_LTC2946_RESOLUTION 		4095.0f

/* VFS Full-Scale Voltage, SENSE+ */
#define VOXLPM_LTC2946_VFS_SENSE 		102.4f

/* VFS Full-Scale Voltage, delta sense */
#define VOXLPM_LTC2946_VFS_DELTA_SENSE 		0.1024f

/* Power sense resistor for battery current */
#define VOXLPM_RSENSE_VBATT			0.0005f

/* Power sense resistor for 5VDC output current */
#define VOXLPM_RSENSE_5VOUT			0.005f

enum VOXLPM_CH_TYPE {
	VOXLPM_CH_TYPE_VBATT = 0,
	VOXLPM_CH_TYPE_P5VDC
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
	void			print_status() override;

	void 			RunImpl();
private:
	void 			start();
	int 			measure();

	static constexpr unsigned 		_meas_interval{100000}; // 100ms
	perf_counter_t		_sample_perf;

	uORB::PublicationMulti<power_monitor_s>		_pm_pub_topic{ORB_ID(power_monitor)};
	uORB::Subscription _parameter_sub{ORB_ID(parameter_update)};

	power_monitor_s 	_pm_status{};

	const VOXLPM_CH_TYPE	_ch_type;
	float			_voltage{0.0f};
	float			_amperage{0.0f};
	float			_rsense{0.0f};

	Battery 		_battery;

	uint8_t 		read_reg(uint8_t addr);
	int 			read_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len);
	int 			write_reg(uint8_t value, uint8_t addr);
};
