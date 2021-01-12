/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/adc_report.h>
#include <uORB/Publication.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#define ADDRESSPOINTER_REG_CONVERSATION    0x00
#define ADDRESSPOINTER_REG_CONFIG    0x01
#define ADDRESSPOINTER_REG_LO_THRESH    0x02
#define ADDRESSPOINTER_REG_HI_THRESH    0x03
#define CONVERSION_REG_RESET    0x00
#define CONFIG_HIGH_OS_RESET    0x80
#define CONFIG_HIGH_OS_NOACT    0x00
#define CONFIG_HIGH_OS_START_SINGLE    0x80
#define CONFIG_HIGH_MUX_RESET    0x00
#define CONFIG_HIGH_MUX_P0N1    0x00
#define CONFIG_HIGH_MUX_P0N3    0x10
#define CONFIG_HIGH_MUX_P1N3    0x20
#define CONFIG_HIGH_MUX_P2N3    0x30
#define CONFIG_HIGH_MUX_P0NG    0x40
#define CONFIG_HIGH_MUX_P1NG    0x50
#define CONFIG_HIGH_MUX_P2NG    0x60
#define CONFIG_HIGH_MUX_P3NG    0x70
#define CONFIG_HIGH_PGA_RESET    0x02
#define CONFIG_HIGH_PGA_6144    0x00
#define CONFIG_HIGH_PGA_4096    0x02
#define CONFIG_HIGH_PGA_2048    0x04
#define CONFIG_HIGH_PGA_1024    0x06
#define CONFIG_HIGH_PGA_0512    0x08
#define CONFIG_HIGH_PGA_0256    0x0a
#define CONFIG_HIGH_MODE_RESET    0x01
#define CONFIG_HIGH_MODE_SS    0x01
#define CONFIG_HIGH_MODE_CC    0x00

#define CONFIG_LOW_DR_RESET    0x80
#define CONFIG_LOW_DR_8SPS    0x00
#define CONFIG_LOW_DR_16SPS    0x20
#define CONFIG_LOW_DR_32SPS    0x40
#define CONFIG_LOW_DR_64SPS    0x60
#define CONFIG_LOW_DR_128SPS    0x80
#define CONFIG_LOW_DR_250SPS    0xa0
#define CONFIG_LOW_DR_475SPS    0xc0
#define CONFIG_LOW_DR_860SPS    0xe0
#define CONFIG_LOW_COMP_MODE_RESET    0x00
#define CONFIG_LOW_COMP_MODE_TRADITIONAL    0x00
#define CONFIG_LOW_COMP_MODE_WINDOW    0x10
#define CONFIG_LOW_COMP_POL_RESET    0x00
#define CONFIG_LOW_COMP_POL_ACTIVE_LOW    0x00
#define CONFIG_LOW_COMP_POL_ACTIVE_HIGH    0x08
#define CONFIG_LOW_COMP_LAT_DEFAULT    0x00
#define CONFIG_LOW_COMP_LAT_NONE    0x00
#define CONFIG_LOW_COMP_LAT_LATCH    0x04
#define CONFIG_LOW_COMP_QU_DEFAULT    0x03
#define CONFIG_LOW_COMP_QU_AFTER1    0x00
#define CONFIG_LOW_COMP_QU_AFTER2    0x01
#define CONFIG_LOW_COMP_QU_AFTER4    0x02
#define CONFIG_LOW_COMP_QU_DISABLE    0x03

using namespace time_literals;

/*
 * This driver configure ADS1115 into 4 channels with gnd as baseline.
 * Start each sample cycle by setting sample channel.
 * PGA set to 6.144V
 * SPS set to 256
 * Valid output ranges from 0 to 32767 on each channel.
 */
class ADS1115 : public device::I2C, public I2CSPIDriver<ADS1115>
{
public:
	ADS1115(I2CSPIBusOption bus_option, int bus, int addr, int bus_frequency);
	~ADS1115() override;

	int Begin();

	int init() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);

	static void print_usage();

	void RunImpl();

protected:

	adc_report_s _adc_report = {};

	void print_status() override;

	void exit_and_cleanup() override;

private:

	uORB::Publication<adc_report_s>		_to_adc_report{ORB_ID(adc_report)};

	static const hrt_abstime	SAMPLE_INTERVAL{50_ms};

	perf_counter_t			_cycle_perf;

	int     _channel_cycle_count = 0;

	// ADS1115 logic part
	enum ChannelSelection {
		Invalid = -1, A0 = 0, A1, A2, A3
	};
	/* set multiplexer to specific channel */
	int setChannel(ChannelSelection ch);
	/* return true if sample result is valid */
	bool isSampleReady();
	/*
	 * get adc sample. return the channel being measured.
	 * Invalid indicates sample failure.
	 */
	ChannelSelection getMeasurement(int16_t *value);
	/*
	 * get adc sample and automatically switch to next channel and start another measurement
	 */
	ChannelSelection cycleMeasure(int16_t *value);

	int readReg(uint8_t addr, uint8_t *buf, size_t len);

	int writeReg(uint8_t addr, uint8_t *buf, size_t len);

};