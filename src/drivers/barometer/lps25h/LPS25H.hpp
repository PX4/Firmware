/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file lps25h.cpp
 *
 * Driver for the LPS25H barometer connected via I2C or SPI.
 */

#pragma once

#include "lps25h.h"

#include <drivers/device/Device.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/i2c_spi_buses.h>

/*
 * LPS25H internal constants and data structures.
 */

/* Max measurement rate is 25Hz */
#define LPS25H_CONVERSION_INTERVAL	(1000000 / 25)	/* microseconds */

#define ADDR_REF_P_XL		0x08
#define ADDR_REF_P_L		0x09
#define ADDR_REF_P_H		0x0A
#define ADDR_WHO_AM_I		0x0F
#define ADDR_RES_CONF		0x10
#define ADDR_CTRL_REG1		0x20
#define ADDR_CTRL_REG2		0x21
#define ADDR_CTRL_REG3		0x22
#define ADDR_CTRL_REG4		0x23
#define ADDR_INT_CFG		0x24
#define ADDR_INT_SOURCE		0x25

#define ADDR_STATUS_REG		0x27
#define ADDR_P_OUT_XL		0x28
#define ADDR_P_OUT_L		0x29
#define ADDR_P_OUT_H		0x2A
#define ADDR_TEMP_OUT_L		0x2B
#define ADDR_TEMP_OUT_H		0x2C

#define ADDR_FIFO_CTRL		0x2E
#define ADDR_FIFO_STATUS	0x2F
#define ADDR_THS_P_L		0x30
#define ADDR_THS_P_H		0x31

#define ADDR_RPDS_L		0x39
#define ADDR_RPDS_H		0x3A

/* Data sheet is ambigious if AVGT or AVGP is first */
#define RES_CONF_AVGT_8		0x00
#define RES_CONF_AVGT_32	0x01
#define RES_CONF_AVGT_128	0x02
#define RES_CONF_AVGT_512	0x03
#define RES_CONF_AVGP_8		0x00
#define RES_CONF_AVGP_32	0x04
#define RES_CONF_AVGP_128	0x08
#define RES_CONF_AVGP_512	0x0C

#define CTRL_REG1_SIM		(1 << 0)
#define CTRL_REG1_RESET_AZ	(1 << 1)
#define CTRL_REG1_BDU		(1 << 2)
#define CTRL_REG1_DIFF_EN	(1 << 3)
#define CTRL_REG1_PD		(1 << 7)
#define CTRL_REG1_ODR_SINGLE	(0 << 4)
#define CTRL_REG1_ODR_1HZ	(1 << 4)
#define CTRL_REG1_ODR_7HZ	(2 << 4)
#define CTRL_REG1_ODR_12HZ5	(3 << 4)
#define CTRL_REG1_ODR_25HZ	(4 << 4)

#define CTRL_REG2_ONE_SHOT	(1 << 0)
#define CTRL_REG2_AUTO_ZERO	(1 << 1)
#define CTRL_REG2_SWRESET	(1 << 2)
#define CTRL_REG2_FIFO_MEAN_DEC	(1 << 4)
#define CTRL_REG2_WTM_EN	(1 << 5)
#define CTRL_REG2_FIFO_EN	(1 << 6)
#define CTRL_REG2_BOOT		(1 << 7)

#define CTRL_REG3_INT1_S_DATA	0x0
#define CTRL_REG3_INT1_S_P_HIGH	0x1
#define CTRL_REG3_INT1_S_P_LOW	0x2
#define CTRL_REG3_INT1_S_P_LIM	0x3
#define CTRL_REG3_PP_OD		(1 << 6)
#define CTRL_REG3_INT_H_L	(1 << 7)

#define CTRL_REG4_P1_DRDY	(1 << 0)
#define CTRL_REG4_P1_OVERRUN	(1 << 1)
#define CTRL_REG4_P1_WTM	(1 << 2)
#define CTRL_REG4_P1_EMPTY	(1 << 3)

#define INTERRUPT_CFG_PH_E	(1 << 0)
#define INTERRUPT_CFG_PL_E	(1 << 1)
#define INTERRUPT_CFG_LIR	(1 << 2)

#define INT_SOURCE_PH		(1 << 0)
#define INT_SOURCE_PL		(1 << 1)
#define INT_SOURCE_IA		(1 << 2)

#define STATUS_REG_T_DA		(1 << 0)
#define STATUS_REG_P_DA		(1 << 1)
#define STATUS_REG_T_OR		(1 << 4)
#define STATUS_REG_P_OR		(1 << 5)

#define FIFO_CTRL_WTM_FMEAN_2	0x01
#define FIFO_CTRL_WTM_FMEAN_4	0x03
#define FIFO_CTRL_WTM_FMEAN_8	0x07
#define FIFO_CTRL_WTM_FMEAN_16	0x0F
#define FIFO_CTRL_WTM_FMEAN_32	0x1F
#define FIFO_CTRL_F_MODE_BYPASS	(0x0 << 5)
#define FIFO_CTRL_F_MODE_FIFO	(0x1 << 5)
#define FIFO_CTRL_F_MODE_STREAM	(0x2 << 5)
#define FIFO_CTRL_F_MODE_SFIFO	(0x3 << 5)
#define FIFO_CTRL_F_MODE_BSTRM	(0x4 << 5)
#define FIFO_CTRL_F_MODE_FMEAN	(0x6 << 5)
#define FIFO_CTRL_F_MODE_BFIFO	(0x7 << 5)

#define FIFO_STATUS_EMPTY	(1 << 5)
#define FIFO_STATUS_FULL	(1 << 6)
#define FIFO_STATUS_WTM		(1 << 7)

class LPS25H : public I2CSPIDriver<LPS25H>
{
public:
	LPS25H(I2CSPIBusOption bus_option, int bus, device::Device *interface);
	~LPS25H() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int		init();
	void		print_status();

	void			RunImpl();
private:

	void			start();
	int			reset();

	int			write_reg(uint8_t reg, uint8_t val);
	int			read_reg(uint8_t reg, uint8_t &val);

	int			measure();
	int			collect();

	PX4Barometer		_px4_barometer;
	device::Device		*_interface;

	unsigned		_measure_interval{0};
	bool			_collect_phase{false};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
};
