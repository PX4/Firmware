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

#include <cstring>

#include <drivers/device/Device.hpp>
#include <drivers/device/i2c.h>
#include <drivers/device/spi.h>
#include <drivers/drv_baro.h>
#include <lib/cdev/CDev.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>
#include <uORB/uORB.h>


static constexpr uint8_t WHO_AM_I = 0x0F;
static constexpr uint8_t LPS22HB_ID_WHO_AM_I = 0xB1;

static constexpr uint8_t CTRL_REG1 = 0x10;
static constexpr uint8_t CTRL_REG2 = 0x11;
static constexpr uint8_t CTRL_REG3 = 0x12;

#define BOOT		(1 << 7)
#define FIFO_EN		(1 << 6)
#define STOP_ON_FTH	(1 << 5)
#define IF_ADD_INC	(1 << 4)
#define I2C_DIS		(1 << 3)
#define SWRESET		(1 << 2)
#define ONE_SHOT	(1 << 0)

static constexpr uint8_t STATUS = 0x27;

#define T_OR (1 << 5) // Temperature data overrun.
#define P_OR (1 << 4) // Pressure data overrun.
#define T_DA (1 << 1) // Temperature data available.
#define P_DA (1 << 0) // Pressure data available.

static constexpr uint8_t PRESS_OUT_XL = 0x28;
static constexpr uint8_t PRESS_OUT_L = 0x29;
static constexpr uint8_t PRESS_OUT_H = 0x2A;

static constexpr uint8_t TEMP_OUT_L = 0x2B;
static constexpr uint8_t TEMP_OUT_H = 0x2C;

/* interface factories */
extern device::Device *LPS22HB_SPI_interface(int bus, uint32_t devid, int bus_frequency, spi_mode_e spi_mode);
extern device::Device *LPS22HB_I2C_interface(int bus, int bus_frequency);

class LPS22HB : public cdev::CDev, public I2CSPIDriver<LPS22HB>
{
public:
	LPS22HB(I2CSPIBusOption bus_option, int bus, device::Device *interface);
	virtual ~LPS22HB();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int		init();

	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	void			print_status();

	void			RunImpl();

protected:
	device::Device			*_interface;

private:
	unsigned		_measure_interval{0};

	bool			_collect_phase{false};

	orb_advert_t		_baro_topic{nullptr};

	int			_orb_class_instance{-1};
	int			_class_instance{-1};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	sensor_baro_s	_last_report{};           /**< used for info() */

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Reset the device
	 */
	int			reset();


	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/* this class has pointer data members, do not allow copying it */
	LPS22HB(const LPS22HB &);
	LPS22HB operator=(const LPS22HB &);
};
