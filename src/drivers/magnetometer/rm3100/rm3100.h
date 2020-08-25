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

/**
 * @file rm3100.h
 *
 * Shared defines for the RM3100 driver.
 */

#pragma once

#include <float.h>

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>

#include <lib/conversion/rotation.h>

#include <perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <systemlib/err.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

/**
 * RM3100 internal constants and data structures.
 */

/* At 146 Hz we encounter errors, 100 Hz is safer */
#define RM3100_CONVERSION_INTERVAL	10000	// Microseconds, corresponds to 100 Hz (cycle count 200 on 3 axis)
#define UTESLA_TO_GAUSS			100.0f
#define RM3100_SENSITIVITY		75.0f

#define ADDR_POLL		0x00
#define ADDR_CMM		0x01
#define ADDR_CCX		0x04
#define ADDR_CCY		0x06
#define ADDR_CCZ		0x08
#define ADDR_TMRC		0x0B
#define ADDR_MX			0x24
#define ADDR_MY			0x27
#define ADDR_MZ			0x2A
#define ADDR_BIST		0x33
#define ADDR_STATUS		0x34
#define ADDR_HSHAKE		0x35
#define ADDR_REVID		0x36

#define CCX_DEFAULT_MSB		0x00
#define CCX_DEFAULT_LSB		0xC8
#define CCY_DEFAULT_MSB		CCX_DEFAULT_MSB
#define CCY_DEFAULT_LSB		CCX_DEFAULT_LSB
#define CCZ_DEFAULT_MSB		CCX_DEFAULT_MSB
#define CCZ_DEFAULT_LSB		CCX_DEFAULT_LSB
#define CMM_DEFAULT		0x70	// No continuous mode
#define CONTINUOUS_MODE		(1 << 0)
#define POLLING_MODE		(0 << 0)
#define TMRC_DEFAULT		0x94
#define BIST_SELFTEST		0x8F
#define BIST_DEFAULT		0x00
#define BIST_XYZ_OK		((1 << 4) | (1 << 5) | (1 << 6))
#define STATUS_DRDY		(1 << 7)
#define POLL_XYZ		0x70
#define RM3100_REVID		0x22

#define NUM_BUS_OPTIONS		(sizeof(bus_options)/sizeof(bus_options[0]))

/* interface factories */
extern device::Device *RM3100_SPI_interface(int bus);
extern device::Device *RM3100_I2C_interface(int bus);
typedef device::Device *(*RM3100_constructor)(int);

enum RM3100_BUS {
	RM3100_BUS_ALL = 0,
	RM3100_BUS_I2C_INTERNAL,
	RM3100_BUS_I2C_EXTERNAL,
	RM3100_BUS_SPI_INTERNAL,
	RM3100_BUS_SPI_EXTERNAL
};

enum OPERATING_MODE {
	CONTINUOUS = 0,
	SINGLE
};


class RM3100 : public device::CDev, public px4::ScheduledWorkItem
{
public:
	RM3100(device::Device *interface, const char *path, enum Rotation rotation);

	virtual ~RM3100();

	virtual int init();

	virtual int ioctl(struct file *file_pointer, int cmd, unsigned long arg);

	virtual int read(struct file *file_pointer, char *buffer, size_t buffer_len);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Configures the device with default register values.
	 */
	int set_default_register_values();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

protected:
	Device *_interface;

private:

	ringbuffer::RingBuffer *_reports;

	struct mag_calibration_s _scale;

	sensor_mag_s _last_report {};      /**< used for info() */

	orb_advert_t _mag_topic;

	perf_counter_t _comms_errors;
	perf_counter_t _conf_errors;
	perf_counter_t _range_errors;
	perf_counter_t _sample_perf;

	/* status reporting */
	bool _calibrated;                       /**< the calibration is valid */
	bool _continuous_mode_set;

	enum OPERATING_MODE _mode;
	enum Rotation _rotation;

	unsigned int _measure_interval;

	int _class_instance;
	int _orb_class_instance;

	float _range_scale;

	uint8_t _check_state_cnt;

	/**
	 * Collect the result of the most recent measurement.
	 */
	int collect();

	/**
	 * Run sensor self-test
	 *
	 * @return 0 if self-test is ok, 1 else
	 */
	int self_test();

	/**
	 * Check whether new data is available or not
	 *
	 * @return 0 if new data is available, 1 else
	 */
	int check_measurement();

	/**
	* Converts int24_t stored in 32-bit container to int32_t
	*/
	void convert_signed(int32_t *n);

	/**
	 * @brief Performs a poll cycle; collect from the previous measurement
	 *        and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void Run() override;

	/**
	 * Issue a measurement command.
	 *
	 * @return              OK if the measurement command was successful.
	 */
	int measure();

	/**
	 * @brief Resets the device
	 */
	int reset();

	/**
	 * @brief Initialises the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/* this class has pointer data members, do not allow copying it */
	RM3100(const RM3100 &);

	RM3100 operator=(const RM3100 &);
}; // class RM3100
