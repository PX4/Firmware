/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file lis3mdl.h
 *
 * Shared defines for the LIS3MDL driver.
 */

#pragma once

#include <float.h>

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>

#include <lib/conversion/rotation.h>
#include <systemlib/err.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <perf/perf_counter.h>
#include <px4_platform_common/defines.h>

/**
 * LIS3MDL internal constants and data structures.
 */

/* Max measurement rate is 80Hz */
#define LIS3MDL_CONVERSION_INTERVAL     (1000000 / 80)  /* 12,500 microseconds */

#define NUM_BUS_OPTIONS                 (sizeof(lis3mdl::bus_options)/sizeof(lis3mdl::bus_options[0]))

#define ADDR_WHO_AM_I                   0x0f
#define ID_WHO_AM_I                     0x3d

#define ADDR_CTRL_REG1                  0x20
#define ADDR_CTRL_REG2                  0x21
#define ADDR_CTRL_REG3                  0x22
#define ADDR_CTRL_REG4                  0x23
#define ADDR_CTRL_REG5                  0x24

#define ADDR_STATUS_REG                 0x27
#define ADDR_OUT_X_L                    0x28
#define ADDR_OUT_X_H                    0x29
#define ADDR_OUT_Y_L                    0x2a
#define ADDR_OUT_Y_H                    0x2b
#define ADDR_OUT_Z_L                    0x2c
#define ADDR_OUT_Z_H                    0x2d
#define ADDR_OUT_T_L                    0x2e
#define ADDR_OUT_T_H                    0x2f

#define MODE_REG_CONTINOUS_MODE         (0 << 0)
#define MODE_REG_SINGLE_MODE            (1 << 0) /* default */

#define CNTL_REG1_DEFAULT 0xFC
#define CNTL_REG2_DEFAULT 0x00
#define CNTL_REG3_DEFAULT 0x00
#define CNTL_REG4_DEFAULT 0x0C
#define CNTL_REG5_DEFAULT 0x00

/* interface factories */
extern device::Device *LIS3MDL_SPI_interface(int bus);
extern device::Device *LIS3MDL_I2C_interface(int bus);
typedef device::Device *(*LIS3MDL_constructor)(int);

enum LIS3MDL_BUS {
	LIS3MDL_BUS_ALL = 0,
	LIS3MDL_BUS_I2C_INTERNAL,
	LIS3MDL_BUS_I2C_EXTERNAL,
	LIS3MDL_BUS_SPI
};

enum OPERATING_MODE {
	CONTINUOUS = 0,
	SINGLE
};


class LIS3MDL : public device::CDev, public px4::ScheduledWorkItem
{
public:
	LIS3MDL(device::Device *interface, const char *path, enum Rotation rotation);

	virtual ~LIS3MDL();

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

	float _range_ga;
	float _range_scale;

	uint8_t _check_state_cnt;
	uint8_t _cntl_reg1;
	uint8_t _cntl_reg2;
	uint8_t _cntl_reg3;
	uint8_t _cntl_reg4;
	uint8_t _cntl_reg5;
	uint8_t _range_bits;
	uint8_t _temperature_counter;
	uint8_t _temperature_error_count;


	/**
	 * @brief Performs the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *       will however reflect the uncalibrated sensor state until
	 *       the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int calibrate(struct file *file_pointer, unsigned enable);

	/**
	 * Collect the result of the most recent measurement.
	 */
	int collect();

	/**
	* Check the current scale calibration
	*
	* @return 0 if scale calibration is ok, 1 else
	*/
	int check_scale();

	/**
	* Check the current offset calibration
	*
	* @return 0 if offset calibration is ok, 1 else
	*/
	int check_offset();

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

	/**
	 * @brief Performs the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *       will however reflect the uncalibrated sensor state until
	 *       the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 */
	int set_excitement(unsigned enable);

	/**
	 * @brief Sets the sensor internal range to handle at least the argument in Gauss.
	 *
	 * @param range The sensor range value to be set.
	 */
	int set_range(unsigned range);

	/**
	 * @brief Reads a register.
	 *
	 * @param reg           The register to read.
	 * @param val           The value read.
	 * @return              OK on read success.
	 */
	int read_reg(uint8_t reg, uint8_t &val);

	/**
	 * @brief  Writes a register.
	 *
	 * @param reg           The register to write.
	 * @param val           The value to write.
	 * @return              OK on write success.
	 */
	int write_reg(uint8_t reg, uint8_t val);

	/* this class has pointer data members, do not allow copying it */
	LIS3MDL(const LIS3MDL &);

	LIS3MDL operator=(const LIS3MDL &);
}; // class LIS3MDL
