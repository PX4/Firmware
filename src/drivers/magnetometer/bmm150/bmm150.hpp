/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <px4_platform_common/log.h>

#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>


#define BMM150_SLAVE_ADDRESS                 0x10

#define BMM150_BUS_SPEED                     1000*100

/* Chip Identification number */
#define BMM150_CHIP_ID                       0x32

/* Chip Id Register */
#define BMM150_CHIP_ID_REG                   0x40

/* Data Registers */
#define BMM150_DATA_X_LSB_REG                0x42
#define BMM150_DATA_X_MSB_REG                0x43
#define BMM150_DATA_Y_LSB_REG                0x44
#define BMM150_DATA_Y_MSB_REG                0x45
#define BMM150_DATA_Z_LSB_REG                0x46
#define BMM150_DATA_Z_MSB_REG                0x47
#define BMM150_R_LSB                         0x48
#define BMM150_R_MSB                         0x49

/* Interrupt status registers */
#define BMM150_INT_STATUS_REG                0x4A

/* Control Registers */
#define BMM150_POWER_CTRL_REG                0x4B
#define BMM150_CTRL_REG                      0x4C
#define BMM150_INT_SETT_CTRL_REG             0x4D
#define BMM150_AXES_EN_CTRL_REG              0x4E
#define BMM150_LOW_THRES_SETT_REG            0x4F
#define BMM150_HIGH_THERS_SETT_REG           0x50

/* Repetitions control registers */
#define BMM150_XY_REP_CTRL_REG               0x51
#define BMM150_Z_REP_CTRL_REG                0x52

/* Preset mode definitions */
#define BMM150_PRESETMODE_LOWPOWER            1
#define BMM150_PRESETMODE_REGULAR             2
#define BMM150_PRESETMODE_HIGHACCURACY        3
#define BMM150_PRESETMODE_ENHANCED            4


/* Data rate value definitions */
#define BMM150_DATA_RATE_10HZ                0x00
#define BMM150_DATA_RATE_02HZ                0x08
#define BMM150_DATA_RATE_06HZ                0x10
#define BMM150_DATA_RATE_08HZ                0x18
#define BMM150_DATA_RATE_15HZ                0x20
#define BMM150_DATA_RATE_20HZ                0x28
#define BMM150_DATA_RATE_25HZ                0x30
#define BMM150_DATA_RATE_30HZ                0x38

/* Advance self-test settings Definitions */
#define BMM150_ADV_ST_OFF                    0x00
#define BMM150_ADV_ST_NEG                    0x80
#define BMM150_ADV_ST_POS                    0xC0


/* Interrupt settings and axes enable bits definitions */
#define BMM150_CHANNEL_X_ENABLE              0x08
#define BMM150_CHANNEL_Y_ENABLE              0x10


/*Overflow Definitions */
/* compensated output value returned if sensor had overflow */
#define BMM150_OVERFLOW_OUTPUT               -32768
#define BMM150_OVERFLOW_OUTPUT_S32           ((int32_t)(-2147483647-1))
#define BMM150_OVERFLOW_OUTPUT_FLOAT         0.0f
#define BMM150_FLIP_OVERFLOW_ADCVAL          -4096
#define BMM150_HALL_OVERFLOW_ADCVAL          -16384


/* Preset modes - Repetitions-XY Rates */
#define BMM150_LOWPOWER_REPXY                 1
#define BMM150_REGULAR_REPXY                  4
#define BMM150_HIGHACCURACY_REPXY             23
#define BMM150_ENHANCED_REPXY                 7

/* Preset modes - Repetitions-Z Rates */
#define BMM150_LOWPOWER_REPZ                  2
#define BMM150_REGULAR_REPZ                   14
#define BMM150_HIGHACCURACY_REPZ              82
#define BMM150_ENHANCED_REPZ                  26

/* Preset modes - Data rates */
#define BMM150_LOWPOWER_DR                   BMM150_DATA_RATE_30HZ
#define BMM150_REGULAR_DR                    BMM150_DATA_RATE_30HZ
#define BMM150_HIGHACCURACY_DR               BMM150_DATA_RATE_20HZ
#define BMM150_ENHANCED_DR                   BMM150_DATA_RATE_10HZ


/* Power modes value definitions */
#define BMM150_NORMAL_MODE                   0x00
#define BMM150_FORCED_MODE                   0x02
#define BMM150_SLEEP_MODE                    0x06

/* Default power mode */
#define BMM150_DEFAULT_POWER_MODE           BMM150_NORMAL_MODE

/* Default output data rate */
#define BMM150_DEFAULT_ODR                  BMM150_DATA_RATE_30HZ

/* Maximum output data rate */
#define BMM150_MAX_DATA_RATE                100

/* Default BMM150_INT_SETT_CTRL_REG Value */
#define BMM150_DEFAULT_INT_SETT             0x3F

/* Trim Extended Registers */
#define BMM150_DIG_X1                       0x5D
#define BMM150_DIG_Y1                       0x5E
#define BMM150_DIG_Z4_LSB                   0x62
#define BMM150_DIG_Z4_MSB                   0x63
#define BMM150_DIG_X2                       0x64
#define BMM150_DIG_Y2                       0x65
#define BMM150_DIG_Z2_LSB                   0x68
#define BMM150_DIG_Z2_MSB                   0x69
#define BMM150_DIG_Z1_LSB                   0x6A
#define BMM150_DIG_Z1_MSB                   0x6B
#define BMM150_DIG_XYZ1_LSB                 0x6C
#define BMM150_DIG_XYZ1_MSB                 0x6D
#define BMM150_DIG_Z3_LSB                   0x6E
#define BMM150_DIG_Z3_MSB                   0x6F
#define BMM150_DIG_XY2                      0x70
#define BMM150_DIG_XY1                      0x71


/* Mask definitions for power mode */
#define BMM150_POWER_MASK                   0x06

/* Mask definitions for data rate */
#define BMM150_OUTPUT_DATA_RATE_MASK        0x38

#define BMM150_SOFT_RESET_VALUE             0x82

/* Mask definitions for Soft-Reset */
#define BMM150_SOFT_RESET_MASK              0x82

/* This value is set based on Max output data rate value */
#define BMM150_CONVERSION_INTERVAL          (1000000 / 100) /* microseconds */



struct bmm150_data {
	int16_t x;
	int16_t y;
	int16_t z;
};


class BMM150 : public device::I2C, public I2CSPIDriver<BMM150>
{
public:
	BMM150(I2CSPIBusOption bus_option, const int bus, int bus_frequency, enum Rotation rotation);
	virtual ~BMM150();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int             init() override;
	ssize_t       read(struct file *filp, char *buffer, size_t buflen) override;
	int       ioctl(struct file *filp, int cmd, unsigned long arg) override;

	void            print_status() override;

	void        print_registers();

	void	RunImpl();

	void custom_method(const BusCLIArguments &cli) override;

protected:
	int       probe() override;

private:

	/* altitude conversion calibration */
	unsigned        _call_interval;


	sensor_mag_s _report {};
	ringbuffer::RingBuffer  *_reports;

	bool            _collect_phase;

	struct mag_calibration_s    _scale;
	float           _range_scale;

	orb_advert_t        _topic;
	int         _orb_class_instance;
	int         _class_instance;
	uint8_t     _power;
	uint8_t     _output_data_rate;
	bool        _calibrated;        /**< the calibration is valid */

	int8_t dig_x1;/**< trim x1 data */
	int8_t dig_y1;/**< trim y1 data */

	int8_t dig_x2;/**< trim x2 data */
	int8_t dig_y2;/**< trim y2 data */

	uint16_t dig_z1;/**< trim z1 data */
	int16_t dig_z2;/**< trim z2 data */
	int16_t dig_z3;/**< trim z3 data */
	int16_t dig_z4;/**< trim z4 data */

	uint8_t dig_xy1;/**< trim xy1 data */
	int8_t dig_xy2;/**< trim xy2 data */

	uint16_t dig_xyz1;/**< trim xyz1 data */

	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _good_transfers;
	perf_counter_t      _measure_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _duplicates;

	enum Rotation       _rotation;
	bool            _got_duplicate;

	sensor_mag_s   _last_report {};          /**< used for info() */

	int             init_trim_registers(void);

	/**
	 * Start automatic measurement.
	 */
	void            start();

	int     measure(); //start measure
	int     collect(); //get results and publish

	/**
	 * Read the specified number of bytes from BMM150.
	 *
	 * @param reg       The register to read.
	 * @param data      Pointer to buffer for bytes read.
	 * @param len       Number of bytes to read
	 * @return          OK if the transfer was successful, -errno otherwise.
	 */
	int             get_data(uint8_t reg, uint8_t *data, unsigned len);

	/**
	 * Resets the chip.
	 */
	int             reset();

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int             self_test();

	/**
	 * Read a register from the BMM150
	 *
	 * @param reg     The register to read.
	 * @return        The value that was read.
	 */
	uint8_t         read_reg(uint8_t reg);

	/**
	 * Write a register in the BMM150
	 *
	 * @param reg       The register to write.
	 * @param value     The new value to write.
	 * @return          OK if the transfer was successful, -errno otherwise.
	 */
	int             write_reg(uint8_t reg, uint8_t value);

	/**
	 * Modify a register in the BMM150
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg       The register to modify.
	 * @param clearbits Bits in the register to clear.
	 * @param setbits   Bits in the register to set.
	 */
	void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/*
	  set the power mode of BMM150.
	*/
	int             set_power_mode(uint8_t power);

	/*
	  Set the data rate of BMM150.
	*/
	int             set_data_rate(uint8_t data_rate);

	/*
	  Set the XY-repetitions
	 */
	int             set_rep_xy(uint8_t rep_xy);

	/*
	  Set the Z- repetitions number
	 */
	int             set_rep_z(uint8_t rep_z);

	/*
	   Set the preset modes for BMM150 sensor.The preset mode setting is
	   depend on Data Rate, XY and Z repetitions
	 */
	int             set_presetmode(uint8_t presetmode);


	/* do not allow to copy this class due to pointer data members */
	BMM150(const BMM150 &);
	BMM150 operator=(const BMM150 &);

};


