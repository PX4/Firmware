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
 * @file bmp388.h
 *
 * Shared defines for the bmp388 driver.
 */
#pragma once

#include <math.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>

#include "board_config.h"

// From https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3_defs.h

#define BMP3_CHIP_ID                      (0x50)

/* Over sampling macros */
#define BMP3_NO_OVERSAMPLING              (0x00)
#define BMP3_OVERSAMPLING_2X              (0x01)
#define BMP3_OVERSAMPLING_4X              (0x02)
#define BMP3_OVERSAMPLING_8X              (0x03)
#define BMP3_OVERSAMPLING_16X             (0x04)
#define BMP3_OVERSAMPLING_32X             (0x05)

/* Filter setting macros */
#define BMP3_IIR_FILTER_DISABLE           (0x00)
#define BMP3_IIR_FILTER_COEFF_1           (0x01)
#define BMP3_IIR_FILTER_COEFF_3           (0x02)
#define BMP3_IIR_FILTER_COEFF_7           (0x03)
#define BMP3_IIR_FILTER_COEFF_15          (0x04)
#define BMP3_IIR_FILTER_COEFF_31          (0x05)
#define BMP3_IIR_FILTER_COEFF_63          (0x06)
#define BMP3_IIR_FILTER_COEFF_127         (0x07)

/* Odr setting macros */
#define BMP3_ODR_200_HZ                   (0x00)
#define BMP3_ODR_100_HZ                   (0x01)
#define BMP3_ODR_50_HZ                    (0x02)
#define BMP3_ODR_25_HZ                    (0x03)

/* Register Address */
#define BMP3_CHIP_ID_ADDR                 (0x00)
#define BMP3_ERR_REG_ADDR                 (0x02)
#define BMP3_SENS_STATUS_REG_ADDR         (0x03)
#define BMP3_DATA_ADDR                    (0x04)
#define BMP3_PWR_CTRL_ADDR                (0x1B)
#define BMP3_OSR_ADDR                     (0X1C)
#define BMP3_CALIB_DATA_ADDR              (0x31)
#define BMP3_CMD_ADDR                     (0x7E)

/* Error status macros */
#define BMP3_FATAL_ERR                    (0x01)
#define BMP3_CMD_ERR                      (0x02)
#define BMP3_CONF_ERR                     (0x04)

/* Status macros */
#define BMP3_CMD_RDY                      (0x10)
#define BMP3_DRDY_PRESS                   (0x20)
#define BMP3_DRDY_TEMP                    (0x40)

/* Power mode macros */
#define BMP3_SLEEP_MODE                   (0x00)
#define BMP3_FORCED_MODE                  (0x01)
#define BMP3_NORMAL_MODE                  (0x03)

#define BMP3_ENABLE                       (0x01)
#define BMP3_DISABLE                      (0x00)

/* Sensor component selection macros.  These values are internal for API implementation.
 * Don't relate this t0 data sheet.
 */
#define BMP3_PRESS                        (1)
#define BMP3_TEMP                         (1 << 1)
#define BMP3_ALL                          (0x03)

/* Macros related to size */
#define BMP3_CALIB_DATA_LEN               (21)
#define BMP3_P_T_DATA_LEN                 (6)

/* Macros to select the which sensor settings are to be set by the user.
 * These values are internal for API implementation. Don't relate this to
 * data sheet. */
#define BMP3_PRESS_EN_SEL                 (1 << 1)
#define BMP3_TEMP_EN_SEL                  (1 << 2)
#define BMP3_PRESS_OS_SEL                 (1 << 4)

/* Macros for bit masking */
#define BMP3_OP_MODE_MSK                  (0x30)
#define BMP3_OP_MODE_POS                  (0x04)

#define BMP3_PRESS_EN_MSK                 (0x01)

#define BMP3_TEMP_EN_MSK                  (0x02)
#define BMP3_TEMP_EN_POS                  (0x01)

#define BMP3_IIR_FILTER_MSK               (0x0E)
#define BMP3_IIR_FILTER_POS               (0x01)

#define BMP3_ODR_MSK                      (0x1F)

#define BMP3_PRESS_OS_MSK                 (0x07)

#define BMP3_TEMP_OS_MSK                  (0x38)
#define BMP3_TEMP_OS_POS                  (0x03)

#define BMP3_SET_BITS(reg_data, bitname, data) \
	((reg_data & ~(bitname##_MSK)) | \
	 ((data << bitname##_POS) & bitname##_MSK))

/* Macro variant to handle the bitname position if it is zero */
#define BMP3_SET_BITS_POS_0(reg_data, bitname, data) \
	((reg_data & ~(bitname##_MSK)) | \
	 (data & bitname##_MSK))

#define BMP3_GET_BITS(reg_data, bitname)       ((reg_data & (bitname##_MSK)) >> \
		(bitname##_POS))

/* Macro variant to handle the bitname position if it is zero */
#define BMP3_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

// From https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/self-test/bmp3_selftest.c
#define BMP3_POST_SLEEP_WAIT_TIME         5000
#define BMP3_POST_RESET_WAIT_TIME         2000
#define BMP3_POST_INIT_WAIT_TIME          40000
#define BMP3_TRIM_CRC_DATA_ADDR           0x30
#define BPM3_CMD_SOFT_RESET               0xB6
#define BMP3_ODR_ADDR                     0x1D
#define BMP3_IIR_ADDR                     0x1F

// https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
/* Power control settings */
#define POWER_CNTL            (0x0006)
/* Odr and filter settings */
#define ODR_FILTER            (0x00F0)
/* Interrupt control settings */
#define INT_CTRL              (0x0708)
/* Advance settings */
#define ADV_SETT              (0x1800)

#pragma pack(push,1)
struct calibration_s {
	uint16_t par_t1;
	uint16_t par_t2;
	int8_t   par_t3;

	int16_t  par_p1;
	int16_t  par_p2;
	int8_t   par_p3;
	int8_t   par_p4;
	uint16_t par_p5;
	uint16_t par_p6;
	int8_t   par_p7;
	int8_t   par_p8;
	int16_t  par_p9;
	int8_t   par_p10;
	int8_t   par_p11;

}; //calibration data

struct data_s {
	uint8_t p_msb;
	uint8_t p_lsb;
	uint8_t p_xlsb;

	uint8_t t_msb;
	uint8_t t_lsb;
	uint8_t t_xlsb;
}; // data

struct bmp3_reg_calib_data {
	/**
	 * @ Trim Variables
	 */

	/**@{*/
	uint16_t par_t1;
	uint16_t par_t2;
	int8_t par_t3;
	int16_t par_p1;
	int16_t par_p2;
	int8_t par_p3;
	int8_t par_p4;
	uint16_t par_p5;
	uint16_t par_p6;
	int8_t par_p7;
	int8_t par_p8;
	int16_t par_p9;
	int8_t par_p10;
	int8_t par_p11;
	int64_t t_lin;

	/**@}*/
};
#pragma pack(pop)

/*!
 * bmp3 sensor structure which comprises of temperature and pressure data.
 */
struct bmp3_data {
	/* Compensated temperature */
	int64_t temperature;

	/* Compensated pressure */
	uint64_t pressure;
};

/*!
 * Calibration data
 */
struct bmp3_calib_data {
	/*! Register data */
	struct bmp3_reg_calib_data reg_calib_data;
};

/*!
 * bmp3 sensor structure which comprises of un-compensated temperature
 * and pressure data.
 */
struct bmp3_uncomp_data {
	/*! un-compensated pressure */
	uint32_t pressure;

	/*! un-compensated temperature */
	uint32_t temperature;
};

struct fcalibration_s {
	float t1;
	float t2;
	float t3;

	float p1;
	float p2;
	float p3;
	float p4;
	float p5;
	float p6;
	float p7;
	float p8;
	float p9;
};

/*
 * BMP388 internal constants and data structures.
 */
class IBMP388
{
public:
	virtual ~IBMP388() = default;

	virtual int init() = 0;

	// read reg value
	virtual uint8_t get_reg(uint8_t addr) = 0;

	// bulk read reg value
	virtual int get_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len) = 0;

	// write reg value
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;

	// bulk read of calibration data into buffer, return same pointer
	virtual calibration_s *get_calibration(uint8_t addr) = 0;

	virtual uint32_t get_device_id() const = 0;

	virtual uint8_t get_device_address() const = 0;
};

class BMP388 : public I2CSPIDriver<BMP388>
{
public:
	BMP388(I2CSPIBusOption bus_option, int bus, IBMP388 *interface);
	virtual ~BMP388();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int		init();

	void			print_status();

	void 			RunImpl();
private:
	PX4Barometer		_px4_baro;
	IBMP388			*_interface{nullptr};

	unsigned		_measure_interval{0};			// interval in microseconds needed to measure
	uint8_t			_osr_t{BMP3_OVERSAMPLING_2X};		// oversampling rate, temperature
	uint8_t			_osr_p{BMP3_OVERSAMPLING_16X};		// oversampling rate, pressure
	uint8_t			_odr{BMP3_ODR_50_HZ};			// output data rate
	uint8_t			_iir_coef{BMP3_IIR_FILTER_DISABLE};	// IIR coefficient

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	calibration_s		*_cal {nullptr}; // stored calibration constants

	bool			_collect_phase{false};

	void 			start();
	int 			measure();
	int			collect(); //get results and publish
	uint32_t		get_measurement_time(uint8_t osr_t, uint8_t osr_p);

	bool			soft_reset();
	bool			get_calib_data();
	bool			validate_trimming_param();
	bool 			set_sensor_settings();
	bool			set_op_mode(uint8_t op_mode);

	bool 			get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data);
	bool			compensate_data(uint8_t sensor_comp, const struct bmp3_uncomp_data *uncomp_data, struct bmp3_data *comp_data);
};


/* interface factories */
extern IBMP388 *bmp388_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
extern IBMP388 *bmp388_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency);
