/****************************************************************************
 *
 *   Copyright (C) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file bmp280.h
 *
 * Shared defines for the bmp280 driver.
 */
#pragma once

#include <drivers/device/spi.h>
#include <inttypes.h>

#define BMP280_ADDR_CAL		0x88	/* address of 12x 2 bytes calibration data */
#define BMP280_ADDR_DATA	0xF7	/* address of 2x 3 bytes p-t data */

#define BMP280_ADDR_CONFIG	0xF5	/* configuration */
#define BMP280_ADDR_CTRL	0xF4	/* controll */
#define BMP280_ADDR_STATUS	0xF3	/* state */
#define BMP280_ADDR_RESET	0xE0	/* reset */
#define BMP280_ADDR_ID		0xD0	/* id */

#define BMP280_VALUE_ID		0x58	/* chip id */
#define BMP280_VALUE_RESET	0xB6	/* reset */

#define BMP280_STATUS_MEASURING	(1<<3)	/* if in process of measure */
#define BMP280_STATUS_COPING	(1<<0)	/* if in process of data copy */

#define BMP280_CTRL_P0		(0x0<<2)		/* no p measure */
#define BMP280_CTRL_P1		(0x1<<2)
#define BMP280_CTRL_P2		(0x2<<2)
#define BMP280_CTRL_P4		(0x3<<2)
#define BMP280_CTRL_P8		(0x4<<2)
#define BMP280_CTRL_P16		(0x5<<2)

#define BMP280_CTRL_T0		(0x0<<5)		/* no t measure */
#define BMP280_CTRL_T1		(0x1<<5)
#define BMP280_CTRL_T2		(0x2<<5)
#define BMP280_CTRL_T4		(0x3<<5)
#define BMP280_CTRL_T8		(0x4<<5)
#define BMP280_CTRL_T16		(0x5<<5)

#define BMP280_CONFIG_F0		(0x0<<2)		/* no filter */
#define BMP280_CONFIG_F2		(0x1<<2)
#define BMP280_CONFIG_F4		(0x2<<2)
#define BMP280_CONFIG_F8		(0x3<<2)
#define BMP280_CONFIG_F16		(0x4<<2)


#define BMP280_CTRL_MODE_SLEEP	0x0
#define BMP280_CTRL_MODE_FORCE	0x1		/* on demand, goes to sleep after */
#define BMP280_CTRL_MODE_NORMAL	0x3

#define BMP280_MT_INIT		6400	/* max measure time of initial p + t in us */
#define BMP280_MT			2300	/* max measure time of p or t in us */

namespace bmp280
{

#pragma pack(push,1)
struct calibration_s {
	uint16_t t1;
	int16_t t2;
	int16_t t3;

	uint16_t p1;
	int16_t p2;
	int16_t p3;
	int16_t p4;
	int16_t p5;
	int16_t p6;
	int16_t p7;
	int16_t p8;
	int16_t p9;
}; //calibration data

struct data_s {
	uint8_t p_msb;
	uint8_t p_lsb;
	uint8_t p_xlsb;

	uint8_t t_msb;
	uint8_t t_lsb;
	uint8_t t_xlsb;
}; // data
#pragma pack(pop)

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

class IBMP280
{
public:
	virtual ~IBMP280() = default;

	virtual int init() = 0;

	// read reg value
	virtual uint8_t get_reg(uint8_t addr) = 0;

	// write reg value
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;

	// bulk read of data into buffer, return same pointer
	virtual bmp280::data_s *get_data(uint8_t addr) = 0;

	// bulk read of calibration data into buffer, return same pointer
	virtual bmp280::calibration_s *get_calibration(uint8_t addr) = 0;

	virtual uint32_t get_device_id() const = 0;

	virtual uint8_t get_device_address() const = 0;
};

} /* namespace */


/* interface factories */
extern bmp280::IBMP280 *bmp280_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
extern bmp280::IBMP280 *bmp280_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency);
