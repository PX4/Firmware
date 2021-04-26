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

/**
 * @file tcbp001ta/defines.h
 *
 * Shared defines for the TCBP001TA barometer driver.
 *
 * @author Xiaowei Zhao <xiaowei_zhao1013@163.com>
 * @author Stone White <stone@thone.io>
 *
 */

#pragma once

#include <inttypes.h>

#define TCBP001TA_ADDR_CAL	0x10	/* address of calibration data */

#define TCBP001TA_ADDR_DATA	0x00	/* address of presure data */
#define TCBP001TA_ADDR_PRS_DATA 0X00    /* address of presure data */
#define TCBP001TA_ADDR_TMP_DATA 0x03    /* address of tempature data */

#define TCBP001TA_ADDR_PRS_CFG  0x06
#define TCBP001TA_ADDR_TMP_CFG  0x07
#define TCBP001TA_ADDR_MEAS_CFG 0x08
#define TCBP001TA_ADDR_CFG_REG  0x09

#define TCBP001TA_ADDR_INT_STS  0x0A
#define TCBP001TA_ADDR_FIFO_STS 0x0B
#define TCBP001TA_ADDR_RESET    0x0C    /* reset */
#define TCBP001TA_ADDR_ID       0x0D    /* product and revision ID */

#define TCBP001TA_ADDR_TMP_COEF_SRCE 0x28

#define TCBP001TA_VALUE_ID	0x10	/* chip id */
#define TCBP001TA_VALUE_RESET	0x89	/* reset */

#define TCBP001TA_PRS_TMP_RATE_1    (0x0 << 4)
#define TCBP001TA_PRS_TMP_RATE_2    (0x1 << 4)
#define TCBP001TA_PRS_TMP_RATE_4    (0x2 << 4)
#define TCBP001TA_PRS_TMP_RATE_8    (0x3 << 4)
#define TCBP001TA_PRS_TMP_RATE_16   (0x4 << 4)
#define TCBP001TA_PRS_TMP_RATE_32   (0x5 << 4)
#define TCBP001TA_PRS_TMP_RATE_64   (0x6 << 4)
#define TCBP001TA_PRS_TMP_RATE_128  (0x7 << 4)

#define TCBP001TA_PRS_TMP_PRC          0x0
#define TCBP001TA_PRS_TMP_PRC_2        0x1
#define TCBP001TA_PRS_TMP_PRC_4        0x2
#define TCBP001TA_PRS_TMP_PRC_8        0x3
#define TCBP001TA_PRS_TMP_PRC_16       0x4
#define TCBP001TA_PRS_TMP_PRC_32       0x5
#define TCBP001TA_PRS_TMP_PRC_64       0x6
#define TCBP001TA_PRS_TMP_PRC_128      0x7

#define TCBP001TA_TMP_EXT_ASIC  0x00
#define TCBP001TA_TMP_EXT_MEMS  0x80

#define TCBP001TA_MT_INIT		6400	/* max measure time of initial p + t in us */
#define TCBP001TA_MT			2300	/* max measure time of p or t in us */

#define POW_2_23_MINUS_1	0x7FFFFF   //implies 2^23-1
#define POW_2_24		0x1000000
#define POW_2_15_MINUS_1	0x7FFF
#define POW_2_16		0x10000
#define POW_2_11_MINUS_1	0x7FF
#define POW_2_12		0x1000
#define POW_2_20		0x100000
#define POW_2_19_MINUS_1	524287

namespace tcbp001ta
{

#pragma pack(push,1)
struct calibration_s {
	uint8_t c0_h;
	uint8_t c0l_1h;
	uint8_t c1l;
	uint8_t c00h;
	uint8_t c00m;
	uint8_t c00l_10h;
	uint8_t c10m;
	uint8_t c10l;
	uint8_t c01h;
	uint8_t c01l;
	uint8_t c11h;
	uint8_t c11l;
	uint8_t c20h;
	uint8_t c20l;
	uint8_t c21h;
	uint8_t c21l;
	uint8_t c30h;
	uint8_t c30l;
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
	int16_t c0;	// 12bit
	int16_t c1;	// 12bit
	int32_t c00;	// 20bit
	int32_t c10;	// 20bit
	int16_t c01;	// 16bit
	int16_t c11;	// 16bit
	int16_t c20;	// 16bit
	int16_t c21;	// 16bit
	int16_t c30;	// 16bit
};

class ITCBP001TA
{
public:
	virtual ~ITCBP001TA() = default;

	virtual int init() = 0;

	// read reg value
	virtual uint8_t get_reg(uint8_t addr) = 0;

	// write reg value
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;

	// bulk read of data into buffer, return same pointer
	virtual tcbp001ta::data_s *get_data(uint8_t addr) = 0;

	// bulk read of calibration data into buffer, return same pointer
	virtual tcbp001ta::calibration_s *get_calibration(uint8_t addr) = 0;

	virtual uint32_t get_device_id() const = 0;

};

} /* namespace */


/* interface factories */
extern tcbp001ta::ITCBP001TA *tcbp001ta_spi_interface(uint8_t busnum, uint32_t device);
extern tcbp001ta::ITCBP001TA *tcbp001ta_i2c_interface(uint8_t busnum, uint32_t device);
typedef tcbp001ta::ITCBP001TA *(*TCBP001TA_constructor)(uint8_t, uint32_t);
