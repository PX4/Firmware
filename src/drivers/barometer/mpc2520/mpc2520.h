/****************************************************************************
 *
 *   Copyright (C) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file mpc2520.h
 *
 * Shared defines for the mpc2520 driver.
 */

#define ADDR_RESET_CMD		0x0C	/* write to this address to reset chip */

// Register map:
#define MPC2520_PSR_B2		0x00    /*the highest byte of three bytes measured pressure value*/
#define MPC2520_PSR_B1		0x01    /*the middle byte of three bytes measured pressure value*/
#define MPC2520_PSR_B0		0x02    /*the lowest byte of three bytes measured pressure value*/
#define MPC2520_TMP_B2		0x03    /*the highest byte of three bytes measured temperature value*/
#define MPC2520_TMP_B1		0x04    /*the middle byte of three bytes measured temperature value*/
#define MPC2520_TMP_B0		0x05    /*the lowest byte of three bytes measured temperature value*/
#define MPC2520_PRS_CFG		0x06    /*configuration of pressure measurement rate and resolution*/
#define MPC2520_TMP_CFG		0x07    /*configuration of temperature measurement rate and resolution*/
#define MPC2520_MEAS_CFG		0x08    /*setup measurement mode*/
#define MPC2520_CFG_REG		0x09    /*configuration of interrupts, measurement data shift, and FIFO enable*/

// All supported sampling rates
enum class MPC2520_SAMPLING_RATE {
	RATE_1_HZ,
	RATE_2_HZ,
	RATE_4_HZ,
	RATE_8_HZ,
	RATE_16_HZ,
	RATE_32_HZ,
	RATE_64_HZ,
	RATE_128_HZ
};

// All supported oversampling rates
enum class MPC2520_OVERSAMPLING_RATE {
	RATE_1_HZ,
	RATE_2_HZ,
	RATE_4_HZ,
	RATE_8_HZ,
	RATE_16_HZ,
	RATE_32_HZ,
	RATE_64_HZ,
	RATE_128_HZ
};

// Temperature scale factors for various oversampling rates
constexpr uint32_t MPC2520_1_HZ_SCALE_FACTOR   = 524288;
constexpr uint32_t MPC2520_2_HZ_SCALE_FACTOR   = 1572864;
constexpr uint32_t MPC2520_4_HZ_SCALE_FACTOR   = 3670016;
constexpr uint32_t MPC2520_8_HZ_SCALE_FACTOR   = 7864320;
constexpr uint32_t MPC2520_16_HZ_SCALE_FACTOR  = 253952;
constexpr uint32_t MPC2520_32_HZ_SCALE_FACTOR  = 516096;
constexpr uint32_t MPC2520_64_HZ_SCALE_FACTOR  = 1040384;
constexpr uint32_t MPC2520_128_HZ_SCALE_FACTOR = 2088960;

/* interface ioctls */
#define IOCTL_RESET				2
#define IOCTL_MEASURE			3

#define PRESSURE_SENSOR         0
#define TEMPERATURE_SENSOR      1

namespace mpc2520
{

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct prom_s {
	int16_t c0;
	int16_t c1;
	int32_t c00;
	int32_t c10;
	int16_t c01;
	int16_t c11;
	int16_t c20;
	int16_t c21;
	int16_t c30;
};

#pragma pack(pop)

} /* namespace */

/* interface factories */
extern device::Device *MPC2520_i2c_interface(mpc2520::prom_s &prom_buf, uint8_t busnum);
typedef device::Device *(*MPC2520_constructor)(mpc2520::prom_s &prom_buf, uint8_t busnum);
