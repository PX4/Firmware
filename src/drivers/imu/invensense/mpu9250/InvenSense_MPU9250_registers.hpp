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

/**
 * @file InvenSense_MPU9250_registers.hpp
 *
 * Invensense MPU9250 registers.
 *
 */

#pragma once

#include <cstdint>

namespace InvenSense_MPU9250
{
// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint32_t I2C_ADDRESS_DEFAULT = 0x69; // 0b110100X
static constexpr uint32_t I2C_SPEED = 400 * 1000;

static constexpr uint32_t SPI_SPEED = 1 * 1000 * 1000;
static constexpr uint32_t SPI_SPEED_SENSOR = 10 * 1000 * 1000; // 20MHz for reading sensor and interrupt registers
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHOAMI = 0x71;

static constexpr float TEMPERATURE_SENSITIVITY = 333.87f; // LSB/C
static constexpr float TEMPERATURE_OFFSET = 21.f; // C

enum class Register : uint8_t {
	CONFIG             = 0x1A,
	GYRO_CONFIG        = 0x1B,
	ACCEL_CONFIG       = 0x1C,
	ACCEL_CONFIG2      = 0x1D,

	FIFO_EN            = 0x23,
	I2C_MST_CTRL       = 0x24,
	I2C_SLV0_ADDR      = 0x25,
	I2C_SLV0_REG       = 0x26,
	I2C_SLV0_CTRL      = 0x27,

	I2C_SLV4_CTRL      = 0x34,

	INT_PIN_CFG        = 0x37,
	INT_ENABLE         = 0x38,

	TEMP_OUT_H         = 0x41,
	TEMP_OUT_L         = 0x42,

	EXT_SENS_DATA_00   = 0x49, // EXT_SENS_DATA_00[7:0]
	// [EXT_SENS_DATA_01, EXT_SENS_DATA_22]
	EXT_SENS_DATA_23   = 0x60, // EXT_SENS_DATA_23[7:0]

	I2C_SLV0_DO        = 0x63,

	I2C_MST_DELAY_CTRL = 0x67,
	SIGNAL_PATH_RESET  = 0x68,

	USER_CTRL          = 0x6A,
	PWR_MGMT_1         = 0x6B,

	FIFO_COUNTH        = 0x72,
	FIFO_COUNTL        = 0x73,
	FIFO_R_W           = 0x74,
	WHO_AM_I           = 0x75,
};

// CONFIG
enum CONFIG_BIT : uint8_t {
	FIFO_MODE = Bit6, // when the FIFO is full, additional writes will not be written to FIFO

	// DLPF_CFG[2:0]
	DLPF_CFG_Fs_1KHZ          = 1, // Rate 1 kHz,  184 Hz Bandwidth
	DLPF_CFG_BYPASS_DLPF_8KHZ = 7, // Rate 8 kHz, 3600 Hz Bandwidth
};

// GYRO_CONFIG
enum GYRO_CONFIG_BIT : uint8_t {
	// GYRO_FS_SEL [4:3]
	GYRO_FS_SEL_250_DPS	= 0,           // 0b00000
	GYRO_FS_SEL_500_DPS	= Bit3,        // 0b01000
	GYRO_FS_SEL_1000_DPS	= Bit4,        // 0b10000
	GYRO_FS_SEL_2000_DPS	= Bit4 | Bit3, // 0b11000

	// FCHOICE_B [1:0]
	FCHOICE_B_BYPASS_DLPF   = Bit1 | Bit0, // 0b00 - 3-dB BW: 3281 Noise BW (Hz): 3451.0   8 kHz
};

// ACCEL_CONFIG
enum ACCEL_CONFIG_BIT : uint8_t {
	// ACCEL_FS_SEL [4:3]
	ACCEL_FS_SEL_2G  = 0,           // 0b00000
	ACCEL_FS_SEL_4G  = Bit3,        // 0b01000
	ACCEL_FS_SEL_8G  = Bit4,        // 0b10000
	ACCEL_FS_SEL_16G = Bit4 | Bit3, // 0b11000
};

// ACCEL_CONFIG2
enum ACCEL_CONFIG2_BIT : uint8_t {
	ACCEL_FCHOICE_B_BYPASS_DLPF = Bit3,

	// [2:0] A_DLPFCFG
	A_DLPFCFG_BW_218HZ_DLPF = 1, // Rate 1 kHz, 218.1 Hz Bandwidth (DLPF filter Block)
};

// FIFO_EN
enum FIFO_EN_BIT : uint8_t {
	TEMP_OUT  = Bit7,
	GYRO_XOUT = Bit6,
	GYRO_YOUT = Bit5,
	GYRO_ZOUT = Bit4,
	ACCEL     = Bit3,
};

// I2C_MST_CTRL
enum I2C_MST_CTRL_BIT : uint8_t {
	I2C_MST_P_NSR = Bit4, // I2C Master’s transition from one slave read to the next slave read

	// I2C_MST_CLK [3:0]
	I2C_MST_CLK_400_kHz = 13,
};

// I2C_SLV0_ADDR
enum I2C_SLV0_ADDR_BIT : uint8_t {
	I2C_SLV0_RNW = Bit7, // 1 – Transfer is a read

	// I2C_ID_0[6:0]
	I2C_ID_0 = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, // Physical address of I2C slave 0
};

// I2C_SLV0_CTRL
enum I2C_SLV0_CTRL_BIT : uint8_t {
	I2C_SLV0_EN      = Bit7, // Enable reading data from this slave
	I2C_SLV0_BYTE_SW = Bit6, // Swap bytes when reading both the low and high byte of a word
	I2C_SLV0_REG_DIS = Bit5, // transaction does not write a register value (only read data)

	I2C_SLV0_LENG    = Bit3 | Bit2 | Bit1 | Bit0, // Number of bytes to be read from I2C slave 0
};

// I2C_SLV4_CTRL
enum I2C_SLV4_CTRL_BIT : uint8_t {
	// I2C_MST_DLY[4:0]
	I2C_MST_DLY = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

// INT_PIN_CFG
enum INT_PIN_CFG_BIT : uint8_t {
	ACTL      = Bit7,
	BYPASS_EN = Bit1, // interface pins(ES_CL and ES_DA) will go into ‘bypass mode’ when the i2c master interface is disabled
};

// INT_ENABLE
enum INT_ENABLE_BIT : uint8_t {
	RAW_RDY_EN = Bit0
};

// I2C_MST_DELAY_CTRL
enum I2C_MST_DELAY_CTRL_BIT : uint8_t {
	I2C_SLVX_DLY_EN = Bit4 | Bit3 | Bit2 | Bit1 | Bit0, // limit all slave access (1+I2C_MST_DLY)
};

// SIGNAL_PATH_RESET
enum SIGNAL_PATH_RESET_BIT : uint8_t {
	GYRO_RESET  = Bit2,
	ACCEL_RESET = Bit1,
	TEMP_RESET  = Bit0,
};

// USER_CTRL
enum USER_CTRL_BIT : uint8_t {
	FIFO_EN      = Bit6,
	I2C_MST_EN   = Bit5,
	I2C_IF_DIS   = Bit4,

	FIFO_RST     = Bit2,
	I2C_MST_RST  = Bit1,
	SIG_COND_RST = Bit0,
};

// PWR_MGMT_1
enum PWR_MGMT_1_BIT : uint8_t {
	H_RESET    = Bit7,
	SLEEP      = Bit6,

	// CLKSEL[2:0]
	CLKSEL_0     = Bit0, // It is required that CLKSEL[2:0] be set to 001 to achieve full gyroscope performance.
};

namespace FIFO
{
static constexpr size_t SIZE = 512;

// FIFO_DATA layout when FIFO_EN has GYRO_{X, Y, Z}OUT and ACCEL set
struct DATA {
	uint8_t ACCEL_XOUT_H;
	uint8_t ACCEL_XOUT_L;
	uint8_t ACCEL_YOUT_H;
	uint8_t ACCEL_YOUT_L;
	uint8_t ACCEL_ZOUT_H;
	uint8_t ACCEL_ZOUT_L;
	uint8_t GYRO_XOUT_H;
	uint8_t GYRO_XOUT_L;
	uint8_t GYRO_YOUT_H;
	uint8_t GYRO_YOUT_L;
	uint8_t GYRO_ZOUT_H;
	uint8_t GYRO_ZOUT_L;
};
}

} // namespace InvenSense_MPU9250
