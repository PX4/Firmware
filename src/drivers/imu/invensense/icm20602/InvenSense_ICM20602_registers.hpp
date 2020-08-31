/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file InvenSense_ICM20602_registers.hpp
 *
 * Invensense ICM-20602 registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace InvenSense_ICM20602
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHOAMI = 0x12;

static constexpr float TEMPERATURE_SENSITIVITY = 326.8f; // LSB/°C
static constexpr float TEMPERATURE_OFFSET = 25.f; // °C
static constexpr float TEMPERATURE_SENSOR_MIN = -40.f; // °C
static constexpr float TEMPERATURE_SENSOR_MAX = 85.f; // °C

enum class Register : uint8_t {
	CONFIG            = 0x1A,
	GYRO_CONFIG       = 0x1B,
	ACCEL_CONFIG      = 0x1C,
	ACCEL_CONFIG2     = 0x1D,

	FIFO_EN           = 0x23,

	INT_PIN_CFG       = 0x37,
	INT_ENABLE        = 0x38,

	TEMP_OUT_H        = 0x41,
	TEMP_OUT_L        = 0x42,

	FIFO_WM_TH1       = 0x60,
	FIFO_WM_TH2       = 0x61,

	SIGNAL_PATH_RESET = 0x68,

	USER_CTRL         = 0x6A,
	PWR_MGMT_1        = 0x6B,

	I2C_IF            = 0x70,

	FIFO_COUNTH       = 0x72,
	FIFO_COUNTL       = 0x73,
	FIFO_R_W          = 0x74,
	WHO_AM_I          = 0x75,
};

// CONFIG
enum CONFIG_BIT : uint8_t {
	// Bit7 - FIFO_WM_TH[9:0] User should ensure that bit 7 of register 0x1A is set to 0 before using this feature
	FIFO_MODE = Bit6, // when the FIFO is full, additional writes will not be written to FIFO

	DLPF_CFG_BYPASS_DLPF_8KHZ = 7, // Rate 8 kHz [2:0]
};

// GYRO_CONFIG
enum GYRO_CONFIG_BIT : uint8_t {
	// FS_SEL [4:3]
	FS_SEL_250_DPS	= 0,           // 0b00000
	FS_SEL_500_DPS	= Bit3,        // 0b01000
	FS_SEL_1000_DPS	= Bit4,        // 0b10000
	FS_SEL_2000_DPS	= Bit4 | Bit3, // 0b11000

	// FCHOICE_B [1:0]
	FCHOICE_B_8KHZ_BYPASS_DLPF = Bit1 | Bit0, // 0b00 - 3-dB BW: 3281 Noise BW (Hz): 3451.0   8 kHz
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
};

// FIFO_EN
enum FIFO_EN_BIT : uint8_t {
	GYRO_FIFO_EN  = Bit4,
	ACCEL_FIFO_EN = Bit3,
};

// INT_PIN_CFG
enum INT_PIN_CFG_BIT : uint8_t {
	INT_LEVEL    = Bit7,

	LATCH_INT_EN = Bit5,
	INT_RD_CLEAR = Bit4,
};

// INT_ENABLE
enum INT_ENABLE_BIT : uint8_t {
	DATA_RDY_INT_EN = Bit0,
};

// SIGNAL_PATH_RESET
enum SIGNAL_PATH_RESET_BIT : uint8_t {
	ACCEL_RST = Bit1,
	TEMP_RST  = Bit0,
};

// USER_CTRL
enum USER_CTRL_BIT : uint8_t {
	FIFO_EN      = Bit6,
	FIFO_RST     = Bit2,
	SIG_COND_RST = Bit0,
};

// PWR_MGMT_1
enum PWR_MGMT_1_BIT : uint8_t {
	DEVICE_RESET = Bit7,
	SLEEP        = Bit6,

	// CLKSEL[2:0]
	CLKSEL_0     = Bit0, // It is required that CLKSEL[2:0] be set to 001 to achieve full gyroscope performance.
};

// I2C_IF
enum I2C_IF_BIT : uint8_t {
	I2C_IF_DIS = Bit6, // 1 – Disable I2C Slave module and put the serial interface in SPI mode only.
};


namespace FIFO
{
static constexpr size_t SIZE = 1008;

// FIFO_DATA layout when FIFO_EN has both GYRO_FIFO_EN and ACCEL_FIFO_EN set
struct DATA {
	uint8_t ACCEL_XOUT_H;
	uint8_t ACCEL_XOUT_L;
	uint8_t ACCEL_YOUT_H;
	uint8_t ACCEL_YOUT_L;
	uint8_t ACCEL_ZOUT_H;
	uint8_t ACCEL_ZOUT_L;
	uint8_t TEMP_OUT_H;
	uint8_t TEMP_OUT_L;
	uint8_t GYRO_XOUT_H;
	uint8_t GYRO_XOUT_L;
	uint8_t GYRO_YOUT_H;
	uint8_t GYRO_YOUT_L;
	uint8_t GYRO_ZOUT_H;
	uint8_t GYRO_ZOUT_L;
};
}

} // namespace InvenSense_ICM20602
