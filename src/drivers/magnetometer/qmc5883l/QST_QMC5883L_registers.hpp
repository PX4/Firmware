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
 * @file QST_QMC5883L_registers.hpp
 *
 * QST QMC5883L registers.
 *
 */

#pragma once

#include <cstdint>

namespace QST_QMC5883L
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

static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0b0001101; // default I2C address

static constexpr uint8_t Chip_ID = 0xFF;

enum class Register : uint8_t {
	X_LSB      = 0x00, // Data Output X LSB Register XOUT[7:0]
	X_MSB      = 0x01, // Data Output X MSB Register XOUT[15:8]
	Y_LSB      = 0x02, // Data Output Y LSB Register YOUT[7:0]
	Y_MSB      = 0x03, // Data Output Y MSB Register YOUT[15:8]
	Z_LSB      = 0x04, // Data Output Z LSB Register ZOUT[7:0]
	Z_MSB      = 0x05, // Data Output Z MSB Register ZOUT[15:8]
	STATUS     = 0x06, // Status Register 1

	CNTL1      = 0x09, // Control Register 1
	CNTL2      = 0x0A, // Control Register 2

	SET_RESET_PERIOD = 0x0B, // SET/RESET Period is controlled by FBR [7:0], it is recommended that the register 0BH is written by 0x01.

	CHIP_ID    = 0x0D,
};

// STATUS
enum STATUS_BIT : uint8_t {
	DOR  = Bit2, // Data Skip
	OVL  = Bit1, // Overflow flag
	DRDY = Bit0, // Data Ready
};

// CNTL1
enum CNTL1_BIT : uint8_t {
	// OSR[7:6]
	OSR_512         = Bit7 | Bit6, // 00

	// RNG[5:4]
	RNG_2G          = Bit5 | Bit4, // 00

	// ODR[3:2]
	ODR_50HZ        = Bit2,        // 01

	// MODE[1:0]
	Mode_Continuous = Bit0,        // 01
};

// CNTL2
enum CNTL2_BIT : uint8_t {
	SOFT_RST = Bit7, // Soft reset, restore default value of all registers.
	ROL_PNT  = Bit6, // Enable pointer roll-over function.
};

// SET_RESET_PERIOD
enum SET_RESET_PERIOD_BIT : uint8_t {
	FBR = Bit0, // it is recommended that the register is written by 0x01.
};

} // namespace QMC5883L
