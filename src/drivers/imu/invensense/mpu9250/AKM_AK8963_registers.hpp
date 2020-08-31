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
 * @file AKM_AK8963_registers.hpp
 *
 * Asahi Kasei Microdevices (AKM) AK8963 registers.
 *
 */

#pragma once

#include <cstdint>

namespace AKM_AK8963
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
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x0C;

static constexpr uint8_t Device_ID = 0x48; // Device ID of AKM

enum class Register : uint8_t {
	WIA   = 0x00, // Device ID

	ST1   = 0x02, // Status 1
	HXL   = 0x03,
	HXH   = 0x04,
	HYL   = 0x05,
	HYH   = 0x06,
	HZL   = 0x07,
	HZH   = 0x08,
	ST2   = 0x09, // Status 2
	CNTL1 = 0x0A, // Control 1
	CNTL2 = 0x0B, // Control 2

	ASAX  = 0x10, // X-axis sensitivity adjustment value
	ASAY  = 0x11, // Y-axis sensitivity adjustment value
	ASAZ  = 0x12, // Z-axis sensitivity adjustment value
};

// ST1
enum ST1_BIT : uint8_t {
	DRDY = Bit0,
};

// ST2
enum ST2_BIT : uint8_t {
	BITM = Bit4, // Output bit setting (mirror)
	HOFL = Bit3, // Magnetic sensor overflow
};

// CNTL1
enum CNTL1_BIT : uint8_t {
	BIT_16 = Bit4, // Output bit setting (16-bit output)

	// MODE[3:0]: Operation mode setting
	POWER_DOWN_MODE         = 0,
	SINGLE_MEASUREMENT_MODE = Bit0,
	CONTINUOUS_MODE_1       = Bit1,        //   8 Hz
	CONTINUOUS_MODE_2       = Bit2 | Bit1, // 100 Hz
	FUSE_ROM_ACCESS_MODE    = Bit3 | Bit2 | Bit1 | Bit0, // MODE[3:0]=“1111”
};

// CNTL2
enum CNTL2_BIT : uint8_t {
	SRST = Bit0, // Reset
};

} // namespace AKM_AK8963
