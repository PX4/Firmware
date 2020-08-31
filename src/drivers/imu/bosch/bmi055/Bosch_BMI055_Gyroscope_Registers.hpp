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

namespace Bosch::BMI055::Gyroscope
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

static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t chip_id = 0x0F;

enum class Register : uint8_t {
	CHIP_ID        = 0x00,

	FIFO_STATUS    = 0x0E,
	RANGE          = 0x0F,

	RATE_HBW       = 0x13,
	BGW_SOFTRESET  = 0x14,
	INT_EN_0       = 0x15,
	INT_EN_1       = 0x16,

	INT_MAP_1      = 0x18,

	FIFO_WM_ENABLE = 0x1E,

	FIFO_CONFIG_0  = 0x3D,
	FIFO_CONFIG_1  = 0x3E,
	FIFO_DATA      = 0x3F,
};

// FIFO_STATUS
enum FIFO_STATUS_BIT : uint8_t {
	fifo_overrun       = Bit7,
	fifo_frame_counter = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, // fifo_frame_counter<6:0>
};

// RANGE
enum RANGE_BIT : uint8_t {
	gyro_range_2000_dps = 0x00, // ±2000
	gyro_range_1000_dps = 0x01, // ±1000
	gyro_range_500_dps  = 0x02, // ±500
	gyro_range_250_dps  = 0x04, // ±250
	gyro_range_125_dps  = 0x05, // ±125
};

// RATE_HBW
enum RATE_HBW_BIT : uint8_t {
	data_high_bw = Bit7, //  1 -> unfiltered
};

// INT_EN_0
enum INT_EN_0_BIT : uint8_t {
	data_en = Bit7,
	fifo_en = Bit6,
};

// INT_EN_1
enum INT_EN_1_BIT : uint8_t {
	int1_od  = Bit1,
	int1_lvl = Bit0,
};

// INT_MAP_1
enum INT_MAP_1_BIT : uint8_t {
	int1_fifo = Bit2,
	int1_data = Bit0,
};

// FIFO_WM_ENABLE
enum FIFO_WM_ENABLE_BIT : uint8_t {
	fifo_wm_enable  = Bit7,
};

// FIFO_CONFIG_0
enum FIFO_CONFIG_0_BIT : uint8_t {
	tag = Bit7,
};

// FIFO_CONFIG_1
enum FIFO_CONFIG_1_BIT : uint8_t {
	fifo_mode = Bit6,
};

namespace FIFO
{
struct DATA {
	uint8_t RATE_X_LSB;
	uint8_t RATE_X_MSB;
	uint8_t RATE_Y_LSB;
	uint8_t RATE_Y_MSB;
	uint8_t RATE_Z_LSB;
	uint8_t RATE_Z_MSB;
};
static_assert(sizeof(DATA) == 6);

// 100 frames of data in FIFO mode
static constexpr size_t SIZE = sizeof(DATA) * 100;

} // namespace FIFO
} // namespace Bosch::BMI055::Gyroscope
