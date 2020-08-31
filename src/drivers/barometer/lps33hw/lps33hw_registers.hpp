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

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace ST_LPS33HW
{

static constexpr uint8_t WHO_AM_I_VALUE = 0b10110001;

enum class
Register : uint8_t {

	WHO_AM_I    = 0x0F,
	CTRL_REG1   = 0x10,
	CTRL_REG2   = 0x11,
	CTRL_REG3   = 0x12,

	REF_P_XL    = 0x15,
	REF_P_L     = 0x16,
	REF_P_H     = 0x17,

	RPDS_L      = 0x18,
	RPDS_H      = 0x19,

	RES_CONF    = 0x1A,

	STATUS      = 0x27,
	PRESS_OUT_XL = 0x28,
	PRESS_OUT_L = 0x29,
	PRESS_OUT_H = 0x2A,
	TEMP_OUT_L  = 0x2B,
	TEMP_OUT_H  = 0x2C,
};

enum CTRL_REG1 : uint8_t {
	ODR_75HZ = Bit4 | Bit6, // Continous 75Hz update rate
	EN_LPFP = Bit3, // enable low-pass filter
	BDU = Bit1, // block data update
};

enum CTRL_REG2 : uint8_t {
	SWRESET = Bit2,
};

enum STATUS : uint8_t {
	P_DA = Bit0, // Pressure data available
	T_DA = Bit1, // Temp data available
};

} // namespace ST_LPS33HW
