/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

namespace PixArt_PAW3902JF
{

static constexpr uint8_t PRODUCT_ID = 0x49;
static constexpr uint8_t REVISION_ID = 0x01;
static constexpr uint8_t PRODUCT_ID_INVERSE = 0xB6;

static constexpr uint32_t SAMPLE_INTERVAL_MODE_0{1000000 / 126};	// 126 fps
static constexpr uint32_t SAMPLE_INTERVAL_MODE_1{1000000 / 126};	// 126 fps
static constexpr uint32_t SAMPLE_INTERVAL_MODE_2{1000000 / 50};		// 50 fps

static constexpr uint32_t SPI_SPEED = 2 * 1000 * 1000; // 2MHz SPI serial interface

// Various time delay needed for PAW3902
static constexpr uint32_t TIME_us_TSWW  = 11; // actually 10.5us
static constexpr uint32_t TIME_us_TSRAD = 2;

enum Register : uint8_t {
	Product_ID         = 0x00,
	Revision_ID        = 0x01,
	Motion             = 0x02,
	Delta_X_L          = 0x03,
	Delta_X_H          = 0x04,
	Delta_Y_L          = 0x05,
	Delta_Y_H          = 0x06,
	Squal              = 0x07,
	RawData_Sum        = 0x08,
	Maximum_RawData    = 0x09,
	Minimum_RawData    = 0x0A,
	Shutter_Lower      = 0x0B,
	Shutter_Upper      = 0x0C,

	Observation        = 0x15,
	Motion_Burst       = 0x16,

	Power_Up_Reset     = 0x3A,

	Resolution	   = 0x4E,

	Inverse_Product_ID = 0x5F,
};

enum Product_ID_Bit : uint8_t {
	Reset = 0x5A,
};

enum class Mode {
	Bright        = 0,
	LowLight      = 1,
	SuperLowLight = 2,
};

struct BURST_TRANSFER {
	uint8_t Motion;
	uint8_t Observation;
	uint8_t Delta_X_L;
	uint8_t Delta_X_H;
	uint8_t Delta_Y_L;
	uint8_t Delta_Y_H;
	uint8_t SQUAL;
	uint8_t RawData_Sum;
	uint8_t Maximum_RawData;
	uint8_t Minimum_RawData;
	uint8_t Shutter_Upper;
	uint8_t Shutter_Lower;
};

}
