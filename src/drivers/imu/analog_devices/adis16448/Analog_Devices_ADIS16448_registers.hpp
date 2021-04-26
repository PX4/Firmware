/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file Analog_Devices_ADIS16448_registers.hpp
 *
 * Analog Devices ADIS16448 registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint16_t Bit0  = (1 << 0);
static constexpr uint16_t Bit1  = (1 << 1);
static constexpr uint16_t Bit2  = (1 << 2);
static constexpr uint16_t Bit3  = (1 << 3);
static constexpr uint16_t Bit4  = (1 << 4);
static constexpr uint16_t Bit5  = (1 << 5);
static constexpr uint16_t Bit6  = (1 << 6);
static constexpr uint16_t Bit7  = (1 << 7);
static constexpr uint16_t Bit8  = (1 << 8);
static constexpr uint16_t Bit9  = (1 << 9);
static constexpr uint16_t Bit10 = (1 << 10);
static constexpr uint16_t Bit11 = (1 << 11);
static constexpr uint16_t Bit12 = (1 << 12);
static constexpr uint16_t Bit13 = (1 << 13);
static constexpr uint16_t Bit14 = (1 << 14);
static constexpr uint16_t Bit15 = (1 << 15);

namespace Analog_Devices_ADIS16448
{
static constexpr uint32_t SPI_SPEED = 2 * 1000 * 1000;       // 2 MHz SPI serial interface
static constexpr uint32_t SPI_SPEED_BURST = 1 * 1000 * 1000; // 1 MHz SPI serial interface for burst read

static constexpr uint32_t SPI_STALL_PERIOD = 9; // 9 us Stall period between data

static constexpr uint16_t DIR_WRITE = 0x80;

static constexpr uint16_t Product_identification = 0x4040;

static constexpr uint32_t SAMPLE_INTERVAL_US = (1e6f / 819.2f); // ~819.2 Hz


enum class Register : uint16_t {
	GPIO_CTRL  = 0x32, // Auxiliary digital input/output control

	MSC_CTRL   = 0x34, // Miscellaneous control
	SMPL_PRD   = 0x36, // Internal sample period (rate) control
	SENS_AVG   = 0x38, // Dynamic range and digital filter control

	DIAG_STAT  = 0x3C, // System status
	GLOB_CMD   = 0x3E, // System command

	LOT_ID1    = 0x52, // Lot identification number
	LOT_ID2    = 0x54, // Lot identification number
	PROD_ID    = 0x56, // Product identifier
	SERIAL_NUM = 0x58, // Lot-specific serial number
};

// MSC_CTRL
enum MSC_CTRL_BIT : uint16_t {
	Checksum_memory_test   = Bit11, // Checksum memory test (cleared upon completion)
	Internal_self_test     = Bit10, // Internal self test (cleared upon completion)
	// Not used            = Bit5
	CRC16_for_burst        = Bit4,  // include the CRC-16 code in burst read output sequence
	// Not used            = Bit3
	Data_ready_enable      = Bit2,
	Data_ready_polarity    = Bit1,  // 1 = active high when data is valid
	Data_ready_line_select = Bit0,  // Data ready line select 1 = DIO2, 0 = DIO1
};

// DIAG_STAT
enum DIAG_STAT_BIT : uint16_t {
	Z_axis_accelerometer_self_test_failure = Bit15, // 1 = fail, 0 = pass
	Y_axis_accelerometer_self_test_failure = Bit14, // 1 = fail, 0 = pass
	X_axis_accelerometer_self_test_failure = Bit13, // 1 = fail, 0 = pass
	Z_axis_gyroscope_self_test_failure     = Bit12, // 1 = fail, 0 = pass
	Y_axis_gyroscope_self_test_failure     = Bit11, // 1 = fail, 0 = pass
	X_axis_gyroscope_self_test_failure     = Bit10, // 1 = fail, 0 = pass

	New_data_xMAGN_OUT_BARO_OUT            = Bit7, // New data, xMAGN_OUT/BARO_OUT
	Flash_test_checksum_flag               = Bit6, // 1 = fail, 0 = pass
	Self_test_diagnostic_error_flag        = Bit5, // 1 = fail, 0 = pass

	SPI_communication_failure              = Bit3, // 1 = fail, 0 = pass

	Barometer_functional_test              = Bit1, // 1 = fail, 0 = pass
	Magnetometer_functional_test           = Bit0, // 1 = fail, 0 = pass
};

// GLOB_CMD
enum GLOB_CMD_BIT : uint16_t {
	Software_reset = Bit7,
};

// SMPL_PRD
enum SMPL_PRD_BIT : uint16_t {
	// [12:8] D, decimation rate setting, binomial,
	decimation_rate = Bit12 | Bit11 | Bit10 | Bit9, // disable

	internal_sampling_clock = Bit0, // 1 = internal sampling clock, 819.2 SPS
};

// SENS_AVG
enum SENS_AVG_BIT : uint16_t {
	// [10:8] Measurement range (sensitivity) selection
	Measurement_range_1000_set   = Bit10, // 100 = ±1000°/sec (default condition)
	Measurement_range_1000_clear = Bit9 | Bit8,

	// [2:0] Filter Size Variable B
	Filter_Size_Variable_B = Bit2 | Bit1 | Bit0, // disable

};

// GPIO_CTRL
enum GPIO_CTRL_BIT : uint16_t {
	GPIO4_DATA_LEVEL = Bit11, // General-Purpose I/O Line 1 (DIO1) data level
	GPIO3_DATA_LEVEL = Bit10, // General-Purpose I/O Line 1 (DIO1) data level
	GPIO2_DATA_LEVEL = Bit9,  // General-Purpose I/O Line 1 (DIO1) data level
	GPIO1_DATA_LEVEL = Bit8,  // General-Purpose I/O Line 1 (DIO1) data level

	GPIO4_DIRECTION  = Bit3,  // General-Purpose I/O Line 4 (DIO4) direction control 1 = output, 0 = input
	GPIO3_DIRECTION  = Bit2,  // General-Purpose I/O Line 3 (DIO3) direction control 1 = output, 0 = input
	GPIO2_DIRECTION  = Bit1,  // General-Purpose I/O Line 2 (DIO2) direction control 1 = output, 0 = input
	GPIO1_DIRECTION  = Bit0,  // General-Purpose I/O Line 1 (DIO1) direction control 1 = output, 0 = input
};

} // namespace Analog_Devices_ADIS16448
