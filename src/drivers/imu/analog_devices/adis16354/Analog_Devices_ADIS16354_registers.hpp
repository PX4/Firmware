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
 * @file Analog_Devices_ADIS16354_registers.hpp
 *
 * Analog Devices ADIS16354 registers.
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

namespace Analog_Devices_ADIS16354
{
static constexpr uint32_t SPI_SPEED = 2 * 1000 * 1000; // 2 MHz SPI serial interface

static constexpr uint32_t SPI_STALL_PERIOD = 9; // 9 us Stall period between data

static constexpr uint16_t DIR_WRITE = 0x80;

static constexpr uint32_t SAMPLE_INTERVAL_US = (1e6f / 819.2f); // ~819.2 Hz


enum class Register : uint16_t {
	SUPPLY_OUT = 0x03, // Power supply measurement
	XGYRO_OUT  = 0x05, // X-axis gyro output
	YGYRO_OUT  = 0x07, // Y-axis gyro output
	ZGYRO_OUT  = 0x09, // Z-axis gyro output
	XACCL_OUT  = 0x0B, // X-axis accelerometer output
	YACCL_OUT  = 0x0D, // Y-axis accelerometer output
	ZACCL_OUT  = 0x0F, // Z-axis accelerometer output
	XTEMP_OUT  = 0x11, // X-axis gyro temperature measurement
	YTEMP_OUT  = 0x13, // Y-axis gyro temperature measurement
	ZTEMP_OUT  = 0x15, // Z-axis gyro temperature measurement
	AUX_ADC    = 0x17, // Auxiliary ADC output

	GPIO_CTRL  = 0x32, // Auxiliary digital input/output control

	MSC_CTRL   = 0x34, // Miscellaneous control
	SMPL_PRD   = 0x36, // Internal sample period (rate) control
	SENS_AVG   = 0x38, // Dynamic range and digital filter control

	STATUS     = 0x3C, // System status
	COMMAND    = 0x3E, // System command
};

// MSC_CTRL
enum MSC_CTRL_BIT : uint16_t {
	Internal_self_test     = Bit10, // Internal self test (cleared upon completion), 1 = enabled, 0 = disabled
	Data_ready_enable      = Bit2,  // Data ready enable, 1 = enabled, 0 = disabled
	Data_ready_polarity    = Bit1,  // Data-ready polarity, 1 = active high, 0 = active low
	Data_ready_line_select = Bit0,  // Data ready line select, 1 = DIO2, 0 = DIO1
};

// STATUS
enum STATUS_BIT : uint16_t {
	Z_axis_accelerometer_self_test_failure = Bit15, // 1 = fail, 0 = pass
	Y_axis_accelerometer_self_test_failure = Bit14, // 1 = fail, 0 = pass
	X_axis_accelerometer_self_test_failure = Bit13, // 1 = fail, 0 = pass
	Z_axis_gyroscope_self_test_failure     = Bit12, // 1 = fail, 0 = pass
	Y_axis_gyroscope_self_test_failure     = Bit11, // 1 = fail, 0 = pass
	X_axis_gyroscope_self_test_failure     = Bit10, // 1 = fail, 0 = pass

	Self_test_diagnostic_error_flag        = Bit5, // 1 = fail, 0 = pass
	Sensor_overrange                       = Bit4, // 1 = fail, 0 = pass
	SPI_communication_failure              = Bit3, // 1 = fail, 0 = pass

	Power_supply_greater_than_5V25	       = Bit1, // 1 = power supply > 5.25 V, 0 = power supply ≤ 5.25 V
	Power_supply_less_than_4V75	       = Bit0, // 1 = power supply < 4.75 V, 0 = power supply ≥ 4.75 V
};

// COMMAND
enum COMMAND_BIT : uint16_t {
	Software_reset = Bit7,
};

// SMPL_PRD
enum SMPL_PRD_BIT : uint16_t {
	// SMPL_PRD default = 0x0001 for sample rate setting of 819.2 SPS
	Time_base = Bit7, // 0 = 0.61035 ms, 1 = 18.921 ms
	Sample_period_multiplier = Bit0, // Ns = 000001: tS = tB × (NS + 1) = 0.61035 × (1 + 1) = 1.2207 ms
};

// SENS_AVG
enum SENS_AVG_BIT : uint16_t {
	// [10:8] Measurement range (sensitivity) selection
	Measurement_range_set = Bit10, // [10:8] = [100] = 300°/sec (default condition)
	Measurement_range_clear = Bit9 | Bit8,

	// [2:0] Number of filter taps
	Number_of_filter_taps = Bit1, // N = 2^M = 2^2 = 4 (bandwidth < 350 Hz)
};

// GPIO_CTRL
enum GPIO_CTRL_BIT : uint16_t {
	GPIO2_DATA_LEVEL = Bit9,  // General-Purpose I/O Line 2 (DIO1) data level
	GPIO1_DATA_LEVEL = Bit8,  // General-Purpose I/O Line 1 (DIO1) data level

	GPIO2_DIRECTION  = Bit1,  // General-Purpose I/O Line 2 (DIO2) direction control 1 = output, 0 = input
	GPIO1_DIRECTION  = Bit0,  // General-Purpose I/O Line 1 (DIO1) direction control 1 = output, 0 = input
};

} // namespace Analog_Devices_ADIS16354
