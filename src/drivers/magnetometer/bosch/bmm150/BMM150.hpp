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
 * @file BMM150.hpp
 *
 * Driver for the Bosch BMM150 connected via I2C.
 *
 */

#pragma once

#include "Bosch_BMM150_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace Bosch_BMM150;

class BMM150 : public device::I2C, public I2CSPIDriver<BMM150>
{
public:
	BMM150(I2CSPIBusOption bus_option, int bus, int bus_frequency, enum Rotation rotation = ROTATION_NONE);
	~BMM150() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:

	struct trim_registers {
		int8_t dig_x1;     // trim x1 data
		int8_t dig_y1;     // trim y1 data
		int8_t dig_x2;     // trim x2 data
		int8_t dig_y2;     // trim y2 data
		uint16_t dig_z1;   // trim z1 data
		int16_t dig_z2;    // trim z2 data
		int16_t dig_z3;    // trim z3 data
		int16_t dig_z4;    // trim z4 data
		uint8_t dig_xy1;   // trim xy1 data
		int8_t dig_xy2;    // trim xy2 data
		uint16_t dig_xyz1; // trim xyz1 data
	} _trim_data{};

	// Sensor Configuration
	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;
	bool Reset();
	bool Configure();
	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);
	void RegisterClearBits(Register reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); };
	void RegisterSetBits(Register reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); };

	// obtain the compensated magnetometer data in micro-tesla.
	float compensate_x(int16_t mag_data_x, uint16_t data_rhall);
	float compensate_y(int16_t mag_data_y, uint16_t data_rhall);
	float compensate_z(int16_t mag_data_z, uint16_t data_rhall);

	PX4Magnetometer _px4_mag;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": overflow")};
	perf_counter_t _self_test_failed_perf{perf_alloc(PC_COUNT, MODULE_NAME": self test failed")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		SELF_TEST_CHECK,
		READ_TRIM,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{4};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                | Set bits, Clear bits
		{ Register::POWER_CONTROL, POWER_CONTROL_BIT::PowerControl, POWER_CONTROL_BIT::SoftReset },
		{ Register::OP_MODE,       OP_MODE_BIT::ODR_20Hz, OP_MODE_BIT::Opmode_Sleep | OP_MODE_BIT::Self_Test },
		{ Register::REPXY,         REPXY_BIT::XY_HA_SET, REPXY_BIT::XY_HA_CLEAR },
		{ Register::REPZ,          REPZ_BIT::Z_HA_SET, REPZ_BIT::Z_HA_CLEAR },
	};
};
