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
 * @file AK8963.hpp
 *
 * Driver for the AKM AK8963 connected via I2C.
 *
 */

#pragma once

#include "AKM_AK8963_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace AKM_AK8963;

class AK8963 : public device::I2C, public I2CSPIDriver<AK8963>
{
public:
	AK8963(I2CSPIBusOption bus_option, int bus, int bus_frequency, enum Rotation rotation = ROTATION_NONE);
	~AK8963() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:

	struct TransferBuffer {
		uint8_t ST1;
		uint8_t HXL;
		uint8_t HXH;
		uint8_t HYL;
		uint8_t HYH;
		uint8_t HZL;
		uint8_t HZH;
		uint8_t ST2;
	};

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

	PX4Magnetometer _px4_mag;

	perf_counter_t _transfer_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": transfer")};
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _magnetic_sensor_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": magnetic sensor overflow")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	unsigned _consecutive_failures{0};

	bool _sensitivity_adjustments_loaded{false};
	float _sensitivity[3] {1.f, 1.f, 1.f};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		READ_SENSITIVITY_ADJUSTMENTS,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{1};
	register_config_t _register_cfg[size_register_cfg] {
		// Register          | Set bits, Clear bits
		{ Register::CNTL1,   CNTL1_BIT::CONTINUOUS_MODE_2 | CNTL1_BIT::BIT_16, 0 },
	};
};
