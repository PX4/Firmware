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
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class MPU9250;

namespace AKM_AK8963
{

class MPU9250_AK8963 : public px4::ScheduledWorkItem
{
public:
	MPU9250_AK8963(MPU9250 &mpu9250, enum Rotation rotation = ROTATION_NONE);
	~MPU9250_AK8963() override;

	bool Reset();
	void PrintInfo();

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
		AKM_AK8963::Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	void Run() override;

	MPU9250 &_mpu9250;

	PX4Magnetometer _px4_mag;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME"_ak8963: bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME"_ak8963: bad transfer")};
	perf_counter_t _magnetic_sensor_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME"_ak09916: magnetic sensor overflow")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	bool _sensitivity_adjustments_loaded{false};
	float _sensitivity[3] {1.f, 1.f, 1.f};

	enum class STATE : uint8_t {
		RESET,
		READ_WHO_AM_I,
		WAIT_FOR_RESET,
		READ_SENSITIVITY_ADJUSTMENTS,
		READ,
	} _state{STATE::RESET};
};

} // namespace AKM_AK8963
