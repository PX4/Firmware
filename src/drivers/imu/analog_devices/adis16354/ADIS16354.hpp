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
 * @file ADIS16354.hpp
 *
 * Driver for the Analog Devices ADIS16354 connected via SPI.
 *
 */

#pragma once

#include "Analog_Devices_ADIS16354_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace Analog_Devices_ADIS16354;

class ADIS16354 : public device::SPI, public I2CSPIDriver<ADIS16354>
{
public:
	ADIS16354(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		  spi_drdy_gpio_t drdy_gpio);
	~ADIS16354() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	struct register_config_t {
		Register reg;
		uint16_t set_bits{0};
		uint16_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint16_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint16_t value);
	void RegisterSetAndClearBits(Register reg, uint16_t setbits, uint16_t clearbits);

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	// perf_counter_t _perf_crc_bad{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": CRC16 bad"))};
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	bool _data_ready_interrupt_enabled{false};
	bool _self_test_passed{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		SELF_TEST_CHECK,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{3};
	register_config_t _register_cfg[size_register_cfg] {
		// Register              | Set bits, Clear bits
		// { Register::MSC_CTRL,    0, MSC_CTRL_BIT::DR_polarity },
		{ Register::SMPL_PRD,     SMPL_PRD_BIT::Sample_period_multiplier, SMPL_PRD_BIT::Time_base },
		{ Register::SENS_AVG,     SENS_AVG_BIT::Measurement_range_set | SENS_AVG_BIT::Number_of_filter_taps, SENS_AVG_BIT::Measurement_range_clear },
		{ Register::GPIO_CTRL,    GPIO_CTRL_BIT::GPIO2_DIRECTION | GPIO_CTRL_BIT::GPIO1_DIRECTION, 0},
	};
};
