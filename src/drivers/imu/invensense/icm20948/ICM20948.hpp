/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file ICM20948.hpp
 *
 * Driver for the Invensense ICM20948 connected via SPI.
 *
 */

#pragma once

#include "InvenSense_ICM20948_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include "ICM20948_AK09916.hpp"

using namespace InvenSense_ICM20948;

class ICM20948 : public device::SPI, public I2CSPIDriver<ICM20948>
{
public:
	ICM20948(const I2CSPIDriverConfig &config);
	~ICM20948() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / 9000.f};
	static constexpr int32_t SAMPLES_PER_TRANSFER{2};                    // ensure at least 1 new accel sample per transfer
	static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};             // 9000 Hz gyro
	static constexpr float ACCEL_RATE{GYRO_RATE / SAMPLES_PER_TRANSFER}; // 4500 Hz accel

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
	static constexpr int32_t FIFO_MAX_SAMPLES{math::min(math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0])), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]) * (int)(GYRO_RATE / ACCEL_RATE))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH) | DIR_READ};
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (3 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_bank0_config_t {
		Register::BANK_0 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct register_bank2_config_t {
		Register::BANK_2 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct register_bank3_config_t {
		Register::BANK_3 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureAccel();
	void ConfigureGyro();
	void ConfigureSampleRate(int sample_rate);

	void SelectRegisterBank(enum REG_BANK_SEL_BIT bank);
	void SelectRegisterBank(Register::BANK_0 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0); }
	void SelectRegisterBank(Register::BANK_2 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_2); }
	void SelectRegisterBank(Register::BANK_3 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_3); }

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	template <typename T> bool RegisterCheck(const T &reg_cfg);
	template <typename T> uint8_t RegisterRead(T reg);
	template <typename T> void RegisterWrite(T reg, uint8_t value);
	template <typename T> void RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits);
	template <typename T> void RegisterSetBits(T reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	template <typename T> void RegisterClearBits(T reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	bool ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void UpdateTemperature();

	const spi_drdy_gpio_t _drdy_gpio;

	// I2C AUX interface (slave 1 - 4)
	AKM_AK09916::ICM20948_AK09916 *_slave_ak09916_magnetometer{nullptr};
	friend class AKM_AK09916::ICM20948_AK09916;

	void I2CSlaveRegisterWrite(uint8_t slave_i2c_addr, uint8_t reg, uint8_t val);
	void I2CSlaveExternalSensorDataEnable(uint8_t slave_i2c_addr, uint8_t reg, uint8_t size);
	bool I2CSlaveExternalSensorDataRead(uint8_t *buffer, uint8_t length);

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	enum REG_BANK_SEL_BIT _last_register_bank {REG_BANK_SEL_BIT::USER_BANK_0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	int32_t _drdy_count{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	uint8_t _checked_register_bank0{0};
	static constexpr uint8_t size_register_bank0_cfg{6};
	register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
		// Register                             | Set bits, Clear bits
		{ Register::BANK_0::USER_CTRL,          USER_CTRL_BIT::FIFO_EN | USER_CTRL_BIT::I2C_MST_EN | USER_CTRL_BIT::I2C_IF_DIS, USER_CTRL_BIT::DMP_EN },
		{ Register::BANK_0::PWR_MGMT_1,         PWR_MGMT_1_BIT::CLKSEL_0, PWR_MGMT_1_BIT::DEVICE_RESET | PWR_MGMT_1_BIT::SLEEP },
		{ Register::BANK_0::INT_PIN_CFG,        INT_PIN_CFG_BIT::INT1_ACTL, 0 },
		{ Register::BANK_0::INT_ENABLE_1,       INT_ENABLE_1_BIT::RAW_DATA_0_RDY_EN, 0 },
		{ Register::BANK_0::FIFO_EN_2,          FIFO_EN_2_BIT::ACCEL_FIFO_EN | FIFO_EN_2_BIT::GYRO_Z_FIFO_EN | FIFO_EN_2_BIT::GYRO_Y_FIFO_EN | FIFO_EN_2_BIT::GYRO_X_FIFO_EN, FIFO_EN_2_BIT::TEMP_FIFO_EN },
		{ Register::BANK_0::FIFO_MODE,          FIFO_MODE_BIT::Snapshot, 0 },
	};

	uint8_t _checked_register_bank2{0};
	static constexpr uint8_t size_register_bank2_cfg{2};
	register_bank2_config_t _register_bank2_cfg[size_register_bank2_cfg] {
		// Register                             | Set bits, Clear bits
		{ Register::BANK_2::GYRO_CONFIG_1,      GYRO_CONFIG_1_BIT::GYRO_FS_SEL_2000_DPS, GYRO_CONFIG_1_BIT::GYRO_FCHOICE },
		{ Register::BANK_2::ACCEL_CONFIG,       ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G, ACCEL_CONFIG_BIT::ACCEL_FCHOICE },
	};

	uint8_t _checked_register_bank3{0};
	static constexpr uint8_t size_register_bank3_cfg{3};
	register_bank3_config_t _register_bank3_cfg[size_register_bank3_cfg] {
		// Register                             | Set bits, Clear bits
		{ Register::BANK_3::I2C_MST_CTRL,       0, 0 },
		{ Register::BANK_3::I2C_MST_DELAY_CTRL, 0, 0 },
		{ Register::BANK_3::I2C_SLV4_CTRL,      0, 0 },
	};
};
