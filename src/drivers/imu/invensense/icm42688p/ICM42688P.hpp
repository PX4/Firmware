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
 * @file ICM42688P.hpp
 *
 * Driver for the Invensense ICM42688P connected via SPI.
 *
 */

#pragma once

#include "InvenSense_ICM42688P_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace InvenSense_ICM42688P;

class ICM42688P : public device::SPI, public I2CSPIDriver<ICM42688P>
{
public:
	ICM42688P(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		  spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
	~ICM42688P() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / 8000.f};     // 8000 Hz accel & gyro ODR configured
	static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};
	static constexpr float ACCEL_RATE{1e6f / FIFO_SAMPLE_DT};

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
	static constexpr uint32_t FIFO_MAX_SAMPLES{math::min(math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0])), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]) * (int)(GYRO_RATE / ACCEL_RATE))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::BANK_0::INT_STATUS) | DIR_READ};
		uint8_t INT_STATUS{0};
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (4 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_bank0_config_t {
		Register::BANK_0 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct register_bank1_config_t {
		Register::BANK_1 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct register_bank2_config_t {
		Register::BANK_2 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureSampleRate(int sample_rate);
	void ConfigureFIFOWatermark(uint8_t samples);

	void SelectRegisterBank(enum REG_BANK_SEL_BIT bank);
	void SelectRegisterBank(Register::BANK_0 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0); }
	void SelectRegisterBank(Register::BANK_1 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_1); }
	void SelectRegisterBank(Register::BANK_2 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_2); }

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

	void ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	bool ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples);

	const spi_drdy_gpio_t _drdy_gpio;

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

	px4::atomic<uint32_t> _drdy_fifo_read_samples{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	uint32_t _fifo_gyro_samples{static_cast<uint32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	uint8_t _checked_register_bank0{0};
	static constexpr uint8_t size_register_bank0_cfg{13};
	register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
		// Register                              | Set bits, Clear bits
		{ Register::BANK_0::INT_CONFIG,           INT_CONFIG_BIT::INT1_MODE | INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT, INT_CONFIG_BIT::INT1_POLARITY },
		{ Register::BANK_0::FIFO_CONFIG,          FIFO_CONFIG_BIT::FIFO_MODE_STOP_ON_FULL, 0 },
		{ Register::BANK_0::PWR_MGMT0,            PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE, 0 },
		{ Register::BANK_0::GYRO_CONFIG0,         GYRO_CONFIG0_BIT::GYRO_ODR_8KHZ_SET, GYRO_CONFIG0_BIT::GYRO_ODR_8KHZ_CLEAR },
		{ Register::BANK_0::ACCEL_CONFIG0,        ACCEL_CONFIG0_BIT::ACCEL_ODR_8KHZ_SET, ACCEL_CONFIG0_BIT::ACCEL_ODR_8KHZ_CLEAR },
		{ Register::BANK_0::GYRO_CONFIG1,         0, GYRO_CONFIG1_BIT::GYRO_UI_FILT_ORD },
		{ Register::BANK_0::GYRO_ACCEL_CONFIG0,   0, GYRO_ACCEL_CONFIG0_BIT::ACCEL_UI_FILT_BW | GYRO_ACCEL_CONFIG0_BIT::GYRO_UI_FILT_BW },
		{ Register::BANK_0::ACCEL_CONFIG1,        0, ACCEL_CONFIG1_BIT::ACCEL_UI_FILT_ORD },
		{ Register::BANK_0::FIFO_CONFIG1,         FIFO_CONFIG1_BIT::FIFO_WM_GT_TH | FIFO_CONFIG1_BIT::FIFO_HIRES_EN | FIFO_CONFIG1_BIT::FIFO_TEMP_EN | FIFO_CONFIG1_BIT::FIFO_GYRO_EN | FIFO_CONFIG1_BIT::FIFO_ACCEL_EN, 0 },
		{ Register::BANK_0::FIFO_CONFIG2,         0, 0 }, // FIFO_WM[7:0] set at runtime
		{ Register::BANK_0::FIFO_CONFIG3,         0, 0 }, // FIFO_WM[11:8] set at runtime
		{ Register::BANK_0::INT_CONFIG0,          INT_CONFIG0_BIT::CLEAR_ON_FIFO_READ, 0 },
		{ Register::BANK_0::INT_SOURCE0,          INT_SOURCE0_BIT::FIFO_THS_INT1_EN, 0 },
	};

	uint8_t _checked_register_bank1{0};
	static constexpr uint8_t size_register_bank1_cfg{1};
	register_bank1_config_t _register_bank1_cfg[size_register_bank1_cfg] {
		// Register                              | Set bits, Clear bits
		{ Register::BANK_1::GYRO_CONFIG_STATIC2,  0, GYRO_CONFIG_STATIC2_BIT::GYRO_AAF_DIS | GYRO_CONFIG_STATIC2_BIT::GYRO_NF_DIS },
	};

	uint8_t _checked_register_bank2{0};
	static constexpr uint8_t size_register_bank2_cfg{1};
	register_bank2_config_t _register_bank2_cfg[size_register_bank2_cfg] {
		// Register                              | Set bits, Clear bits
		{ Register::BANK_2::ACCEL_CONFIG_STATIC2, 0, ACCEL_CONFIG_STATIC2_BIT::ACCEL_AAF_DIS },
	};
};
