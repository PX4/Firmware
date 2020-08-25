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

#include "MPU6000.hpp"

#include <px4_platform/board_dma_alloc.h>

using namespace time_literals;
using namespace InvenSense_MPU6000;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb) { return (msb << 8u) | lsb; }

static constexpr uint32_t GYRO_RATE{8000};  // 8 kHz gyro
static constexpr uint32_t ACCEL_RATE{1000}; // 1 kHz accel

static constexpr uint32_t FIFO_INTERVAL{1000}; // 1000 us / 1000 Hz interval

static constexpr uint32_t FIFO_GYRO_SAMPLES{FIFO_INTERVAL / (1000000 / GYRO_RATE)};
static constexpr uint32_t FIFO_ACCEL_SAMPLES{FIFO_INTERVAL / (1000000 / ACCEL_RATE)};

MPU6000::MPU6000(int bus, uint32_t device, enum Rotation rotation) :
	SPI(MODULE_NAME, nullptr, bus, device, SPIDEV_MODE3, SPI_SPEED),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), ORB_PRIO_VERY_HIGH, rotation),
	_px4_gyro(get_device_id(), ORB_PRIO_VERY_HIGH, rotation)
{
	set_device_type(DRV_IMU_DEVTYPE_MPU6000);
	_px4_accel.set_device_type(DRV_IMU_DEVTYPE_MPU6000);
	_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_MPU6000);

	_px4_accel.set_update_rate(1000000 / FIFO_INTERVAL);
	_px4_gyro.set_update_rate(1000000 / FIFO_INTERVAL);
}

MPU6000::~MPU6000()
{
	Stop();

	if (_dma_data_buffer != nullptr) {
		board_dma_free(_dma_data_buffer, FIFO::SIZE);
	}

	perf_free(_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_interval_perf);
}

int MPU6000::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHOAMI) {
		PX4_WARN("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

bool MPU6000::Init()
{
	if (SPI::init() != PX4_OK) {
		PX4_ERR("SPI::init failed");
		return false;
	}

	if (!Reset()) {
		PX4_ERR("reset failed");
		return false;
	}

	// allocate DMA capable buffer
	_dma_data_buffer = (uint8_t *)board_dma_alloc(FIFO::SIZE);

	if (_dma_data_buffer == nullptr) {
		PX4_ERR("DMA alloc failed");
		return false;
	}

	Start();

	return true;
}

bool MPU6000::Reset()
{
	// PWR_MGMT_1: Device Reset
	// CLKSEL[2:0] must be set to 001 to achieve full gyroscope performance.
	RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
	usleep(2000);

	// PWR_MGMT_1: CLKSEL[2:0] must be set to 001 to achieve full gyroscope performance.
	RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
	usleep(100);

	// ACCEL_CONFIG: Accel 16 G range
	RegisterSetBits(Register::ACCEL_CONFIG, ACCEL_CONFIG_BIT::AFS_SEL_16G);
	_px4_accel.set_scale(CONSTANTS_ONE_G / 2048);
	_px4_accel.set_range(16.0f * CONSTANTS_ONE_G);

	// GYRO_CONFIG: Gyro 2000 degrees/second
	RegisterSetBits(Register::GYRO_CONFIG, GYRO_CONFIG_BIT::FS_SEL_2000_DPS);
	_px4_gyro.set_scale(math::radians(1.0f / 16.4f));
	_px4_gyro.set_range(math::radians(2000.0f));

	// reset done once data is ready
	const bool reset_done = !(RegisterRead(Register::PWR_MGMT_1) & PWR_MGMT_1_BIT::DEVICE_RESET);
	const bool clksel_done = (RegisterRead(Register::PWR_MGMT_1) & PWR_MGMT_1_BIT::CLKSEL_0);
	const bool data_ready = (RegisterRead(Register::INT_STATUS) & INT_STATUS_BIT::DATA_RDY_INT);

	return reset_done && clksel_done && data_ready;
}

void MPU6000::ResetFIFO()
{
	perf_count(_fifo_reset_perf);

	// FIFO_EN: disable FIFO
	RegisterWrite(Register::FIFO_EN, 0);
	RegisterClearBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_EN | USER_CTRL_BIT::FIFO_RESET);

	// USER_CTRL: reset FIFO then re-enable
	RegisterSetBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_RESET);
	up_udelay(1); // bit auto clears after one clock cycle of the internal 20 MHz clock
	RegisterSetBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_EN);

	// CONFIG:
	RegisterSetBits(Register::CONFIG, CONFIG_BIT::DLPF_CFG_BYPASS_DLPF);

	// FIFO_EN: enable both gyro and accel
	_data_ready_count = 0;
	RegisterWrite(Register::FIFO_EN, FIFO_EN_BIT::XG_FIFO_EN | FIFO_EN_BIT::YG_FIFO_EN | FIFO_EN_BIT::ZG_FIFO_EN |
		      FIFO_EN_BIT::ACCEL_FIFO_EN);
	up_udelay(10);
}

uint8_t MPU6000::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void MPU6000::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void MPU6000::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void MPU6000::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

int MPU6000::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	MPU6000 *dev = reinterpret_cast<MPU6000 *>(arg);
	dev->DataReady();
	return 0;
}

void MPU6000::DataReady()
{
	perf_count(_drdy_interval_perf);

	_data_ready_count++;

	if (_data_ready_count >= 8) {
		_time_data_ready = hrt_absolute_time();

		_data_ready_count = 0;

		// make another measurement
		ScheduleNow();
	}
}

void MPU6000::Start()
{
	Stop();

	ResetFIFO();

	// TODO: cleanup horrible DRDY define mess
#if defined(GPIO_SPI1_EXTI_MPU_DRDY)
	// Setup data ready on rising edge
	px4_arch_gpiosetevent(GPIO_SPI1_EXTI_MPU_DRDY, true, false, true, &MPU6000::DataReadyInterruptCallback, this);
	RegisterSetBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#else
	ScheduleOnInterval(FIFO_INTERVAL, FIFO_INTERVAL);
#endif
}

void MPU6000::Stop()
{
	// TODO: cleanup horrible DRDY define mess
#if defined(GPIO_SPI1_EXTI_MPU_DRDY)
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_SPI1_EXTI_MPU_DRDY, false, false, false, nullptr, nullptr);
	RegisterClearBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#else
	ScheduleClear();
#endif
}

void MPU6000::Run()
{
	// use timestamp from the data ready interrupt if available,
	//  otherwise use the time now roughly corresponding with the last sample we'll pull from the FIFO
	const hrt_abstime timestamp_sample = (hrt_elapsed_time(&_time_data_ready) < FIFO_INTERVAL) ? _time_data_ready :
					     hrt_absolute_time();

	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ;

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		return;
	}

	const size_t fifo_count = combine(fifo_count_buf[1], fifo_count_buf[2]);
	const int samples = (fifo_count / sizeof(FIFO::DATA) / 2) * 2; // round down to nearest 2

	if (samples < 1) {
		perf_count(_fifo_empty_perf);
		return;

	} else if (samples < 8) {
		// don't transfer fewer than 8 samples (1 accel + 8 gyro)
		return;

	} else if (samples > 16) {
		// not technically an overflow, but more samples than we expected
		perf_count(_fifo_overflow_perf);
		ResetFIFO();
		return;
	}

	// Transfer data
	struct TransferBuffer {
		uint8_t cmd;
		FIFO::DATA f[16]; // max 16 samples
	};
	static_assert(sizeof(TransferBuffer) == (sizeof(uint8_t) + 16 * sizeof(FIFO::DATA))); // ensure no struct padding

	TransferBuffer *report = (TransferBuffer *)_dma_data_buffer;
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);
	memset(report, 0, transfer_size);
	report->cmd = static_cast<uint8_t>(Register::FIFO_R_W) | DIR_READ;

	perf_begin(_transfer_perf);

	if (transfer(_dma_data_buffer, _dma_data_buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		return;
	}

	perf_end(_transfer_perf);

	PX4Accelerometer::FIFOSample accel;
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples / 8;
	accel.dt = FIFO_INTERVAL / FIFO_ACCEL_SAMPLES;

	PX4Gyroscope::FIFOSample gyro;
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = FIFO_INTERVAL / FIFO_GYRO_SAMPLES;

	// accel data is duplicated 8 times
	for (int i = 0; i < accel.samples; i++) {
		const FIFO::DATA &fifo_sample = report->f[i];

		// coordinate convention (x forward, y right, z down)
		accel.x[i] = combine(fifo_sample.ACCEL_XOUT_H, fifo_sample.ACCEL_XOUT_L);
		accel.y[i] = -combine(fifo_sample.ACCEL_YOUT_H, fifo_sample.ACCEL_YOUT_L);
		accel.z[i] = -combine(fifo_sample.ACCEL_ZOUT_H, fifo_sample.ACCEL_ZOUT_L);
	}

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = report->f[i];

		// coordinate convention (x forward, y right, z down)
		gyro.x[i] = combine(fifo_sample.GYRO_XOUT_H, fifo_sample.GYRO_XOUT_L);
		gyro.y[i] = -combine(fifo_sample.GYRO_YOUT_H, fifo_sample.GYRO_YOUT_L);
		gyro.z[i] = -combine(fifo_sample.GYRO_ZOUT_H, fifo_sample.GYRO_ZOUT_L);
	}


	// Temperature
	if (hrt_elapsed_time(&_time_last_temperature_update) > 1_s) {
		// read current temperature
		uint8_t temperature_buf[3] {};
		temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_OUT_H) | DIR_READ;

		if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
			return;
		}

		const int16_t TEMP_OUT = combine(temperature_buf[1], temperature_buf[2]);

		static constexpr float RoomTemp_Offset = 25.0f; // Room Temperature Offset 25°C
		static constexpr float Temp_Sensitivity = 326.8f; // Sensitivity 326.8 LSB/°C

		const float TEMP_degC = ((TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity) + 25.0f;

		_px4_accel.set_temperature(TEMP_degC);
		_px4_gyro.set_temperature(TEMP_degC);
	}

	_px4_gyro.updateFIFO(gyro);
	_px4_accel.updateFIFO(accel);
}

void MPU6000::PrintInfo()
{
	perf_print_counter(_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_interval_perf);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}
