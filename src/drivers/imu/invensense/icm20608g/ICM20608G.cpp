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

#include "ICM20608G.hpp"

#include <px4_platform/board_dma_alloc.h>

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM20608G::ICM20608G(int bus, uint32_t device, enum Rotation rotation) :
	SPI(MODULE_NAME, nullptr, bus, device, SPIDEV_MODE3, SPI_SPEED),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), ORB_PRIO_VERY_HIGH, rotation),
	_px4_gyro(get_device_id(), ORB_PRIO_VERY_HIGH, rotation)
{
	set_device_type(DRV_IMU_DEVTYPE_ICM20608);

	_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ICM20608);
	_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ICM20608);

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ICM20608G::~ICM20608G()
{
	Stop();

	if (_dma_data_buffer != nullptr) {
		board_dma_free(_dma_data_buffer, FIFO::SIZE);
	}

	perf_free(_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_interval_perf);
}

bool ICM20608G::Init()
{
	if (SPI::init() != PX4_OK) {
		PX4_ERR("SPI::init failed");
		return false;
	}

	// allocate DMA capable buffer
	_dma_data_buffer = (uint8_t *)board_dma_alloc(FIFO::SIZE);

	if (_dma_data_buffer == nullptr) {
		PX4_ERR("DMA alloc failed");
		return false;
	}

	return Reset();
}

void ICM20608G::Stop()
{
	// wait until stopped
	while (_state.load() != STATE::STOPPED) {
		_state.store(STATE::REQUEST_STOP);
		ScheduleNow();
		px4_usleep(10);
	}
}

bool ICM20608G::Reset()
{
	_state.store(STATE::RESET);
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM20608G::PrintInfo()
{
	PX4_INFO("FIFO empty interval: %d us (%.3f Hz)", _fifo_empty_interval_us,
		 static_cast<double>(1000000 / _fifo_empty_interval_us));

	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_interval_perf);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}

int ICM20608G::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHOAMI) {
		PX4_WARN("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void ICM20608G::Run()
{
	switch (_state.load()) {
	case STATE::RESET:
		// PWR_MGMT_1: Device Reset
		RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
		_reset_timestamp = hrt_absolute_time();
		_state.store(STATE::WAIT_FOR_RESET);
		ScheduleDelayed(100);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		//  Document Number: RM-000030 Page 5 of 23
		if ((RegisterRead(Register::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(Register::PWR_MGMT_1) == 0x40)) {

			// if reset succeeded then configure
			_state.store(STATE::CONFIGURE);
			ScheduleNow();

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 10_ms) {
				PX4_ERR("Reset failed, retrying");
				_state.store(STATE::RESET);
				ScheduleDelayed(10_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 1 ms");
				ScheduleDelayed(1_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state.store(STATE::FIFO_READ);

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(10_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

		} else {
			PX4_DEBUG("Configure failed, retrying");
			// try again in 1 ms
			ScheduleDelayed(1_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = 0;
			uint8_t samples = 0;

			if (_data_ready_interrupt_enabled) {
				// re-schedule as watchdog timeout
				ScheduleDelayed(10_ms);

				// timestamp set in data ready interrupt
				samples = _fifo_read_samples.load();
				timestamp_sample = _fifo_watermark_interrupt_timestamp;
			}

			bool failure = false;

			// manually check FIFO count if no samples from DRDY or timestamp looks bogus
			if (!_data_ready_interrupt_enabled || (samples == 0)
			    || (hrt_elapsed_time(&timestamp_sample) > (_fifo_empty_interval_us / 2))) {

				// use the time now roughly corresponding with the last sample we'll pull from the FIFO
				timestamp_sample = hrt_absolute_time();
				const uint16_t fifo_count = FIFOReadCount();

				if (fifo_count == 0) {
					failure = true;
					perf_count(_fifo_empty_perf);
				}

				samples = (fifo_count / sizeof(FIFO::DATA) / 2) * 2; // round down to nearest 2
			}

			if (samples > FIFO_MAX_SAMPLES) {
				// not technically an overflow, but more samples than we expected or can publish
				perf_count(_fifo_overflow_perf);
				failure = true;
				FIFOReset();

			} else if (samples >= 2) {
				// require at least 2 samples (we want at least 1 new accel sample per transfer)
				if (!FIFORead(timestamp_sample, samples)) {
					failure = true;
					_px4_accel.increase_error_count();
					_px4_gyro.increase_error_count();
				}
			}

			if (failure || hrt_elapsed_time(&_last_config_check_timestamp) > 10_ms) {
				// check registers incrementally
				if (RegisterCheck(_register_cfg[_checked_register], true)) {
					_last_config_check_timestamp = timestamp_sample;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reconfigure
					PX4_DEBUG("Health check failed, reconfiguring");
					_state.store(STATE::CONFIGURE);
					ScheduleNow();
				}

			} else {
				// periodically update temperature (1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) > 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = timestamp_sample;
				}
			}
		}

		break;

	case STATE::REQUEST_STOP:
		DataReadyInterruptDisable();
		ScheduleClear();
		_state.store(STATE::STOPPED);
		break;

	case STATE::STOPPED:
		// DO NOTHING
		break;
	}
}

void ICM20608G::ConfigureAccel()
{
	const uint8_t ACCEL_FS_SEL = RegisterRead(Register::ACCEL_CONFIG) & (Bit4 | Bit3); // [4:3] ACCEL_FS_SEL[1:0]

	switch (ACCEL_FS_SEL) {
	case ACCEL_FS_SEL_2G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 16384);
		_px4_accel.set_range(2 * CONSTANTS_ONE_G);
		break;

	case ACCEL_FS_SEL_4G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 8192);
		_px4_accel.set_range(4 * CONSTANTS_ONE_G);
		break;

	case ACCEL_FS_SEL_8G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 4096);
		_px4_accel.set_range(8 * CONSTANTS_ONE_G);
		break;

	case ACCEL_FS_SEL_16G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 2048);
		_px4_accel.set_range(16 * CONSTANTS_ONE_G);
		break;
	}
}

void ICM20608G::ConfigureGyro()
{
	const uint8_t FS_SEL = RegisterRead(Register::GYRO_CONFIG) & (Bit4 | Bit3); // [4:3] FS_SEL[1:0]

	switch (FS_SEL) {
	case FS_SEL_250_DPS:
		_px4_gyro.set_scale(math::radians(1.0f / 131.f));
		_px4_gyro.set_range(math::radians(250.f));
		break;

	case FS_SEL_500_DPS:
		_px4_gyro.set_scale(math::radians(1.0f / 65.5f));
		_px4_gyro.set_range(math::radians(500.f));
		break;

	case FS_SEL_1000_DPS:
		_px4_gyro.set_scale(math::radians(1.0f / 32.8f));
		_px4_gyro.set_range(math::radians(1000.0f));
		break;

	case FS_SEL_2000_DPS:
		_px4_gyro.set_scale(math::radians(1.0f / 16.4f));
		_px4_gyro.set_range(math::radians(2000.0f));
		break;
	}
}

void ICM20608G::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = 1000; // default to 1 kHz
	}

	_fifo_empty_interval_us = math::max(((1000000 / sample_rate) / 250) * 250, 250); // round down to nearest 250 us
	_fifo_gyro_samples = math::min(_fifo_empty_interval_us / (1000000 / GYRO_RATE), FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1000000 / GYRO_RATE);

	_fifo_accel_samples = math::min(_fifo_empty_interval_us / (1000000 / ACCEL_RATE), FIFO_MAX_SAMPLES);

	_px4_accel.set_update_rate(1000000 / _fifo_empty_interval_us);
	_px4_gyro.set_update_rate(1000000 / _fifo_empty_interval_us);
}

bool ICM20608G::Configure()
{
	bool success = true;

	for (const auto &reg : _register_cfg) {
		if (!RegisterCheck(reg)) {
			success = false;
		}
	}

	ConfigureAccel();
	ConfigureGyro();

	return success;
}

int ICM20608G::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM20608G *>(arg)->DataReady();
	return 0;
}

void ICM20608G::DataReady()
{
	if (_data_ready_count.fetch_add(1) >= (_fifo_gyro_samples - 1)) {
		_data_ready_count.store(0);
		_fifo_watermark_interrupt_timestamp = hrt_absolute_time();
		_fifo_read_samples.store(_fifo_gyro_samples);
		ScheduleNow();
	}

	perf_count(_drdy_interval_perf);
}

bool ICM20608G::DataReadyInterruptConfigure()
{
	int ret_setevent = -1;

	// Setup data ready on rising edge
	// TODO: cleanup horrible DRDY define mess
#if defined(GPIO_DRDY_PORTC_PIN14)
	ret_setevent = px4_arch_gpiosetevent(GPIO_DRDY_PORTC_PIN14, true, false, true, &ICM20608G::DataReadyInterruptCallback,
					     this);
#elif defined(GPIO_DRDY_ICM_2060X)
	ret_setevent = px4_arch_gpiosetevent(GPIO_DRDY_ICM_2060X, true, false, true, &ICM20608G::DataReadyInterruptCallback,
					     this);
#endif

	return (ret_setevent == 0);
}

bool ICM20608G::DataReadyInterruptDisable()
{
	int ret_setevent = -1;

	// Disable data ready callback
	// TODO: cleanup horrible DRDY define mess
#if defined(GPIO_DRDY_PORTC_PIN14)
	ret_setevent = px4_arch_gpiosetevent(GPIO_DRDY_PORTC_PIN14, false, false, false, nullptr, nullptr);
#elif defined(GPIO_DRDY_ICM_2060X)
	ret_setevent = px4_arch_gpiosetevent(GPIO_DRDY_ICM_2060X, false, false, false, nullptr, nullptr);
#endif

	return (ret_setevent == 0);
}

bool ICM20608G::RegisterCheck(const register_config_t &reg_cfg, bool notify)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && !(reg_value & reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && (reg_value & reg_cfg.clear_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	if (!success) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

		if (reg_cfg.reg == Register::ACCEL_CONFIG) {
			ConfigureAccel();

		} else if (reg_cfg.reg == Register::GYRO_CONFIG) {
			ConfigureGyro();
		}

		if (notify) {
			perf_count(_bad_register_perf);
			_px4_accel.increase_error_count();
			_px4_gyro.increase_error_count();
		}
	}

	return success;
}

uint8_t ICM20608G::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void ICM20608G::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ICM20608G::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = orig_val;

	if (setbits) {
		val |= setbits;
	}

	if (clearbits) {
		val &= ~clearbits;
	}

	RegisterWrite(reg, val);
}

void ICM20608G::RegisterSetBits(Register reg, uint8_t setbits)
{
	RegisterSetAndClearBits(reg, setbits, 0);
}

void ICM20608G::RegisterClearBits(Register reg, uint8_t clearbits)
{
	RegisterSetAndClearBits(reg, 0, clearbits);
}

uint16_t ICM20608G::FIFOReadCount()
{
	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ;

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return combine(fifo_count_buf[1], fifo_count_buf[2]);
}

bool ICM20608G::FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples)
{
	TransferBuffer *report = (TransferBuffer *)_dma_data_buffer;
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);
	memset(report, 0, transfer_size);
	report->cmd = static_cast<uint8_t>(Register::FIFO_R_W) | DIR_READ;

	perf_begin(_transfer_perf);

	if (transfer(_dma_data_buffer, _dma_data_buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		perf_count(_bad_transfer_perf);
		return false;
	}

	perf_end(_transfer_perf);

	ProcessGyro(timestamp_sample, report, samples);
	return ProcessAccel(timestamp_sample, report, samples);
}

void ICM20608G::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO_EN: disable FIFO
	RegisterWrite(Register::FIFO_EN, 0);

	// USER_CTRL: disable FIFO and reset all signal paths
	RegisterSetAndClearBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_RST | USER_CTRL_BIT::SIG_COND_RST,
				USER_CTRL_BIT::FIFO_EN);

	// reset while FIFO is disabled
	_data_ready_count.store(0);
	_fifo_watermark_interrupt_timestamp = 0;
	_fifo_read_samples.store(0);

	// FIFO_EN: enable both gyro and accel
	// USER_CTRL: re-enable FIFO
	for (const auto &r : _register_cfg) {
		if ((r.reg == Register::FIFO_EN) || (r.reg == Register::USER_CTRL)) {
			RegisterSetAndClearBits(r.reg, r.set_bits, r.clear_bits);
		}
	}
}

static bool fifo_accel_equal(const FIFO::DATA &f0, const FIFO::DATA &f1)
{
	return (memcmp(&f0.ACCEL_XOUT_H, &f1.ACCEL_XOUT_H, 6) == 0);
}

bool ICM20608G::ProcessAccel(const hrt_abstime &timestamp_sample, const TransferBuffer *const report, uint8_t samples)
{
	PX4Accelerometer::FIFOSample accel;
	accel.timestamp_sample = timestamp_sample;
	accel.dt = _fifo_empty_interval_us / _fifo_accel_samples;

	bool bad_data = false;

	// accel data is doubled in FIFO, but might be shifted
	int accel_first_sample = 1;

	if (samples >= 3) {
		if (fifo_accel_equal(report->f[0], report->f[1])) {
			// [A0, A1, A2, A3]
			//  A0==A1, A2==A3
			accel_first_sample = 1;

		} else if (fifo_accel_equal(report->f[1], report->f[2])) {
			// [A0, A1, A2, A3]
			//  A0, A1==A2, A3
			accel_first_sample = 0;

		} else {
			perf_count(_bad_transfer_perf);
			bad_data = true;
		}
	}

	int accel_samples = 0;

	for (int i = accel_first_sample; i < samples; i = i + 2) {
		const FIFO::DATA &fifo_sample = report->f[i];
		int16_t accel_x = combine(fifo_sample.ACCEL_XOUT_H, fifo_sample.ACCEL_XOUT_L);
		int16_t accel_y = combine(fifo_sample.ACCEL_YOUT_H, fifo_sample.ACCEL_YOUT_L);
		int16_t accel_z = combine(fifo_sample.ACCEL_ZOUT_H, fifo_sample.ACCEL_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[accel_samples] = accel_x;
		accel.y[accel_samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		accel.z[accel_samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
		accel_samples++;
	}

	accel.samples = accel_samples;

	_px4_accel.updateFIFO(accel);

	return !bad_data;
}

void ICM20608G::ProcessGyro(const hrt_abstime &timestamp_sample, const TransferBuffer *const report, uint8_t samples)
{
	PX4Gyroscope::FIFOSample gyro;
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = _fifo_empty_interval_us / _fifo_gyro_samples;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = report->f[i];

		const int16_t gyro_x = combine(fifo_sample.GYRO_XOUT_H, fifo_sample.GYRO_XOUT_L);
		const int16_t gyro_y = combine(fifo_sample.GYRO_YOUT_H, fifo_sample.GYRO_YOUT_L);
		const int16_t gyro_z = combine(fifo_sample.GYRO_ZOUT_H, fifo_sample.GYRO_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	_px4_gyro.updateFIFO(gyro);
}

void ICM20608G::UpdateTemperature()
{
	// read current temperature
	uint8_t temperature_buf[3] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_OUT_H) | DIR_READ;

	if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const int16_t TEMP_OUT = combine(temperature_buf[1], temperature_buf[2]);
	const float TEMP_degC = ((TEMP_OUT - ROOM_TEMPERATURE_OFFSET) / TEMPERATURE_SENSITIVITY) + ROOM_TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(TEMP_degC)) {
		_px4_accel.set_temperature(TEMP_degC);
		_px4_gyro.set_temperature(TEMP_degC);
	}
}
