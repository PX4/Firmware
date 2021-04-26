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

#include "BMI088_Gyroscope.hpp"

#include <px4_platform/board_dma_alloc.h>

using namespace time_literals;

namespace Bosch::BMI088::Gyroscope
{

BMI088_Gyroscope::BMI088_Gyroscope(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation,
				   int bus_frequency, spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	BMI088(DRV_GYR_DEVTYPE_BMI088, "BMI088_Gyroscope", bus_option, bus, device, spi_mode, bus_frequency, drdy_gpio),
	_px4_gyro(get_device_id(), rotation)
{
	if (drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME"_gyro: DRDY missed");
	}

	ConfigureSampleRate(2000);
}

BMI088_Gyroscope::~BMI088_Gyroscope()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

void BMI088_Gyroscope::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BMI088_Gyroscope::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

int BMI088_Gyroscope::probe()
{
	const uint8_t chipid = RegisterRead(Register::GYRO_CHIP_ID);

	if (chipid != ID) {
		DEVICE_DEBUG("unexpected GYRO_CHIP_ID 0x%02x", chipid);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI088_Gyroscope::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {

	case STATE::SELFTEST:
		//SelfTest();
		_state = STATE::RESET;
		ScheduleDelayed(1_ms);
		break;

	case STATE::RESET:
		// GYRO_SOFTRESET: Writing a value of 0xB6 to this register resets the sensor.
		// Following a delay of 30 ms, all configuration settings are overwritten with their reset value.
		RegisterWrite(Register::GYRO_SOFTRESET, 0xB6);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(30_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::GYRO_CHIP_ID) == ID)) {
			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(1_ms);

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_READ: {
			SimpleFIFORead(now);
		}
		break;
	}
}

void BMI088_Gyroscope::ConfigureGyro()
{
	const uint8_t GYRO_RANGE = RegisterRead(Register::GYRO_RANGE) & (Bit3 | Bit2 | Bit1 | Bit0);

	switch (GYRO_RANGE) {
	case gyro_range_2000_dps:
		_px4_gyro.set_scale(math::radians(1.f / 16.384f));
		_px4_gyro.set_range(math::radians(2000.f));
		break;

	case gyro_range_1000_dps:
		_px4_gyro.set_scale(math::radians(1.f / 32.768f));
		_px4_gyro.set_range(math::radians(1000.f));
		break;

	case gyro_range_500_dps:
		_px4_gyro.set_scale(math::radians(1.f / 65.536f));
		_px4_gyro.set_range(math::radians(500.f));
		break;

	case gyro_range_250_dps:
		_px4_gyro.set_scale(math::radians(1.f / 131.072f));
		_px4_gyro.set_range(math::radians(250.f));
		break;

	case gyro_range_125_dps:
		_px4_gyro.set_scale(math::radians(1.f / 262.144f));
		_px4_gyro.set_range(math::radians(125.f));
		break;
	}
}

void BMI088_Gyroscope::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_samples = math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual sample limit
	_fifo_empty_interval_us = _fifo_samples * (1e6f / RATE);

	ConfigureFIFOWatermark(_fifo_samples);
}

void BMI088_Gyroscope::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold
	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CONFIG_0) {
			r.set_bits = samples;
			r.clear_bits = ~r.set_bits;
		}
	}
}

bool BMI088_Gyroscope::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	ConfigureGyro();

	return success;
}

int BMI088_Gyroscope::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BMI088_Gyroscope *>(arg)->DataReady();
	return 0;
}

void BMI088_Gyroscope::DataReady()
{
	uint32_t expected = 0;

	if (_drdy_fifo_read_samples.compare_exchange(&expected, _fifo_samples)) {
		ScheduleNow();
	}
}

bool BMI088_Gyroscope::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool BMI088_Gyroscope::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool BMI088_Gyroscope::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t BMI088_Gyroscope::RegisterRead(Register reg)
{
	uint8_t add = static_cast<uint8_t>(reg);
	uint8_t cmd[2] = {add, 0};
	transfer(&cmd[0], 1, &cmd[1], 1);
	return cmd[1];
}

void BMI088_Gyroscope::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t add = static_cast<uint8_t>(reg);
	uint8_t cmd[2] = {add, value};
	transfer(cmd, sizeof(cmd), nullptr, 0);
}

void BMI088_Gyroscope::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool BMI088_Gyroscope::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);

	//PX4_WARN("Estimated transfer size: %d", transfer_size);
	if (transfer((uint8_t *)&buffer, 1, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = buffer.f[i];

		const int16_t gyro_x = combine(fifo_sample.RATE_X_MSB, fifo_sample.RATE_X_LSB);
		const int16_t gyro_y = combine(fifo_sample.RATE_Y_MSB, fifo_sample.RATE_Y_LSB);
		const int16_t gyro_z = combine(fifo_sample.RATE_Z_MSB, fifo_sample.RATE_Z_LSB);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	_px4_gyro.updateFIFO(gyro);

	return true;
}

void BMI088_Gyroscope::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO_CONFIG_0: Writing to water mark level trigger in register 0x3D (FIFO_CONFIG_0) clears the FIFO buffer.
	RegisterWrite(Register::FIFO_CONFIG_0, 0);

	// FIFO_CONFIG_1: FIFO overrun condition can only be cleared by writing to the FIFO configuration register FIFO_CONFIG_1
	RegisterWrite(Register::FIFO_CONFIG_1, 0);

	// reset while FIFO is disabled
	_drdy_fifo_read_samples.store(0);

	// FIFO_CONFIG_0: restore FIFO watermark
	// FIFO_CONFIG_1: re-enable FIFO
	for (const auto &r : _register_cfg) {
		if ((r.reg == Register::FIFO_CONFIG_0) || (r.reg == Register::FIFO_CONFIG_1)) {
			RegisterSetAndClearBits(r.reg, r.set_bits, r.clear_bits);
		}
	}
}

bool BMI088_Gyroscope::SelfTest()
{
	//Datasheet page 17 self test

	//Set bit0 to enable built in self test
	RegisterWrite(Register::SELF_TEST, 0x01);
	usleep(10000);
	uint8_t res = 0;
	uint8_t test_res = false;

	while (true) {
		res = RegisterRead(Register::SELF_TEST);

		if ((res & 0x02) == 0x02) {
			if ((res & 0x04) == 0x00) {
				PX4_WARN("Gyro Self-test success");
				test_res = true;

			} else {
				PX4_WARN("Gyro Self-test error");
			}

			break;
		}
	}

	RegisterWrite(Register::SELF_TEST, 0x00);
	return test_res;
}

bool BMI088_Gyroscope::NormalRead(const hrt_abstime &timestamp_sample)
{
	float x = 0;
	float y = 0;
	float z = 0;
	uint8_t buffer[6] = {0};
	uint8_t cmd[1] = {static_cast<uint8_t>(Register::READ_GYRO)};

	transfer(&cmd[0], 1, &buffer[0], 6);

	uint8_t RATE_X_LSB = buffer[0];
	uint8_t RATE_X_MSB = buffer[1];
	uint8_t RATE_Y_LSB = buffer[2];
	uint8_t RATE_Y_MSB = buffer[3];
	uint8_t RATE_Z_LSB = buffer[4];
	uint8_t RATE_Z_MSB = buffer[5];

	const int16_t gyro_x = combine(RATE_X_MSB, RATE_X_LSB);
	const int16_t gyro_y = combine(RATE_Y_MSB, RATE_Y_LSB);
	const int16_t gyro_z = combine(RATE_Z_MSB, RATE_Z_LSB);

	// sensor's frame is +x forward, +y left, +z up
	//  flip y & z to publish right handed with z down (x forward, y right, z down)
	x = gyro_x;
	y = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
	z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

	_px4_gyro.update(timestamp_sample, x, y, z);

	return true;
}

bool BMI088_Gyroscope::SimpleFIFORead(const hrt_abstime &timestamp_sample)
{
	uint8_t n_frames;
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;
	gyro.dt = FIFO_SAMPLE_DT;

	uint8_t data_i[1] = {static_cast<uint8_t>(Register::FIFO_STATUS)};

	transfer(&data_i[0], 1, &n_frames, 1);

	n_frames &= 0x7F;

	int n_frames_to_read = 6;

	// don't read more than 8 frames at a time
	if (n_frames > n_frames_to_read) {
		n_frames = n_frames_to_read;
	}

	if (n_frames == 0) {
		return false;
	}

	uint8_t data[6 * n_frames];
	data[0] = static_cast<uint8_t>(Register::FIFO_DATA);

	if (transfer(&data[0], 1, &data[0], 6 * n_frames) != PX4_OK) {
		//PX4_WARN("transfer(&data[0], 1, &data[0], fifo_fill_level) != PX4_OK");
		return false;
	}

	for (uint8_t i = 0; i < n_frames; i++) {
		const uint8_t *d = &data[i * 6];
		int16_t xyz[3] {
			int16_t(uint16_t(d[0] | d[1] << 8)),
			int16_t(uint16_t(d[2] | d[3] << 8)),
			int16_t(uint16_t(d[4] | d[5] << 8))
		};

		gyro.x[i] = xyz[0];
		gyro.y[i] = (xyz[1] == INT16_MIN) ? INT16_MAX : -xyz[1];
		gyro.z[i] = (xyz[2] == INT16_MIN) ? INT16_MAX : -xyz[2];
		gyro.samples++;
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (gyro.samples > 0) {
		//PX4_WARN("accel.samples: %d", accel.samples);
		_px4_gyro.updateFIFO(gyro);
		return true;
	}

	return true;

}
} // namespace Bosch::BMI088::Gyroscope
