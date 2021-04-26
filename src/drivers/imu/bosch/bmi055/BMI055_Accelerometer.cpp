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

#include "BMI055_Accelerometer.hpp"

#include <ecl/geo/geo.h> // CONSTANTS_ONE_G

using namespace time_literals;

namespace Bosch::BMI055::Accelerometer
{

BMI055_Accelerometer::BMI055_Accelerometer(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation,
		int bus_frequency, spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	BMI055(DRV_ACC_DEVTYPE_BMI055, "BMI055_Accelerometer", bus_option, bus, device, spi_mode, bus_frequency, drdy_gpio),
	_px4_accel(get_device_id(), rotation)
{
	if (drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME"_accel: DRDY missed");
	}

	ConfigureSampleRate(_px4_accel.get_max_rate_hz());
}

BMI055_Accelerometer::~BMI055_Accelerometer()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

void BMI055_Accelerometer::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BMI055_Accelerometer::print_status()
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

int BMI055_Accelerometer::probe()
{
	const uint8_t BGW_CHIPID = RegisterRead(Register::BGW_CHIPID);

	if (BGW_CHIPID != chip_id) {
		DEVICE_DEBUG("unexpected BGW_CHIPID 0x%02x", BGW_CHIPID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI055_Accelerometer::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// BGW_SOFTRESET: Writing a value of 0xB6 to this register resets the sensor.
		RegisterWrite(Register::BGW_SOFTRESET, 0xB6);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(25_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::BGW_CHIPID) == chip_id) {
			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(25_ms);

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
			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_fifo_read_samples was set
				if (_drdy_fifo_read_samples.fetch_and(0) != _fifo_samples) {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			// always check current FIFO status/count
			bool success = false;
			const uint8_t FIFO_STATUS = RegisterRead(Register::FIFO_STATUS);

			if (FIFO_STATUS & FIFO_STATUS_BIT::fifo_overrun) {
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else {
				const uint8_t fifo_frame_counter = FIFO_STATUS & FIFO_STATUS_BIT::fifo_frame_counter;

				if (fifo_frame_counter > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (fifo_frame_counter == 0) {
					perf_count(_fifo_empty_perf);

				} else if (fifo_frame_counter >= 1) {
					if (FIFORead(now, fifo_frame_counter)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void BMI055_Accelerometer::ConfigureAccel()
{
	const uint8_t PMU_RANGE = RegisterRead(Register::PMU_RANGE) & (Bit3 | Bit2 | Bit1 | Bit0);

	switch (PMU_RANGE) {
	case range_2g:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 1024.f); // 1024 LSB/g, 0.98mg/LSB
		_px4_accel.set_range(2.f * CONSTANTS_ONE_G);
		break;

	case range_4g:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 512.f); // 512 LSB/g, 1.95mg/LSB
		_px4_accel.set_range(4.f * CONSTANTS_ONE_G);
		break;

	case range_8g:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 256.f); // 256 LSB/g, 3.91mg/LSB
		_px4_accel.set_range(8.f * CONSTANTS_ONE_G);
		break;

	case range_16g:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 128.f); // 128 LSB/g, 7.81mg/LSB
		_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
		break;
	}
}

void BMI055_Accelerometer::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_samples = math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual sample limit
	_fifo_empty_interval_us = _fifo_samples * (1e6f / RATE);

	ConfigureFIFOWatermark(_fifo_samples);
}

void BMI055_Accelerometer::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold
	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_CONFIG_0) {
			r.set_bits = samples;
			r.clear_bits = ~r.set_bits;
		}
	}
}

bool BMI055_Accelerometer::Configure()
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

	ConfigureAccel();

	return success;
}

int BMI055_Accelerometer::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BMI055_Accelerometer *>(arg)->DataReady();
	return 0;
}

void BMI055_Accelerometer::DataReady()
{
	uint32_t expected = 0;

	if (_drdy_fifo_read_samples.compare_exchange(&expected, _fifo_samples)) {
		ScheduleNow();
	}
}

bool BMI055_Accelerometer::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool BMI055_Accelerometer::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool BMI055_Accelerometer::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t BMI055_Accelerometer::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void BMI055_Accelerometer::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void BMI055_Accelerometer::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool BMI055_Accelerometer::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples;
	accel.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = buffer.f[i];

		// acc_x_msb<11:4> + acc_x_lsb<3:0>
		const int16_t accel_x = combine(fifo_sample.ACCD_X_MSB, fifo_sample.ACCD_X_LSB) >> 4;
		const int16_t accel_y = combine(fifo_sample.ACCD_Y_MSB, fifo_sample.ACCD_Y_LSB) >> 4;
		const int16_t accel_z = combine(fifo_sample.ACCD_Z_MSB, fifo_sample.ACCD_Z_LSB) >> 4;

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = accel_x;
		accel.y[i] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		accel.z[i] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	_px4_accel.updateFIFO(accel);

	return true;
}

void BMI055_Accelerometer::FIFOReset()
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

void BMI055_Accelerometer::UpdateTemperature()
{
	// The slope of the temperature sensor is 0.5K/LSB, its center temperature is 23°C [(ACC 0x08) temp = 0x00].
	// The register contains the current chip temperature represented in two’s complement format.
	float temperature = static_cast<int8_t>(RegisterRead(Register::ACCD_TEMP)) * 0.5f + 23.f;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);

	} else {
		perf_count(_bad_transfer_perf);
	}
}

} // namespace Bosch::BMI055::Accelerometer
