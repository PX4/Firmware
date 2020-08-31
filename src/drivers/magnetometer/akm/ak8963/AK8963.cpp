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

#include "AK8963.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

AK8963::AK8963(I2CSPIBusOption bus_option, int bus, int bus_frequency, enum Rotation rotation) :
	I2C(DRV_MAG_DEVTYPE_AK8963, MODULE_NAME, bus, I2C_ADDRESS_DEFAULT, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_mag(get_device_id(), rotation)
{
	_px4_mag.set_external(external());
}

AK8963::~AK8963()
{
	perf_free(_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_magnetic_sensor_overflow_perf);
}

int AK8963::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool AK8963::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void AK8963::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_magnetic_sensor_overflow_perf);
}

int AK8963::probe()
{
	const uint8_t WIA = RegisterRead(Register::WIA);

	if (WIA != Device_ID) {
		DEVICE_DEBUG("unexpected WIA 0x%02x", WIA);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void AK8963::RunImpl()
{
	switch (_state) {
	case STATE::RESET:
		// CNTL2 SRST: Soft reset
		RegisterWrite(Register::CNTL2, CNTL2_BIT::SRST);
		_reset_timestamp = hrt_absolute_time();
		_consecutive_failures = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::WIA) == Device_ID) {
			// if reset succeeded then configure
			if (!_sensitivity_adjustments_loaded) {
				// Set Fuse ROM Access mode before reading Fuse ROM data.
				RegisterWrite(Register::CNTL1, CNTL1_BIT::BIT_16 | CNTL1_BIT::FUSE_ROM_ACCESS_MODE);
				_state = STATE::READ_SENSITIVITY_ADJUSTMENTS;
				ScheduleDelayed(100_ms);

			} else {
				// if reset succeeded then configure
				RegisterWrite(Register::CNTL1, CNTL1_BIT::CONTINUOUS_MODE_2 | CNTL1_BIT::BIT_16);
				_state = STATE::CONFIGURE;
				ScheduleDelayed(100_ms);
			}

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 100 ms");
				ScheduleDelayed(100_ms);
			}
		}

		break;

	case STATE::READ_SENSITIVITY_ADJUSTMENTS: {
			// read FUSE ROM (to get ASA corrections)
			uint8_t response[3] {};
			uint8_t cmd = static_cast<uint8_t>(Register::ASAX);

			if (transfer(&cmd, 1, response, 3) == PX4_OK) {

				bool valid = true;

				for (int i = 0; i < 3; i++) {
					if (response[i] != 0 && response[i] != 0xFF) {
						_sensitivity[i] = ((float)(response[i] - 128) / 256.f) + 1.f;

					} else {
						valid = false;
					}
				}

				_sensitivity_adjustments_loaded = valid;

				// After reading fuse ROM data, set power-down mode (MODE[3:0]=“0000”) before the transition to another mode.
			}

			// reset on success or failure
			RegisterWrite(Register::CNTL1, 0);
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
		}
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::READ;
			ScheduleOnInterval(10_ms, 10_ms); // 100 Hz

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

	case STATE::READ: {
			perf_begin(_transfer_perf);
			TransferBuffer buffer{};
			const hrt_abstime timestamp_sample = hrt_absolute_time();
			uint8_t cmd = static_cast<uint8_t>(Register::ST1);
			int ret = transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(TransferBuffer));
			perf_end(_transfer_perf);

			bool success = false;

			if (ret == PX4_OK) {
				if (buffer.ST2 & ST2_BIT::HOFL) {
					perf_count(_magnetic_sensor_overflow_perf);

				} else if ((buffer.ST1 & ST1_BIT::DRDY) && (buffer.ST2 & ST2_BIT::BITM)) {

					const int16_t x = combine(buffer.HXH, buffer.HXL);
					const int16_t y = combine(buffer.HYH, buffer.HYL);
					const int16_t z = combine(buffer.HZH, buffer.HZL);

					// sensor's frame is +Y forward (X), -X right (Y), +Z down (Z)
					// adjust with sensitivity scale factors
					float x_f = y * _sensitivity[0];  // X := +Y
					float y_f = -x * _sensitivity[1]; // Y := -X
					float z_f = z * _sensitivity[2];  // Z := +Z

					_px4_mag.update(timestamp_sample, x_f, y_f, z_f);

					success = true;

					_consecutive_failures = 0;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = timestamp_sample;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
					return;
				}
			}

			if (_consecutive_failures > 10) {
				Reset();
			}
		}

		break;
	}
}

bool AK8963::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterWrite(reg_cfg.reg, reg_cfg.set_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// in 16-bit sampling mode (ST2 BITM) the mag resolution is 1.5 milli Gauss per bit (0.15 μT/LSB)
	_px4_mag.set_scale(1.5e-3f);

	return success;
}

bool AK8963::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t AK8963::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void AK8963::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void AK8963::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
