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

#include "ICM20948_I2C_Passthrough.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM20948_I2C_Passthrough::ICM20948_I2C_Passthrough(I2CSPIBusOption bus_option, int bus, int bus_frequency) :
	I2C(DRV_IMU_DEVTYPE_ICM20948, MODULE_NAME, bus, I2C_ADDRESS_DEFAULT, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus)
{
}

ICM20948_I2C_Passthrough::~ICM20948_I2C_Passthrough()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int ICM20948_I2C_Passthrough::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM20948_I2C_Passthrough::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM20948_I2C_Passthrough::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("temperature: %.1f degC", (double)_temperature);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int ICM20948_I2C_Passthrough::probe()
{
	const uint8_t whoami = RegisterRead(Register::BANK_0::WHO_AM_I);

	if (whoami != WHOAMI) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void ICM20948_I2C_Passthrough::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// PWR_MGMT_1: Device Reset
		RegisterWrite(Register::BANK_0::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		if ((RegisterRead(Register::BANK_0::WHO_AM_I) == WHOAMI) && (RegisterRead(Register::BANK_0::PWR_MGMT_1) == 0x41)) {
			// Wakeup and reset
			RegisterWrite(Register::BANK_0::USER_CTRL, USER_CTRL_BIT::SRAM_RST | USER_CTRL_BIT::I2C_MST_RST);

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
			_state = STATE::READ;
			ScheduleOnInterval(500_ms);

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(10_ms);
		}

		break;

	case STATE::READ: {
			if (hrt_elapsed_time(&_last_config_check_timestamp) > 1000_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;

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

void ICM20948_I2C_Passthrough::SelectRegisterBank(enum REG_BANK_SEL_BIT bank)
{
	if (bank != _last_register_bank) {
		// select BANK_0
		uint8_t cmd_bank_sel[2];
		cmd_bank_sel[0] = static_cast<uint8_t>(Register::BANK_0::REG_BANK_SEL);
		cmd_bank_sel[1] = bank;

		transfer(cmd_bank_sel, 2, nullptr, 0);

		_last_register_bank = bank;
	}
}

bool ICM20948_I2C_Passthrough::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	return success;
}

template <typename T>
bool ICM20948_I2C_Passthrough::RegisterCheck(const T &reg_cfg)
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

template <typename T>
uint8_t ICM20948_I2C_Passthrough::RegisterRead(T reg)
{
	SelectRegisterBank(reg);

	uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t value = 0;
	transfer(&cmd, 1, &value, 1);
	return value;
}

template <typename T>
void ICM20948_I2C_Passthrough::RegisterWrite(T reg, uint8_t value)
{
	SelectRegisterBank(reg);

	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(reg);
	cmd[1] = value;
	transfer(cmd, sizeof(cmd), nullptr, 0);
}

template <typename T>
void ICM20948_I2C_Passthrough::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

void ICM20948_I2C_Passthrough::UpdateTemperature()
{
	SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	// read current temperature
	uint8_t cmd = static_cast<uint8_t>(Register::BANK_0::TEMP_OUT_H);
	uint8_t temperature_buf[2] {};

	if (transfer(&cmd, 1, temperature_buf, 2) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const int16_t TEMP_OUT = combine(temperature_buf[0], temperature_buf[1]);
	const float TEMP_degC = (TEMP_OUT / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(TEMP_degC)) {
		_temperature = TEMP_degC;
	}
}
