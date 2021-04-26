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

#include "BMM150.hpp"

using namespace time_literals;

BMM150::BMM150(I2CSPIBusOption bus_option, int bus, int bus_frequency, enum Rotation rotation) :
	I2C(DRV_MAG_DEVTYPE_BMM150, MODULE_NAME, bus, I2C_ADDRESS_DEFAULT, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_mag(get_device_id(), rotation)
{
	_px4_mag.set_external(external());
}

BMM150::~BMM150()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_overflow_perf);
}

int BMM150::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool BMM150::Reset()
{
	RegisterWrite(Register::POWER_CONTROL, 0);
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleDelayed(1_ms);
	return true;
}

void BMM150::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_overflow_perf);
}

int BMM150::probe()
{
	const uint8_t POWER_CONTROL = RegisterRead(Register::POWER_CONTROL);
	const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

	PX4_DEBUG("POWER_CONTROL: 0x%02hhX, CHIP_ID: 0x%02hhX", POWER_CONTROL, CHIP_ID);

	// either power control bit is set and chip ID can be read, or both registers are 0x00
	if ((POWER_CONTROL & POWER_CONTROL_BIT::PowerControl) && (CHIP_ID == chip_identification_number)) {
		return PX4_OK;

	} else if ((POWER_CONTROL == 0) && (CHIP_ID == 0)) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

float BMM150::compensate_x(int16_t mag_data_x, uint16_t data_rhall)
{
	float retval = 0;

	// Processing compensation equations
	//  not documented, but derived from https://github.com/BoschSensortec/BMM150-Sensor-API/blob/a20641f216057f0c54de115fe81b57368e119c01/bmm150.c#L1624-L1633 as of 2020-09-25
	float process_comp_x0 = (((float)_trim_data.dig_xyz1) * 16384.f / data_rhall);
	retval = (process_comp_x0 - 16384.f);
	float process_comp_x1 = ((float)_trim_data.dig_xy2) * (retval * retval / 268435456.f);
	float process_comp_x2 = process_comp_x1 + retval * ((float)_trim_data.dig_xy1) / 16384.f;
	float process_comp_x3 = ((float)_trim_data.dig_x2) + 160.f;
	float process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.f) * process_comp_x3);
	retval = ((process_comp_x4 / 8192.f) + (((float)_trim_data.dig_x1) * 8.f)) / 16.f;

	return retval;
}

float BMM150::compensate_y(int16_t mag_data_y, uint16_t data_rhall)
{
	float retval = 0;

	// Processing compensation equations
	//  not documented, but derived from https://github.com/BoschSensortec/BMM150-Sensor-API/blob/a20641f216057f0c54de115fe81b57368e119c01/bmm150.c#L1660-L1667 as of 2020-09-25
	float process_comp_y0 = ((float)_trim_data.dig_xyz1) * 16384.f / data_rhall;
	retval = process_comp_y0 - 16384.f;
	float process_comp_y1 = ((float)_trim_data.dig_xy2) * (retval * retval / 268435456.f);
	float process_comp_y2 = process_comp_y1 + retval * ((float)_trim_data.dig_xy1) / 16384.f;
	float process_comp_y3 = ((float)_trim_data.dig_y2) + 160.0f;
	float process_comp_y4 = mag_data_y * (((process_comp_y2) + 256.f) * process_comp_y3);
	retval = ((process_comp_y4 / 8192.f) + (((float)_trim_data.dig_y1) * 8.f)) / 16.f;

	return retval;
}

float BMM150::compensate_z(int16_t mag_data_z, uint16_t data_rhall)
{
	float retval = 0;

	// Processing compensation equations
	//  not documented, but derived from https://github.com/BoschSensortec/BMM150-Sensor-API/blob/a20641f216057f0c54de115fe81b57368e119c01/bmm150.c#L1696-L1703 as of 2020-09-25
	float process_comp_z0 = ((float)mag_data_z) - ((float)_trim_data.dig_z4);
	float process_comp_z1 = ((float)data_rhall) - ((float)_trim_data.dig_xyz1);
	float process_comp_z2 = (((float)_trim_data.dig_z3) * process_comp_z1);
	float process_comp_z3 = ((float)_trim_data.dig_z1) * ((float)data_rhall) / 32768.f;
	float process_comp_z4 = ((float)_trim_data.dig_z2) + process_comp_z3;
	float process_comp_z5 = (process_comp_z0 * 131072.f) - process_comp_z2;
	retval = (process_comp_z5 / ((process_comp_z4) * 4.f)) / 16.f;

	return retval;
}

static constexpr int16_t combine_xy_int13(const uint8_t msb, const uint8_t lsb)
{
	int16_t x = ((msb << 8) | lsb);
	return x / 8; // arithmetic shift by 3 (13 bit signed integer)
}

static constexpr int16_t combine_z_int15(const uint8_t msb, const uint8_t lsb)
{
	int16_t z = ((msb << 8) | lsb);
	return z / 2; // arithmetic shift by 1 (15 bit signed integer)
}

static constexpr uint16_t combine_rhall_uint14(const uint8_t msb, const uint8_t lsb)
{
	uint16_t rhall = ((msb << 8) | lsb);
	return (rhall >> 2) & 0x3FFF; // 14 bit unsigned integer
}

void BMM150::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// POWER_CONTROL: soft reset
		RegisterWrite(Register::POWER_CONTROL, POWER_CONTROL_BIT::SoftReset | POWER_CONTROL_BIT::PowerControl);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(3_ms); // 3.0 ms start-up time from suspend to sleep
		break;

	case STATE::WAIT_FOR_RESET:

		// Soft reset always brings the device into sleep mode (power off -> suspend -> sleep)
		if ((RegisterRead(Register::CHIP_ID) == chip_identification_number)
		    && (RegisterRead(Register::POWER_CONTROL) == POWER_CONTROL_BIT::PowerControl)
		    && (RegisterRead(Register::OP_MODE) == OP_MODE_BIT::Opmode_Sleep)) {

			// if reset succeeded then start self test
			RegisterSetBits(Register::OP_MODE, OP_MODE_BIT::Self_Test);

			_state = STATE::SELF_TEST_CHECK;
			ScheduleDelayed(10_ms);

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

	case STATE::SELF_TEST_CHECK: {
			// After performing self test OpMode "Self test" bit is set to 0
			const bool opmode_self_test_cleared = ((RegisterRead(Register::OP_MODE) & OP_MODE_BIT::Self_Test) == 0);

			// When self-test is successful, the corresponding self-test result bits are set
			//  “X-Self-Test” register 0x42 bit0
			//  “Y-Self-Test” register 0x44 bit0
			//  “Z-Self-Test” register 0x46 bit0
			const bool x_success = RegisterRead(Register::DATAX_LSB) & Bit0;
			const bool y_success = RegisterRead(Register::DATAY_LSB) & Bit0;
			const bool z_success = RegisterRead(Register::DATAZ_LSB) & Bit0;

			if (opmode_self_test_cleared && (!x_success || !y_success || !z_success)) {
				PX4_DEBUG("self test failed, resetting");
				perf_count(_self_test_failed_perf);
				_state = STATE::RESET;
				ScheduleDelayed(1000_ms);

			} else {
				_state = STATE::READ_TRIM;
				ScheduleDelayed(1_ms);
			}
		}

		break;

	case STATE::READ_TRIM: {
			// Trim register value is read
			uint8_t cmd = static_cast<uint8_t>(Register::DIG_X1);
			uint8_t trim_x1y1[2] {};

			if (transfer(&cmd, 1, trim_x1y1, 2) == PX4_OK) {
				cmd = static_cast<uint8_t>(Register::DIG_Z4_LSB);
				uint8_t trim_xyz_data[4] {};

				if (transfer(&cmd, 1, trim_xyz_data, 4) == PX4_OK) {
					cmd = static_cast<uint8_t>(Register::DIG_Z2_LSB);
					uint8_t trim_xy1xy2[10] {};

					if (transfer(&cmd, 1, trim_xy1xy2, 10) == PX4_OK) {
						_trim_data.dig_x1 = (int8_t)trim_x1y1[0];
						_trim_data.dig_y1 = (int8_t)trim_x1y1[1];

						_trim_data.dig_x2 = (int8_t)trim_xyz_data[2];
						_trim_data.dig_y2 = (int8_t)trim_xyz_data[3];

						uint16_t temp_msb;
						temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
						_trim_data.dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);

						temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
						_trim_data.dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);

						temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
						_trim_data.dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);

						temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
						_trim_data.dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);

						_trim_data.dig_xy1 = trim_xy1xy2[9];
						_trim_data.dig_xy2 = (int8_t)trim_xy1xy2[8];

						temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
						_trim_data.dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);

						if ((_trim_data.dig_xyz1 != 0) && (_trim_data.dig_z2 != 0) && (_trim_data.dig_z1 != 0)) {
							_state = STATE::CONFIGURE;
							ScheduleDelayed(1_ms);
							return;
						}
					}
				}
			}

			// reset if reading trim failed
			PX4_DEBUG("reading trim failed, resetting");
			perf_count(_bad_register_perf);
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading every 50 ms (20 Hz)
			_state = STATE::READ;
			ScheduleOnInterval(50_ms, 50_ms);

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
			struct TransferBuffer {
				uint8_t DATAX_LSB;
				uint8_t DATAX_MSB;
				uint8_t DATAY_LSB;
				uint8_t DATAY_MSB;
				uint8_t DATAZ_LSB;
				uint8_t DATAZ_MSB;
				uint8_t RHALL_LSB;
				uint8_t RHALL_MSB;
				uint8_t STATUS;
			} buffer{};

			bool success = false;
			// 0x42 to 0x4A with a burst read.
			uint8_t cmd = static_cast<uint8_t>(Register::DATAX_LSB);

			if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {

				int16_t x = combine_xy_int13(buffer.DATAX_MSB, buffer.DATAX_LSB);
				int16_t y = combine_xy_int13(buffer.DATAY_MSB, buffer.DATAY_LSB);
				int16_t z = combine_z_int15(buffer.DATAZ_MSB, buffer.DATAZ_LSB);
				uint16_t rhall = combine_rhall_uint14(buffer.RHALL_MSB, buffer.RHALL_LSB);

				const bool data_ready = buffer.RHALL_LSB & Bit0;

				if (data_ready && (rhall != 0)) {
					if ((buffer.STATUS & STATUS_BIT::Overflow) ||
					    (x == OVERFLOW_XYAXES) || (y == OVERFLOW_XYAXES) || (z == OVERFLOW_ZAXIS)) {
						// overflow ADC value, record error, but don't publish
						perf_count(_overflow_perf);

					} else {
						_px4_mag.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
						_px4_mag.update(now, compensate_x(x, rhall), compensate_y(y, rhall), compensate_z(z, rhall));

						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}

			} else {
				perf_count(_bad_transfer_perf);
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
			}
		}

		break;
	}
}

bool BMM150::Configure()
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

	// microTesla -> Gauss
	_px4_mag.set_scale(0.01f);

	return success;
}

bool BMM150::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t BMM150::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	int ret = transfer(&cmd, 1, &buffer, 1);

	if (ret != PX4_OK) {
		PX4_DEBUG("register read 0x%02hhX failed, ret = %d", cmd, ret);
		return -1;
	}

	return buffer;
}

void BMM150::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	int ret = transfer(buffer, sizeof(buffer), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_DEBUG("register write 0x%02hhX failed, ret = %d", (uint8_t)reg, ret);
	}
}

void BMM150::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
