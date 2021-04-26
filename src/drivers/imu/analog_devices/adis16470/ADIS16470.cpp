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

#include "ADIS16470.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ADIS16470::ADIS16470(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		     spi_drdy_gpio_t drdy_gpio) :
	SPI(DRV_IMU_DEVTYPE_ADIS16470, MODULE_NAME, bus, device, SPIDEV_MODE3, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_drdy_gpio(drdy_gpio),
	_px4_accel(get_device_id(), rotation),
	_px4_gyro(get_device_id(), rotation)
{
	_debug_enabled = true;
}

ADIS16470::~ADIS16470()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int ADIS16470::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ADIS16470::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ADIS16470::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ADIS16470::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int ADIS16470::probe()
{
	// Power-On Start-Up Time 205 ms
	if (hrt_absolute_time() < 205_ms) {
		PX4_WARN("Power-On Start-Up Time is 205 ms");
	}

	const uint16_t PROD_ID = RegisterRead(Register::PROD_ID);

	if (PROD_ID != Product_identification) {
		DEVICE_DEBUG("unexpected PROD_ID 0x%02x", PROD_ID);
		return PX4_ERROR;
	}

	const uint16_t SERIAL_NUM = RegisterRead(Register::SERIAL_NUM);
	const uint16_t FIRM_REV = RegisterRead(Register::FIRM_REV);
	const uint16_t FIRM_DM = RegisterRead(Register::FIRM_DM);
	const uint16_t FIRM_Y = RegisterRead(Register::FIRM_Y);

	PX4_INFO("Serial Number: 0x%X, Firmware revision: 0x%X Date: Y %X DM %X", SERIAL_NUM, FIRM_REV, FIRM_Y, FIRM_DM);

	return PX4_OK;
}

void ADIS16470::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		perf_count(_reset_perf);
		// GLOB_CMD: software reset
		RegisterWrite(Register::GLOB_CMD, GLOB_CMD_BIT::Software_reset);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(193_ms); // 193 ms Software Reset Recovery Time
		break;

	case STATE::WAIT_FOR_RESET:

		if (_self_test_passed) {
			if ((RegisterRead(Register::PROD_ID) == Product_identification)) {
				// if reset succeeded then configure
				_state = STATE::CONFIGURE;
				ScheduleNow();

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

		} else {
			RegisterWrite(Register::GLOB_CMD, GLOB_CMD_BIT::Sensor_self_test);
			_state = STATE::SELF_TEST_CHECK;
			ScheduleDelayed(14_ms); // Self Test Time
		}

		break;

	case STATE::SELF_TEST_CHECK: {
			// read DIAG_STAT to check result
			const uint16_t DIAG_STAT = RegisterRead(Register::DIAG_STAT);

			if (DIAG_STAT != 0) {
				PX4_ERR("DIAG_STAT: %#X", DIAG_STAT);

			} else {
				PX4_DEBUG("self test passed");
				_self_test_passed = true;
				_state = STATE::RESET;
				ScheduleNow();
			}
		}
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
			}

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
			if (_data_ready_interrupt_enabled) {
				// push backup schedule back
				ScheduleDelayed(SAMPLE_INTERVAL_US * 2);
			}

			bool success = false;

			struct BurstRead {
				uint16_t cmd;
				uint16_t DIAG_STAT;
				int16_t X_GYRO_OUT;
				int16_t Y_GYRO_OUT;
				int16_t Z_GYRO_OUT;
				int16_t X_ACCL_OUT;
				int16_t Y_ACCL_OUT;
				int16_t Z_ACCL_OUT;
				int16_t TEMP_OUT;
				uint16_t DATA_CNTR;
				uint16_t checksum;
			} buffer{};

			// ADIS16470 burst report should be 176 bits
			static_assert(sizeof(BurstRead) == (176 / 8), "ADIS16470 report not 176 bits");

			buffer.cmd = static_cast<uint16_t>(Register::GLOB_CMD) << 8;

			set_frequency(SPI_SPEED_BURST);

			if (transferhword((uint16_t *)&buffer, (uint16_t *)&buffer, sizeof(buffer) / sizeof(uint16_t)) == PX4_OK) {

				// Calculate checksum and compare

				// Checksum = DIAG_STAT, Bits[15:8] + DIAG_STAT, Bits[7:0] +
				//  X_GYRO_OUT, Bits[15:8] + X_GYRO_OUT, Bits[7:0] +
				//  Y_GYRO_OUT, Bits[15:8] + Y_GYRO_OUT, Bits[7:0] +
				//  Z_GYRO_OUT, Bits[15:8] + Z_GYRO_OUT, Bits[7:0] +
				//  X_ACCL_OUT, Bits[15:8] + X_ACCL_OUT, Bits[7:0] +
				//  Y_ACCL_OUT, Bits[15:8] + Y_ACCL_OUT, Bits[7:0] +
				//  Z_ACCL_OUT, Bits[15:8] + Z_ACCL_OUT, Bits[7:0] +
				//  TEMP_OUT, Bits[15:8] + TEMP_OUT, Bits[7:0] +
				//  DATA_CNTR, Bits[15:8] + DATA_CNTR, Bits[7:0]
				uint8_t *checksum_helper = (uint8_t *)&buffer.DIAG_STAT;

				uint16_t checksum = 0;

				for (int i = 0; i < 18; i++) {
					checksum += checksum_helper[i];
				}

				if (buffer.checksum != checksum) {
					//PX4_DEBUG("adis_report.checksum: %X vs calculated: %X", buffer.checksum, checksum);
					perf_count(_bad_transfer_perf);
				}

				if (buffer.DIAG_STAT != DIAG_STAT_BIT::Data_path_overrun) {
					// Data path overrun. A 1 indicates that one of the
					// data paths have experienced an overrun condition.
					// If this occurs, initiate a reset,

					//Reset();
					//return;
				}

				// Check all Status/Error Flag Indicators (DIAG_STAT)
				if (buffer.DIAG_STAT != 0) {
					perf_count(_bad_transfer_perf);
				}

				// temperature 1 LSB = 0.1°C
				const float temperature = buffer.TEMP_OUT * 0.1f;
				_px4_accel.set_temperature(temperature);
				_px4_gyro.set_temperature(temperature);


				int16_t accel_x = buffer.X_ACCL_OUT;
				int16_t accel_y = buffer.Y_ACCL_OUT;
				int16_t accel_z = buffer.Z_ACCL_OUT;

				// sensor's frame is +x forward, +y left, +z up
				//  flip y & z to publish right handed with z down (x forward, y right, z down)
				accel_y = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
				accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

				_px4_accel.update(now, accel_x, accel_y, accel_z);


				int16_t gyro_x = buffer.X_GYRO_OUT;
				int16_t gyro_y = buffer.Y_GYRO_OUT;
				int16_t gyro_z = buffer.Z_GYRO_OUT;
				// sensor's frame is +x forward, +y left, +z up
				//  flip y & z to publish right handed with z down (x forward, y right, z down)
				gyro_y = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
				gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
				_px4_gyro.update(now, gyro_x, gyro_y, gyro_z);

				success = true;

				if (_failure_count > 0) {
					_failure_count--;
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

bool ADIS16470::Configure()
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

	_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);
	_px4_accel.set_range(40.f * CONSTANTS_ONE_G);
	_px4_gyro.set_scale(math::radians(2000.f / 32768.f));
	_px4_gyro.set_range(math::radians(2000.f));

	_px4_accel.set_scale(1.25f * CONSTANTS_ONE_G / 1000.0f); // accel 1.25 mg/LSB
	_px4_gyro.set_scale(math::radians(0.025f)); // gyro 0.025 °/sec/LSB

	return success;
}

int ADIS16470::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ADIS16470 *>(arg)->DataReady();
	return 0;
}

void ADIS16470::DataReady()
{
	ScheduleNow();
}

bool ADIS16470::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0;
}

bool ADIS16470::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool ADIS16470::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint16_t reg_value = RegisterRead(reg_cfg.reg);

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

uint16_t ADIS16470::RegisterRead(Register reg)
{
	set_frequency(SPI_SPEED);

	uint16_t cmd[1];
	cmd[0] = (static_cast<uint16_t>(reg) << 8);

	transferhword(cmd, nullptr, 1);
	px4_udelay(SPI_STALL_PERIOD);
	transferhword(nullptr, cmd, 1);

	return cmd[0];
}

void ADIS16470::RegisterWrite(Register reg, uint16_t value)
{
	set_frequency(SPI_SPEED);

	uint16_t cmd[2];
	cmd[0] = (((static_cast<uint16_t>(reg))     | DIR_WRITE) << 8) | ((0x00FF & value));
	cmd[1] = (((static_cast<uint16_t>(reg) + 1) | DIR_WRITE) << 8) | ((0xFF00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	px4_udelay(SPI_STALL_PERIOD);
	transferhword(cmd + 1, nullptr, 1);
}

void ADIS16470::RegisterSetAndClearBits(Register reg, uint16_t setbits, uint16_t clearbits)
{
	const uint16_t orig_val = RegisterRead(reg);

	uint16_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
