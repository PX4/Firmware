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

#include "ADIS16448.hpp"

using namespace time_literals;

// computes the CCITT CRC16 on the data received from a burst read
static uint16_t ComputeCRC16(uint16_t burstData[13])
{
	uint16_t crc = 0xFFFF; // Holds the CRC value

	unsigned int data; // Holds the lower/Upper byte for CRC computation
	static constexpr unsigned int POLY = 0x1021; // Divisor used during CRC computation

	// Compute CRC on burst data starting from XGYRO_OUT and ending with TEMP_OUT.
	// Start with the lower byte and then the upper byte of each word.
	// i.e. Compute XGYRO_OUT_LSB CRC first and then compute XGYRO_OUT_MSB CRC.
	for (int i = 1; i < 12; i++) {
		unsigned int upperByte = (burstData[i] >> 8) & 0xFF;
		unsigned int lowerByte = (burstData[i] & 0xFF);
		data = lowerByte; // Compute lower byte CRC first

		for (int ii = 0; ii < 8; ii++, data >>= 1) {
			if ((crc & 0x0001) ^ (data & 0x0001)) {
				crc = (crc >> 1) ^ POLY;

			} else {
				crc >>= 1;
			}
		}

		data = upperByte; // Compute upper byte of CRC

		for (int ii = 0; ii < 8; ii++, data >>= 1) {
			if ((crc & 0x0001) ^ (data & 0x0001)) {
				crc = (crc >> 1) ^ POLY;

			} else {
				crc >>= 1;
			}
		}
	}

	crc = ~crc; // Compute complement of CRC
	data = crc;
	crc = (crc << 8) | (data >> 8 & 0xFF); // Perform byte swap prior to returning CRC

	return crc;
}

// convert 12 bit integer format to int16.
static int16_t convert12BitToINT16(uint16_t word)
{
	int16_t output = 0;

	if ((word >> 11) & 0x1) {
		// sign extend
		output = (word & 0xFFF) | 0xF000;

	} else {
		output = (word & 0x0FFF);
	}

	return output;
}

ADIS16448::ADIS16448(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		     spi_drdy_gpio_t drdy_gpio) :
	SPI(DRV_IMU_DEVTYPE_ADIS16448, MODULE_NAME, bus, device, SPIDEV_MODE3, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_drdy_gpio(drdy_gpio), // TODO: DRDY disabled
	_px4_accel(get_device_id(), rotation),
	_px4_baro(get_device_id()),
	_px4_gyro(get_device_id(), rotation),
	_px4_mag(get_device_id(), rotation)
{
}

ADIS16448::~ADIS16448()
{
	perf_free(_reset_perf);
	perf_free(_perf_crc_bad);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int ADIS16448::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ADIS16448::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ADIS16448::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ADIS16448::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int ADIS16448::probe()
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
	const uint16_t LOT_ID1 = RegisterRead(Register::LOT_ID1);
	const uint16_t LOT_ID2 = RegisterRead(Register::LOT_ID2);

	PX4_INFO("Serial Number: 0x%02x, Lot ID1: 0x%02x ID2: 0x%02x", SERIAL_NUM, LOT_ID1, LOT_ID2);

	// Only enable CRC-16 for verified lots (HACK to support older ADIS16448AMLZ with no explicit detection)
	if (LOT_ID1 == 0x1824) {
		_check_crc = true;
	}

	return PX4_OK;
}

void ADIS16448::RunImpl()
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
		ScheduleDelayed(90_ms); // Reset Recovery Time 90 ms
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
					PX4_DEBUG("Reset not complete, check again in 10 ms");
					ScheduleDelayed(10_ms);
				}
			}

		} else {
			RegisterWrite(Register::MSC_CTRL, MSC_CTRL_BIT::Internal_self_test);
			_state = STATE::SELF_TEST_CHECK;
			ScheduleDelayed(45_ms); // Automatic Self-Test Time 45 ms
		}

		break;

	case STATE::SELF_TEST_CHECK: {
			const uint16_t DIAG_STAT = RegisterRead(Register::DIAG_STAT);

			if (DIAG_STAT & DIAG_STAT_BIT::Self_test_diagnostic_error_flag) {
				PX4_ERR("self test failed");

				// Magnetometer
				if (DIAG_STAT & DIAG_STAT_BIT::Magnetometer_functional_test) {
					// tolerate mag test failure (likely due to surrounding magnetic field)
					PX4_ERR("Magnetometer functional test fail");
				}

				// Barometer
				if (DIAG_STAT & DIAG_STAT_BIT::Barometer_functional_test) {
					PX4_ERR("Barometer functional test test fail");
				}

				// Gyroscope
				const bool gyro_x_fail = DIAG_STAT & DIAG_STAT_BIT::X_axis_gyroscope_self_test_failure;
				const bool gyro_y_fail = DIAG_STAT & DIAG_STAT_BIT::Y_axis_gyroscope_self_test_failure;
				const bool gyro_z_fail = DIAG_STAT & DIAG_STAT_BIT::Z_axis_gyroscope_self_test_failure;

				if (gyro_x_fail || gyro_y_fail || gyro_z_fail) {
					PX4_ERR("gyroscope self-test failure");
				}

				// Accelerometer
				const bool accel_x_fail = DIAG_STAT & DIAG_STAT_BIT::X_axis_accelerometer_self_test_failure;
				const bool accel_y_fail = DIAG_STAT & DIAG_STAT_BIT::Y_axis_accelerometer_self_test_failure;
				const bool accel_z_fail = DIAG_STAT & DIAG_STAT_BIT::Z_axis_accelerometer_self_test_failure;

				if (accel_x_fail || accel_y_fail || accel_z_fail) {
					PX4_ERR("accelerometer self-test failure");
				}

				_self_test_passed = false;
				_state = STATE::RESET;
				ScheduleDelayed(1000_ms);

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
				int16_t XGYRO_OUT;
				int16_t YGYRO_OUT;
				int16_t ZGYRO_OUT;
				int16_t XACCL_OUT;
				int16_t YACCL_OUT;
				int16_t ZACCL_OUT;
				int16_t XMAGN_OUT;
				int16_t YMAGN_OUT;
				int16_t ZMAGN_OUT;
				uint16_t BARO_OUT;
				uint16_t TEMP_OUT;
				uint16_t CRC16;
			} buffer{};

			// ADIS16448 burst report should be 224 bits
			static_assert(sizeof(BurstRead) == (224 / 8), "ADIS16448 report not 224 bits");

			buffer.cmd = static_cast<uint16_t>(Register::GLOB_CMD) << 8;
			set_frequency(SPI_SPEED_BURST);

			if (transferhword((uint16_t *)&buffer, (uint16_t *)&buffer, sizeof(buffer) / sizeof(uint16_t)) == PX4_OK) {

				bool publish_data = true;

				// checksum
				if (_check_crc) {
					if (buffer.CRC16 != ComputeCRC16((uint16_t *)&buffer.DIAG_STAT)) {
						perf_count(_perf_crc_bad);
						publish_data = false;
					}
				}

				if (buffer.DIAG_STAT == DIAG_STAT_BIT::SPI_communication_failure) {
					perf_count(_bad_transfer_perf);
					publish_data = false;
				}

				if (publish_data) {

					const uint32_t error_count = perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf);
					_px4_accel.set_error_count(error_count);
					_px4_gyro.set_error_count(error_count);

					// temperature 0.07386°C/LSB, 31°C = 0x000
					const float temperature = (convert12BitToINT16(buffer.TEMP_OUT) * 0.07386f) + 31.f;

					_px4_accel.set_temperature(temperature);
					_px4_gyro.set_temperature(temperature);

					// sensor's frame is +x forward, +y left, +z up
					//  flip y & z to publish right handed with z down (x forward, y right, z down)
					const int16_t accel_x = buffer.XACCL_OUT;
					const int16_t accel_y = (buffer.YACCL_OUT == INT16_MIN) ? INT16_MAX : -buffer.YACCL_OUT;
					const int16_t accel_z = (buffer.ZACCL_OUT == INT16_MIN) ? INT16_MAX : -buffer.ZACCL_OUT;

					const int16_t gyro_x = buffer.XGYRO_OUT;
					const int16_t gyro_y = (buffer.YGYRO_OUT == INT16_MIN) ? INT16_MAX : -buffer.YGYRO_OUT;
					const int16_t gyro_z = (buffer.ZGYRO_OUT == INT16_MIN) ? INT16_MAX : -buffer.ZGYRO_OUT;

					_px4_accel.update(now, accel_x, accel_y, accel_z);
					_px4_gyro.update(now, gyro_x, gyro_y, gyro_z);

					// DIAG_STAT bit 7: New data, xMAGN_OUT/BARO_OUT
					if (buffer.DIAG_STAT & DIAG_STAT_BIT::New_data_xMAGN_OUT_BARO_OUT) {
						_px4_mag.set_error_count(error_count);
						_px4_mag.set_temperature(temperature);

						const int16_t mag_x = buffer.XMAGN_OUT;
						const int16_t mag_y = (buffer.YMAGN_OUT == INT16_MIN) ? INT16_MAX : -buffer.YMAGN_OUT;
						const int16_t mag_z = (buffer.ZMAGN_OUT == INT16_MIN) ? INT16_MAX : -buffer.ZMAGN_OUT;
						_px4_mag.update(now, mag_x, mag_y, mag_z);

						_px4_baro.set_error_count(error_count);
						_px4_baro.set_temperature(temperature);

						float pressure_pa = buffer.BARO_OUT * 0.02f; // 20 μbar per LSB
						_px4_baro.update(now, pressure_pa);
					}

					success = true;

					if (_failure_count > 0) {
						_failure_count--;
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

bool ADIS16448::Configure()
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

	_px4_accel.set_scale(0.833f * 1e-3f * CONSTANTS_ONE_G); // 0.833 mg/LSB
	_px4_gyro.set_scale(math::radians(0.04f));              // 0.04 °/sec/LSB
	_px4_mag.set_scale(142.9f * 1e-6f);                     // μgauss/LSB

	_px4_accel.set_range(18.f * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math::radians(1000.f));

	_px4_mag.set_external(external());

	return success;
}

int ADIS16448::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ADIS16448 *>(arg)->DataReady();
	return 0;
}

void ADIS16448::DataReady()
{
	ScheduleNow();
}

bool ADIS16448::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// check if DIO1 is connected to data ready
	{
		// DIO1 output set low
		RegisterWrite(Register::GPIO_CTRL, GPIO_CTRL_BIT::GPIO1_DIRECTION);
		bool write0_valid = (px4_arch_gpioread(_drdy_gpio) == 1);

		// DIO1 output set high
		RegisterWrite(Register::GPIO_CTRL, GPIO_CTRL_BIT::GPIO1_DATA_LEVEL | GPIO_CTRL_BIT::GPIO1_DIRECTION);
		bool write1_valid = (px4_arch_gpioread(_drdy_gpio) == 0);

		// DIO1 output set low again
		RegisterWrite(Register::GPIO_CTRL, GPIO_CTRL_BIT::GPIO1_DIRECTION);
		bool write2_valid = (px4_arch_gpioread(_drdy_gpio) == 1);

		if (write0_valid && write1_valid && write2_valid) {
			PX4_INFO("DIO1 DRDY valid");
			// Setup data ready on falling edge
			return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;

		} else {
			PX4_DEBUG("DIO1 DRDY invalid");
		}
	}

	// check if DIO2 is connected to data ready
	{
		// DIO2 output set low
		RegisterWrite(Register::GPIO_CTRL, GPIO_CTRL_BIT::GPIO2_DIRECTION);
		bool write0_valid = (px4_arch_gpioread(_drdy_gpio) == 1);

		// DIO2 output set high
		RegisterWrite(Register::GPIO_CTRL, GPIO_CTRL_BIT::GPIO2_DATA_LEVEL | GPIO_CTRL_BIT::GPIO2_DIRECTION);
		bool write1_valid = (px4_arch_gpioread(_drdy_gpio) == 0);

		// DIO2 output set low again
		RegisterWrite(Register::GPIO_CTRL, GPIO_CTRL_BIT::GPIO2_DIRECTION);
		bool write2_valid = (px4_arch_gpioread(_drdy_gpio) == 1);

		if (write0_valid && write1_valid && write2_valid) {
			PX4_INFO("DIO2 DRDY valid");

		} else {
			PX4_DEBUG("DIO2 DRDY invalid");
		}
	}

	return false;
}

bool ADIS16448::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool ADIS16448::RegisterCheck(const register_config_t &reg_cfg)
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

uint16_t ADIS16448::RegisterRead(Register reg)
{
	set_frequency(SPI_SPEED);

	uint16_t cmd[1];
	cmd[0] = (static_cast<uint16_t>(reg) << 8);

	transferhword(cmd, nullptr, 1);
	px4_udelay(SPI_STALL_PERIOD);
	transferhword(nullptr, cmd, 1);

	return cmd[0];
}

void ADIS16448::RegisterWrite(Register reg, uint16_t value)
{
	set_frequency(SPI_SPEED);

	uint16_t cmd[2];
	cmd[0] = (((static_cast<uint16_t>(reg))     | DIR_WRITE) << 8) | ((0x00FF & value));
	cmd[1] = (((static_cast<uint16_t>(reg) + 1) | DIR_WRITE) << 8) | ((0xFF00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	px4_udelay(SPI_STALL_PERIOD);
	transferhword(cmd + 1, nullptr, 1);
}

void ADIS16448::RegisterSetAndClearBits(Register reg, uint16_t setbits, uint16_t clearbits)
{
	const uint16_t orig_val = RegisterRead(reg);

	uint16_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
