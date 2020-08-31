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

#include "LSM9DS1.hpp"

using namespace time_literals;
using namespace ST_LSM9DS1;

static constexpr int16_t combine(uint8_t lsb, uint8_t msb) { return (msb << 8u) | lsb; }

LSM9DS1::LSM9DS1(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		 spi_mode_e spi_mode) :
	SPI(DRV_IMU_DEVTYPE_ST_LSM9DS1_AG, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_accel(get_device_id(), rotation),
	_px4_gyro(get_device_id(), rotation)
{
}

LSM9DS1::~LSM9DS1()
{
	perf_free(_interval_perf);
	perf_free(_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
}

int LSM9DS1::probe()
{
	uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami == LSM9DS1_WHO_AM_I) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

int LSM9DS1::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed (%i)", ret);
		return ret;
	}

	if (!Reset()) {
		PX4_ERR("reset failed");
		return PX4_ERROR;
	}

	Start();

	return PX4_OK;
}

bool LSM9DS1::Reset()
{
	// Reset
	// CTRL_REG8: SW_RESET
	RegisterSetBits(Register::CTRL_REG8, CTRL_REG8_BIT::SW_RESET);
	usleep(50); // Wait 50 μs (or wait until the SW_RESET bit of the CTRL_REG8 register returns to 0).

	RegisterSetBits(Register::CTRL_REG9, CTRL_REG9_BIT::I2C_DISABLE);

	// Gyroscope configuration
	// CTRL_REG1_G: Gyroscope 2000 degrees/second and ODR 952 Hz
	RegisterWrite(Register::CTRL_REG1_G,
		      CTRL_REG1_G_BIT::ODR_G_952HZ | CTRL_REG1_G_BIT::FS_G_2000DPS | CTRL_REG1_G_BIT::BW_G_100Hz);
	_px4_gyro.set_scale(math::radians(70.0f / 1000.0f)); // 70 mdps/LSB
	_px4_gyro.set_range(math::radians(2000.0f));

	// Accelerometer configuration
	// CTRL_REG6_XL: Accelerometer 16 G range and ODR 952 Hz
	RegisterWrite(Register::CTRL_REG6_XL, CTRL_REG6_XL_BIT::ODR_XL_952HZ | CTRL_REG6_XL_BIT::FS_XL_16);
	_px4_accel.set_scale(0.732f * (CONSTANTS_ONE_G / 1000.0f));	// 0.732 mg/LSB
	_px4_accel.set_range(16.0f * CONSTANTS_ONE_G);

	return true;
}

void LSM9DS1::ResetFIFO()
{
	perf_count(_fifo_reset_perf);

	// CTRL_REG9: disable FIFO
	RegisterClearBits(Register::CTRL_REG9, CTRL_REG9_BIT::FIFO_EN);

	// FIFO_CTRL: to reset FIFO content, Bypass mode (0) should be selected
	RegisterWrite(Register::FIFO_CTRL, 0);

	// CTRL_REG9: enable FIFO
	RegisterSetBits(Register::CTRL_REG9, CTRL_REG9_BIT::FIFO_EN);

	// CTRL_REG8: Note: When the FIFO is used, the IF_INC and BDU bits must be equal to 1.
	RegisterWrite(Register::CTRL_REG8, CTRL_REG8_BIT::BDU | CTRL_REG8_BIT::IF_ADD_INC);
	usleep(1);

	// FIFO_CTRL: FIFO continuous mode enabled
	RegisterWrite(Register::FIFO_CTRL, FIFO_CTRL_BIT::FIFO_MODE_CONTINUOUS);
	usleep(1);
}

uint8_t LSM9DS1::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void LSM9DS1::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void LSM9DS1::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void LSM9DS1::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

void LSM9DS1::Start()
{
	ResetFIFO();

	ScheduleOnInterval(_fifo_interval / 2, _fifo_interval);
}

void LSM9DS1::RunImpl()
{
	perf_count(_interval_perf);

	// Number of unread words (16-bit axes) stored in FIFO.
	const hrt_abstime timestamp_fifo_level = hrt_absolute_time();
	const uint8_t FIFO_SRC = RegisterRead(Register::FIFO_SRC);

	if (FIFO_SRC & FIFO_SRC_BIT::OVRN) {
		// overflow
		perf_count(_fifo_overflow_perf);
		ResetFIFO();
		return;
	}

	const uint8_t samples = FIFO_SRC & static_cast<uint8_t>(FIFO_SRC_BIT::FSS);

	if (samples < 1) {
		perf_count(_fifo_empty_perf);
		return;

	} else if (samples > 16) {
		// not technically an overflow, but more samples than we expected
		perf_count(_fifo_overflow_perf);
		ResetFIFO();
		return;
	}

	// estimate timestamp of first sample in the FIFO from number of samples and fill rate
	const hrt_abstime timestamp_sample = timestamp_fifo_level;

	int16_t accel_data[3] {};
	int16_t gyro_data[3] {};
	int accel_samples = 0;
	int gyro_samples = 0;

	perf_begin(_transfer_perf);

	for (int i = 0; i < samples; i++) {
		// Gyro
		{
			struct GyroReport {
				uint8_t cmd;
				uint8_t STATUS_REG;
				uint8_t OUT_X_L_G;
				uint8_t OUT_X_H_G;
				uint8_t OUT_Y_L_G;
				uint8_t OUT_Y_H_G;
				uint8_t OUT_Z_L_G;
				uint8_t OUT_Z_H_G;
			} greport{};
			greport.cmd = static_cast<uint8_t>(Register::STATUS_REG_G) | DIR_READ;

			if (transfer((uint8_t *)&greport, (uint8_t *)&greport, sizeof(GyroReport)) != PX4_OK) {
				perf_end(_transfer_perf);
				return;
			}

			if (greport.STATUS_REG & STATUS_REG_BIT::GDA) {
				// Gyroscope new data available

				// sensor Z is up (RHC), flip z for publication
				gyro_data[0] = combine(greport.OUT_X_L_G, greport.OUT_X_H_G);
				gyro_data[1] = combine(greport.OUT_Y_L_G, greport.OUT_Y_H_G);
				gyro_data[2] = -combine(greport.OUT_Z_L_G, greport.OUT_Z_H_G);
				gyro_samples++;
			}
		}

		// Accel
		{
			struct AccelReport {
				uint8_t cmd;
				uint8_t STATUS_REG;
				uint8_t OUT_X_L_XL;
				uint8_t OUT_X_H_XL;
				uint8_t OUT_Y_L_XL;
				uint8_t OUT_Y_H_XL;
				uint8_t OUT_Z_L_XL;
				uint8_t OUT_Z_H_XL;
			} areport{};
			areport.cmd = static_cast<uint8_t>(Register::STATUS_REG_A) | DIR_READ;

			if (transfer((uint8_t *)&areport, (uint8_t *)&areport, sizeof(AccelReport)) != PX4_OK) {
				perf_end(_transfer_perf);
				return;
			}

			if (areport.STATUS_REG & STATUS_REG_BIT::XLDA) {
				// Accelerometer new data available

				// sensor Z is up (RHC), flip z for publication
				accel_data[0] = combine(areport.OUT_X_L_XL, areport.OUT_X_H_XL);
				accel_data[1] = combine(areport.OUT_Y_L_XL, areport.OUT_Y_H_XL);
				accel_data[2] = -combine(areport.OUT_Z_L_XL, areport.OUT_Z_H_XL);
				accel_samples++;
			}
		}
	}

	perf_end(_transfer_perf);

	// get current temperature at 1 Hz
	if (hrt_elapsed_time(&_time_last_temperature_update) > 1_s) {
		uint8_t temperature_buf[3] {};
		temperature_buf[0] = static_cast<uint8_t>(Register::OUT_TEMP_L) | DIR_READ;

		if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
			return;
		}

		// 16 bits in two’s complement format with a sensitivity of 256 LSB/°C. The output zero level corresponds to 25 °C.
		const int16_t OUT_TEMP = combine(temperature_buf[1], temperature_buf[2] & 0x0F);
		const float temperature = (OUT_TEMP / 256.0f) + 25.0f;

		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}

	if ((accel_samples > 0) && (gyro_samples > 0)) {
		// published synchronized IMU
		_px4_accel.update(timestamp_sample, accel_data[0], accel_data[1], accel_data[2]);
		_px4_gyro.update(timestamp_sample, gyro_data[0], gyro_data[1], gyro_data[2]);
	}
}

void LSM9DS1::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_interval_perf);
	perf_print_counter(_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);

}
