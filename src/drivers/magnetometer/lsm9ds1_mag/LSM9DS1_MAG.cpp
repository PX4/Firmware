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

#include "LSM9DS1_MAG.hpp"

using namespace ST_LSM9DS1_MAG;
using ST_LSM9DS1_MAG::Register;

using namespace time_literals;

static constexpr int16_t combine(uint8_t lsb, uint8_t msb) { return (msb << 8u) | lsb; }

LSM9DS1_MAG::LSM9DS1_MAG(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation,
			 int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_MAG_DEVTYPE_ST_LSM9DS1_M, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_mag(get_device_id(), rotation)
{
	_px4_mag.set_external(external());
}

LSM9DS1_MAG::~LSM9DS1_MAG()
{
	perf_free(_interval_perf);
	perf_free(_transfer_perf);
	perf_free(_data_overrun_perf);
}

int LSM9DS1_MAG::probe()
{
	if (RegisterRead(Register::WHO_AM_I) == LSM9DS1_MAG_WHO_AM_I) {
		return PX4_OK;
	}

	return PX4_ERROR;
}

int LSM9DS1_MAG::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		return ret;
	}

	if (!Reset()) {
		PX4_ERR("reset failed");
		return PX4_ERROR;
	}

	Start();

	return PX4_OK;
}

bool LSM9DS1_MAG::Reset()
{
	// Reset
	// CTRL_REG2_M: SOFT_RST
	RegisterWrite(Register::CTRL_REG2_M, CTRL_REG2_M_BIT::SOFT_RST);
	usleep(50);

	// CTRL_REG1_M: Temp comp, ultra high perofmrance mode, ODR 80 Hz, fast ODR
	RegisterWrite(Register::CTRL_REG1_M,
		      CTRL_REG1_M_BIT::TEMP_COMP | CTRL_REG1_M_BIT::OM_ULTRA_HIGH_PERFORMANCE | CTRL_REG1_M_BIT::DO_80HZ);

	// CTRL_REG2_M:
	RegisterSetBits(Register::CTRL_REG2_M, CTRL_REG2_M_BIT::FS_16_GAUSS);
	_px4_mag.set_scale(0.58f / 1000.0f); // Magnetic FS = ±16 gauss 0.58 mgauss/LSB

	// CTRL_REG3_M: I2C_DISABLE, Continuous-conversion mode
	RegisterClearBits(Register::CTRL_REG3_M, CTRL_REG3_M_BIT::MD_CONTINUOUS_MODE);

	// CTRL_REG4_M: Z-axis Ultra-high performance mode
	RegisterSetBits(Register::CTRL_REG4_M, CTRL_REG4_M_BIT::OMZ_ULTRA_HIGH_PERFORMANCE);

	// CTRL_REG5_M: Block data update for magnetic data.
	RegisterSetBits(Register::CTRL_REG5_M, CTRL_REG5_M_BIT::BDU);

	return true;
}

uint8_t LSM9DS1_MAG::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | RW_BIT_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void LSM9DS1_MAG::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void LSM9DS1_MAG::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void LSM9DS1_MAG::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

void LSM9DS1_MAG::Start()
{
	ScheduleOnInterval(1000000 / ST_LSM9DS1_MAG::M_ODR / 2);
}

void LSM9DS1_MAG::RunImpl()
{
	perf_count(_interval_perf);

	struct MagReport {
		uint8_t cmd;
		uint8_t STATUS_REG_M;
		uint8_t OUT_X_L_M;
		uint8_t OUT_X_H_M;
		uint8_t OUT_Y_L_M;
		uint8_t OUT_Y_H_M;
		uint8_t OUT_Z_L_M;
		uint8_t OUT_Z_H_M;
	} mreport{};
	mreport.cmd = static_cast<uint8_t>(Register::STATUS_REG_M) | RW_BIT_READ | MS_BIT_AUTO_INCREMENT;

	perf_begin(_transfer_perf);
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (transfer((uint8_t *)&mreport, (uint8_t *)&mreport, sizeof(MagReport)) != PX4_OK) {
		perf_end(_transfer_perf);
		return;
	}

	perf_end(_transfer_perf);

	if (mreport.STATUS_REG_M & STATUS_REG_M_BIT::ZYXOR) {
		// X, Y and Z-axis data overrun.
		perf_count(_data_overrun_perf);
		return;
	}

	if (mreport.STATUS_REG_M & STATUS_REG_M_BIT::ZYXDA) {
		// X, Y and Z-axis new data available.

		// sensor Z is up (RHC), flip z for publication
		// sensor X is aligned with -X of lsm9ds1 accel/gyro
		int16_t x = -combine(mreport.OUT_X_L_M, mreport.OUT_X_H_M);
		int16_t y = combine(mreport.OUT_Y_L_M, mreport.OUT_Y_H_M);
		int16_t z = -combine(mreport.OUT_Z_L_M, mreport.OUT_Z_H_M);

		_px4_mag.update(timestamp_sample, x, y, z);
	}
}

void LSM9DS1_MAG::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_interval_perf);
	perf_print_counter(_transfer_perf);
	perf_print_counter(_data_overrun_perf);
}
