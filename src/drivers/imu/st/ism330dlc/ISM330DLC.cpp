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

#include "ISM330DLC.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t lsb, uint8_t msb) { return (msb << 8u) | lsb; }

static constexpr uint32_t FIFO_INTERVAL{1000}; // 1000 us / 1000 Hz interval

ISM330DLC::ISM330DLC(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		     spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	SPI(DRV_IMU_DEVTYPE_ST_ISM330DLC, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_drdy_gpio(drdy_gpio),
	_px4_accel(get_device_id(), rotation),
	_px4_gyro(get_device_id(), rotation)
{
}

ISM330DLC::~ISM330DLC()
{
	perf_free(_interval_perf);
	perf_free(_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_count_perf);
	perf_free(_drdy_interval_perf);
}

void ISM330DLC::exit_and_cleanup()
{
	if (_drdy_gpio != 0) {
		// Disable data ready callback
		px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);

		RegisterWrite(Register::INT1_CTRL, 0);
	}

	I2CSPIDriverBase::exit_and_cleanup();
}

int ISM330DLC::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != ISM330DLC_WHO_AM_I) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int ISM330DLC::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	if (!Reset()) {
		PX4_ERR("reset failed");
		return PX4_ERROR;
	}

	Start();

	return PX4_OK;
}

bool ISM330DLC::Reset()
{
	// CTRL3_C: SW_RESET
	// Note: When the FIFO is used, the IF_INC and BDU bits must be equal to 1.
	RegisterSetBits(Register::CTRL3_C, CTRL3_C_BIT::BDU | CTRL3_C_BIT::IF_INC | CTRL3_C_BIT::SW_RESET);
	usleep(50);	// Wait 50 μs (or wait until the SW_RESET bit of the CTRL3_C register returns to 0).

	// Accelerometer configuration
	// CTRL1_XL: Accelerometer 16 G range and ODR 6.66 kHz
	RegisterWrite(Register::CTRL1_XL, CTRL1_XL_BIT::ODR_XL_6_66KHZ | CTRL1_XL_BIT::FS_XL_16);
	_px4_accel.set_scale(0.488f * (CONSTANTS_ONE_G / 1000.0f));	// 0.488 mg/LSB
	_px4_accel.set_range(16.0f * CONSTANTS_ONE_G);

	// Gyroscope configuration
	// CTRL2_G: Gyroscope 2000 degrees/second and ODR 6.66 kHz
	// CTRL6_C: Gyroscope low-pass filter bandwidth 937 Hz (maximum)
	RegisterWrite(Register::CTRL2_G, CTRL2_G_BIT::ODR_G_6_66KHZ | CTRL2_G_BIT::FS_G_2000);
	RegisterSetBits(Register::CTRL6_C, CTRL6_C_BIT::FTYPE_GYRO_LPF_BW_937_HZ);
	_px4_gyro.set_scale(math::radians(70.0f / 1000.0f));	// 70 mdps/LSB
	_px4_gyro.set_range(math::radians(2000.0f));

	// reset is considered done once data is ready
	const bool reset_done = ((RegisterRead(Register::CTRL3_C) & CTRL3_C_BIT::SW_RESET) == 0);

	return reset_done;
}

void ISM330DLC::ResetFIFO()
{
	perf_count(_fifo_reset_perf);

	// FIFO_CTRL5 - disable FIFO
	RegisterWrite(Register::FIFO_CTRL5, 0);

	// CTRL5_C: rounding mode gyro + accel
	RegisterWrite(Register::CTRL5_C, CTRL5_C_BIT::ROUNDING_GYRO_ACCEL);

	// FIFO_CTRL3: full gyro and accel data to FIFO
	RegisterWrite(Register::FIFO_CTRL3, FIFO_CTRL3_BIT::DEC_FIFO_GYRO | FIFO_CTRL3_BIT::DEC_FIFO_XL);

	// FIFO_CTRL5: FIFO ODR is set to 6.66 kHz, and FIFO continuous mode enabled
	RegisterWrite(Register::FIFO_CTRL5, FIFO_CTRL5_BIT::ODR_FIFO_6_66_KHZ | FIFO_CTRL5_BIT::FIFO_MODE_CONTINUOUS);
}

uint8_t ISM330DLC::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void ISM330DLC::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ISM330DLC::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void ISM330DLC::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

int ISM330DLC::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	ISM330DLC *dev = reinterpret_cast<ISM330DLC *>(arg);
	dev->DataReady();
	return 0;
}

void ISM330DLC::DataReady()
{
	_time_data_ready = hrt_absolute_time();

	perf_count(_drdy_count_perf);
	perf_count(_drdy_interval_perf);

	// make another measurement
	ScheduleNow();
}

void ISM330DLC::Start()
{
	ResetFIFO();

	if (_drdy_gpio != 0 && false) { // TODO: enable
		// Setup data ready on rising edge
		px4_arch_gpiosetevent(_drdy_gpio, true, true, false, &ISM330DLC::DataReadyInterruptCallback, this);

		// FIFO threshold level setting
		// FIFO_CTRL1: FTH_[7:0]
		// FIFO_CTRL2: FTH_[10:8]
		const uint8_t fifo_threshold = 12;
		RegisterWrite(Register::FIFO_CTRL1, fifo_threshold);

		// INT1: FIFO full, overrun, or threshold
		RegisterWrite(Register::INT1_CTRL, INT1_CTRL_BIT::INT1_FULL_FLAG | INT1_CTRL_BIT::INT1_FIFO_OVR |
			      INT1_CTRL_BIT::INT1_FTH);

	} else {
		ScheduleOnInterval(FIFO_INTERVAL, FIFO_INTERVAL);
	}
}

void ISM330DLC::RunImpl()
{
	perf_count(_interval_perf);

	// use timestamp from the data ready interrupt if available,
	//  otherwise use the time now roughly corresponding with the last sample we'll pull from the FIFO
	const hrt_abstime timestamp_sample = (hrt_elapsed_time(&_time_data_ready) < FIFO_INTERVAL) ? _time_data_ready :
					     hrt_absolute_time();

	// Number of unread words (16-bit axes) stored in FIFO.
	const uint8_t fifo_words = RegisterRead(Register::FIFO_STATUS1);

	// check for FIFO status
	const uint8_t FIFO_STATUS2 = RegisterRead(Register::FIFO_STATUS2);

	if (FIFO_STATUS2 & FIFO_STATUS2_BIT::OVER_RUN) {
		// overflow
		perf_count(_fifo_overflow_perf);
		ResetFIFO();
		return;

	} else if (FIFO_STATUS2 & FIFO_STATUS2_BIT::FIFO_EMPTY) {
		// fifo empty could indicate a problem, reset
		perf_count(_fifo_empty_perf);
		ResetFIFO();
		return;
	}

	// FIFO pattern: indicates Next reading from FIFO output registers (Gx, Gy, Gz, XLx, XLy, XLz)
	const uint8_t fifo_pattern = RegisterRead(Register::FIFO_STATUS3);

	if (fifo_pattern != 0) {
		PX4_DEBUG("check FIFO pattern: %d", fifo_pattern);
	}

	// Transfer data
	// only transfer out complete sets of gyro + accel
	const int samples = (fifo_words / 2) / sizeof(FIFO::DATA);

	if (samples < 1) {
		perf_count(_fifo_empty_perf);
		return;

	} else if (samples > 16) {
		// not technically an overflow, but more samples than we expected
		perf_count(_fifo_overflow_perf);
		ResetFIFO();
		return;
	}

	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);

	perf_begin(_transfer_perf);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		return;
	}

	perf_end(_transfer_perf);

	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = samples;
	accel.dt = 1000000 / ST_ISM330DLC::LA_ODR;

	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = 1000000 / ST_ISM330DLC::G_ODR;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = buffer.f[i];

		// sensor Z is up (RHC), flip y & z for publication
		gyro.x[i] = combine(fifo_sample.OUTX_L_G, fifo_sample.OUTX_H_G);
		gyro.y[i] = -combine(fifo_sample.OUTY_L_G, fifo_sample.OUTY_H_G);
		gyro.z[i] = -combine(fifo_sample.OUTZ_L_G, fifo_sample.OUTZ_H_G);

		accel.x[i] = combine(fifo_sample.OUTX_L_XL, fifo_sample.OUTX_H_XL);
		accel.y[i] = -combine(fifo_sample.OUTY_L_XL, fifo_sample.OUTY_H_XL);
		accel.z[i] = -combine(fifo_sample.OUTZ_L_XL, fifo_sample.OUTZ_H_XL);
	}

	// get current temperature at 1 Hz
	if (hrt_elapsed_time(&_time_last_temperature_update) > 1_s) {
		uint8_t temperature_buf[3] {};
		temperature_buf[0] = static_cast<uint8_t>(Register::OUT_TEMP_L) | DIR_READ;

		if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
			return;
		}

		// 16 bits in two’s complement format with a sensitivity of 256 LSB/°C. The output zero level corresponds to 25 °C.
		const int16_t OUT_TEMP = combine(temperature_buf[1], temperature_buf[2]);
		const float temperature = ((float)OUT_TEMP / 256.0f) + 25.0f;

		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}

	_px4_gyro.updateFIFO(gyro);
	_px4_accel.updateFIFO(accel);
}

void ISM330DLC::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_interval_perf);
	perf_print_counter(_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_count_perf);
	perf_print_counter(_drdy_interval_perf);

}
