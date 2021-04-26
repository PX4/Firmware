/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include "PAW3902.hpp"

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

PAW3902::PAW3902(I2CSPIBusOption bus_option, int bus, int devid, int bus_frequency, spi_mode_e spi_mode,
		 spi_drdy_gpio_t drdy_gpio, float yaw_rotation_degrees) :
	SPI(DRV_FLOW_DEVTYPE_PAW3902, MODULE_NAME, bus, devid, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_drdy_gpio(drdy_gpio)
{
	if (PX4_ISFINITE(yaw_rotation_degrees)) {
		PX4_INFO("using yaw rotation %.3f degrees (%.3f radians)",
			 (double)yaw_rotation_degrees, (double)math::radians(yaw_rotation_degrees));

		_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, math::radians(yaw_rotation_degrees)}};

	} else {
		// otherwise use the parameter SENS_FLOW_ROT
		param_t rot = param_find("SENS_FLOW_ROT");
		int32_t val = 0;

		if (param_get(rot, &val) == PX4_OK) {
			_rotation = get_rot_matrix((enum Rotation)val);

		} else {
			_rotation.identity();
		}
	}
}

PAW3902::~PAW3902()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_interval_perf);
	perf_free(_comms_errors);
	perf_free(_false_motion_perf);
	perf_free(_register_write_fail_perf);
	perf_free(_mode_change_bright_perf);
	perf_free(_mode_change_low_light_perf);
	perf_free(_mode_change_super_low_light_perf);
}

int PAW3902::init()
{
	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		return PX4_ERROR;
	}

	Reset();

	// force to low light mode (1) initially
	ChangeMode(Mode::LowLight, true);

	_previous_collect_timestamp = hrt_absolute_time();

	return PX4_OK;
}

int PAW3902::probe()
{
	const uint8_t Product_ID = RegisterRead(Register::Product_ID);

	if (Product_ID != PRODUCT_ID) {
		PX4_ERR("Product_ID: %X", Product_ID);
		return PX4_ERROR;
	}

	const uint8_t Revision_ID = RegisterRead(Register::Revision_ID);

	if (Revision_ID != REVISION_ID) {
		PX4_ERR("Revision_ID: %X", Revision_ID);
		return PX4_ERROR;
	}

	const uint8_t Inverse_Product_ID = RegisterRead(Register::Inverse_Product_ID);

	if (Inverse_Product_ID != PRODUCT_ID_INVERSE) {
		PX4_ERR("Inverse_Product_ID: %X", Inverse_Product_ID);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int PAW3902::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<PAW3902 *>(arg)->DataReady();
	return 0;
}

void PAW3902::DataReady()
{
	ScheduleNow();
}

bool PAW3902::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool PAW3902::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool PAW3902::Reset()
{
	DataReadyInterruptDisable();

	// Power on reset
	RegisterWrite(Register::Power_Up_Reset, 0x5A);
	usleep(1000);

	// Read from registers 0x02, 0x03, 0x04, 0x05 and 0x06 one time regardless of the motion state
	RegisterRead(Register::Motion);
	RegisterRead(Register::Delta_X_L);
	RegisterRead(Register::Delta_X_H);
	RegisterRead(Register::Delta_Y_L);
	RegisterRead(Register::Delta_Y_H);

	return true;
}

void PAW3902::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

bool PAW3902::ChangeMode(Mode newMode, bool force)
{
	if (newMode != _mode || force) {
		PX4_DEBUG("changing from mode %d -> %d", static_cast<int>(_mode), static_cast<int>(newMode));
		DataReadyInterruptDisable();
		ScheduleClear();

		// Issue a soft reset
		RegisterWrite(Register::Power_Up_Reset, 0x5A);

		uint32_t interval_us = 0;

		switch (newMode) {
		case Mode::Bright:
			ModeBright();
			interval_us = SAMPLE_INTERVAL_MODE_0;
			perf_count(_mode_change_bright_perf);
			break;

		case Mode::LowLight:
			ModeLowLight();
			interval_us = SAMPLE_INTERVAL_MODE_1;
			perf_count(_mode_change_low_light_perf);
			break;

		case Mode::SuperLowLight:
			ModeSuperLowLight();
			interval_us = SAMPLE_INTERVAL_MODE_2;
			perf_count(_mode_change_super_low_light_perf);
			break;
		}

		if (DataReadyInterruptConfigure()) {
			// backup schedule as a watchdog timeout
			ScheduleDelayed(500_ms);

		} else {
			ScheduleOnInterval(interval_us);
		}

		// Discard the first three motion data.
		for (int i = 0; i < 3; i++) {
			RegisterRead(Register::Motion);
			RegisterRead(Register::Delta_X_L);
			RegisterRead(Register::Delta_X_H);
			RegisterRead(Register::Delta_Y_L);
			RegisterRead(Register::Delta_Y_H);
		}

		EnableLed();

		_mode = newMode;
	}

	_bright_to_low_counter = 0;
	_low_to_superlow_counter = 0;
	_low_to_bright_counter = 0;
	_superlow_to_low_counter = 0;
	// Approximate Resolution = (Register Value + 1) * (50 / 8450) ≈ 0.6% of data point in Figure 19
	// The maximum register value is 0xA8. The minimum register value is 0.
	uint8_t resolution = RegisterRead(Register::Resolution);
	PX4_DEBUG("Resolution: %X", resolution);
	PX4_DEBUG("Resolution is approx: %.3f", (double)((resolution + 1.0f) * (50.0f / 8450.0f)));

	return true;
}

void PAW3902::ModeBright()
{
	// Mode 0: Bright (126 fps) 60 Lux typical

	// set performance optimization registers
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x55, 0x01);
	RegisterWriteVerified(0x50, 0x07);
	RegisterWriteVerified(0x7f, 0x0e);
	RegisterWriteVerified(0x43, 0x10);

	RegisterWriteVerified(0x48, 0x02);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x51, 0x7b);
	RegisterWriteVerified(0x50, 0x00);
	RegisterWriteVerified(0x55, 0x00);

	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x61, 0xAD);
	RegisterWriteVerified(0x7F, 0x03);
	RegisterWriteVerified(0x40, 0x00);
	RegisterWriteVerified(0x7F, 0x05);
	RegisterWriteVerified(0x41, 0xB3);
	RegisterWriteVerified(0x43, 0xF1);
	RegisterWriteVerified(0x45, 0x14);
	RegisterWriteVerified(0x5F, 0x34);
	RegisterWriteVerified(0x7B, 0x08);
	RegisterWriteVerified(0x5e, 0x34);
	RegisterWriteVerified(0x5b, 0x32);
	RegisterWriteVerified(0x6d, 0x32);
	RegisterWriteVerified(0x45, 0x17);
	RegisterWriteVerified(0x70, 0xe5);
	RegisterWriteVerified(0x71, 0xe5);
	RegisterWriteVerified(0x7F, 0x06);
	RegisterWriteVerified(0x44, 0x1B);
	RegisterWriteVerified(0x40, 0xBF);
	RegisterWriteVerified(0x4E, 0x3F);
	RegisterWriteVerified(0x7F, 0x08);
	RegisterWriteVerified(0x66, 0x44);
	RegisterWriteVerified(0x65, 0x20);
	RegisterWriteVerified(0x6a, 0x3a);
	RegisterWriteVerified(0x61, 0x05);
	RegisterWriteVerified(0x62, 0x05);
	RegisterWriteVerified(0x7F, 0x09);
	RegisterWriteVerified(0x4F, 0xAF);
	RegisterWriteVerified(0x48, 0x80);
	RegisterWriteVerified(0x49, 0x80);
	RegisterWriteVerified(0x57, 0x77);
	RegisterWriteVerified(0x5F, 0x40);
	RegisterWriteVerified(0x60, 0x78);
	RegisterWriteVerified(0x61, 0x78);
	RegisterWriteVerified(0x62, 0x08);
	RegisterWriteVerified(0x63, 0x50);
	RegisterWriteVerified(0x7F, 0x0A);
	RegisterWriteVerified(0x45, 0x60);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x4D, 0x11);
	RegisterWriteVerified(0x55, 0x80);
	RegisterWriteVerified(0x74, 0x21);
	RegisterWriteVerified(0x75, 0x1F);
	RegisterWriteVerified(0x4A, 0x78);
	RegisterWriteVerified(0x4B, 0x78);
	RegisterWriteVerified(0x44, 0x08);
	RegisterWriteVerified(0x45, 0x50);
	RegisterWriteVerified(0x64, 0xFE);
	RegisterWriteVerified(0x65, 0x1F);
	RegisterWriteVerified(0x72, 0x0A);
	RegisterWriteVerified(0x73, 0x00);
	RegisterWriteVerified(0x7F, 0x14);
	RegisterWriteVerified(0x44, 0x84);
	RegisterWriteVerified(0x65, 0x47);
	RegisterWriteVerified(0x66, 0x18);
	RegisterWriteVerified(0x63, 0x70);
	RegisterWriteVerified(0x6f, 0x2c);
	RegisterWriteVerified(0x7F, 0x15);
	RegisterWriteVerified(0x48, 0x48);
	RegisterWriteVerified(0x7F, 0x07);
	RegisterWriteVerified(0x41, 0x0D);
	RegisterWriteVerified(0x43, 0x14);
	RegisterWriteVerified(0x4B, 0x0E);
	RegisterWriteVerified(0x45, 0x0F);
	RegisterWriteVerified(0x44, 0x42);
	RegisterWriteVerified(0x4C, 0x80);
	RegisterWriteVerified(0x7F, 0x10);
	RegisterWriteVerified(0x5B, 0x03);
	RegisterWriteVerified(0x7F, 0x07);
	RegisterWriteVerified(0x40, 0x41);

	usleep(10_ms); // delay 10ms

	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x32, 0x00);
	RegisterWriteVerified(0x7F, 0x07);
	RegisterWriteVerified(0x40, 0x40);
	RegisterWriteVerified(0x7F, 0x06);
	RegisterWriteVerified(0x68, 0x70);
	RegisterWriteVerified(0x69, 0x01);
	RegisterWriteVerified(0x7F, 0x0D);
	RegisterWriteVerified(0x48, 0xC0);
	RegisterWriteVerified(0x6F, 0xD5);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x5B, 0xA0);
	RegisterWriteVerified(0x4E, 0xA8);
	RegisterWriteVerified(0x5A, 0x50);
	RegisterWriteVerified(0x40, 0x80);
	RegisterWriteVerified(0x73, 0x1f);

	usleep(10_ms); // delay 10ms

	RegisterWrite(0x73, 0x00);
}

void PAW3902::ModeLowLight()
{
	// Mode 1: Low Light (126 fps) 30 Lux typical
	// low light and low speed motion tracking

	// set performance optimization registers
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x55, 0x01);
	RegisterWriteVerified(0x50, 0x07);
	RegisterWriteVerified(0x7f, 0x0e);
	RegisterWriteVerified(0x43, 0x10);

	RegisterWriteVerified(0x48, 0x02);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x51, 0x7b);
	RegisterWriteVerified(0x50, 0x00);
	RegisterWriteVerified(0x55, 0x00);

	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x61, 0xAD);
	RegisterWriteVerified(0x7F, 0x03);
	RegisterWriteVerified(0x40, 0x00);
	RegisterWriteVerified(0x7F, 0x05);
	RegisterWriteVerified(0x41, 0xB3);
	RegisterWriteVerified(0x43, 0xF1);
	RegisterWriteVerified(0x45, 0x14);
	RegisterWriteVerified(0x5F, 0x34);
	RegisterWriteVerified(0x7B, 0x08);
	RegisterWriteVerified(0x5e, 0x34);
	RegisterWriteVerified(0x5b, 0x65);
	RegisterWriteVerified(0x6d, 0x65);
	RegisterWriteVerified(0x45, 0x17);
	RegisterWriteVerified(0x70, 0xe5);
	RegisterWriteVerified(0x71, 0xe5);
	RegisterWriteVerified(0x7F, 0x06);
	RegisterWriteVerified(0x44, 0x1B);
	RegisterWriteVerified(0x40, 0xBF);
	RegisterWriteVerified(0x4E, 0x3F);
	RegisterWriteVerified(0x7F, 0x08);
	RegisterWriteVerified(0x66, 0x44);
	RegisterWriteVerified(0x65, 0x20);
	RegisterWriteVerified(0x6a, 0x3a);
	RegisterWriteVerified(0x61, 0x05);
	RegisterWriteVerified(0x62, 0x05);
	RegisterWriteVerified(0x7F, 0x09);
	RegisterWriteVerified(0x4F, 0xAF);
	RegisterWriteVerified(0x48, 0x80);
	RegisterWriteVerified(0x49, 0x80);
	RegisterWriteVerified(0x57, 0x77);
	RegisterWriteVerified(0x5F, 0x40);
	RegisterWriteVerified(0x60, 0x78);
	RegisterWriteVerified(0x61, 0x78);
	RegisterWriteVerified(0x62, 0x08);
	RegisterWriteVerified(0x63, 0x50);
	RegisterWriteVerified(0x7F, 0x0A);
	RegisterWriteVerified(0x45, 0x60);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x4D, 0x11);
	RegisterWriteVerified(0x55, 0x80);
	RegisterWriteVerified(0x74, 0x21);
	RegisterWriteVerified(0x75, 0x1F);
	RegisterWriteVerified(0x4A, 0x78);
	RegisterWriteVerified(0x4B, 0x78);
	RegisterWriteVerified(0x44, 0x08);
	RegisterWriteVerified(0x45, 0x50);
	RegisterWriteVerified(0x64, 0xFE);
	RegisterWriteVerified(0x65, 0x1F);
	RegisterWriteVerified(0x72, 0x0A);
	RegisterWriteVerified(0x73, 0x00);
	RegisterWriteVerified(0x7F, 0x14);
	RegisterWriteVerified(0x44, 0x84);
	RegisterWriteVerified(0x65, 0x67);
	RegisterWriteVerified(0x66, 0x18);
	RegisterWriteVerified(0x63, 0x70);
	RegisterWriteVerified(0x6f, 0x2c);
	RegisterWriteVerified(0x7F, 0x15);
	RegisterWriteVerified(0x48, 0x48);
	RegisterWriteVerified(0x7F, 0x07);
	RegisterWriteVerified(0x41, 0x0D);
	RegisterWriteVerified(0x43, 0x14);
	RegisterWriteVerified(0x4B, 0x0E);
	RegisterWriteVerified(0x45, 0x0F);
	RegisterWriteVerified(0x44, 0x42);
	RegisterWriteVerified(0x4C, 0x80);
	RegisterWriteVerified(0x7F, 0x10);
	RegisterWriteVerified(0x5B, 0x03);
	RegisterWriteVerified(0x7F, 0x07);
	RegisterWriteVerified(0x40, 0x41);

	usleep(10_ms); // delay 10ms

	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x32, 0x00);
	RegisterWriteVerified(0x7F, 0x07);
	RegisterWriteVerified(0x40, 0x40);
	RegisterWriteVerified(0x7F, 0x06);
	RegisterWriteVerified(0x68, 0x70);
	RegisterWriteVerified(0x69, 0x01);
	RegisterWriteVerified(0x7F, 0x0D);
	RegisterWriteVerified(0x48, 0xC0);
	RegisterWriteVerified(0x6F, 0xD5);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x5B, 0xA0);
	RegisterWriteVerified(0x4E, 0xA8);
	RegisterWriteVerified(0x5A, 0x50);
	RegisterWriteVerified(0x40, 0x80);
	RegisterWriteVerified(0x73, 0x1f);

	usleep(10_ms); // delay 10ms

	RegisterWrite(0x73, 0x00);
}

void PAW3902::ModeSuperLowLight()
{
	// Mode 2: Super Low Light (50 fps) 9 Lux typical
	// super low light and low speed motion tracking

	// set performance optimization registers
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x55, 0x01);
	RegisterWriteVerified(0x50, 0x07);
	RegisterWriteVerified(0x7f, 0x0e);
	RegisterWriteVerified(0x43, 0x10);

	RegisterWriteVerified(0x48, 0x04);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x51, 0x7b);
	RegisterWriteVerified(0x50, 0x00);
	RegisterWriteVerified(0x55, 0x00);

	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x61, 0xAD);
	RegisterWriteVerified(0x7F, 0x03);
	RegisterWriteVerified(0x40, 0x00);
	RegisterWriteVerified(0x7F, 0x05);
	RegisterWriteVerified(0x41, 0xB3);
	RegisterWriteVerified(0x43, 0xF1);
	RegisterWriteVerified(0x45, 0x14);
	RegisterWriteVerified(0x5F, 0x34);
	RegisterWriteVerified(0x7B, 0x08);
	RegisterWriteVerified(0x5E, 0x34);
	RegisterWriteVerified(0x5B, 0x32);
	RegisterWriteVerified(0x6D, 0x32);
	RegisterWriteVerified(0x45, 0x17);
	RegisterWriteVerified(0x70, 0xE5);
	RegisterWriteVerified(0x71, 0xE5);
	RegisterWriteVerified(0x7F, 0x06);
	RegisterWriteVerified(0x44, 0x1B);
	RegisterWriteVerified(0x40, 0xBF);
	RegisterWriteVerified(0x4E, 0x3F);
	RegisterWriteVerified(0x7F, 0x08);
	RegisterWriteVerified(0x66, 0x44);
	RegisterWriteVerified(0x65, 0x20);
	RegisterWriteVerified(0x6A, 0x3a);
	RegisterWriteVerified(0x61, 0x05);
	RegisterWriteVerified(0x62, 0x05);
	RegisterWriteVerified(0x7F, 0x09);
	RegisterWriteVerified(0x4F, 0xAF);
	RegisterWriteVerified(0x48, 0x80);
	RegisterWriteVerified(0x49, 0x80);
	RegisterWriteVerified(0x57, 0x77);
	RegisterWriteVerified(0x5F, 0x40);
	RegisterWriteVerified(0x60, 0x78);
	RegisterWriteVerified(0x61, 0x78);
	RegisterWriteVerified(0x62, 0x08);
	RegisterWriteVerified(0x63, 0x50);
	RegisterWriteVerified(0x7F, 0x0A);
	RegisterWriteVerified(0x45, 0x60);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x4D, 0x11);
	RegisterWriteVerified(0x55, 0x80);
	RegisterWriteVerified(0x74, 0x21);
	RegisterWriteVerified(0x75, 0x1F);
	RegisterWriteVerified(0x4A, 0x78);
	RegisterWriteVerified(0x4B, 0x78);
	RegisterWriteVerified(0x44, 0x08);
	RegisterWriteVerified(0x45, 0x50);
	RegisterWriteVerified(0x64, 0xCE);
	RegisterWriteVerified(0x65, 0x0B);
	RegisterWriteVerified(0x72, 0x0A);
	RegisterWriteVerified(0x73, 0x00);
	RegisterWriteVerified(0x7F, 0x14);
	RegisterWriteVerified(0x44, 0x84);
	RegisterWriteVerified(0x65, 0x67);
	RegisterWriteVerified(0x66, 0x18);
	RegisterWriteVerified(0x63, 0x70);
	RegisterWriteVerified(0x6f, 0x2c);
	RegisterWriteVerified(0x7F, 0x15);
	RegisterWriteVerified(0x48, 0x48);
	RegisterWriteVerified(0x7F, 0x07);
	RegisterWriteVerified(0x41, 0x0D);
	RegisterWriteVerified(0x43, 0x14);
	RegisterWriteVerified(0x4B, 0x0E);
	RegisterWriteVerified(0x45, 0x0F);
	RegisterWriteVerified(0x44, 0x42);
	RegisterWriteVerified(0x4C, 0x80);
	RegisterWriteVerified(0x7F, 0x10);
	RegisterWriteVerified(0x5B, 0x02);
	RegisterWriteVerified(0x7F, 0x07);
	RegisterWriteVerified(0x40, 0x41);

	usleep(25_ms); // delay 25ms

	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x32, 0x44);
	RegisterWriteVerified(0x7F, 0x07);
	RegisterWriteVerified(0x40, 0x40);
	RegisterWriteVerified(0x7F, 0x06);
	RegisterWriteVerified(0x68, 0x40);
	RegisterWriteVerified(0x69, 0x02);
	RegisterWriteVerified(0x7F, 0x0D);
	RegisterWriteVerified(0x48, 0xC0);
	RegisterWriteVerified(0x6F, 0xD5);
	RegisterWriteVerified(0x7F, 0x00);
	RegisterWriteVerified(0x5B, 0xA0);
	RegisterWriteVerified(0x4E, 0xA8);
	RegisterWriteVerified(0x5A, 0x50);
	RegisterWriteVerified(0x40, 0x80);
	RegisterWriteVerified(0x73, 0x0B);

	usleep(25_ms); // delay 25ms

	RegisterWrite(0x73, 0x00);
}

void PAW3902::EnableLed()
{
	// Enable LED_N controls
	RegisterWriteVerified(0x7F, 0x14);
	RegisterWriteVerified(0x6F, 0x1c);
	RegisterWriteVerified(0x7F, 0x00);
}

uint8_t PAW3902::RegisterRead(uint8_t reg, int retries)
{
	for (int i = 0; i < retries; i++) {
		px4_udelay(TIME_us_TSRAD);
		uint8_t cmd[2] {reg, 0};

		if (transfer(&cmd[0], &cmd[0], sizeof(cmd)) == 0) {
			return cmd[1];
		}
	}

	perf_count(_comms_errors);
	return 0;
}

void PAW3902::RegisterWrite(uint8_t reg, uint8_t data)
{
	uint8_t cmd[2];
	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	if (transfer(&cmd[0], nullptr, sizeof(cmd)) != 0) {
		perf_count(_comms_errors);
	}
}

bool PAW3902::RegisterWriteVerified(uint8_t reg, uint8_t data, int retries)
{
	for (int i = 0; i < retries; i++) {
		uint8_t cmd[2];
		cmd[0] = DIR_WRITE(reg);
		cmd[1] = data;
		transfer(&cmd[0], nullptr, sizeof(cmd));
		px4_udelay(TIME_us_TSWW);

		// read back to verify
		uint8_t data_read = RegisterRead(reg);

		if (data_read == data) {
			return true;
		}

		PX4_DEBUG("Register write failed 0x%02hhX: 0x%02hhX (actual value 0x%02hhX)", reg, data, data_read);
	}

	perf_count(_register_write_fail_perf);

	return false;
}

void PAW3902::RunImpl()
{
	perf_begin(_sample_perf);
	perf_count(_interval_perf);

	struct TransferBuffer {
		uint8_t cmd = Register::Motion_Burst;
		BURST_TRANSFER data{};
	} buf{};
	static_assert(sizeof(buf) == (12 + 1));

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (transfer((uint8_t *)&buf, (uint8_t *)&buf, sizeof(buf)) != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return;
	}

	perf_end(_sample_perf);

	const uint64_t dt_flow = timestamp_sample - _previous_collect_timestamp;

	// update for next iteration
	_previous_collect_timestamp = timestamp_sample;

	// check SQUAL & Shutter values
	// To suppress false motion reports, discard Delta X and Delta Y values if the SQUAL and Shutter values meet the condition
	// Bright Mode,			SQUAL < 0x19, Shutter ≥ 0x1FF0
	// Low Light Mode,		SQUAL < 0x46, Shutter ≥ 0x1FF0
	// Super Low Light Mode,	SQUAL < 0x55, Shutter ≥ 0x0BC0
	const uint16_t shutter = (buf.data.Shutter_Upper << 8) | buf.data.Shutter_Lower;

	bool data_valid = true;

	switch (_mode) {
	case Mode::Bright:
		if ((buf.data.SQUAL < 0x19) && (shutter >= 0x1FF0)) {
			// false motion report, discarding
			perf_count(_false_motion_perf);
			ResetAccumulatedData();
			data_valid = false;
		}

		if ((shutter >= 0x1FFE) && (buf.data.RawData_Sum < 0x3C)) {
			// Bright -> LowLight
			_bright_to_low_counter++;

			if (_bright_to_low_counter >= 10) { // AND valid for 10 consecutive frames
				ChangeMode(Mode::LowLight);
			}

		} else {
			_bright_to_low_counter = 0;
		}

		break;

	case Mode::LowLight:
		if ((buf.data.SQUAL < 0x46) && (shutter >= 0x1FF0)) {
			// false motion report, discarding
			perf_count(_false_motion_perf);
			ResetAccumulatedData();
			data_valid = false;
		}

		if ((shutter >= 0x1FFE) && (buf.data.RawData_Sum < 0x5A)) {
			// LowLight -> SuperLowLight
			_low_to_bright_counter = 0;
			_low_to_superlow_counter++;

			if (_low_to_superlow_counter >= 10) { // AND valid for 10 consecutive frames
				ChangeMode(Mode::SuperLowLight);
			}

		} else if (shutter < 0x0BB8) {
			// LowLight -> Bright
			_low_to_bright_counter++;
			_low_to_superlow_counter = 0;

			if (_low_to_bright_counter >= 10) { // AND valid for 10 consecutive frames
				ChangeMode(Mode::Bright);
			}

		} else {
			_low_to_bright_counter = 0;
			_low_to_superlow_counter = 0;
		}

		break;

	case Mode::SuperLowLight:
		if ((buf.data.SQUAL < 0x55) && (shutter >= 0x0BC0)) {
			// false motion report, discarding
			perf_count(_false_motion_perf);
			ResetAccumulatedData();
			data_valid = false;
		}

		if (shutter < 0x03E8) {
			// SuperLowLight -> LowLight
			_superlow_to_low_counter++;

			if (_superlow_to_low_counter >= 10) { // AND valid for 10 consecutive frames
				ChangeMode(Mode::LowLight);
			}

		} else {
			_superlow_to_low_counter = 0;
		}

		break;
	}

	if (data_valid && (buf.data.SQUAL > 0)) {
		const int16_t delta_x_raw = combine(buf.data.Delta_X_H, buf.data.Delta_X_L);
		const int16_t delta_y_raw = combine(buf.data.Delta_Y_H, buf.data.Delta_Y_L);

		_flow_dt_sum_usec += dt_flow;
		_flow_sum_x += delta_x_raw;
		_flow_sum_y += delta_y_raw;
		_flow_sample_counter++;
		_flow_quality_sum += buf.data.SQUAL;

	} else {
		ResetAccumulatedData();
		return;
	}

	// returns if the collect time has not been reached
	if (_flow_dt_sum_usec < COLLECT_TIME) {
		return;
	}

	optical_flow_s report{};
	report.timestamp = timestamp_sample;
	//report.device_id = get_device_id();

	float pixel_flow_x_integral = (float)_flow_sum_x / 500.0f;	// proportional factor + convert from pixels to radians
	float pixel_flow_y_integral = (float)_flow_sum_y / 500.0f;	// proportional factor + convert from pixels to radians

	// rotate measurements in yaw from sensor frame to body frame
	const matrix::Vector3f pixel_flow_rotated = _rotation * matrix::Vector3f{pixel_flow_x_integral, pixel_flow_y_integral, 0.f};
	report.pixel_flow_x_integral = pixel_flow_rotated(0);
	report.pixel_flow_y_integral = pixel_flow_rotated(1);

	report.frame_count_since_last_readout = _flow_sample_counter; // number of frames
	report.integration_timespan = _flow_dt_sum_usec;              // microseconds

	report.quality = _flow_sample_counter > 0 ? _flow_quality_sum / _flow_sample_counter : 0;

	// No gyro on this board
	report.gyro_x_rate_integral = NAN;
	report.gyro_y_rate_integral = NAN;
	report.gyro_z_rate_integral = NAN;

	// set (conservative) specs according to datasheet
	report.max_flow_rate = 7.4f;        // Datasheet: 7.4 rad/s
	report.min_ground_distance = 0.08f; // Datasheet: 80mm
	report.max_ground_distance = 30.0f; // Datasheet: infinity


	switch (_mode) {
	case Mode::Bright:
		report.mode = optical_flow_s::MODE_BRIGHT;
		break;

	case Mode::LowLight:
		report.mode = optical_flow_s::MODE_LOWLIGHT;
		break;

	case Mode::SuperLowLight:
		report.mode = optical_flow_s::MODE_SUPER_LOWLIGHT;
		break;
	}

	report.timestamp = hrt_absolute_time();
	_optical_flow_pub.publish(report);

	ResetAccumulatedData();
}

void PAW3902::ResetAccumulatedData()
{
	// reset
	_flow_dt_sum_usec = 0;
	_flow_sum_x = 0;
	_flow_sum_y = 0;
	_flow_sample_counter = 0;
	_flow_quality_sum = 0;
}

void PAW3902::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_false_motion_perf);
	perf_print_counter(_register_write_fail_perf);
	perf_print_counter(_mode_change_bright_perf);
	perf_print_counter(_mode_change_low_light_perf);
	perf_print_counter(_mode_change_super_low_light_perf);
}
