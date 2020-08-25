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

#include "PAW3902.hpp"

PAW3902::PAW3902(I2CSPIBusOption bus_option, int bus, int devid, enum Rotation yaw_rotation, int bus_frequency,
		 spi_mode_e spi_mode) :
	SPI("PAW3902", nullptr, bus, devid, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_sample_perf(perf_alloc(PC_ELAPSED, "paw3902: read")),
	_comms_errors(perf_alloc(PC_COUNT, "paw3902: com_err")),
	_dupe_count_perf(perf_alloc(PC_COUNT, "paw3902: duplicate reading")),
	_yaw_rotation(yaw_rotation)
{
}

PAW3902::~PAW3902()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_dupe_count_perf);
}

int
PAW3902::init()
{
	// get yaw rotation from sensor frame to body frame
	param_t rot = param_find("SENS_FLOW_ROT");

	if (rot != PARAM_INVALID) {
		int32_t val = 0;

		if (param_get(rot, &val) == PX4_OK) {
			_yaw_rotation = (enum Rotation)val;
		}
	}

	/* For devices competing with NuttX SPI drivers on a bus (Crazyflie SD Card expansion board) */
	SPI::set_lockmode(LOCK_THREADS);

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		return PX4_ERROR;
	}

	reset();

	// default to low light mode (1)
	modeLowLight();

	_previous_collect_timestamp = hrt_absolute_time();

	start();

	return PX4_OK;
}

int
PAW3902::probe()
{
	uint8_t product_id = registerRead(Register::Product_ID);

	PX4_INFO("DEVICE_ID: %X", product_id);

	// Test the SPI communication, checking chipId and inverse chipId
	if (product_id != PRODUCT_ID) {
		return PX4_ERROR;
	}

	uint8_t revision_id = registerRead(Register::Revision_ID);
	PX4_INFO("REVISION_ID: %X", revision_id);

	if (revision_id != REVISION_ID) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

bool
PAW3902::reset()
{
	// Power on reset
	registerWrite(Register::Power_Up_Reset, 0x5A);
	usleep(5000);

	// Read from registers 0x02, 0x03, 0x04, 0x05 and 0x06 one time regardless of the motion state
	registerRead(Register::Motion);
	registerRead(Register::Delta_X_L);
	registerRead(Register::Delta_X_H);
	registerRead(Register::Delta_Y_L);
	registerRead(Register::Delta_Y_H);

	usleep(1000);

	return true;
}

bool
PAW3902::changeMode(Mode newMode)
{
	if (newMode != _mode) {
		PX4_INFO("changing from mode %d -> %d", static_cast<int>(_mode), static_cast<int>(newMode));
		ScheduleClear();
		reset();

		switch (newMode) {
		case Mode::Bright:
			modeBright();
			ScheduleOnInterval(SAMPLE_INTERVAL_MODE_0);
			break;

		case Mode::LowLight:
			modeLowLight();
			ScheduleOnInterval(SAMPLE_INTERVAL_MODE_1);
			break;

		case Mode::SuperLowLight:
			modeSuperLowLight();
			ScheduleOnInterval(SAMPLE_INTERVAL_MODE_2);
			break;
		}

		_mode = newMode;
	}

	// Approximate Resolution = (Register Value + 1) * (50 / 8450) ≈ 0.6% of data point in Figure 19
	// The maximum register value is 0xA8. The minimum register value is 0.
	uint8_t resolution = registerRead(Register::Resolution);
	PX4_INFO("Resolution: %X", resolution);
	PX4_INFO("Resolution is approx: %.3f", (double)((resolution + 1.0f) * (50.0f / 8450.0f)));

	return true;
}

bool
PAW3902::modeBright()
{
	// Mode 0: Bright (126 fps) 60 Lux typical

	// set performance optimization registers
	registerWrite(0x7F, 0x00);
	registerWrite(0x55, 0x01);
	registerWrite(0x50, 0x07);
	registerWrite(0x7f, 0x0e);
	registerWrite(0x43, 0x10);

	registerWrite(0x48, 0x02);
	registerWrite(0x7F, 0x00);
	registerWrite(0x51, 0x7b);
	registerWrite(0x50, 0x00);
	registerWrite(0x55, 0x00);

	registerWrite(0x7F, 0x00);
	registerWrite(0x61, 0xAD);
	registerWrite(0x7F, 0x03);
	registerWrite(0x40, 0x00);
	registerWrite(0x7F, 0x05);
	registerWrite(0x41, 0xB3);
	registerWrite(0x43, 0xF1);
	registerWrite(0x45, 0x14);
	registerWrite(0x5F, 0x34);
	registerWrite(0x7B, 0x08);
	registerWrite(0x5e, 0x34);
	registerWrite(0x5b, 0x32);
	registerWrite(0x6d, 0x32);
	registerWrite(0x45, 0x17);
	registerWrite(0x70, 0xe5);
	registerWrite(0x71, 0xe5);
	registerWrite(0x7F, 0x06);
	registerWrite(0x44, 0x1B);
	registerWrite(0x40, 0xBF);
	registerWrite(0x4E, 0x3F);
	registerWrite(0x7F, 0x08);
	registerWrite(0x66, 0x44);
	registerWrite(0x65, 0x20);
	registerWrite(0x6a, 0x3a);
	registerWrite(0x61, 0x05);
	registerWrite(0x62, 0x05);
	registerWrite(0x7F, 0x09);
	registerWrite(0x4F, 0xAF);
	registerWrite(0x48, 0x80);
	registerWrite(0x49, 0x80);
	registerWrite(0x57, 0x77);
	registerWrite(0x5F, 0x40);
	registerWrite(0x60, 0x78);
	registerWrite(0x61, 0x78);
	registerWrite(0x62, 0x08);
	registerWrite(0x63, 0x50);
	registerWrite(0x7F, 0x0A);
	registerWrite(0x45, 0x60);
	registerWrite(0x7F, 0x00);
	registerWrite(0x4D, 0x11);
	registerWrite(0x55, 0x80);
	registerWrite(0x74, 0x21);
	registerWrite(0x75, 0x1F);
	registerWrite(0x4A, 0x78);
	registerWrite(0x4B, 0x78);
	registerWrite(0x44, 0x08);
	registerWrite(0x45, 0x50);
	registerWrite(0x64, 0xFE);
	registerWrite(0x65, 0x1F);
	registerWrite(0x72, 0x0A);
	registerWrite(0x73, 0x00);
	registerWrite(0x7F, 0x14);
	registerWrite(0x44, 0x84);
	registerWrite(0x65, 0x47);
	registerWrite(0x66, 0x18);
	registerWrite(0x63, 0x70);
	registerWrite(0x6f, 0x2c);
	registerWrite(0x7F, 0x15);
	registerWrite(0x48, 0x48);
	registerWrite(0x7F, 0x07);
	registerWrite(0x41, 0x0D);
	registerWrite(0x43, 0x14);
	registerWrite(0x4B, 0x0E);
	registerWrite(0x45, 0x0F);
	registerWrite(0x44, 0x42);
	registerWrite(0x4C, 0x80);
	registerWrite(0x7F, 0x10);
	registerWrite(0x5B, 0x03);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x41);

	usleep(10_ms); // delay 10ms

	registerWrite(0x7F, 0x00);
	registerWrite(0x32, 0x00);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x40);
	registerWrite(0x7F, 0x06);
	registerWrite(0x68, 0x70);
	registerWrite(0x69, 0x01);
	registerWrite(0x7F, 0x0D);
	registerWrite(0x48, 0xC0);
	registerWrite(0x6F, 0xD5);
	registerWrite(0x7F, 0x00);
	registerWrite(0x5B, 0xA0);
	registerWrite(0x4E, 0xA8);
	registerWrite(0x5A, 0x50);
	registerWrite(0x40, 0x80);
	registerWrite(0x73, 0x1f);

	usleep(10_ms); // delay 10ms

	registerWrite(0x73, 0x00);

	return false;
}

bool
PAW3902::modeLowLight()
{
	// Mode 1: Low Light (126 fps) 30 Lux typical
	// low light and low speed motion tracking

	// set performance optimization registers
	registerWrite(0x7F, 0x00);
	registerWrite(0x55, 0x01);
	registerWrite(0x50, 0x07);
	registerWrite(0x7f, 0x0e);
	registerWrite(0x43, 0x10);

	registerWrite(0x48, 0x02);
	registerWrite(0x7F, 0x00);
	registerWrite(0x51, 0x7b);
	registerWrite(0x50, 0x00);
	registerWrite(0x55, 0x00);

	registerWrite(0x7F, 0x00);
	registerWrite(0x61, 0xAD);
	registerWrite(0x7F, 0x03);
	registerWrite(0x40, 0x00);
	registerWrite(0x7F, 0x05);
	registerWrite(0x41, 0xB3);
	registerWrite(0x43, 0xF1);
	registerWrite(0x45, 0x14);
	registerWrite(0x5F, 0x34);
	registerWrite(0x7B, 0x08);
	registerWrite(0x5e, 0x34);
	registerWrite(0x5b, 0x65);
	registerWrite(0x6d, 0x65);
	registerWrite(0x45, 0x17);
	registerWrite(0x70, 0xe5);
	registerWrite(0x71, 0xe5);
	registerWrite(0x7F, 0x06);
	registerWrite(0x44, 0x1B);
	registerWrite(0x40, 0xBF);
	registerWrite(0x4E, 0x3F);
	registerWrite(0x7F, 0x08);
	registerWrite(0x66, 0x44);
	registerWrite(0x65, 0x20);
	registerWrite(0x6a, 0x3a);
	registerWrite(0x61, 0x05);
	registerWrite(0x62, 0x05);
	registerWrite(0x7F, 0x09);
	registerWrite(0x4F, 0xAF);
	registerWrite(0x48, 0x80);
	registerWrite(0x49, 0x80);
	registerWrite(0x57, 0x77);
	registerWrite(0x5F, 0x40);
	registerWrite(0x60, 0x78);
	registerWrite(0x61, 0x78);
	registerWrite(0x62, 0x08);
	registerWrite(0x63, 0x50);
	registerWrite(0x7F, 0x0A);
	registerWrite(0x45, 0x60);
	registerWrite(0x7F, 0x00);
	registerWrite(0x4D, 0x11);
	registerWrite(0x55, 0x80);
	registerWrite(0x74, 0x21);
	registerWrite(0x75, 0x1F);
	registerWrite(0x4A, 0x78);
	registerWrite(0x4B, 0x78);
	registerWrite(0x44, 0x08);
	registerWrite(0x45, 0x50);
	registerWrite(0x64, 0xFE);
	registerWrite(0x65, 0x1F);
	registerWrite(0x72, 0x0A);
	registerWrite(0x73, 0x00);
	registerWrite(0x7F, 0x14);
	registerWrite(0x44, 0x84);
	registerWrite(0x65, 0x67);
	registerWrite(0x66, 0x18);
	registerWrite(0x63, 0x70);
	registerWrite(0x6f, 0x2c);
	registerWrite(0x7F, 0x15);
	registerWrite(0x48, 0x48);
	registerWrite(0x7F, 0x07);
	registerWrite(0x41, 0x0D);
	registerWrite(0x43, 0x14);
	registerWrite(0x4B, 0x0E);
	registerWrite(0x45, 0x0F);
	registerWrite(0x44, 0x42);
	registerWrite(0x4C, 0x80);
	registerWrite(0x7F, 0x10);
	registerWrite(0x5B, 0x03);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x41);

	usleep(10_ms); // delay 10ms

	registerWrite(0x7F, 0x00);
	registerWrite(0x32, 0x00);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x40);
	registerWrite(0x7F, 0x06);
	registerWrite(0x68, 0x70);
	registerWrite(0x69, 0x01);
	registerWrite(0x7F, 0x0D);
	registerWrite(0x48, 0xC0);
	registerWrite(0x6F, 0xD5);
	registerWrite(0x7F, 0x00);
	registerWrite(0x5B, 0xA0);
	registerWrite(0x4E, 0xA8);
	registerWrite(0x5A, 0x50);
	registerWrite(0x40, 0x80);
	registerWrite(0x73, 0x1f);

	usleep(10_ms); // delay 10ms

	registerWrite(0x73, 0x00);

	return false;
}

bool
PAW3902::modeSuperLowLight()
{
	// Mode 2: Super Low Light (50 fps) 9 Lux typical
	// super low light and low speed motion tracking

	// set performance optimization registers
	registerWrite(0x7F, 0x00);
	registerWrite(0x55, 0x01);
	registerWrite(0x50, 0x07);
	registerWrite(0x7f, 0x0e);
	registerWrite(0x43, 0x10);

	registerWrite(0x48, 0x04);
	registerWrite(0x7F, 0x00);
	registerWrite(0x51, 0x7b);
	registerWrite(0x50, 0x00);
	registerWrite(0x55, 0x00);

	registerWrite(0x7F, 0x00);
	registerWrite(0x61, 0xAD);
	registerWrite(0x7F, 0x03);
	registerWrite(0x40, 0x00);
	registerWrite(0x7F, 0x05);
	registerWrite(0x41, 0xB3);
	registerWrite(0x43, 0xF1);
	registerWrite(0x45, 0x14);
	registerWrite(0x5F, 0x34);
	registerWrite(0x7B, 0x08);
	registerWrite(0x5E, 0x34);
	registerWrite(0x5B, 0x32);
	registerWrite(0x6D, 0x32);
	registerWrite(0x45, 0x17);
	registerWrite(0x70, 0xE5);
	registerWrite(0x71, 0xE5);
	registerWrite(0x7F, 0x06);
	registerWrite(0x44, 0x1B);
	registerWrite(0x40, 0xBF);
	registerWrite(0x4E, 0x3F);
	registerWrite(0x7F, 0x08);
	registerWrite(0x66, 0x44);
	registerWrite(0x65, 0x20);
	registerWrite(0x6A, 0x3a);
	registerWrite(0x61, 0x05);
	registerWrite(0x62, 0x05);
	registerWrite(0x7F, 0x09);
	registerWrite(0x4F, 0xAF);
	registerWrite(0x48, 0x80);
	registerWrite(0x49, 0x80);
	registerWrite(0x57, 0x77);
	registerWrite(0x5F, 0x40);
	registerWrite(0x60, 0x78);
	registerWrite(0x61, 0x78);
	registerWrite(0x62, 0x08);
	registerWrite(0x63, 0x50);
	registerWrite(0x7F, 0x0A);
	registerWrite(0x45, 0x60);
	registerWrite(0x7F, 0x00);
	registerWrite(0x4D, 0x11);
	registerWrite(0x55, 0x80);
	registerWrite(0x74, 0x21);
	registerWrite(0x75, 0x1F);
	registerWrite(0x4A, 0x78);
	registerWrite(0x4B, 0x78);
	registerWrite(0x44, 0x08);
	registerWrite(0x45, 0x50);
	registerWrite(0x64, 0xCE);
	registerWrite(0x65, 0x0B);
	registerWrite(0x72, 0x0A);
	registerWrite(0x73, 0x00);
	registerWrite(0x7F, 0x14);
	registerWrite(0x44, 0x84);
	registerWrite(0x65, 0x67);
	registerWrite(0x66, 0x18);
	registerWrite(0x63, 0x70);
	registerWrite(0x6f, 0x2c);
	registerWrite(0x7F, 0x15);
	registerWrite(0x48, 0x48);
	registerWrite(0x7F, 0x07);
	registerWrite(0x41, 0x0D);
	registerWrite(0x43, 0x14);
	registerWrite(0x4B, 0x0E);
	registerWrite(0x45, 0x0F);
	registerWrite(0x44, 0x42);
	registerWrite(0x4C, 0x80);
	registerWrite(0x7F, 0x10);
	registerWrite(0x5B, 0x02);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x41);

	usleep(25_ms); // delay 25ms

	registerWrite(0x7F, 0x00);
	registerWrite(0x32, 0x44);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x40);
	registerWrite(0x7F, 0x06);
	registerWrite(0x68, 0x40);
	registerWrite(0x69, 0x02);
	registerWrite(0x7F, 0x0D);
	registerWrite(0x48, 0xC0);
	registerWrite(0x6F, 0xD5);
	registerWrite(0x7F, 0x00);
	registerWrite(0x5B, 0xA0);
	registerWrite(0x4E, 0xA8);
	registerWrite(0x5A, 0x50);
	registerWrite(0x40, 0x80);
	registerWrite(0x73, 0x0B);

	usleep(25_ms); // delay 25ms

	registerWrite(0x73, 0x00);

	return true;
}

uint8_t
PAW3902::registerRead(uint8_t reg)
{
	uint8_t cmd[2] {};
	cmd[0] = reg;
	transfer(&cmd[0], &cmd[0], sizeof(cmd));
	up_udelay(T_SRR);
	return cmd[1];
}

void
PAW3902::registerWrite(uint8_t reg, uint8_t data)
{
	uint8_t cmd[2];
	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;
	transfer(&cmd[0], nullptr, sizeof(cmd));
	up_udelay(T_SWW);
}

void
PAW3902::RunImpl()
{
	perf_begin(_sample_perf);

	struct TransferBuffer {
		uint8_t cmd = Register::Motion_Burst;
		BURST_TRANSFER data{};
	};
	TransferBuffer buf;
	static_assert(sizeof(buf) == (12 + 1));

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (transfer((uint8_t *)&buf, (uint8_t *)&buf, sizeof(buf)) != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return;
	}

	const uint64_t dt_flow = timestamp_sample - _previous_collect_timestamp;
	_flow_dt_sum_usec += dt_flow;
	_frame_count_since_last++;

	// update for next iteration
	_previous_collect_timestamp = timestamp_sample;

	// PX4_INFO("data.Motion %d", buf.data.Motion);
	// PX4_INFO("data.Observation %d", buf.data.Observation);
	// PX4_INFO("data.Delta_X_L %d", buf.data.Delta_X_L);
	// PX4_INFO("data.Delta_X_H %d", buf.data.Delta_X_H);
	// PX4_INFO("data.Delta_Y_L %d", buf.data.Delta_Y_L);
	// PX4_INFO("data.Delta_Y_H %d", buf.data.Delta_Y_H);
	// PX4_INFO("data.SQUAL %d", buf.data.SQUAL);
	// PX4_INFO("data.RawData_Sum %d", buf.data.RawData_Sum);
	// PX4_INFO("data.Maximum_RawData %d", buf.data.Maximum_RawData);
	// PX4_INFO("data.Minimum_RawData %d", buf.data.Minimum_RawData);
	// PX4_INFO("data.Shutter_Upper %d", buf.data.Shutter_Upper);
	// PX4_INFO("data.Shutter_Lower %d", buf.data.Shutter_Lower);

	const int16_t delta_x_raw = ((int16_t)buf.data.Delta_X_H << 8) | buf.data.Delta_X_L;
	const int16_t delta_y_raw = ((int16_t)buf.data.Delta_Y_H << 8) | buf.data.Delta_Y_L;

	// check SQUAL & Shutter values
	// To suppress false motion reports, discard Delta X and Delta Y values if the SQUAL and Shutter values meet the condition
	// Bright Mode,			SQUAL < 0x19, Shutter ≥ 0x1FF0
	// Low Light Mode,		SQUAL < 0x46, Shutter ≥ 0x1FF0
	// Super Low Light Mode,	SQUAL < 0x55, Shutter ≥ 0x0BC0
	const uint16_t shutter = (buf.data.Shutter_Upper << 8) | buf.data.Shutter_Lower;

	if ((buf.data.SQUAL < 0x19) && (shutter >= 0x0BC0)) {
		PX4_ERR("false motion report, discarding");
		perf_end(_sample_perf);
		return;
	}

	switch (_mode) {
	case Mode::Bright:
		if ((shutter >= 0x1FFE) && (buf.data.RawData_Sum < 0x3C)) { // AND valid for 10 consecutive frames?
			// Bright -> LowLight
			changeMode(Mode::LowLight);
		}

		break;

	case Mode::LowLight:
		if ((shutter >= 0x1FFE) && (buf.data.RawData_Sum < 0x5A)) {	// AND valid for 10 consecutive frames?
			// LowLight -> SuperLowLight
			changeMode(Mode::SuperLowLight);

		} else if ((shutter < 0x0BB8)) {	// AND valid for 10 consecutive frames?
			// LowLight -> Bright
			changeMode(Mode::Bright);
		}

		break;

	case Mode::SuperLowLight:

		// SuperLowLight -> LowLight
		if ((shutter < 0x03E8)) { // AND valid for 10 consecutive frames?
			changeMode(Mode::LowLight);
		}

		// PAW3902JF should not operate with Shutter < 0x01F4 in Mode 2
		if (shutter >= 0x01F4) {
			changeMode(Mode::LowLight);
		}

		break;
	}

	// TODO: page 35 switching scheme

	// As a minimum requirement, PAW3902JF should not operate with Shutter < 0x01F4 in Mode 2, and must switch to the next operation mode.

	_flow_sum_x += delta_x_raw;
	_flow_sum_y += delta_y_raw;

	// returns if the collect time has not been reached
	if (_flow_dt_sum_usec < _collect_time) {
		perf_end(_sample_perf);
		return;
	}

	optical_flow_s report{};
	report.timestamp = timestamp_sample;


	//PX4_INFO("X: %d Y: %d", _flow_sum_x, _flow_sum_y);

	report.pixel_flow_x_integral = (float)_flow_sum_x / 500.0f;	// proportional factor + convert from pixels to radians
	report.pixel_flow_y_integral = (float)_flow_sum_y / 500.0f;	// proportional factor + convert from pixels to radians

	// rotate measurements in yaw from sensor frame to body frame according to parameter SENS_FLOW_ROT
	float zeroval = 0.0f;
	rotate_3f(_yaw_rotation, report.pixel_flow_x_integral, report.pixel_flow_y_integral, zeroval);

	report.frame_count_since_last_readout = _frame_count_since_last;
	report.integration_timespan = _flow_dt_sum_usec;	// microseconds

	report.sensor_id = 0;
	report.quality = buf.data.SQUAL;

	/* No gyro on this board */
	report.gyro_x_rate_integral = NAN;
	report.gyro_y_rate_integral = NAN;
	report.gyro_z_rate_integral = NAN;

	// set (conservative) specs according to datasheet
	report.max_flow_rate = 5.0f;       // Datasheet: 7.4 rad/s
	report.min_ground_distance = 0.08f; // Datasheet: 80mm
	report.max_ground_distance = 30.0f; // Datasheet: infinity

	_optical_flow_pub.publish(report);

	// reset
	_flow_dt_sum_usec = 0;
	_flow_sum_x = 0;
	_flow_sum_y = 0;
	_frame_count_since_last = 0;

	perf_end(_sample_perf);
}

void
PAW3902::start()
{
	// schedule a cycle to start things
	ScheduleOnInterval(SAMPLE_INTERVAL_MODE_1);
}

void
PAW3902::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_dupe_count_perf);
}
