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

#include "BMI088_Accelerometer.hpp"

#include <ecl/geo/geo.h> // CONSTANTS_ONE_G

using namespace time_literals;

namespace Bosch::BMI088::Accelerometer
{
BMI088_Accelerometer::BMI088_Accelerometer(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation,
		int bus_frequency, spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	BMI088(DRV_ACC_DEVTYPE_BMI088, "BMI088_Accelerometer", bus_option, bus, device, spi_mode, bus_frequency, drdy_gpio),
	_px4_accel(get_device_id(), rotation)
{
	if (drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME"_accel: DRDY missed");
	}

	ConfigureSampleRate(1600);
}

BMI088_Accelerometer::~BMI088_Accelerometer()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

void BMI088_Accelerometer::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BMI088_Accelerometer::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

uint8_t BMI088_Accelerometer::RegisterRead(Register reg)
{
	uint8_t add = static_cast<uint8_t>(reg);
	uint8_t cmd[2] = {add, 0};
	transfer(&cmd[0], 1, &cmd[1], 1);
	return cmd[1];
}

uint8_t BMI088_Accelerometer::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t add = static_cast<uint8_t>(reg);
	uint8_t cmd[2] = { add, value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

int BMI088_Accelerometer::probe()
{
	const uint8_t ACC_CHIP_ID = RegisterRead(Register::ACC_CHIP_ID);

	if (ACC_CHIP_ID != ID) {
		DEVICE_DEBUG("unexpected ACC_CHIP_ID 0x%02x", ACC_CHIP_ID);
		return PX4_ERROR;
	}

	PX4_WARN("Probe success, ACC_CHIP_ID: 0x%02x", ACC_CHIP_ID);

	return PX4_OK;
}

void BMI088_Accelerometer::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::SELFTEST:
		//PX4_WARN("Selftest state");
		//SelfTest();
		_state = STATE::RESET;
		ScheduleDelayed(10_ms);
		break;

	case STATE::RESET:
		// ACC_SOFTRESET: Writing a value of 0xB6 to this register resets the sensor
		RegisterWrite(Register::ACC_SOFTRESET, 0xB6);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;


		ScheduleDelayed(1_ms); // Following a delay of 1 ms, all configuration settings are overwritten with their reset value.

		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::ACC_CHIP_ID) == ID) {
			// ACC_PWR_CONF: Power on sensor
			RegisterWrite(Register::ACC_PWR_CONF, 0);

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
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

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

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

	case STATE::FIFO_READ: {
			SimpleFIFORead(now);
		}
		break;
	}
}

void BMI088_Accelerometer::ConfigureAccel()
{
	//PX4_WARN("ConfigureAccel");
	const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE) & (Bit1 | Bit0);

	switch (ACC_RANGE) {
	case acc_range_3g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(3.f);
		break;

	case acc_range_6g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(6.f);
		break;

	case acc_range_12g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(12.f);
		break;

	case acc_range_24g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(24.f);
		break;
	}
}

void BMI088_Accelerometer::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	PX4_WARN("_fifo_empty_interval_us %d", _fifo_empty_interval_us);
	_fifo_samples = math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES);

	PX4_WARN("_fifo_samples %d", _fifo_samples);
	// recompute FIFO empty interval (us) with actual sample limit
	_fifo_empty_interval_us = _fifo_samples * (1e6f / RATE);

	PX4_WARN("_fifo_empty_interval_us %d", _fifo_empty_interval_us);
	//PX4_WARN("_fifo_samples %d", _fifo_samples);
	ConfigureFIFOWatermark(_fifo_samples);
}

void BMI088_Accelerometer::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO_WTM: 13 bit FIFO watermark level value
	// unit of the fifo watermark is one byte
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::DATA);

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_WTM_0) {
			// fifo_water_mark[7:0]
			r.set_bits = fifo_watermark_threshold & 0x00FF;
			r.clear_bits = ~r.set_bits;

		} else if (r.reg == Register::FIFO_WTM_1) {
			// fifo_water_mark[12:8]
			r.set_bits = (fifo_watermark_threshold & 0x0700) >> 8;
			r.clear_bits = ~r.set_bits;
		}
	}
}

bool BMI088_Accelerometer::Configure()
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

	ConfigureAccel();

	return success;
}

int BMI088_Accelerometer::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BMI088_Accelerometer *>(arg)->DataReady();
	return 0;
}

void BMI088_Accelerometer::DataReady()
{
	uint32_t expected = 0;

	if (_drdy_fifo_read_samples.compare_exchange(&expected, _fifo_samples)) {
		ScheduleNow();
	}
}

bool BMI088_Accelerometer::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool BMI088_Accelerometer::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool BMI088_Accelerometer::RegisterCheck(const register_config_t &reg_cfg)
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

void BMI088_Accelerometer::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t BMI088_Accelerometer::FIFOReadCount()
{
	// FIFO length registers FIFO_LENGTH_1 and FIFO_LENGTH_0 contain the 14 bit FIFO byte
	uint8_t fifo_len_buf[2] {};
	fifo_len_buf[0] = static_cast<uint8_t>(Register::FIFO_LENGTH_0) | DIR_READ;
	// fifo_len_buf[1] dummy byte

	if (transfer(&fifo_len_buf[0], 1, &fifo_len_buf[0], 2) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	const uint8_t FIFO_LENGTH_0 = fifo_len_buf[0];        // fifo_byte_counter[7:0]
	const uint8_t FIFO_LENGTH_1 = fifo_len_buf[1] & 0x3F; // fifo_byte_counter[13:8]

	return combine(FIFO_LENGTH_1, FIFO_LENGTH_0);
}

bool BMI088_Accelerometer::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 4, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, 1, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	//PX4_WARN("Accel transfer success");
	const size_t fifo_byte_counter = combine(buffer.FIFO_LENGTH_1 & 0x3F, buffer.FIFO_LENGTH_0);

	// An empty FIFO corresponds to 0x8000
	if (fifo_byte_counter == 0x8000) {
		perf_count(_fifo_empty_perf);
		return false;

	} else if (fifo_byte_counter >= FIFO::SIZE) {
		perf_count(_fifo_overflow_perf);
		return false;
	}

	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT;

	// first find all sensor data frames in the buffer
	uint8_t *data_buffer = (uint8_t *)&buffer.f[0];
	unsigned fifo_buffer_index = 0; // start of buffer

	while (fifo_buffer_index < math::min(fifo_byte_counter, transfer_size - 4)) {
		// look for header signature (first 6 bits) followed by two bits indicating the status of INT1 and INT2
		switch (data_buffer[fifo_buffer_index] & 0xFC) {
		case FIFO::header::sensor_data_frame: {
				// Acceleration sensor data frame
				// Frame length: 7 bytes (1 byte header + 6 bytes payload)

				FIFO::DATA *fifo_sample = (FIFO::DATA *)&data_buffer[fifo_buffer_index];
				const int16_t accel_x = combine(fifo_sample->ACC_X_MSB, fifo_sample->ACC_X_LSB);
				const int16_t accel_y = combine(fifo_sample->ACC_Y_MSB, fifo_sample->ACC_Y_LSB);
				const int16_t accel_z = combine(fifo_sample->ACC_Z_MSB, fifo_sample->ACC_Z_LSB);

				// sensor's frame is +x forward, +y left, +z up
				//  flip y & z to publish right handed with z down (x forward, y right, z down)
				accel.x[accel.samples] = accel_x;
				accel.y[accel.samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
				accel.z[accel.samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
				accel.samples++;

				fifo_buffer_index += 7; // move forward to next record
			}
			break;

		case FIFO::header::skip_frame:
			// Skip Frame
			// Frame length: 2 bytes (1 byte header + 1 byte payload)
			PX4_DEBUG("Skip Frame");
			fifo_buffer_index += 2;
			break;

		case FIFO::header::sensor_time_frame:
			// Sensortime Frame
			// Frame length: 4 bytes (1 byte header + 3 bytes payload)
			PX4_DEBUG("Sensortime Frame");
			fifo_buffer_index += 4;
			break;

		case FIFO::header::FIFO_input_config_frame:
			// FIFO input config Frame
			// Frame length: 2 bytes (1 byte header + 1 byte payload)
			PX4_DEBUG("FIFO input config Frame");
			fifo_buffer_index += 2;
			break;

		case FIFO::header::sample_drop_frame:
			// Sample drop Frame
			// Frame length: 2 bytes (1 byte header + 1 byte payload)
			PX4_DEBUG("Sample drop Frame");
			fifo_buffer_index += 2;
			break;

		default:
			fifo_buffer_index++;
			break;
		}
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
		return true;
	}

	return false;
}

bool BMI088_Accelerometer::SimpleFIFORead(const hrt_abstime &timestamp_sample)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT;

	int fifo_fill_level = 0;

	uint8_t data_o[2] = { 0, 0 };
	uint8_t data_i[1] = {static_cast<uint8_t>(Register::FIFO_LENGTH_0)};
	data_i[0] = static_cast<uint8_t>(Register::FIFO_LENGTH_0);

	transfer(&data_i[0], 1, &data_o[0], 2);
	fifo_fill_level = data_o[0] + (data_o[1] << 8);

	if (fifo_fill_level & 0x8000) {
		return false;
	}

	int n_frames_to_read = 6;

	// don't read more than 6 frames at a time
	if (fifo_fill_level > n_frames_to_read * 7) {
		fifo_fill_level = n_frames_to_read * 7;
	}

	if (fifo_fill_level == 0) {
		return false;
	}

	uint8_t data[fifo_fill_level];

	data[0] = static_cast<uint8_t>(Register::FIFO_DATA);

	if (transfer(&data[0], 1, &data[0], fifo_fill_level) != PX4_OK) {
		return false;
	}

	const uint8_t *p = &data[0];

	while (fifo_fill_level >= 7) {
		uint8_t frame_len = 2;

		switch (p[0] & 0xFC) {
		case 0x84: {
				// accel frame
				frame_len = 7;
				const uint8_t *d = p + 1;
				int16_t xyz[3] {
					int16_t(uint16_t(d[0] | (d[1] << 8))),
					int16_t(uint16_t(d[2] | (d[3] << 8))),
					int16_t(uint16_t(d[4] | (d[5] << 8)))
				};


				const int16_t tX[3] = {1, 0, 0};
				const int16_t tY[3] = {0, -1, 0};
				const int16_t tZ[3] = {0, 0, -1};

				float x = 0;
				float y = 0;
				float z = 0;

				x = xyz[0] * tX[0] + xyz[1] * tX[1] + xyz[2] * tX[2];
				y = xyz[0] * tY[0] + xyz[1] * tY[1] + xyz[2] * tY[2];
				z = xyz[0] * tZ[0] + xyz[1] * tZ[1] + xyz[2] * tZ[2];

				accel.x[accel.samples] = x;
				accel.y[accel.samples] = y;
				accel.z[accel.samples] = z;
				accel.samples++;

				break;
			}

		case 0x40:
			// skip frame
			frame_len = 2;
			break;

		case 0x44:
			// sensortime frame
			frame_len = 4;
			break;

		case 0x48:
			// fifo config frame
			frame_len = 2;
			break;

		case 0x50:
			// sample drop frame
			frame_len = 2;
			break;
		}

		p += frame_len;
		fifo_fill_level -= frame_len;
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		//PX4_WARN("accel.samples: %d", accel.samples);
		_px4_accel.updateFIFO(accel);
		return true;
	}

	return true;
}

void BMI088_Accelerometer::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// ACC_SOFTRESET: trigger a FIFO reset by writing 0xB0 to ACC_SOFTRESET (register 0x7E).
	RegisterWrite(Register::ACC_SOFTRESET, 0xB0);

	// reset while FIFO is disabled
	_drdy_fifo_read_samples.store(0);
}

void BMI088_Accelerometer::UpdateTemperature()
{
	// stored in an 11-bit value in 2’s complement format
	uint8_t temperature_buf[4] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_MSB) | ACC_I2C_ADDR_PRIMARY;
	// temperature_buf[1] dummy byte

	if (transfer(&temperature_buf[0], 1, &temperature_buf[0], sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const uint8_t TEMP_MSB = temperature_buf[2];
	const uint8_t TEMP_LSB = temperature_buf[3];

	// Datasheet 5.3.7: Register 0x22 – 0x23: Temperature sensor data
	uint16_t Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32);
	int16_t Temp_int11 = 0;

	if (Temp_uint11 > 1023) {
		Temp_int11 = Temp_uint11 - 2048;

	} else {
		Temp_int11 = Temp_uint11;
	}

	float temperature = (Temp_int11 * 0.125f) + 23.f; // Temp_int11 * 0.125°C/LSB + 23°C

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);

	} else {
		perf_count(_bad_transfer_perf);
	}
}

bool BMI088_Accelerometer::SelfTest()
{
	PX4_WARN("Running self-test with datasheet recomended steps(page 17)");
	// Reset
	PX4_WARN("Reseting the sensor");

	if (RegisterWrite(Register::ACC_SOFTRESET, 0xB6) == PX4_OK) {
		PX4_WARN("Reset success");
	}

	usleep(100000);
	PX4_WARN("Accel on");

	if (RegisterWrite(Register::ACC_PWR_CTRL, 0x04) == PX4_OK) {
		PX4_WARN("Accel on success");
	}

	usleep(100000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());
	Configure();
	usleep(1000000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());
	const uint8_t ACC_CHIP_ID = RegisterRead(Register::ACC_CHIP_ID);
	PX4_WARN("ACC_CHIP_ID: 0x%02x", ACC_CHIP_ID);
	usleep(30000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	if (RegisterWrite(Register::ACC_PWR_CONF, 0) == PX4_OK) {
		PX4_WARN("Start sensor success");
		PX4_WARN("ACC_PWR_CONF(0): 0x%02x", RegisterRead(Register::ACC_PWR_CONF));
	}

	usleep(2000000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	if (RegisterWrite(Register::ACC_RANGE, 0x03) == PX4_OK) {
		PX4_WARN("Range set success");
		PX4_WARN("ACC_RANGE(0x03): 0x%02x", RegisterRead(Register::ACC_RANGE));
	}

	usleep(100000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	if (RegisterWrite(Register::ACC_CONF, 0xA7) == PX4_OK) {
		PX4_WARN("Conf set success");
		PX4_WARN("ACC_CONF(0xA7): 0x%02x", RegisterRead(Register::ACC_CONF));
	}

	usleep(100000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	// Positive sel-test polarity
	if (RegisterWrite(Register::ACC_SELF_TEST, 0x0D) == PX4_OK) {
		PX4_WARN("Self-test positive mode set success");
		PX4_WARN("ACC_SELF_TEST(0x0D): 0x%02x", RegisterRead(Register::ACC_SELF_TEST));
	}

	usleep(100000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());

	float *accel_mss = ReadAccelDataFIFO();
	PX4_WARN("Positive value");
	PX4_WARN("X %f", (double)accel_mss[0]);
	PX4_WARN("Y %f", (double)accel_mss[1]);
	PX4_WARN("Z %f", (double)accel_mss[2]);

	// Negative sel-test polarity
	if (RegisterWrite(Register::ACC_SELF_TEST, 0x09) == PX4_OK) {
		PX4_WARN("Self-test negative mode set success");
		PX4_WARN("ACC_SELF_TEST(0x09): 0x%02x", RegisterRead(Register::ACC_SELF_TEST));
	}

	usleep(600000);
	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());
	float *accel_mss2 = ReadAccelDataFIFO();
	PX4_WARN("Negative value");
	PX4_WARN("X %f", (double)accel_mss2[0]);
	PX4_WARN("Y %f", (double)accel_mss2[1]);
	PX4_WARN("Z %f", (double)accel_mss2[2]);

	// Calculate difference between positive and negative sef-test response
	float diff_x = accel_mss[0] - accel_mss2[0];
	float diff_y = accel_mss[1] - accel_mss2[1];
	float diff_z = accel_mss[2] - accel_mss2[2];

	PX4_WARN("Diff value");
	PX4_WARN("diff_x %f", (double)diff_x);
	PX4_WARN("diff_y %f", (double)diff_y);
	PX4_WARN("diff_z %f", (double)diff_z);


	if (diff_x >= 1000) {
		PX4_WARN("X Axis self-test success");
	}

	if (diff_y >= 1000) {
		PX4_WARN("Y Axis self-test success");
	}

	if (diff_z >= 500) {
		PX4_WARN("Z Axis self-test success");
	}


	// Disable self-test
	RegisterWrite(Register::ACC_SELF_TEST, 0x00);
	usleep(60000);

	PX4_WARN("Sensor ErrReg: 0x%02x", CheckSensorErrReg());
	// Reset
	//PX4_WARN("Reseting the sensor again");
	//RegisterWrite(Register::ACC_SOFTRESET, 0xB6);
	//usleep(100000);
	return true;
}

float *BMI088_Accelerometer::ReadAccelData()
{
	uint8_t cmd[1] = {0x12};

	uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
	uint8_t *buffer = buf;

	int16_t accel[3];

	if (transfer(&cmd[0], 1, buffer, sizeof(buf)) == PX4_OK) {
		PX4_WARN("ReadAccelData transfer success");
	}

	for (uint8_t i = 0; i < sizeof(buf); i++) {
		PX4_WARN("buf[%d]: %f", i, (double)buf[i]);
	}

	accel[0] = (buf[1] << 8) | buf[0];
	accel[1] = (buf[3] << 8) | buf[2];
	accel[2] = (buf[5] << 8) | buf[4];

	float *accel_mss = new float[3];

	accel_mss[0] = (float) accel[0] / 32768.0f * 1000.0f * powf(2.0f, 24.0f + 1.0f) * 1.50f;
	accel_mss[1] = (float) accel[1] / 32768.0f * 1000.0f * powf(2.0f, 24.0f + 1.0f) * 1.50f;
	accel_mss[2] = (float) accel[2] / 32768.0f * 1000.0f * powf(2.0f, 24.0f + 1.0f) * 1.50f;

	return accel_mss;
}

float *BMI088_Accelerometer::ReadAccelDataFIFO()
{
	float *accel_mg = new float[3];
	struct FIFO::bmi08x_sensor_data bmi08x_accel;
	uint8_t buffer[50] = {0};

	PX4_WARN("FIFO mode is stop-at-full");
	/* Desired FIFO mode is stop-at-full: set bit #0 to 1 in 0x48. Bit #1 must always be one! */
	buffer[0] = 0x00 | 0x02;
	RegisterWrite(Register::FIFO_CONFIG_0, buffer[0]);
	PX4_WARN("FIFO_CONFIG_0(0x%02x): 0x%02x", buffer[0], RegisterRead(Register::FIFO_CONFIG_0));

	PX4_WARN("Downsampling factor 2**4 = 16");
	/* Downsampling factor 2**4 = 16: write 4 into bit #4-6 of reg. 0x45. Bit #7 must always be one! */
	buffer[0] = 0x10 | 0x80;
	RegisterWrite(Register::FIFO_DOWN_SAMPLING, buffer[0]);
	PX4_WARN("FIFO_DOWN_SAMPLING(0x%02x): 0x%02x", buffer[0], RegisterRead(Register::FIFO_DOWN_SAMPLING));

	/* Set water mark to 42 bytes (aka 6 frames, each 7 bytes: 1 byte header + 6 bytes accel data) */
	// uint16_t wml = 42;
	// buffer[0] = (uint8_t) wml & 0xff;
	// buffer[1] = (uint8_t) (wml >> 8) & 0xff;
	// uint8_t add = static_cast<uint8_t>(Register::FIFO_WTM_0);
	// uint8_t cmd[3] = { add, buffer[0], buffer[1]};
	// transfer(cmd, sizeof(cmd), nullptr, 0);
	// PX4_WARN("FIFO_WTM_0(0x%02x): 0x%02x",cmd[0], RegisterRead(Register::FIFO_WTM_0));

	/* Enable the actual FIFO functionality: write 0x50 to 0x49. Bit #4 must always be one! */
	buffer[0] = 0x10 | 0x40;
	RegisterWrite(Register::FIFO_CONFIG_1, buffer[0]);
	PX4_WARN("FIFO_CONFIG_1(0x%02x): 0x%02x", buffer[0], RegisterRead(Register::FIFO_CONFIG_1));
	usleep(1000000);

	int fifo_fill_level = 0;

	uint8_t data_o[2] = { 0, 0 };
	uint8_t data_i[1] = {static_cast<uint8_t>(Register::FIFO_LENGTH_0)};
	data_i[0] = static_cast<uint8_t>(Register::FIFO_LENGTH_0);

	transfer(&data_i[0], 1, &data_o[0], 2);
	fifo_fill_level = data_o[0] + 256 * data_o[1];
	PX4_WARN("fifo_fill_level %d", fifo_fill_level);

	// while(fifo_fill_level < wml)
	// {
	// 	transfer(&data_i[0], 1, &data_o[0], 2);
	// 	fifo_fill_level = data_o[0] + 256 * data_o[1];
	// 	PX4_WARN("fifo_fill_level %d", fifo_fill_level);
	// }

	uint8_t custom_size = 42;
	uint8_t buffer_data[custom_size] = {0};
	buffer[0] = static_cast<uint8_t>(Register::FIFO_DATA);
	bmi08x_accel.x = 10;
	PX4_WARN("bmi08x_accel %d", bmi08x_accel.x);
	transfer(&buffer[0], 1, &buffer_data[0], custom_size);

	/* This is a super-simple FIFO parsing loop, hoping it will only find valid accel data packets */
	for (int i = 1; i < custom_size;) {
		/* Header of acceleration sensor data frame: 100001xxb, where x is INT1/INT2 tag, ignored here */
		if (buffer_data[i] == (0x84 & 0x8c)) {
			UnpackSensorData(&bmi08x_accel, &buffer_data[i + 1]);
			PX4_WARN("Frame: %03d ax:%f ay:%f az:%f", i / 6, (double)bmi08x_accel.x, (double)bmi08x_accel.y,
				 (double)bmi08x_accel.z);
			accel_mg[0] = bmi08x_accel.x;
			accel_mg[1] = bmi08x_accel.y;
			accel_mg[2] = bmi08x_accel.z;
			float *data_in_mg = SensorDataTomg(accel_mg);
			PX4_WARN("Frame mg: %03d ax:%f ay:%f az:%f", i / 6, (double)data_in_mg[0], (double)data_in_mg[1],
				 (double)data_in_mg[2]);
			i += 7;

		} else {
			i++;
		}
	}

	return accel_mg;
}

uint8_t  BMI088_Accelerometer::CheckSensorErrReg()
{
	return RegisterRead(Register::ACC_ERR_REG);
}

void BMI088_Accelerometer::UnpackSensorData(struct FIFO::bmi08x_sensor_data *sens_data, uint8_t *buffer)
{
	uint16_t data_lsb;
	uint16_t data_msb;
	uint16_t start_idx = 0;
	/* Gyro raw x data */
	data_lsb = buffer[start_idx++];
	data_msb = buffer[start_idx++];
	sens_data->x = (int16_t)((data_msb << 8) | data_lsb);
	/* Gyro raw y data */
	data_lsb = buffer[start_idx++];
	data_msb = buffer[start_idx++];
	sens_data->y = (int16_t)((data_msb << 8) | data_lsb);
	/* Gyro raw z data */
	data_lsb = buffer[start_idx++];
	data_msb = buffer[start_idx++];
	sens_data->z = (int16_t)((data_msb << 8) | data_lsb);
}

float *BMI088_Accelerometer::SensorDataTomg(float *data)
{
	data[0] = (float) data[0] / 32768.0f * 1000.0f * powf(2.0f, 24.0f + 1.0f) * 1.50f;
	data[1] = (float) data[1] / 32768.0f * 1000.0f * powf(2.0f, 24.0f + 1.0f) * 1.50f;
	data[2] = (float) data[2] / 32768.0f * 1000.0f * powf(2.0f, 24.0f + 1.0f) * 1.50f;
	return data;
}

bool BMI088_Accelerometer::NormalRead(const hrt_abstime &timestamp_sample)
{
	const int16_t tX[3] = {1, 0, 0};
	const int16_t tY[3] = {0, -1, 0};
	const int16_t tZ[3] = {0, 0, -1};

	float x = 0;
	float y = 0;
	float z = 0;
	uint8_t buffer[6] = {0};
	uint8_t cmd[1] = {static_cast<uint8_t>(Register::ACC_READ)};

	transfer(&cmd[0], 1, &buffer[0], 6);

	uint8_t RATE_X_LSB = buffer[0];
	uint8_t RATE_X_MSB = buffer[1];

	uint8_t RATE_Y_LSB = buffer[2];
	uint8_t RATE_Y_MSB = buffer[3];

	uint8_t RATE_Z_LSB = buffer[4];
	uint8_t RATE_Z_MSB = buffer[5];

	const int16_t accel_x = combine(RATE_X_MSB, RATE_X_LSB);
	const int16_t accel_y = combine(RATE_Y_MSB, RATE_Y_LSB);
	const int16_t accel_z = combine(RATE_Z_MSB, RATE_Z_LSB);

	// sensor's frame is +x forward, +y left, +z up
	//  flip y & z to publish right handed with z down (x forward, y right, z down)
	x = accel_x * tX[0] + accel_y * tX[1] + accel_z * tX[2];
	y = accel_x * tY[0] + accel_y * tY[1] + accel_z * tY[2];
	z = accel_x * tZ[0] + accel_y * tZ[1] + accel_z * tZ[2];

	//PX4_WARN("x: %f | y: %f | z: %f", (double)x, (double)y ,(double)z);
	_px4_accel.update(timestamp_sample, x, y, z);

	return true;
}

} // namespace Bosch::BMI088::Accelerometer
