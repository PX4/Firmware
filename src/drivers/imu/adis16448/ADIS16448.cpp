/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file ADIS16448.cpp
 */

#include "ADIS16448.h"

ADIS16448::ADIS16448(I2CSPIBusOption bus_option, int bus, int32_t device, enum Rotation rotation, int bus_frequency,
		     spi_mode_e spi_mode) :
	SPI(DRV_IMU_DEVTYPE_ADIS16448, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_accel(get_device_id(), rotation),
	_px4_baro(get_device_id()),
	_px4_gyro(get_device_id(), rotation),
	_px4_mag(get_device_id(), rotation)
{
	_px4_accel.set_scale(ADIS16448_ACCEL_SENSITIVITY);
	_px4_gyro.set_scale(ADIS16448_GYRO_INITIAL_SENSITIVITY);
	_px4_mag.set_scale(ADIS16448_MAG_SENSITIVITY);

	_px4_mag.set_external(external());
}

ADIS16448::~ADIS16448()
{
	// Delete the perf counter.
	perf_free(_perf_read);
	perf_free(_perf_transfer);
	perf_free(_perf_bad_transfer);
	perf_free(_perf_crc_bad);
}

int
ADIS16448::init()
{
	// Do SPI init (and probe) first.
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI setup failed %d", ret);

		// If probe/setup failed, return result.
		return ret;
	}

	ret = measure();

	if (ret != PX4_OK) {
		PX4_ERR("measure failed");
		return PX4_ERROR;
	}

	start();

	return OK;
}

bool ADIS16448::reset()
{
	// Software reset
	write_reg16(ADIS16448_GLOB_CMD, 1 << 7); // GLOB_CMD bit 7 Software reset

	// Reset Recovery Time 90 ms
	usleep(90000);

	if (!self_test()) {
		return false;
	}

	// Factory calibration restore
	//write_reg16(ADIS16448_GLOB_CMD, 1 << 1); // GLOB_CMD bit 1 Factory calibration restore

	// include the CRC-16 code in burst read output sequence
	write_reg16(ADIS16448_MSC_CTRL, 1 << 4);

	// Set digital FIR filter tap.
	//if (!set_dlpf_filter(BITS_FIR_NO_TAP_CFG)) {
	//	return PX4_ERROR;
	//}

	// Set IMU sample rate.
	if (!set_sample_rate(_sample_rate)) {
		return false;
	}

	// Set gyroscope scale to default value.
	//if (!set_gyro_dyn_range(GYRO_INITIAL_SENSITIVITY)) {
	//	return false;
	//}

	// Settling time.
	usleep(100000);

	return true;
}

bool ADIS16448::self_test()
{
	bool ret = true;

	// start internal self test routine
	write_reg16(ADIS16448_MSC_CTRL, 0x04);	// MSC_CTRL bit 10 Internal self test (cleared upon completion)

	// Automatic Self-Test Time 45 ms
	usleep(45000);

	// check test status (ADIS16448_DIAG_STAT)
	const uint16_t status = read_reg16(ADIS16448_DIAG_STAT);

	const bool self_test_error = (status & (1 << 5));	// 5: Self-test diagnostic error flag

	if (self_test_error) {
		//PX4_ERR("self test failed DIAG_STAT: 0x%04X", status);

		// Magnetometer
		const bool mag_fail = (status & (1 << 0));			// 0: Magnetometer functional test

		if (mag_fail) {
			// tolerate mag test failure (likely due to surrounding magnetic field)
			PX4_WARN("Magnetometer functional test fail");
		}

		// Barometer
		const bool baro_fail = (status & (1 << 1));			//  1: Barometer functional test

		if (baro_fail) {
			PX4_ERR("Barometer functional test test fail");
			ret = false;
		}

		// Gyroscope
		const bool gyro_x_fail = (status & (1 << 10));		// 10: X-axis gyroscope self-test failure
		const bool gyro_y_fail = (status & (1 << 11));		// 11: Y-axis gyroscope self-test failure
		const bool gyro_z_fail = (status & (1 << 12));		// 12: Z-axis gyroscope self-test failure

		if (gyro_x_fail || gyro_y_fail || gyro_z_fail) {
			PX4_ERR("gyroscope self-test failure");
			ret = false;
		}

		// Accelerometer
		const bool accel_x_fail = (status & (1 << 13));		// 13: X-axis accelerometer self-test failure
		const bool accel_y_fail = (status & (1 << 14));		// 14: Y-axis accelerometer self-test failure
		const bool accel_z_fail = (status & (1 << 15));		// 15: Z-axis accelerometer self-test failure

		if (accel_x_fail || accel_y_fail || accel_z_fail) {
			PX4_ERR("accelerometer self-test failure");
			ret = false;
		}
	}

	return ret;
}

int
ADIS16448::probe()
{
	bool reset_success = reset();

	// Retry 5 time to get the ADIS16448 PRODUCT ID number.
	for (size_t i = 0; i < 5; i++) {
		// Read product ID.
		_product_ID = read_reg16(ADIS16448_PRODUCT_ID);

		if (_product_ID == ADIS16448_Product) {
			break;
		}

		reset_success = reset();
	}

	if (!reset_success) {
		DEVICE_DEBUG("unable to successfully reset");
		return PX4_ERROR;
	}

	// Recognize product serial number.
	uint16_t serial_number = (read_reg16(ADIS16334_SERIAL_NUMBER) & 0xfff);

	// Verify product ID.
	switch (_product_ID) {
	case ADIS16448_Product:
		DEVICE_DEBUG("ADIS16448 is detected ID: 0x%02x, Serial: 0x%02x", _product_ID, serial_number);
		modify_reg16(ADIS16448_GPIO_CTRL, 0x0200, 0x0002);  // Turn on ADIS16448 adaptor board led.
		return OK;
	}

	DEVICE_DEBUG("unexpected ID 0x%02x", _product_ID);

	return -EIO;
}

bool
ADIS16448::set_sample_rate(uint16_t desired_sample_rate_hz)
{
	uint16_t smpl_prd = 0;

	if (desired_sample_rate_hz <= 51) {
		smpl_prd = BITS_SMPL_PRD_16_TAP_CFG;

	} else if (desired_sample_rate_hz <= 102) {
		smpl_prd = BITS_SMPL_PRD_8_TAP_CFG;

	} else if (desired_sample_rate_hz <= 204) {
		smpl_prd = BITS_SMPL_PRD_4_TAP_CFG;

	} else if (desired_sample_rate_hz <= 409) {
		smpl_prd = BITS_SMPL_PRD_2_TAP_CFG;

	} else {
		smpl_prd = BITS_SMPL_PRD_NO_TAP_CFG;
	}

	modify_reg16(ADIS16448_SMPL_PRD, 0x1F00, smpl_prd);

	if ((read_reg16(ADIS16448_SMPL_PRD) & 0x1F00) != smpl_prd) {
		PX4_ERR("failed to set IMU sample rate");

		return false;
	}

	return true;
}

bool
ADIS16448::set_dlpf_filter(uint16_t desired_filter_tap)
{
	// Set the DLPF FIR filter tap. This affects both accelerometer and gyroscope.
	modify_reg16(ADIS16448_SENS_AVG, 0x0007, desired_filter_tap);

	// Verify data write on the IMU.
	if ((read_reg16(ADIS16448_SENS_AVG) & 0x0007) != desired_filter_tap) {
		PX4_ERR("failed to set IMU filter");

		return false;
	}

	return true;
}

bool
ADIS16448::set_gyro_dyn_range(uint16_t desired_gyro_dyn_range)
{
	uint16_t gyro_range_selection = 0;

	if (desired_gyro_dyn_range <= 250) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_250_CFG;

	} else if (desired_gyro_dyn_range <= 500) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_500_CFG;

	} else {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_1000_CFG;
	}

	modify_reg16(ADIS16448_SENS_AVG, 0x0700, gyro_range_selection);

	// Verify data write on the IMU.
	if ((read_reg16(ADIS16448_SENS_AVG) & 0x0700) != gyro_range_selection) {
		PX4_ERR("failed to set gyro range");
		return false;

	} else {
		_px4_gyro.set_scale(((gyro_range_selection >> 8) / 100.0f) * M_PI_F / 180.0f);
	}

	return true;
}

void
ADIS16448::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_perf_read);
	perf_print_counter(_perf_transfer);
	perf_print_counter(_perf_bad_transfer);
	perf_print_counter(_perf_crc_bad);
}

void
ADIS16448::modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits)
{
	uint16_t val = read_reg16(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg16(reg, val);
}

uint16_t
ADIS16448::read_reg16(unsigned reg)
{
	uint16_t cmd[1];

	cmd[0] = ((reg | DIR_READ) << 8) & 0xff00;

	transferhword(cmd, nullptr, 1);
	usleep(T_STALL);
	transferhword(nullptr, cmd, 1);
	usleep(T_STALL);

	return cmd[0];
}

void
ADIS16448::write_reg16(unsigned reg, uint16_t value)
{
	uint16_t cmd[2];

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	usleep(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	usleep(T_STALL);
}

void
ADIS16448::start()
{
	// Start polling at the specified interval
	ScheduleOnInterval((1_s / _sample_rate), 10000);
}

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

/**
 * convert 12 bit integer format to int16.
 */
static int16_t
convert12BitToINT16(uint16_t word)
{
	int16_t outputbuffer = 0;

	if ((word >> 11) & 0x1) {
		outputbuffer = (word & 0xfff) | 0xf000;

	} else {
		outputbuffer = (word & 0x0fff);
	}

	return (outputbuffer);
}

int
ADIS16448::measure()
{
	// Start measuring.
	perf_begin(_perf_read);

	// Fetch the full set of measurements from the ADIS16448 in one pass (burst read).
#pragma pack(push, 1) // Ensure proper memory alignment.
	struct Report {
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
	} report{};
#pragma pack(pop)

	report.cmd = ((ADIS16448_GLOB_CMD | DIR_READ) << 8) & 0xff00;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	perf_begin(_perf_transfer);

	if (OK != transferhword((uint16_t *)&report, ((uint16_t *)&report), sizeof(report) / sizeof(int16_t))) {
		perf_end(_perf_transfer);
		perf_end(_perf_read);

		perf_count(_perf_bad_transfer);

		return -EIO;
	}

	perf_end(_perf_transfer);

	// checksum
	if (report.CRC16 != ComputeCRC16((uint16_t *)&report.DIAG_STAT)) {
		perf_count(_perf_crc_bad);
		perf_end(_perf_read);
		return -EIO;
	}

	// error count
	const uint64_t error_count = perf_event_count(_perf_bad_transfer) + perf_event_count(_perf_crc_bad);

	// temperature
	const float temperature = (convert12BitToINT16(report.TEMP_OUT) * 0.07386f) + 31.0f; // 0.07386°C/LSB, 31°C = 0x000

	_px4_accel.set_error_count(error_count);
	_px4_accel.set_temperature(temperature);
	_px4_accel.update(timestamp_sample, report.XACCL_OUT, report.YACCL_OUT, report.ZACCL_OUT);

	_px4_gyro.set_error_count(error_count);
	_px4_gyro.set_temperature(temperature);
	_px4_gyro.update(timestamp_sample, report.XGYRO_OUT, report.YGYRO_OUT, report.ZGYRO_OUT);

	// DIAG_STAT bit 7: New data, xMAGN_OUT/BARO_OUT
	const bool baro_mag_update = (report.DIAG_STAT & (1 << 7));

	if (baro_mag_update) {
		_px4_mag.set_error_count(error_count);
		_px4_mag.set_temperature(temperature);
		_px4_mag.update(timestamp_sample, report.XMAGN_OUT, report.YMAGN_OUT, report.ZMAGN_OUT);

		_px4_baro.set_error_count(error_count);
		_px4_baro.set_temperature(temperature);
		_px4_baro.update(timestamp_sample, report.BARO_OUT * ADIS16448_BARO_SENSITIVITY);
	}

	// Stop measuring.
	perf_end(_perf_read);

	return OK;
}

void
ADIS16448::RunImpl()
{
	// Make another measurement.
	measure();
}
