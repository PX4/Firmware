/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file LSM303agr.cpp
 * Driver for the ST LSM303AGR MEMS accelerometer / magnetometer connected via SPI.
 * NOTE: currently only the mag is implemented
 */

#include "LSM303AGR.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <ecl/geo/geo.h>

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT			(1<<6)

/* Max measurement rate is 100Hz */
#define CONVERSION_INTERVAL	(1000000 / 100)	/* microseconds */

static constexpr uint8_t LSM303AGR_WHO_AM_I_M = 0x40;

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
 */
#define LSM303AGR_TIMER_REDUCTION				200

LSM303AGR::LSM303AGR(I2CSPIBusOption bus_option, int bus, int device, enum Rotation rotation, int bus_frequency,
		     spi_mode_e spi_mode) :
	SPI(DRV_MAG_DEVTYPE_LSM303AGR, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_mag(get_device_id(), rotation),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": mag_read")),
	_bad_registers(perf_alloc(PC_COUNT, MODULE_NAME": bad_reg")),
	_bad_values(perf_alloc(PC_COUNT, MODULE_NAME": bad_val"))
{
	_px4_mag.set_external(external());

	_px4_mag.set_scale(1.5f / 1000.f); // 1.5 milligauss/LSB
}

LSM303AGR::~LSM303AGR()
{
	/* delete the perf counter */
	perf_free(_mag_sample_perf);
	perf_free(_bad_registers);
	perf_free(_bad_values);
}

int LSM303AGR::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed (%i)", ret);
		return ret;
	}

	reset();

	self_test();

	reset();

	/* fill report structures */
	measure();

	_measure_interval = CONVERSION_INTERVAL;
	start();

	return ret;
}

int LSM303AGR::reset()
{
	// Single mode
	// Output data rate configuration: 100Hz
	write_reg(CFG_REG_A_M, CFG_REG_A_M_MD0 | CFG_REG_A_M_ODR1 | CFG_REG_A_M_ODR0);

	// Enable low pass filter
	write_reg(CFG_REG_B_M, CFG_REG_B_M_OFF_CANC | CFG_REG_B_M_OFF_LPF);

	// Disable I2C
	write_reg(CFG_REG_C_M, CFG_REG_C_M_I2C_DIS);

	return PX4_OK;
}

bool LSM303AGR::self_test()
{
	// Magnetometer self-test procedure (LSM303AGR DocID027765 Rev 5 page 25/68)
	uint8_t status_m = 0;

	write_reg(CFG_REG_A_M, 0x0C);
	write_reg(CFG_REG_B_M, 0x02);
	write_reg(CFG_REG_C_M, 0x10);

	// sleep 20ms
	usleep(20000);

	uint16_t OUTX_NOST = 0;
	uint16_t OUTY_NOST = 0;
	uint16_t OUTZ_NOST = 0;

	// Check Zyxda 50 times and discard
	// average x, y, z
	for (int i = 0; i < 50; i++) {

		status_m = read_reg(STATUS_REG_M);

		OUTX_NOST += read_reg(OUTX_L_REG_M) + (read_reg(OUTX_H_REG_M) << 8);
		OUTY_NOST += read_reg(OUTY_L_REG_M) + (read_reg(OUTY_H_REG_M) << 8);
		OUTZ_NOST += read_reg(OUTZ_L_REG_M) + (read_reg(OUTZ_H_REG_M) << 8);
	}

	// enable self-test
	write_reg(CFG_REG_C_M, 0x12);

	// wait for 60ms
	usleep(60000);

	// Check Zyxda
	status_m = read_reg(STATUS_REG_M);

	// Read mag x, y, z to clear Zyxda bit
	read_reg(OUTX_L_REG_M);
	read_reg(OUTX_H_REG_M);
	read_reg(OUTY_L_REG_M);
	read_reg(OUTY_H_REG_M);
	read_reg(OUTZ_L_REG_M);
	read_reg(OUTZ_H_REG_M);

	uint16_t OUTX_ST = 0;
	uint16_t OUTY_ST = 0;
	uint16_t OUTZ_ST = 0;

	// Read the output registers after checking the Zyxda bit 50 times
	// average x, y, z
	for (int i = 0; i < 50; i++) {

		status_m = read_reg(STATUS_REG_M);

		OUTX_NOST += read_reg(OUTX_L_REG_M) + (read_reg(OUTX_H_REG_M) << 8);
		OUTY_NOST += read_reg(OUTY_L_REG_M) + (read_reg(OUTY_H_REG_M) << 8);
		OUTZ_NOST += read_reg(OUTZ_L_REG_M) + (read_reg(OUTZ_H_REG_M) << 8);
	}

	const uint16_t abs_x = abs(OUTX_ST - OUTX_NOST);
	const uint16_t abs_y = abs(OUTY_ST - OUTY_NOST);
	const uint16_t abs_z = abs(OUTZ_ST - OUTZ_NOST);

	// TODO: proper ranges?
	const bool x_valid = (abs_x > 0 && abs_x < UINT16_MAX);
	const bool y_valid = (abs_y > 0 && abs_y < UINT16_MAX);
	const bool z_valid = (abs_z > 0 && abs_z < UINT16_MAX);

	if (!x_valid || !y_valid || !z_valid) {

		PX4_ERR("self-test failed");

		PX4_INFO("STATUS_M: %X", status_m);
		PX4_INFO("ABS(OUTX_ST - OUTX_NOST) = %d", abs_x);
		PX4_INFO("ABS(OUTY_ST - OUTY_NOST) = %d", abs_y);
		PX4_INFO("ABS(OUTZ_ST - OUTZ_NOST) = %d", abs_z);
	}

	// disable self test
	write_reg(CFG_REG_C_M, 0x10);

	// Idle mode
	write_reg(CFG_REG_A_M, 0x03);

	return true;
}

int LSM303AGR::probe()
{
	/* verify that the device is attached and functioning */
	bool success = (read_reg(WHO_AM_I_M) == LSM303AGR_WHO_AM_I_M);

	if (success) {
		return OK;
	}

	return -EIO;
}

uint8_t LSM303AGR::read_reg(unsigned reg)
{
	uint8_t cmd[2];
	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

int LSM303AGR::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];
	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	return transfer(cmd, nullptr, sizeof(cmd));
}

void LSM303AGR::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void LSM303AGR::RunImpl()
{
	if (_measure_interval == 0) {
		return;
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_interval > CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	measure();

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_interval > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(CONVERSION_INTERVAL);
	}
}

void LSM303AGR::measure()
{
	// Send the command to begin a measurement.
	write_reg(CFG_REG_A_M, CFG_REG_A_M_MD0 | CFG_REG_A_M_ODR1 | CFG_REG_A_M_ODR0);
}

int LSM303AGR::collect()
{
	const uint8_t status = read_reg(STATUS_REG_M);

	_px4_mag.set_error_count(perf_event_count(_bad_registers) + perf_event_count(_bad_values));

	// only publish new data
	if (status & STATUS_REG_M_Zyxda) {
		/* start the performance counter */
		perf_begin(_mag_sample_perf);

		const hrt_abstime timestamp_sample = hrt_absolute_time();

		// switch to right hand coordinate system in place
		int16_t x_raw = read_reg(OUTX_L_REG_M) + (read_reg(OUTX_H_REG_M) << 8);
		int16_t y_raw = read_reg(OUTY_L_REG_M) + (read_reg(OUTY_H_REG_M) << 8);
		int16_t z_raw = -(read_reg(OUTZ_L_REG_M) + (read_reg(OUTZ_H_REG_M) << 8));

		_px4_mag.update(timestamp_sample, x_raw, y_raw, z_raw);

		/* stop the perf counter */
		perf_end(_mag_sample_perf);
	}

	return OK;
}

void LSM303AGR::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_mag_sample_perf);
	perf_print_counter(_bad_registers);
	perf_print_counter(_bad_values);
}
