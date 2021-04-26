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

#include "ADIS16477.hpp"

#define DIR_READ				0x00
#define DIR_WRITE				0x80

//  ADIS16477 registers
static constexpr uint8_t DIAG_STAT	= 0x02; // Output, system error flags
static constexpr uint8_t FILT_CTRL	= 0x5C;
static constexpr uint8_t MSC_CTRL	= 0x60;
static constexpr uint8_t DEC_RATE	= 0x64;
static constexpr uint8_t GLOB_CMD	= 0x68;
static constexpr uint8_t PROD_ID	= 0x72;

static constexpr uint16_t PROD_ID_ADIS16477 = 0x405D;	// ADIS16477 Identification, device number

// Stall time between SPI transfers
static constexpr uint8_t T_STALL = 16;

static constexpr uint32_t ADIS16477_DEFAULT_RATE = 1000;

using namespace time_literals;

ADIS16477::ADIS16477(I2CSPIBusOption bus_option, int bus, int32_t device, enum Rotation rotation, int bus_frequency,
		     spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	SPI(DRV_IMU_DEVTYPE_ADIS16477, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_accel(get_device_id(), rotation),
	_px4_gyro(get_device_id(), rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_bad_transfers(perf_alloc(PC_COUNT, MODULE_NAME": bad transfers")),
	_drdy_gpio(drdy_gpio)
{
#ifdef GPIO_SPI1_RESET_ADIS16477
	// Configure hardware reset line
	px4_arch_configgpio(GPIO_SPI1_RESET_ADIS16477);
#endif // GPIO_SPI1_RESET_ADIS16477

	_px4_accel.set_scale(1.25f * CONSTANTS_ONE_G / 1000.0f); // accel 1.25 mg/LSB
	_px4_gyro.set_scale(math::radians(0.025f)); // gyro 0.025 °/sec/LSB
}

ADIS16477::~ADIS16477()
{
	// delete the perf counters
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
}

int
ADIS16477::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		// if probe/setup failed, bail now
		DEVICE_DEBUG("SPI init failed (%i)", ret);
		return ret;
	}

	start();

	return PX4_OK;
}

int ADIS16477::reset()
{
#ifdef GPIO_SPI1_RESET_ADIS16477
	// Hardware reset
	px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16477, 0);

	// The RST line must be in a low state for at least 10 μs to ensure a proper reset initiation and recovery.
	usleep(10_us);

	px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16477, 1);
#else
	// Software reset (global command bit 7)
	uint8_t value[2] {};
	value[0] = (1 << 7);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);
#endif // GPIO_SPI1_RESET_ADIS16477

	// Reset recovery time
	usleep(193_ms);


	// Miscellaneous Control Register (MSC_CTRL)
	static constexpr uint16_t MSC_CTRL_DEFAULT = 0x00C1;
	usleep(100);
	// verify
	const uint16_t msc_ctrl = read_reg16(MSC_CTRL);

	if (msc_ctrl != MSC_CTRL_DEFAULT) {
		PX4_ERR("invalid setup, MSC_CTRL=%#X", msc_ctrl);
		return PX4_ERROR;
	}


	// Bartlett Window FIR Filter
	static constexpr uint16_t FILT_CTRL_SETUP = 0x0004; // (disabled: 0x0000, 2 taps: 0x0001, 16 taps: 0x0004)
	write_reg16(FILT_CTRL, FILT_CTRL_SETUP);
	usleep(100);
	// verify
	const uint16_t filt_ctrl = read_reg16(FILT_CTRL);

	if (filt_ctrl != FILT_CTRL_SETUP) {
		PX4_ERR("invalid setup, FILT_CTRL=%#X", filt_ctrl);
		return PX4_ERROR;
	}


	// Decimation Filter
	//  set for 1000 samples per second
	static constexpr uint16_t DEC_RATE_SETUP = 0x0001;
	write_reg16(DEC_RATE, DEC_RATE_SETUP);
	usleep(100);
	// verify
	const uint16_t dec_rate = read_reg16(DEC_RATE);

	if (dec_rate != DEC_RATE_SETUP) {
		PX4_ERR("invalid setup, DEC_RATE=%#X", dec_rate);
		return PX4_ERROR;
	}

	return OK;
}

int
ADIS16477::probe()
{
	reset();

	// read product id (5 attempts)
	for (int i = 0; i < 5; i++) {
		uint16_t product_id = read_reg16(PROD_ID);

		if (product_id == PROD_ID_ADIS16477) {
			DEVICE_DEBUG("PRODUCT: %X", product_id);

			if (self_test_memory() && self_test_sensor()) {
				return PX4_OK;

			} else {
				DEVICE_DEBUG("probe attempt %d: self test failed, resetting", i);
				reset();
			}

		} else {
			DEVICE_DEBUG("probe attempt %d: read product id failed, resetting", i);
			reset();
		}
	}

	return -EIO;
}

bool
ADIS16477::self_test_memory()
{
	DEVICE_DEBUG("self test memory");

	// self test (global command bit 4)
	uint8_t value[2] {};
	value[0] = (1 << 4);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);
	usleep(32_ms); // Flash Memory Test Time

	// read DIAG_STAT to check result
	uint16_t diag_stat = read_reg16(DIAG_STAT);

	if (diag_stat != 0) {
		PX4_ERR("DIAG_STAT: %#X", diag_stat);
		return false;
	}

	return true;
}

bool
ADIS16477::self_test_sensor()
{
	PX4_DEBUG("self test sensor");

	// self test (global command bit 2)
	uint8_t value[2] {};
	value[0] = (1 << 2);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);
	usleep(14_ms); // Self Test Time

	// read DIAG_STAT to check result
	uint16_t diag_stat = read_reg16(DIAG_STAT);

	if (diag_stat != 0) {
		PX4_ERR("DIAG_STAT: %#X", diag_stat);
		return false;
	}

	return true;
}

uint16_t
ADIS16477::read_reg16(uint8_t reg)
{
	uint16_t cmd[1] {};

	cmd[0] = ((reg | DIR_READ) << 8) & 0xff00;
	transferhword(cmd, nullptr, 1);
	px4_udelay(T_STALL);
	transferhword(nullptr, cmd, 1);
	px4_udelay(T_STALL);

	return cmd[0];
}

int
ADIS16477::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t cmd[2] {};
	cmd[0] = reg | 0x8;
	cmd[1] = val;
	return transfer(cmd, cmd, sizeof(cmd));
}

void
ADIS16477::write_reg16(uint8_t reg, uint16_t value)
{
	uint16_t cmd[2] {};

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	px4_udelay(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	px4_udelay(T_STALL);
}

void
ADIS16477::start()
{
	if (_drdy_gpio != 0) {
		// Setup data ready on rising edge
		px4_arch_gpiosetevent(_drdy_gpio, true, false, true, &ADIS16477::data_ready_interrupt, this);

	} else {
		// start polling at the specified rate
		ScheduleOnInterval((1_s / ADIS16477_DEFAULT_RATE), 10000);
	}
}

void
ADIS16477::exit_and_cleanup()
{
	if (_drdy_gpio != 0) {
		// Disable data ready callback
		px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);
	}

	I2CSPIDriverBase::exit_and_cleanup();
}

int
ADIS16477::data_ready_interrupt(int irq, void *context, void *arg)
{
	ADIS16477 *dev = static_cast<ADIS16477 *>(arg);

	// make another measurement
	dev->ScheduleNow();

	return PX4_OK;
}

void
ADIS16477::RunImpl()
{
	// make another measurement
	measure();
}

int
ADIS16477::measure()
{
	perf_begin(_sample_perf);

	// Fetch the full set of measurements from the ADIS16477 in one pass (burst read).
	ADISReport adis_report{};
	adis_report.cmd = ((GLOB_CMD | DIR_READ) << 8) & 0xff00;

	// ADIS16477 burst report should be 176 bits
	static_assert(sizeof(adis_report) == (176 / 8), "ADIS16477 report not 176 bits");

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (OK != transferhword((uint16_t *)&adis_report, ((uint16_t *)&adis_report), sizeof(adis_report) / sizeof(uint16_t))) {
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

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
	uint8_t *checksum_helper = (uint8_t *)&adis_report.diag_stat;

	uint8_t checksum = 0;

	for (int i = 0; i < 18; i++) {
		checksum += checksum_helper[i];
	}

	if (adis_report.checksum != checksum) {
		PX4_DEBUG("adis_report.checksum: %X vs calculated: %X", adis_report.checksum, checksum);

		perf_count(_bad_transfers);
		perf_end(_sample_perf);

		return -EIO;
	}

	// Check all Status/Error Flag Indicators (DIAG_STAT)
	if (adis_report.diag_stat != 0) {
		perf_count(_bad_transfers);
		perf_end(_sample_perf);

		return -EIO;
	}

	// temperature 1 LSB = 0.1°C
	const float temperature = adis_report.temp * 0.1f;
	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	_px4_accel.update(timestamp_sample, adis_report.accel_x, adis_report.accel_y, adis_report.accel_z);
	_px4_gyro.update(timestamp_sample, adis_report.gyro_x, adis_report.gyro_y, adis_report.gyro_z);

	perf_end(_sample_perf);

	return OK;
}

void
ADIS16477::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);

}
