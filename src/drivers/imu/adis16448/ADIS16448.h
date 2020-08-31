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
 * @file adis16448.cpp
 *
 * Driver for the Analog device ADIS16448 connected via SPI.
 *
 * @author Amir Melzer
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <drivers/device/spi.h>
#include <ecl/geo/geo.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace time_literals;

#define DIR_READ                     0x00
#define DIR_WRITE                    0x80

#define ADIS16448_GPIO_CTRL          0x32  /* Auxiliary digital input/output control */
#define ADIS16448_MSC_CTRL           0x34  /* Miscellaneous control */
#define ADIS16448_SMPL_PRD           0x36  /* Internal sample period (rate) control */
#define ADIS16448_SENS_AVG           0x38  /* Dynamic range and digital filter control */
#define ADIS16448_DIAG_STAT          0x3C  /* System status */
#define ADIS16448_GLOB_CMD           0x3E  /* System command */
#define ADIS16448_PRODUCT_ID         0x56  /* Product identifier */
#define ADIS16334_SERIAL_NUMBER      0x58  /* Serial number, lot specific */

#define ADIS16448_Product            0x4040/* Product ID Description for ADIS16448 */

#define BITS_SMPL_PRD_NO_TAP_CFG     (0<<8)
#define BITS_SMPL_PRD_2_TAP_CFG      (1<<8)
#define BITS_SMPL_PRD_4_TAP_CFG      (2<<8)
#define BITS_SMPL_PRD_8_TAP_CFG      (3<<8)
#define BITS_SMPL_PRD_16_TAP_CFG     (4<<8)

#define BITS_GYRO_DYN_RANGE_1000_CFG (4<<8)
#define BITS_GYRO_DYN_RANGE_500_CFG  (2<<8)
#define BITS_GYRO_DYN_RANGE_250_CFG  (1<<8)

#define BITS_FIR_NO_TAP_CFG          (0<<0)
#define BITS_FIR_2_TAP_CFG           (1<<0)
#define BITS_FIR_4_TAP_CFG           (2<<0)
#define BITS_FIR_8_TAP_CFG           (3<<0)
#define BITS_FIR_16_TAP_CFG          (4<<0)
#define BITS_FIR_32_TAP_CFG          (5<<0)
#define BITS_FIR_64_TAP_CFG          (6<<0)
#define BITS_FIR_128_TAP_CFG         (7<<0)

#define T_STALL                         9

static constexpr float ADIS16448_ACCEL_SENSITIVITY{1.0f / 1200.0f * CONSTANTS_ONE_G};		// 1200 LSB/g
static constexpr float ADIS16448_GYRO_INITIAL_SENSITIVITY{math::radians(1.0 / 25.0)};		// 25 LSB/°/sec
static constexpr float ADIS16448_BARO_SENSITIVITY{0.02f};									// 20 microbar per LSB,
static constexpr float ADIS16448_MAG_SENSITIVITY{1.0 / 7.0 / 1000.0};						// 7 LSB/mgauss


static constexpr float ADIS16448_ACCEL_GYRO_UPDATE_RATE{819.2}; // accel and gryo max update 819.2 samples per second
static constexpr float ADIS16448_MAG_BARO_UPDATE_RATE{51.2}; // xMAGN_OUT and BARO_OUT registers update at 51.2 samples per second

class ADIS16448 : public device::SPI, public I2CSPIDriver<ADIS16448>
{
public:
	ADIS16448(I2CSPIBusOption bus_option, int bus, int32_t device, enum Rotation rotation, int bus_frequency,
		  spi_mode_e spi_mode);
	virtual ~ADIS16448();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init() override;

	void print_status() override;

	void RunImpl();
protected:
	int probe() override;

private:

	enum class
	Register : uint8_t {
		GPIO_CTRL		= 0x32,	// Auxiliary digital input/output control
		MSC_CTRL		= 0x34,	// Miscellaneous control
		SMPL_PRD		= 0x36,	// Internal sample period (rate) control
		SENS_AVG		= 0x38,	// Dynamic range and digital filter control

		DIAG_STAT		= 0x3C,	// System status
		GLOB_CMD		= 0x3E,	// System command

		PRODUCT_ID		= 0x56,	// Product identifier
		SERIAL_NUMBER	= 0x58,	// Serial number, lot specific
	};

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	int measure();

	/**
	 * Modify a register in the ADIS16448
	 * Bits are cleared before bits are set.
	 *
	 * @param reg       The register to modify.
	 * @param clearbits Bits in the register to clear.
	 * @param setbits   Bits in the register to set.
	 */
	void modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits);

	/**
	 * Resets the chip and measurements ranges
	 */
	bool reset();

	bool self_test();

	/**
	 * Read a register from the ADIS16448
	 * @arg reg The register to read.
	 * @return Returns the register value.
	 */
	uint16_t read_reg16(unsigned reg);

	/**
	 * Write a register in the ADIS16448
	 * @param reg       The register to write.
	 * @param value     The new value to write.
	 */
	void write_reg16(unsigned reg, uint16_t value);

	/**
	 * Set low pass filter frequency.
	 */
	bool set_dlpf_filter(uint16_t frequency_hz);

	/**
	 * Set the gyroscope dynamic range.
	 */
	bool set_gyro_dyn_range(uint16_t desired_gyro_dyn_range);

	/**
	 * Set sample rate (approximate) - 1kHz to 5Hz.
	 */
	bool set_sample_rate(uint16_t desired_sample_rate_hz);

	/**
	 * Start automatic measurement.
	 */
	void start();

	PX4Accelerometer	_px4_accel;
	PX4Barometer		_px4_baro;
	PX4Gyroscope		_px4_gyro;
	PX4Magnetometer		_px4_mag;

	uint16_t _product_ID{0};    // Product ID code.

	static constexpr float _sample_rate{ADIS16448_ACCEL_GYRO_UPDATE_RATE};

	perf_counter_t _perf_read{perf_counter_t(perf_alloc(PC_ELAPSED, "ADIS16448: read"))};
	perf_counter_t _perf_transfer{perf_counter_t(perf_alloc(PC_ELAPSED, "ADIS16448: transfer"))};
	perf_counter_t _perf_bad_transfer{perf_counter_t(perf_alloc(PC_COUNT, "ADIS16448: bad transfers"))};
	perf_counter_t _perf_crc_bad{perf_counter_t(perf_alloc(PC_COUNT, "ADIS16448: CRC16 bad"))};

};
