/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file batt_smbus.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2 or BQ40Z80
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 */

#pragma once

#include <ecl/geo/geo.h>
#include <lib/drivers/smbus/SMBus.hpp>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/battery_status.h>

#include <board_config.h>

using namespace time_literals;

#define BATT_SMBUS_MEASUREMENT_INTERVAL_US              100_ms         ///< time in microseconds, measure at 10Hz

#define MAC_DATA_BUFFER_SIZE                            32

#define BATT_CELL_VOLTAGE_THRESHOLD_RTL                 0.5f            ///< Threshold in volts to RTL if cells are imbalanced
#define BATT_CELL_VOLTAGE_THRESHOLD_FAILED              1.5f            ///< Threshold in volts to Land if cells are imbalanced

#define BATT_CURRENT_UNDERVOLTAGE_THRESHOLD             5.0f            ///< Threshold in amps to disable undervoltage protection
#define BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD             3.4f            ///< Threshold in volts to re-enable undervoltage protection

#define BATT_SMBUS_ADDR                                 0x0B            ///< Default 7 bit address I2C address. 8 bit = 0x16

#define BATT_SMBUS_TEMP                                 0x08            ///< temperature register
#define BATT_SMBUS_VOLTAGE                              0x09            ///< voltage register
#define BATT_SMBUS_CURRENT                              0x0A            ///< current register
#define BATT_SMBUS_AVERAGE_CURRENT                      0x0B            ///< average current register
#define BATT_SMBUS_MAX_ERROR                            0x0C            ///< max error
#define BATT_SMBUS_RELATIVE_SOC                         0x0D            ///< Relative State Of Charge
#define BATT_SMBUS_ABSOLUTE_SOC                         0x0E            ///< Absolute State of charge
#define BATT_SMBUS_REMAINING_CAPACITY                   0x0F            ///< predicted remaining battery capacity as a percentage
#define BATT_SMBUS_FULL_CHARGE_CAPACITY                 0x10            ///< capacity when fully charged
#define BATT_SMBUS_RUN_TIME_TO_EMPTY                    0x11            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_AVERAGE_TIME_TO_EMPTY                0x12            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_CYCLE_COUNT                          0x17            ///< number of cycles the battery has experienced
#define BATT_SMBUS_DESIGN_CAPACITY                      0x18            ///< design capacity register
#define BATT_SMBUS_DESIGN_VOLTAGE                       0x19            ///< design voltage register
#define BATT_SMBUS_MANUFACTURER_NAME                    0x20            ///< manufacturer name
#define BATT_SMBUS_MANUFACTURER_NAME_SIZE               21              ///< manufacturer name data size
#define BATT_SMBUS_MANUFACTURE_DATE                     0x1B            ///< manufacture date register
#define BATT_SMBUS_SERIAL_NUMBER                        0x1C            ///< serial number register

#define BATT_SMBUS_BQ40Z50_CELL_4_VOLTAGE               0x3C
#define BATT_SMBUS_BQ40Z50_CELL_3_VOLTAGE               0x3D
#define BATT_SMBUS_BQ40Z50_CELL_2_VOLTAGE               0x3E
#define BATT_SMBUS_BQ40Z50_CELL_1_VOLTAGE               0x3F

#define BATT_SMBUS_BQ40Z80_CELL_7_VOLTAGE               0x3C
#define BATT_SMBUS_BQ40Z80_CELL_6_VOLTAGE               0x3D
#define BATT_SMBUS_BQ40Z80_CELL_5_VOLTAGE               0x3E
#define BATT_SMBUS_BQ40Z80_CELL_4_VOLTAGE               0x3F

#define BATT_SMBUS_STATE_OF_HEALTH                      0x4F            ///< State of Health. The SOH information of the battery in percentage of Design Capacity

#define BATT_SMBUS_MANUFACTURER_ACCESS                  0x00
#define BATT_SMBUS_MANUFACTURER_DATA                    0x23
#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS            0x44

#define BATT_SMBUS_SECURITY_KEYS                        0x0035

#define BATT_SMBUS_LIFETIME_FLUSH                       0x002E
#define BATT_SMBUS_LIFETIME_BLOCK_ONE                   0x0060
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS        0x4938
#define BATT_SMBUS_SEAL                                 0x0030
#define BATT_SMBUS_DASTATUS1                            0x0071
#define BATT_SMBUS_DASTATUS2                            0x0072
#define BATT_SMBUS_DASTATUS3                            0x007B

#define BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT        0xcf
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED   0xce


enum class SMBUS_DEVICE_TYPE {
	UNDEFINED  = 0,
	BQ40Z50    = 1,
	BQ40Z80    = 2,
};

class BATT_SMBUS : public I2CSPIDriver<BATT_SMBUS>
{
public:
	BATT_SMBUS(I2CSPIBusOption bus_option, const int bus, SMBus *interface);

	~BATT_SMBUS();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	friend SMBus;

	void RunImpl();

	void custom_method(const BusCLIArguments &cli) override;

	/**
	 * @brief Reads data from flash.
	 * @param address The address to start the read from.
	 * @param data The returned data.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_read(const uint16_t address, void *data, const unsigned length);

	/**
	 * @brief Writes data to flash.
	 * @param address The start address of the write.
	 * @param data The data to be written.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_write(const uint16_t address, void *data, const unsigned length);

	/**
	* @brief Read info from battery on startup.
	* @return Returns PX4_OK on success, PX4_ERROR on failure.
	*/
	int get_startup_info();

	/**
	 * @brief Performs a ManufacturerBlockAccess() read command.
	 * @param cmd_code The command code.
	 * @param data The returned data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Performs a ManufacturerBlockAccess() write command.
	 * @param cmd_code The command code.
	 * @param data The sent data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Unseals the battery to allow writing to restricted flash.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int unseal();

	/**
	 * @brief Seals the battery to disallow writing to restricted flash.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int seal();

	/**
	 * @brief This command flushes the RAM Lifetime Data to data flash to help streamline evaluation testing.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_data_flush();

	/**
	 * @brief Reads the lifetime data from block 1.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_read_block_one();

	/**
	 * @brief Reads the cell voltages.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int get_cell_voltages();

	/**
	 * @brief Enables or disables the cell under voltage protection emergency shut off.
	 */
	void set_undervoltage_protection(float average_current);

	void suspend();

	void resume();

private:

	SMBus *_interface;

	SMBUS_DEVICE_TYPE _device_type{SMBUS_DEVICE_TYPE::UNDEFINED};

	perf_counter_t _cycle{perf_alloc(PC_ELAPSED, "batt_smbus_cycle")};

	static const uint8_t MAX_NUM_OF_CELLS = 7;
	float _cell_voltages[MAX_NUM_OF_CELLS] {};

	float _max_cell_voltage_delta{0};

	float _min_cell_voltage{0};

	float _pack_power{0};
	float _pack_average_power{0};

	/** @param _last_report Last published report, used for test(). */
	battery_status_s _last_report{};

	/** @param _batt_topic uORB battery topic. */
	orb_advert_t _batt_topic{nullptr};

	/** @param _cell_count Number of series cell. */
	uint8_t _cell_count{0};

	/** @param _batt_capacity Battery design capacity in mAh (0 means unknown). */
	uint16_t _batt_capacity{0};

	/** @param _batt_startup_capacity Battery remaining capacity in mAh on startup. */
	uint16_t _batt_startup_capacity{0};

	/** @param _cycle_count The number of cycles the battery has experienced. */
	uint16_t _cycle_count{0};

	/** @param _serial_number Serial number register. */
	uint16_t _serial_number{0};

	/** @param _crit_thr Critical battery threshold param. */
	float _crit_thr{0.f};

	/** @param _emergency_thr Emergency battery threshold param. */
	float _emergency_thr{0.f};

	/** @param _low_thr Low battery threshold param. */
	float _low_thr{0.f};

	/** @parama _c_mult Capacity/current multiplier param  */
	float _c_mult{0.f};

	/** @param _manufacturer_name Name of the battery manufacturer. */
	char _manufacturer_name[BATT_SMBUS_MANUFACTURER_NAME_SIZE + 1] {};	// Plus one for terminator

	/** @param _manufacture_date Date of the battery manufacturing. */
	uint16_t _manufacture_date{0};

	/** @param _state_of_health state of health as read on connection  */
	float _state_of_health{0.f};

	/** @param _lifetime_max_delta_cell_voltage Max lifetime delta of the battery cells */
	float _lifetime_max_delta_cell_voltage{0.f};

	/** @param _cell_undervoltage_protection_status 0 if protection disabled, 1 if enabled */
	uint8_t _cell_undervoltage_protection_status{1};

	BATT_SMBUS(const BATT_SMBUS &) = delete;
	BATT_SMBUS operator=(const BATT_SMBUS &) = delete;
};
