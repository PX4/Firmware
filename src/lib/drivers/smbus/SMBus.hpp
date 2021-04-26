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
 * @file SMBus.hpp
 * SMBus v2.0 protocol implementation.
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 */

#include <drivers/device/i2c.h>
#include <perf/perf_counter.h>
#include <string.h>

#define SMBUS_PEC_POLYNOMIAL	0x07	///< Polynomial for calculating PEC

class SMBus : public device::I2C
{
public:
	static constexpr uint8_t MAX_BLOCK_LEN = 34;

	SMBus(int bus_num, uint16_t address);
	~SMBus() override;

	/**
	 * @brief Sends a block write command.
	 * @param cmd_code The command code.
	 * @param data The data to be written.
	 * @param length The number of bytes being written. Maximum is SMBus::MAX_BLOCK_LEN.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	int block_write(const uint8_t cmd_code, const void *data, uint8_t byte_count, const bool use_pec);

	/**
	 * @brief Sends a block read command.
	 * @param cmd_code The command code.
	 * @param data The returned data.
	 * @param length The number of bytes being read. Maximum is SMBus::MAX_BLOCK_LEN.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	int block_read(const uint8_t cmd_code, void *data, const uint8_t length, const bool use_pec);

	/**
	 * @brief Sends a read word command.
	 * @param cmd_code The command code.
	 * @param data The 2 bytes of returned data plus a 1 byte CRC if used.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	int read_word(const uint8_t cmd_code, uint16_t &data);

	/**
	 * @brief Sends a write word command.
	 * @param cmd_code The command code.
	 * @param data The 2 bytes of data to be transfered.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	int write_word(const uint8_t cmd_code, uint16_t data);

	/**
	 * @brief Calculates the PEC from the data.
	 * @param buffer The buffer that stores the data to perform the CRC over.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, -errno on failure.
	 */
	uint8_t get_pec(uint8_t *buffer, uint8_t length);

	perf_counter_t _interface_errors{perf_alloc(PC_COUNT, MODULE_NAME": errors")};

};
